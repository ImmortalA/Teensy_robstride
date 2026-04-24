#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

#define USE_STATIC_IP 1
#if USE_STATIC_IP
IPAddress teensyIP(192, 168, 0, 101);
IPAddress teensySubnet(255, 255, 255, 0);
IPAddress teensyGateway(192, 168, 0, 1);
#endif
constexpr uint32_t kDHCPTimeout = 15000; // 15 seconds
constexpr uint16_t kPort = 8003;         // udp port (board 0; test_spine listens here, PC = 192.168.0.100)
constexpr int MAX_NODES = 3;             // Maximum number of nodes
constexpr int MAX_NUM_SAMPLES = 5000;

EthernetUDP udp;

uint8_t can_data[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command[MAX_NODES][8]; // CAN command buffer for each node

uint8_t can_data_bus2[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command_bus2[MAX_NODES][8]; // CAN command buffer for each node

unsigned long last_packet_time_bus1[MAX_NODES] = {0};
unsigned long total_latency_bus1[MAX_NODES] = {0};
unsigned int packet_count_bus1[MAX_NODES] = {0};

unsigned long last_packet_time_bus2[MAX_NODES] = {0};
unsigned long total_latency_bus2[MAX_NODES] = {0};
unsigned int packet_count_bus2[MAX_NODES] = {0};

// Timestamp of last command send per node (for command->feedback latency)
unsigned long last_cmd_time_bus1[MAX_NODES] = {0};
unsigned long last_cmd_time_bus2[MAX_NODES] = {0};
// Accumulators for command->feedback latency (for averaging / debug)
unsigned long total_cmd_latency_bus1[MAX_NODES] = {0};
unsigned int  cmd_count_bus1[MAX_NODES]        = {0};
unsigned long total_cmd_latency_bus2[MAX_NODES] = {0};
unsigned int  cmd_count_bus2[MAX_NODES]        = {0};

// One-shot timing: UDP control receive -> next feedback frame
unsigned long udp_cmd_start_time_bus1[MAX_NODES] = {0};
bool          udp_cmd_waiting_fb_bus1[MAX_NODES] = {false};
unsigned long udp_cmd_start_time_bus2[MAX_NODES] = {0};
bool          udp_cmd_waiting_fb_bus2[MAX_NODES] = {false};

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

// Simple motor enable-only helper (RobStride O2, 29-bit extended CAN) – stop -> zero -> enable once at startup.
// Change MOTOR_ID to match your motor if needed (e.g. 0 or 1).
#define MOTOR_ID  1
#define CANCOM_MOTOR_ENABLE   3
#define CANCOM_MOTOR_RESET    4
#define CANCOM_MOTOR_ZERO     6

static inline uint32_t makeO2ExtendedId(uint8_t motor_id, uint8_t type) {
    return ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)(motor_id & 0xFF));
}
// Arduino/RobStride use 29-bit ID: (mode<<24)|(data<<8)|id. Type 1 puts torque in data (16-bit).
// Use this for Type 1 so motor keeps running; 0 Nm = 32768 in 16-bit (-12..+12 Nm).
static inline uint32_t makeO2ExtendedIdWithData(uint8_t motor_id, uint8_t type, uint16_t data16) {
    return ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)(data16 & 0xFFFF) << 8) | ((uint32_t)(motor_id & 0xFF));
}

// Number of motors to enable at startup on each bus (1..MAX_NODES). Use 2 for daisy chain of 2 motors.
#define NUM_MOTORS_ENABLE_AT_STARTUP  2

static void sendSimpleEnableSequence() {
    CAN_message_t msg;
    uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(msg.buf, payload, 8);

    Serial.println("Simple O2 enable sequence on both CAN buses.");
    for (int n = 0; n < NUM_MOTORS_ENABLE_AT_STARTUP && n < MAX_NODES; n++) {
        uint8_t id = (n == 0) ? MOTOR_ID : (MOTOR_ID + n);
        // Stop (Type 4)
        msg.id = makeO2ExtendedId(id, CANCOM_MOTOR_RESET);
        Serial.print("Stop (Type 4) motor ID ");
        Serial.println(id);
        Can0.write(msg);
        Can1.write(msg);
        delay(200);

        // Zero (Type 6)
        msg.id = makeO2ExtendedId(id, CANCOM_MOTOR_ZERO);
        Serial.print("Zero (Type 6) motor ID ");
        Serial.println(id);
        Can0.write(msg);
        Can1.write(msg);
        delay(200);

        // Enable (Type 3) — no Type 1 here so motor stays still until host sends.
        msg.id = makeO2ExtendedId(id, CANCOM_MOTOR_ENABLE);
        Serial.print("Enable (Type 3) motor ID ");
        Serial.println(id);
        Can0.write(msg);
        Can1.write(msg);
        delay(200);
    }
    Serial.println("Enable sequence sent.");
}

// Calculate CRC-8 checksum
// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;
uint8_t calculate_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
        ;
    printf("Starting...\r\n");

    setupEthernet();
    setupCAN();
    sendSimpleEnableSequence();
    Serial.println("======== Setup ends ========");
}

void setupEthernet()
{
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Ethernet.onLinkState([](bool state)
                         { printf("[Ethernet] Link %s\r\n", state ? "ON" : "OFF"); });

#if USE_STATIC_IP
    printf("Starting Ethernet with static IP...\r\n");
    if (!Ethernet.begin(teensyIP, teensySubnet, teensyGateway))
    {
        printf("Failed to start Ethernet (static)\r\n");
        return;
    }
    printf("Static IP ");
    printIPAddress();
#else
    printf("Starting Ethernet with DHCP...\r\n");
    if (!Ethernet.begin())
    {
        printf("Failed to start Ethernet\r\n");
        return;
    }
    if (!Ethernet.waitForLocalIP(kDHCPTimeout))
    {
        printf("Failed to get IP address from DHCP\r\n");
        return;
    }
    printf("Ethernet speed: %d\r\n", Ethernet.linkSpeed());
    printIPAddress();
#endif

    udp.begin(kPort);
    printf("Done setting Ethernet\r\n");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void setupCAN()
{
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can0.setMBFilter(MB1, 0, 0x0);
    Can0.enhanceFilter(MB1);
    Can0.distribute();
    Can0.enableMBInterrupts();

    Can0.onReceive(canReceive);
    Can0.setClock(CLK_60MHz);
    printf("Done setting up CAN 0\n");

    Can1.begin();
    Can1.setBaudRate(1000000);
    Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can1.setMBFilter(MB1, 0, 0x0);
    Can1.enhanceFilter(MB1);
    Can1.enableMBInterrupts();
    Can1.onReceive(canReceive2);
    Can1.distribute();
    Can1.setClock(CLK_60MHz);
    printf("Done setting up CAN 1\n");
}


void printIPAddress()
{
    IPAddress ip = Ethernet.localIP();
    printf("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    printf("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.broadcastIP();
    printf("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    printf("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    printf("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

// O2 Type 2 feedback: type in ID bits 28-24, motor ID in bits 7-0. Payload is p, v, torque, temp.
void canReceive(const CAN_message_t &msg)
{
    uint8_t type = (uint8_t)((msg.id >> 24) & 0x1F);
    if (type != 2)
        return;  // only accept Type 2 (feedback)
    uint8_t motor_id = (uint8_t)(msg.id & 0xFF);
    int node_id = (int)(motor_id - MOTOR_ID);
    if (node_id < 0)
        node_id = 0;  // single-motor fallback: some drives send ID 0
    if (node_id >= MAX_NODES)
        return;
    memcpy(can_data[node_id], msg.buf, 8);
    unsigned long current_time = micros();

    // (A) Existing measurement: time between feedback frames
    unsigned long fb_interval = current_time - last_packet_time_bus1[node_id];
    total_latency_bus1[node_id] += fb_interval;
    packet_count_bus1[node_id]++;

    // (B) New measurement: time from last command send to this feedback
    if (last_cmd_time_bus1[node_id] != 0)
    {
        unsigned long cmd_to_fb = current_time - last_cmd_time_bus1[node_id];
        total_cmd_latency_bus1[node_id] += cmd_to_fb;
        cmd_count_bus1[node_id]++;
    }

    // (C) One-shot measurement: from last UDP control receive to first feedback (also as Hz)
    if (udp_cmd_waiting_fb_bus1[node_id] && udp_cmd_start_time_bus1[node_id] != 0)
    {
        unsigned long udp_to_fb = current_time - udp_cmd_start_time_bus1[node_id];
        float udp_rate_hz = 0.0f;
        if (udp_to_fb > 0)
            udp_rate_hz = 1000000.0f / (float)udp_to_fb;
        printf("Bus 1 node %d: UDP->feedback one-shot=%.2f Hz (%lu us)\n",
               node_id + 1, udp_rate_hz, udp_to_fb);
        udp_cmd_waiting_fb_bus1[node_id] = false;
    }

    if (packet_count_bus1[node_id] == MAX_NUM_SAMPLES)
    {
        unsigned long average_fb_interval = total_latency_bus1[node_id] / MAX_NUM_SAMPLES;
        unsigned long average_cmd_to_fb = 0;
        if (cmd_count_bus1[node_id] > 0)
            average_cmd_to_fb = total_cmd_latency_bus1[node_id] / cmd_count_bus1[node_id];

        // Only keep avg cmd->fb in us; fb interval in Hz not printed anymore
        printf("Bus 1 node %d: avg cmd->fb=%lu us (avg fb interval=%lu us)\n",
               node_id + 1, average_cmd_to_fb, average_fb_interval);

        total_latency_bus1[node_id] = 0;
        packet_count_bus1[node_id] = 0;
        total_cmd_latency_bus1[node_id] = 0;
        cmd_count_bus1[node_id] = 0;
    }

    last_packet_time_bus1[node_id] = current_time;
}

// O2 Type 2 feedback: type in ID bits 28-24, motor ID in bits 7-0.
void canReceive2(const CAN_message_t &msg)
{
    uint8_t type = (uint8_t)((msg.id >> 24) & 0x1F);
    if (type != 2)
        return;
    uint8_t motor_id = (uint8_t)(msg.id & 0xFF);
    int node_id = (int)(motor_id - MOTOR_ID);
    if (node_id < 0)
        node_id = 0;
    if (node_id >= MAX_NODES)
        return;
    memcpy(can_data_bus2[node_id], msg.buf, 8);
    unsigned long current_time = micros();

    // (A) Existing measurement: time between feedback frames
    unsigned long fb_interval = current_time - last_packet_time_bus2[node_id];
    total_latency_bus2[node_id] += fb_interval;
    packet_count_bus2[node_id]++;

    // (B) New measurement: time from last command send to this feedback
    if (last_cmd_time_bus2[node_id] != 0)
    {
        unsigned long cmd_to_fb = current_time - last_cmd_time_bus2[node_id];
        total_cmd_latency_bus2[node_id] += cmd_to_fb;
        cmd_count_bus2[node_id]++;
    }

    // (C) One-shot measurement: from last UDP control receive to first feedback (also as Hz)
    if (udp_cmd_waiting_fb_bus2[node_id] && udp_cmd_start_time_bus2[node_id] != 0)
    {
        unsigned long udp_to_fb = current_time - udp_cmd_start_time_bus2[node_id];
        float udp_rate_hz = 0.0f;
        if (udp_to_fb > 0)
            udp_rate_hz = 1000000.0f / (float)udp_to_fb;
        printf("Bus 2 node %d: UDP->feedback one-shot=%.2f Hz (%lu us)\n",
               node_id + 1, udp_rate_hz, udp_to_fb);
        udp_cmd_waiting_fb_bus2[node_id] = false;
    }

    if (packet_count_bus2[node_id] == MAX_NUM_SAMPLES)
    {
        unsigned long average_fb_interval = total_latency_bus2[node_id] / MAX_NUM_SAMPLES;
        unsigned long average_cmd_to_fb = 0;
        if (cmd_count_bus2[node_id] > 0)
            average_cmd_to_fb = total_cmd_latency_bus2[node_id] / cmd_count_bus2[node_id];

        printf("Bus 2 node %d: avg cmd->fb=%lu us (avg fb interval=%lu us)\n",
               node_id + 1, average_cmd_to_fb, average_fb_interval);

        total_latency_bus2[node_id] = 0;
        packet_count_bus2[node_id] = 0;
        total_cmd_latency_bus2[node_id] = 0;
        cmd_count_bus2[node_id] = 0;
    }

    last_packet_time_bus2[node_id] = current_time;
}

void loop()
{
    Can0.events();
    Can1.events();
    receiveUDPPacket();

    if (!first_packet_recv)
        return;

    sendCANCMD();
    sendUDPPacket();
}

// Process all pending UDP packets so can_command is always the latest (avoids "rotate once" lag).
void receiveUDPPacket()
{
    int size;
    while ((size = udp.parsePacket()) > 0)
    {
        const uint8_t *data = udp.data();

        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
        }
        else if (size >= 49 && data[0] == 0x12)
        {
            uint8_t calculated_crc = calculate_crc8(data, 48);
            if (data[48] == calculated_crc)
            {
                CAN_message_t msg;
                msg.flags.extended = 1;
                msg.id = makeO2ExtendedId(MOTOR_ID, 18);
                msg.len = 8;
                memcpy(msg.buf, data + 1, 8);
                Can0.write(msg);
            }
        }
        else if (size >= 2 * MAX_NODES * 8)
        {
            first_packet_recv = true;
            if (size == 2 * MAX_NODES * 8 + 1)
            {
                uint8_t calculated_crc = calculate_crc8(data, 2 * MAX_NODES * 8);
                if (data[2 * MAX_NODES * 8] == calculated_crc)
                {
                    unsigned long now = micros();
                    for (int i = 0; i < MAX_NODES; i++)
                        for (int j = 0; j < 8; j++)
                            can_command[i][j] = data[i * 8 + j];
                    for (int i = 0; i < MAX_NODES; i++)
                        for (int j = 0; j < 8; j++)
                            can_command_bus2[i][j] = data[MAX_NODES * 8 + i * 8 + j];

                    // Mark start of one-shot UDP->feedback timing for each node
                    for (int i = 0; i < MAX_NODES; i++)
                    {
                        udp_cmd_start_time_bus1[i] = now;
                        udp_cmd_waiting_fb_bus1[i] = true;
                        udp_cmd_start_time_bus2[i] = now;
                        udp_cmd_waiting_fb_bus2[i] = true;
                    }
                }
            }
        }
    }
}
void printCANCommand()
{
    printf("CAN Command bus 1:\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        printf("Node %d: ", i + 1);
        for (int j = 0; j < 8; j++)
            printf("%02X ", can_command[i][j]);
        printf("\n");
    }
    printf("CAN Command bus 2:\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        printf("Node %d: ", i + 1);
        for (int j = 0; j < 8; j++)
            printf("%02X ", can_command_bus2[i][j]);
        printf("\n");
    }
}
// RobStride O2: payload byte 7 magic → CAN type. Type in 29-bit ID so motor actually stops/enables.
#define O2_MAGIC_EXIT  0xFD  // exit motor mode → Type 4 stop
#define O2_MAGIC_ENTER 0xFC  // enter motor mode → Type 3 enable
#define O2_MAGIC_ZERO  0xFE  // zero encoder → Type 6
#define O2_TYPE_CTRL   1     // normal position/velocity control

// 0 Nm in 16-bit for -12..+12 Nm range: (0 - (-12)) / 24 * 65535 ≈ 32768
#define O2_TYPE1_DATA_ZERO_NM  32768u

static uint32_t getO2CanId(uint8_t node_index, const uint8_t *cmd8) {
    uint8_t motor_id = (node_index == 0) ? MOTOR_ID : (MOTOR_ID + node_index);
    uint8_t type;
    if (cmd8[7] == O2_MAGIC_EXIT)
        type = 4;
    else if (cmd8[7] == O2_MAGIC_ENTER)
        type = 3;
    else if (cmd8[7] == O2_MAGIC_ZERO)
        type = 6;
    else {
        type = O2_TYPE_CTRL;
        return makeO2ExtendedIdWithData(motor_id, type, (uint16_t)O2_TYPE1_DATA_ZERO_NM);
    }
    return makeO2ExtendedId(motor_id, type);
}

void sendCANCMD()
{
    for (int i = 0; i < MAX_NODES; i++)
    {
        CAN_message_t msg;
        CAN_message_t msg2;

        msg.flags.extended = 1;
        msg2.flags.extended = 1;
        msg.id = getO2CanId(i, can_command[i]);
        msg2.id = getO2CanId(i, can_command_bus2[i]);
        msg.len = 8;
        msg2.len = 8;

        memcpy(msg.buf, can_command[i], 8);
        memcpy(msg2.buf, can_command_bus2[i], 8);

        Can0.write(msg);
        last_cmd_time_bus1[i] = micros();
        Can1.write(msg2);
        last_cmd_time_bus2[i] = micros();
    }
}
void sendUDPPacket()
{
    uint8_t buffer[MAX_NODES * 8 * 2];
    uint8_t *buffer_ptr = buffer;

    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(buffer_ptr, can_data[i], 8);
        buffer_ptr += 8;
    }

    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(buffer_ptr, can_data_bus2[i], 8);
        buffer_ptr += 8;
    }
    udp.send("192.168.0.100", kPort, buffer, MAX_NODES * 8 * 2);  // PC host (match ETHERNET_SETUP.md)
}

void reset()
{
    memset(can_command, 0, sizeof(can_command));
    memset(can_data, 0, sizeof(can_data));
    memset(can_command_bus2, 0, sizeof(can_command_bus2));
    memset(can_data_bus2, 0, sizeof(can_data_bus2));
    first_packet_recv = false;
}
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

static void sendSimpleEnableSequence() {
    CAN_message_t msg;
    uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(msg.buf, payload, 8);

    Serial.println("Simple O2 enable sequence on both CAN buses.");

    // Stop (Type 4)
    msg.id = makeO2ExtendedId(MOTOR_ID, CANCOM_MOTOR_RESET);
    Serial.println("Stop (Type 4)...");
    Can0.write(msg);
    Can1.write(msg);
    delay(200);

    // Zero (Type 6)
    msg.id = makeO2ExtendedId(MOTOR_ID, CANCOM_MOTOR_ZERO);
    Serial.println("Zero (Type 6)...");
    Can0.write(msg);
    Can1.write(msg);
    delay(200);

    // Enable (Type 3)
    msg.id = makeO2ExtendedId(MOTOR_ID, CANCOM_MOTOR_ENABLE);
    Serial.println("Enable (Type 3)...");
    Can0.write(msg);
    Can1.write(msg);
    delay(200);

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
    {
        // Wait for Serial
    }
    printf("Starting...\r\n");

    setupEthernet();
    setupCAN();

    // One-shot stop -> zero -> enable sequence on both CAN buses (same pattern as enable_motor_only.ino).
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
    // Static IP: no DHCP, use teensyIP / teensySubnet / teensyGateway (QNEthernet: ip, netmask, gateway)
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
    // Can0.mailboxStatus();
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
    // Can1.mailboxStatus();
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

void canReceive(const CAN_message_t &msg)
{
    int node_id = msg.buf[0] - 1;
    memcpy(can_data[node_id], msg.buf, 8);
    if (node_id >= 0 && node_id < MAX_NODES)
    {
        unsigned long current_time = micros();
        unsigned long latency = current_time - last_packet_time_bus1[node_id];
        total_latency_bus1[node_id] += latency;
        packet_count_bus1[node_id]++;

        if (packet_count_bus1[node_id] == MAX_NUM_SAMPLES)
        {
            unsigned long average_latency = total_latency_bus1[node_id] / MAX_NUM_SAMPLES;
            printf("Average latency of %d CAN receives for node %d on bus 1: %lu us\n", MAX_NUM_SAMPLES, node_id + 1, average_latency);
            total_latency_bus1[node_id] = 0;
            packet_count_bus1[node_id] = 0;
        }

        last_packet_time_bus1[node_id] = current_time;
    }
}

void canReceive2(const CAN_message_t &msg)
{
    int node_id = msg.buf[0] - 1;
    memcpy(can_data_bus2[node_id], msg.buf, 8);
    if (node_id >= 0 && node_id < MAX_NODES)
    {
        unsigned long current_time = micros();
        unsigned long latency = current_time - last_packet_time_bus2[node_id];
        total_latency_bus2[node_id] += latency;
        packet_count_bus2[node_id]++;

        if (packet_count_bus2[node_id] == MAX_NUM_SAMPLES)
        {
            unsigned long average_latency = total_latency_bus2[node_id] / MAX_NUM_SAMPLES;
            printf("Average latency of %d CAN receives for node %d on bus 2: %lu us\n", MAX_NUM_SAMPLES, node_id + 1, average_latency);
            total_latency_bus2[node_id] = 0;
            packet_count_bus2[node_id] = 0;
        }

        last_packet_time_bus2[node_id] = current_time;
    }
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

void receiveUDPPacket()
{
    // noInterrupts();
    int size = udp.parsePacket();
    if (size > 0)
    {
        const uint8_t *data = udp.data();
        // Print received UDP payload (length + hex bytes)
        Serial.printf("UDP RX len=%d data=", size);
        for (int i = 0; i < size && i < 64; i++)
            Serial.printf("%02X ", data[i]);
        Serial.println();

        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
            printf("Reset command received. Buffers and variables reset.\n");
        }
        else if (size >= 2 * MAX_NODES * 8)
        {
            if (!first_packet_recv)
                printf("first packet recv\n");
            first_packet_recv = true;

            // Extract the payload data
            std::vector<uint8_t> payload(data, data + 2 * MAX_NODES * 8);

            // Calculate CRC-8 for the payload
            uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());

            if (size == 2 * MAX_NODES * 8 + 1)
            {
                const uint8_t received_crc = data[2 * MAX_NODES * 8];

                if (received_crc == calculated_crc)
                {
                    // CRC check passed, process the data
                    for (int i = 0; i < MAX_NODES; i++)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            can_command[i][j] = data[i * 8 + j];
                        }
                    }
                    for (int i = 0; i < MAX_NODES; i++)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            can_command_bus2[i][j] = data[MAX_NODES * 8 + i * 8 + j];
                        }
                    }
                    // sendCANCMD();
                }
                else
                {
                    // CRC check failed, discard the data
                    printf("CRC check failed\n");
                    printf("Received CRC: %02X\n", received_crc);
                    printf("Calculated CRC: %02X\n", calculated_crc);
                }
                // printCANCommand();
            }
            else
            {
                printf("Invalid packet size\n");
            }
        }
    }

    // interrupts();
}
void printCANCommand()
{
    printf("CAN Command bus 1:\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        printf("Node %d: ", i + 1);
        for (int j = 0; j < 8; j++)
        {
            printf("%02X ", can_command[i][j]);
        }
        printf("\n");
    }
    printf("CAN Command bus 2:\n");
    for (int i = 0; i < MAX_NODES; i++)
    {
        printf("Node %d: ", i + 1);
        for (int j = 0; j < 8; j++)
        {
            printf("%02X ", can_command_bus2[i][j]);
        }
        printf("\n");
    }
}
// RobStride O2: payload byte 7 magic → CAN type. Type in 29-bit ID so motor actually stops/enables.
#define O2_MAGIC_EXIT  0xFD  // exit motor mode → Type 4 stop
#define O2_MAGIC_ENTER 0xFC  // enter motor mode → Type 3 enable
#define O2_MAGIC_ZERO  0xFE  // zero encoder → Type 6
#define O2_TYPE_CTRL   1     // normal position/velocity control

static uint32_t getO2CanId(uint8_t node_index, const uint8_t *cmd8) {
    uint8_t motor_id = (node_index == 0) ? MOTOR_ID : (MOTOR_ID + node_index);
    uint8_t type;
    if (cmd8[7] == O2_MAGIC_EXIT)
        type = 4;
    else if (cmd8[7] == O2_MAGIC_ENTER)
        type = 3;
    else if (cmd8[7] == O2_MAGIC_ZERO)
        type = 6;
    else
        type = O2_TYPE_CTRL;
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
        Can1.write(msg2);
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
    if (!udp.send("192.168.0.222", kPort, buffer, MAX_NODES * 8 * 2))
    {
        printf("ERROR.");
    }
}

void reset()
{
    memset(can_command, 0, sizeof(can_command));
    memset(can_data, 0, sizeof(can_data));
    memset(can_command_bus2, 0, sizeof(can_command_bus2));
    memset(can_data_bus2, 0, sizeof(can_data_bus2));
    first_packet_recv = false;
}
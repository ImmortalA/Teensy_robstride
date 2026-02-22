#include <FlexCAN_T4.h>
#include <QNEthernet.h>
#define DEBUG_MODE
#ifdef DEBUG_MODE
#define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) do {} while (0)
#endif
#define FIFO_VERSION
// Robostride O2: use extended 29-bit CAN ID with mode (id:8, data:16, mode:5)
#define USE_ROBOSTRIDE_O2 1
#if USE_ROBOSTRIDE_O2
#define CANCOM_MOTOR_CTRL    1
#define CANCOM_MOTOR_FEEDBACK 2
#define CANCOM_MOTOR_IN      3
#define CANCOM_MOTOR_RESET   4
#define CANCOM_MOTOR_ZERO    6
#endif

#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32
#define CAN_LED_PIN 15

using namespace qindesign::network;

FlexCAN_T4<CAN1, RX_SIZE_2, TX_SIZE_2> Can0;
FlexCAN_T4<CAN2, RX_SIZE_2, TX_SIZE_2> Can1;
FlexCAN_T4<CAN3, RX_SIZE_2, TX_SIZE_2> Can2;

constexpr uint32_t kDHCPTimeout = 5000; // 15 seconds
constexpr uint16_t kPort = 8003;         // udp port
constexpr int MAX_NODES = 2;             // Maximum number of nodes
constexpr int MAX_NUM_SAMPLES = 5000;
constexpr int NUM_BUSES = 3;

IPAddress ip(172, 17, 0, 5);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(172, 17, 0, 0);

EthernetUDP udp;
bool ethernet_setup_done = false;

uint8_t can_data[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command[MAX_NODES][8]; // CAN command buffer for each node

uint8_t can_data_bus2[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command_bus2[MAX_NODES][8]; // CAN command buffer for each node

uint8_t can_data_bus3[MAX_NODES][8];    // CAN data buffer for each node
uint8_t can_command_bus3[MAX_NODES][8]; // CAN command buffer for each node

unsigned long last_packet_time_bus1[MAX_NODES] = {0};
unsigned long total_latency_bus1[MAX_NODES] = {0};
unsigned int packet_count_bus1[MAX_NODES] = {0};
unsigned int max_latency_bus1[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus1[MAX_NODES] = {0};

unsigned long last_packet_time_bus2[MAX_NODES] = {0};
unsigned long total_latency_bus2[MAX_NODES] = {0};
unsigned int packet_count_bus2[MAX_NODES] = {0};
unsigned int max_latency_bus2[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus2[MAX_NODES] = {0};


unsigned long last_packet_time_bus3[MAX_NODES] = {0};
unsigned long total_latency_bus3[MAX_NODES] = {0};
unsigned int packet_count_bus3[MAX_NODES] = {0};
unsigned int max_latency_bus3[MAX_NODES] = {0};
unsigned long sum_squares_latency_bus3[MAX_NODES] = {0};

const uint8_t exit_motor_mode_cmd[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

#if USE_ROBOSTRIDE_O2
// Get Robostride O2 CAN mode from 8-byte payload (host sends same magic bytes; we map to O2 mode in ID)
static uint8_t getO2ModeFromPayload(const uint8_t *buf) {
  if (buf[0] == 0xFF && buf[1] == 0xFF && buf[2] == 0xFF && buf[3] == 0xFF &&
      buf[4] == 0xFF && buf[5] == 0xFF && buf[6] == 0xFF) {
    if (buf[7] == 0xFD) return CANCOM_MOTOR_RESET;   // exit motor
    if (buf[7] == 0xFC) return CANCOM_MOTOR_IN;     // enter motor
    if (buf[7] == 0xFE) return CANCOM_MOTOR_ZERO;   // zero encoder
  }
  return CANCOM_MOTOR_CTRL;
}
// Build 29-bit extended ID: id(8) | data(16) | mode(5)
static uint32_t makeO2ExtendedId(uint8_t node_id_1based, uint8_t mode) {
  return ((uint32_t)(node_id_1based & 0xFF) << 21) | ((uint32_t)(mode & 0x1F));
}
#endif

CAN_message_t msg;
CAN_message_t msg2;
CAN_message_t msg3;

unsigned long last_udp_packet_time = 0;

bool first_packet_recv = false;
const uint8_t RESET_COMMAND = 0xFF;

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
#ifdef DEBUG_MODE
    Serial.begin(115200);
    while (!Serial && millis() < 4000) {
        // Wait for Serial
    }
#endif

  DEBUG_PRINT("Starting...\r\n");
  reset();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAN_LED_PIN, OUTPUT);
  setupEthernetStatic();
noInterrupts();
  setupCAN();
interrupts();
  DEBUG_PRINT("Setup done...\r\n");

}

void setupEthernet()
{
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    DEBUG_PRINT("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Ethernet.onLinkState([](bool state)
                         { DEBUG_PRINT("[Ethernet] Link %s\r\n", state ? "ON" : "OFF"); });

    DEBUG_PRINT("Starting Ethernet with DHCP...\r\n");
    if (!Ethernet.begin())
    {
        DEBUG_PRINT("Failed to start Ethernet\r\n");
        return;
    }
    if (!Ethernet.waitForLocalIP(kDHCPTimeout))
    {
        DEBUG_PRINT("Failed to get IP address from DHCP\r\n");
        return;
    }

    DEBUG_PRINT("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    udp.begin(kPort);
    DEBUG_PRINT("Done setting Ethernet\r\n");

}
void setupEthernetStatic()
{
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    DEBUG_PRINT("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    Ethernet.onLinkState([](bool state)
                         {ethernet_setup_done = state; 
                         if (state)
                            //  digitalWrite(LED_BUILTIN, HIGH);
                            digitalWrite(CAN_LED_PIN, HIGH);

                          else
                            // digitalWrite(LED_BUILTIN, LOW);
                              digitalWrite(CAN_LED_PIN, LOW);

                         DEBUG_PRINT("[Ethernet] Link %s\r\n", state ? "ON" : "OFF"); });

    // Set the static IP address

    DEBUG_PRINT("Starting Ethernet with static IP...\r\n");
    if (!Ethernet.begin(ip, subnet, gateway))
    {
        DEBUG_PRINT("Failed to start Ethernet with static IP\r\n");
        return;
    }

    // if (!Ethernet.waitForLink(kDHCPTimeout))
    // {
    //     DEBUG_PRINT("Failed to get IP address from DHCP\r\n");
    //     return;
    // }

    DEBUG_PRINT("Ethernet speed: %d\r\n", Ethernet.linkSpeed());

    printIPAddress();

    udp.begin(kPort);
    DEBUG_PRINT("Done setting Ethernet\r\n");
    
}
void setupCAN()
{
  
    Can0.begin();
    Can0.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.setFIFOFilter(REJECT_ALL);
    Can0.setFIFOFilter(0, 0x0, STD);
    Can0.onReceive(canReceive);
#else
    Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can0.setMBFilter(MB1, 0, 0x0);
    Can0.enhanceFilter(MB1);
    Can0.distribute();
    Can0.enableMBInterrupts();
    Can0.onReceive(canReceive);
    Can0.setClock(CLK_60MHz);
#endif
    Can0.mailboxStatus();
    DEBUG_PRINT("Done setting up CAN 0\n");

    delayMicroseconds(10);

    Can1.begin();
    Can1.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can1.enableFIFO();
    Can1.enableFIFOInterrupt();
    Can1.onReceive(canReceive2);
#else

    Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can1.setMBFilter(MB1, 0, 0x0);
    Can1.enhanceFilter(MB1);
    Can1.enableMBInterrupts();
    Can1.onReceive(canReceive2);
    Can1.distribute();
    Can1.setClock(CLK_60MHz);
    Can1.mailboxStatus();
#endif
    DEBUG_PRINT("Done setting up CAN 1\n");
    delayMicroseconds(10);

    if (NUM_BUSES < 3)
    {
        return;
    }

    Can2.begin();
    Can2.setBaudRate(1000000);
#ifdef FIFO_VERSION
    Can2.enableFIFO();
    Can2.enableFIFOInterrupt();
    Can2.onReceive(canReceive3);
#else
    Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
    Can2.setMBFilter(MB1, 0, 0x0);
    Can2.enhanceFilter(MB1);
    Can2.enableMBInterrupts();
    Can2.onReceive(canReceive3);
    Can2.distribute();
    Can2.setClock(CLK_60MHz);
#endif

    DEBUG_PRINT("Done setting up CAN 2\n");
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void printIPAddress()
{
    IPAddress ip = Ethernet.localIP();
    DEBUG_PRINT("    Local IP     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.subnetMask();
    DEBUG_PRINT("    Subnet mask  = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.broadcastIP();
    DEBUG_PRINT("    Broadcast IP = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.gatewayIP();
    DEBUG_PRINT("    Gateway      = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
    ip = Ethernet.dnsServerIP();
    DEBUG_PRINT("    DNS          = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
}

void canReceive(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
#if USE_ROBOSTRIDE_O2
      int node_id;
      if (msg.flags.extended) {
        uint32_t mode = msg.id & 0x1F;
        if (mode != CANCOM_MOTOR_FEEDBACK) return;
        node_id = (msg.id >> 21) & 0xFF;
        node_id = node_id - 1;
      } else
        node_id = msg.buf[0] - 1;
      if (node_id < 0 || node_id >= MAX_NODES) return;
#else
      int node_id = msg.buf[0] - 1;
#endif
      memcpy(can_data[node_id], msg.buf, sizeof(msg.buf));
#ifdef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus1[node_id];
            total_latency_bus1[node_id] += latency;
            sum_squares_latency_bus1[node_id] += latency * latency;
            if (latency > max_latency_bus1[node_id]) max_latency_bus1[node_id] = latency;
            packet_count_bus1[node_id]++;

            if (packet_count_bus1[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus1[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus1[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 1, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus1[node_id]);
                
                total_latency_bus1[node_id] = 0;
                sum_squares_latency_bus1[node_id] = 0;
                max_latency_bus1[node_id] = 0;
                packet_count_bus1[node_id] = 0;
            }

            last_packet_time_bus1[node_id] = current_time;
        }
#endif
    }
}

void canReceive2(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
#if USE_ROBOSTRIDE_O2
      int node_id;
      if (msg.flags.extended) {
        uint32_t mode = msg.id & 0x1F;
        if (mode != CANCOM_MOTOR_FEEDBACK) return;
        node_id = (msg.id >> 21) & 0xFF;
        node_id = node_id - 1;
      } else
        node_id = msg.buf[0] - 1;
      if (node_id < 0 || node_id >= MAX_NODES) return;
#else
      int node_id = msg.buf[0] - 1;
#endif
      memcpy(can_data_bus2[node_id], msg.buf, sizeof(msg.buf));
    


#ifdef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus2[node_id];
            total_latency_bus2[node_id] += latency;
            sum_squares_latency_bus2[node_id] += latency * latency;
            if (latency > max_latency_bus2[node_id]) max_latency_bus2[node_id] = latency;
            packet_count_bus2[node_id]++;

            if (packet_count_bus2[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus2[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus2[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 2, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus2[node_id]);
                
                total_latency_bus2[node_id] = 0;
                sum_squares_latency_bus2[node_id] = 0;
                max_latency_bus2[node_id] = 0;
                packet_count_bus2[node_id] = 0;
            }

            last_packet_time_bus2[node_id] = current_time;
        }
#endif
}
}
void canReceive3(const CAN_message_t &msg)
{
    if(msg.len == 8)
    {
#if USE_ROBOSTRIDE_O2
      int node_id;
      if (msg.flags.extended) {
        uint32_t mode = msg.id & 0x1F;
        if (mode != CANCOM_MOTOR_FEEDBACK) return;
        node_id = (msg.id >> 21) & 0xFF;
        node_id = node_id - 1;
      } else
        node_id = msg.buf[0] - 1;
      if (node_id < 0 || node_id >= MAX_NODES) return;
#else
      int node_id = msg.buf[0] - 1;
#endif
      memcpy(can_data_bus3[node_id], msg.buf, sizeof(msg.buf));
    

#ifdef DEBUG_MODE
        if (node_id >= 0 && node_id < MAX_NODES)
        {
            unsigned long current_time = micros();
            unsigned long latency = current_time - last_packet_time_bus3[node_id];
            total_latency_bus3[node_id] += latency;
            sum_squares_latency_bus3[node_id] += latency * latency;
            if (latency > max_latency_bus3[node_id]) max_latency_bus3[node_id] = latency;
            packet_count_bus3[node_id]++;

            if (packet_count_bus3[node_id] == MAX_NUM_SAMPLES)
            {
                unsigned long average_latency = total_latency_bus3[node_id] / MAX_NUM_SAMPLES;
                unsigned long variance = (sum_squares_latency_bus3[node_id] / MAX_NUM_SAMPLES) - (average_latency * average_latency);
                unsigned long std_dev = sqrt(variance);
                
                DEBUG_PRINT("Bus 3, Node %d: Avg latency: %lu us, Std Dev: %lu us, Max: %lu us\n", 
                            node_id + 1, average_latency, std_dev, max_latency_bus3[node_id]);
                
                total_latency_bus3[node_id] = 0;
                sum_squares_latency_bus3[node_id] = 0;
                max_latency_bus3[node_id] = 0;
                packet_count_bus3[node_id] = 0;
            }

            last_packet_time_bus3[node_id] = current_time;
        }
#endif
}}

void loop()
{
    Can0.events();
    Can1.events();
    if (NUM_BUSES == 3)
        Can2.events();

    receiveUDPPacket();

    if (!first_packet_recv)
        return;

    sendCANCMD();
    sendUDPPacket();

    // delayMicroseconds(100);
}

void receiveUDPPacket()
{
    // noInterrupts();
    int size = udp.parsePacket();
    if (size > 0)
    {
        const uint8_t *data = udp.data();
        if (size == 2 && data[0] == RESET_COMMAND)
        {
            reset();
            DEBUG_PRINT("Reset command received. Buffers and variables reset.\n");
        }
        else if (size >= NUM_BUSES * MAX_NODES * 8)
        {
            if (!first_packet_recv)
                DEBUG_PRINT("first packet recv\n");
            first_packet_recv = true;

            // Extract the payload data
            std::vector<uint8_t> payload(data, data + NUM_BUSES * MAX_NODES * 8);

            // Calculate CRC-8 for the payload
            uint8_t calculated_crc = calculate_crc8(payload.data(), payload.size());

            if (size == NUM_BUSES * MAX_NODES * 8 + 1)
            {
                const uint8_t received_crc = data[NUM_BUSES * MAX_NODES * 8];

                if (received_crc == calculated_crc)
                {
                    last_udp_packet_time = micros();
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
                    if (NUM_BUSES == 3)
                    {

                        for (int i = 0; i < MAX_NODES; i++)
                        {
                            for (int j = 0; j < 8; j++)
                            {
                                can_command_bus3[i][j] = data[MAX_NODES * 8 * 2 + i * 8 + j];
                            }
                        }
                    }
                    // printCANCommand();
                    // sendCANCMD();
                }
                else
                {
                    // CRC check failed, discard the data
                    DEBUG_PRINT("CRC check failed\n");
                    DEBUG_PRINT("Received CRC: %02X\n", received_crc);
                    DEBUG_PRINT("Calculated CRC: %02X\n", calculated_crc);
                }
            }
            else
            {
                DEBUG_PRINT("Invalid packet size\n");
            }
        }
    }

    // interrupts();
}
void printCANCommand()
{
    // DEBUG_PRINT("CAN Command bus 1:\n");
    // for (int i = 0; i < MAX_NODES; i++)
    // {
    //     DEBUG_PRINT("Node %d: ", i + 1);
    //     for (int j = 0; j < 8; j++)
    //     {
    //         DEBUG_PRINT("%02X ", can_command[i][j]);
    //     }
    //     DEBUG_PRINT("\n");
    // }
    // DEBUG_PRINT("CAN Command bus 2:\n");
    // for (int i = 0; i < MAX_NODES; i++)
    // {
    //     DEBUG_PRINT("Node %d: ", i + 1);
    //     for (int j = 0; j < 8; j++)
    //     {
    //         DEBUG_PRINT("%02X ", can_command_bus2[i][j]);
    //     }
    //     DEBUG_PRINT("\n");
    // }

    if (NUM_BUSES == 3)
    {
        DEBUG_PRINT("CAN Command bus 3:\n");
        for (int i = 0; i < MAX_NODES; i++)
        {
            DEBUG_PRINT("Node %d: ", i + 1);
            for (int j = 0; j < 8; j++)
            {
                DEBUG_PRINT("%02X ", can_command_bus3[i][j]);
            }
            DEBUG_PRINT("\n");
        }
    }
    
}
void sendCANCMD()
{

    // noInterrupts();
    // exit_motor_mode();

    for (int i = 0; i < MAX_NODES; i++)
    {
        uint8_t node_id_1based = i + 1;

#if USE_ROBOSTRIDE_O2
        uint8_t mode0 = getO2ModeFromPayload(can_command[i]);
        uint8_t mode1 = getO2ModeFromPayload(can_command_bus2[i]);
        uint8_t mode2 = getO2ModeFromPayload(can_command_bus3[i]);

        msg.flags.extended = 1;
        msg.id = makeO2ExtendedId(node_id_1based, mode0);
        msg.len = 8;

        msg2.flags.extended = 1;
        msg2.id = makeO2ExtendedId(node_id_1based, mode1);
        msg2.len = 8;

        msg3.flags.extended = 1;
        msg3.id = makeO2ExtendedId(node_id_1based, mode2);
        msg3.len = 8;
#else
        msg.flags.extended = 0;
        msg.id = node_id_1based;
        msg.len = 8;

        msg2.flags.extended = 0;
        msg2.id = node_id_1based;
        msg2.len = 8;

        msg3.flags.extended = 0;
        msg3.id = node_id_1based;
        msg3.len = 8;
#endif

        memcpy(msg.buf, can_command[i], 8);
        memcpy(msg2.buf, can_command_bus2[i], 8);
        memcpy(msg3.buf, can_command_bus3[i], 8);

        Can0.write(msg);
        Can1.write(msg2);
        if (NUM_BUSES == 3)
            Can2.write(msg3);
        delayMicroseconds(100);
    }
    // interrupts();
}
void sendUDPPacket()
{
    uint8_t buffer[MAX_NODES * 8 * NUM_BUSES];
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
    if (NUM_BUSES == 3)
    {
        for (int i = 0; i < MAX_NODES; i++)
        {
            memcpy(buffer_ptr, can_data_bus3[i], 8);
            buffer_ptr += 8;
        }
    }

    if (!udp.send("172.17.0.56", kPort, buffer, MAX_NODES * 8 * NUM_BUSES))
    {
        DEBUG_PRINT("ERROR.");
    }
}

void exit_motor_mode()
{
    for (int i = 0; i < MAX_NODES; i++)
    {
        memcpy(can_command[i], exit_motor_mode_cmd, 8);
        memcpy(can_command_bus2[i], exit_motor_mode_cmd, 8);
        memcpy(can_command_bus3[i], exit_motor_mode_cmd, 8);
    }
}

void reset()
{
    // memset(can_command, 0, sizeof(can_command));
    // memset(can_command_bus2, 0, sizeof(can_command_bus2));
    // memset(can_command_bus3, 0, sizeof(can_command_bus3));

    // memset(can_data, 0, sizeof(can_data));
    // memset(can_data_bus2, 0, sizeof(can_data_bus2));
    // memset(can_data_bus3, 0, sizeof(can_data_bus3));


    for (int i = 0; i < MAX_NODES; i++)
    {
        last_packet_time_bus1[i] = 0;
        total_latency_bus1[i] = 0;
        packet_count_bus1[i] = 0;

        last_packet_time_bus2[i] = 0;
        total_latency_bus2[i] = 0;
        packet_count_bus2[i] = 0;

        last_packet_time_bus3[i] = 0;
        total_latency_bus3[i] = 0;
        packet_count_bus3[i] = 0;

    }
    exit_motor_mode();
    // sendCANCMD();
    first_packet_recv = false;
}
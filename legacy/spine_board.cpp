#include "spine_board.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

// -----------------------------------------------------------------------------
// Teensy packet layout and helpers
// -----------------------------------------------------------------------------
// Teensy firmware expects 48 bytes: 2 buses * 3 nodes * 8. Layout: bytes 0-23 = Can0, 24-47 = Can1.
// We have 1 node per bus. Command always in slot 0; Teensy sends slot 0 to motor_id = MOTOR_CAN_ID+0.
// Feedback: if MOTOR_CAN_ID=0 motor feedback is in slot 0 (24-31); if MOTOR_CAN_ID=1 in slot 1 (32-39).
constexpr int TEENSY_PAYLOAD_BYTES = 48;
constexpr int CAN1_FEEDBACK_SLOT = 1;  // 0 if Teensy MOTOR_CAN_ID=0, 1 if MOTOR_CAN_ID=1

static void to_teensy_48(const uint8_t* logical_24, uint8_t* out48) {
    memset(out48, 0, TEENSY_PAYLOAD_BYTES);
    memcpy(out48 + 0, logical_24 + 0, 8);   // bus 0 node 0 -> Can0 node0
    memcpy(out48 + 24, logical_24 + 8, 8);  // bus 1 -> Can1 node0 (sent to MOTOR_CAN_ID+0)
}

// -----------------------------------------------------------------------------
// Constructor: resolve interface, bind sockets, init bus_list
// -----------------------------------------------------------------------------
SpineBoard::SpineBoard(const std::string &ip, const std::string &interface, int port, int nodes, int buses, std::string _board_name)
    : num_nodes(nodes), num_buses(buses), teensy_ip(ip), teensy_port(port),
      first_state_received(false), bus_list(buses),
      sock_send(io_context), server_socket(io_context), board_name(_board_name)
{
    // -------- Resolve bind address: requested interface or first non-loopback --------
    asio::ip::address_v4 bind_address;
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    bool found = false;

    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (ifa->ifa_addr == NULL)
            continue;

        family = ifa->ifa_addr->sa_family;

        if (family == AF_INET && strcmp(ifa->ifa_name, interface.c_str()) == 0)
        {
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0)
            {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                exit(EXIT_FAILURE);
            }
            bind_address = asio::ip::make_address_v4(host);
            found = true;
            break;
        }
    }

    // Fallback: first non-loopback IPv4 interface (e.g. eth0, enp0s3 when eno1 missing)
    if (!found)
    {
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)
                continue;
            family = ifa->ifa_addr->sa_family;
            if (family != AF_INET)
                continue;
            s = getnameinfo(ifa->ifa_addr, sizeof(struct sockaddr_in),
                            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
            if (s != 0)
                continue;
            asio::ip::address_v4 addr = asio::ip::make_address_v4(host);
            if (addr.is_loopback())
                continue;
            bind_address = addr;
            found = true;
            std::cout << "Interface '" << interface << "' not found, using " << ifa->ifa_name
                      << " (" << host << ")" << std::endl;
            break;
        }
    }

    freeifaddrs(ifaddr);

    if (!found || bind_address.is_unspecified())
    {
        std::cerr << "Failed to find the " << interface << " interface IP address (no fallback)" << std::endl;
        throw std::runtime_error("Failed to find network interface IP address");
    }

    // -------- Bind UDP sockets --------
    sock_send.open(asio::ip::udp::v4());
    sock_send.bind(asio::ip::udp::endpoint(bind_address, 0));

    server_socket.open(asio::ip::udp::v4());
    server_socket.bind(asio::ip::udp::endpoint(bind_address, teensy_port));

    std::cout << "Server bound to " << server_socket.local_endpoint() << std::endl;

    // -------- Allocate bus_list (state, command, params) --------
    for (int j = 0; j < num_buses; j++)
    {
        bus_list[j].state.j = new joint_state[num_nodes];
        bus_list[j].command.j = new joint_control[num_nodes];
        bus_list[j].params = new ActuatorParams[num_nodes];
        bus_list[j].params_vec = std::vector<ActuatorParams>(num_nodes);

        for (int i = 0; i < num_nodes; i++)
        {
            bus_list[j].params[i] = getActuatorParams(ActuatorType::ROBOSTRIDE_O2);
            bus_list[j].params_vec[i] = getActuatorParams(ActuatorType::ROBOSTRIDE_O2);
        }
    }
}

// -----------------------------------------------------------------------------
// initBoard: interactive sequence (reset -> exit -> zero -> enter -> Type1 zero -> final zero)
// -----------------------------------------------------------------------------
void SpineBoard::initBoard()
{
    if (!actuator_params_set)
    {
        std::cerr << "Actuator parameters not set. Exiting..." << std::endl;
        exit(-1);
    }
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

    // -------- Step 1: Reset --------
    std::vector<uint8_t> reset_data(1, 0xFF);
    send_data_to_teensy(reset_data, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    printf("Reset sent \n");
    printf("num_buses: %d\n", num_buses);

    // -------- Step 2: Exit motor mode --------
    for (int j = 0; j < num_buses; j++)
    {
        // bus &current_bus = bus_list[j];
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            pack_exit_motor_mode_cmd(bus_data + i * 8);
        }
    }

    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);

    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    printf("Exit motor mode sent \n");
    // exit(0);

// #ifdef ZERO_ENCODERS
//     // send zero encoder command
//     data_to_send = std::vector<uint8_t>(num_nodes * 8 * num_buses);

//     for (int j = 0; j < num_buses; j++)
//     {
//         // bus &current_bus = bus_list[j];
//         uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

//         for (int i = 0; i < num_nodes; i++)
//         {
//             pack_zero_encoder(bus_data + i * 8);
//         }
//     }
//     send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
//     std::this_thread::sleep_for(std::chrono::microseconds(1000000));
//     printf("Zero encoder sent \n");

// #endif

    // -------- Step 3: Zero command (before enter motor mode) --------
    for (int j(0); j < num_buses; j++)
    {
        for (int i = 0; i < num_nodes; i++)
        {
            bus_list[j].command.j[i].v_des = 0.0f;
            bus_list[j].command.j[i].p_des = 0.0f;
            bus_list[j].command.j[i].kp = 0.0f;
            bus_list[j].command.j[i].kd = 0.0f;
            bus_list[j].command.j[i].t_ff = 0.0f;
        }
    }

    // NOTE: Previously we sent a Type 1 \"zero\" command from the host here. This is now\n+    // disabled so motor init does not send any Type 1 frames from the PC.\n+    // for (int j = 0; j < num_buses; j++)\n+    // {\n+    //     bus &current_bus = bus_list[j];\n+    //     uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;\n+    //     for (int i = 0; i < num_nodes; i++)\n+    //     {\n+    //         pack_cmd_private_o2(bus_data + i * 8, current_bus, i);\n+    //     }\n+    // }\n+    // send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);\n+    // std::this_thread::sleep_for(std::chrono::microseconds(1000000));\n+    // printf(\"Zero Command sent \\n\");

    // -------- Step 4: Enter motor mode + immediate Type 1 (zero) so motor has setpoint --------
    for (int j = 0; j < num_buses; j++)
    {
        // bus &current_bus = bus_list[j];
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            pack_enter_motor_mode_cmd(bus_data + i * 8);
        }
    }

    // Send enter motor mode
    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // NOTE: Sending a burst of Type 1 \"zero\" commands from the host used to happen here.
    // This is now disabled so that motor init does not send any Type 1 frames from the PC.
    // for (int j(0); j < num_buses; j++)
    // {
    //     for (int i = 0; i < num_nodes; i++)
    //     {
    //         bus_list[j].command.j[i].v_des = 0.0f;
    //         bus_list[j].command.j[i].p_des = 0.0f;
    //         bus_list[j].command.j[i].kp = 0.0f;
    //         bus_list[j].command.j[i].kd = 0.0f;
    //         bus_list[j].command.j[i].t_ff = 0.0f;
    //     }
    // }
    // for (int j = 0; j < num_buses; j++)
    // {
    //     bus &current_bus = bus_list[j];
    //     uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
    //     for (int i = 0; i < num_nodes; i++)
    //         pack_cmd_private_o2(bus_data + i * 8, current_bus, i);
    // }
    // for (int k = 0; k < 20; k++)
    // {
    //     send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    std::this_thread::sleep_for(std::chrono::microseconds(500000));

    printf("Enter motor mode sent \n");

    // -------- Step 5: Final zero (keep state only; no Type 1 send) --------
    // Keep bus_list zeroed so the send thread can later use it, but do not send Type 1 packets.
    // for (int j(0); j < num_buses; j++)
    // {
    //     for (int i = 0; i < num_nodes; i++)
    //     {
    //         bus_list[j].command.j[i].v_des = 0.0f;
    //         bus_list[j].command.j[i].p_des = 0.0f;
    //         bus_list[j].command.j[i].kp = 0.0f;
    //         bus_list[j].command.j[i].kd = 0.0f;
    //         bus_list[j].command.j[i].t_ff = 0.0f;
    //     }
    // }
    // for (int j = 0; j < num_buses; j++)
    // {
    //     bus &current_bus = bus_list[j];
    //     uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;
    //     for (int i = 0; i < num_nodes; i++)
    //         pack_cmd_private_o2(bus_data + i * 8, current_bus, i);
    // }
    // send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    // std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    // printf("Zero Command sent After motor mode \n");
}

// -------- Alternative init (commented out) --------
// void SpineBoard::initBoard()
// {
//     restBoard();
//     exitMotorMode();
//     zeroEncoders();
//     zeroMotorCommand();
//     enterMotorMode();
//     zeroMotorCommand();
//     printf("Board initialized\n");
// }

// -----------------------------------------------------------------------------
// Standalone init helpers (restBoard, exitMotorMode, enterMotorMode, zeroMotorCommand, zeroEncoders)
// -----------------------------------------------------------------------------
void SpineBoard::restBoard()
{
    printf("Sending rest command...\n");
    if (!actuator_params_set)
    {
        std::cerr << "Actuator parameters not set. Exiting..." << std::endl;
        exit(-1);
    }
    // Spine board reset command
    std::vector<uint8_t> reset_data(1, 0xFF);
    send_data_to_teensy(reset_data, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    printf("Reset sent \n");
}

void SpineBoard::exitMotorMode()
{
    printf("Sending exit motor mode command\n");
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);
    for (int j = 0; j < num_buses; j++)
    {
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            pack_exit_motor_mode_cmd(bus_data + i * 8);
        }
    }

    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);

    std::this_thread::sleep_for(std::chrono::microseconds(1000000));

    printf("Exit motor mode sent \n");
}

void SpineBoard::enterMotorMode()
{
    printf("Sending enter motor mode command ...\n");
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);
    // Send enter motor mode command
    for (int j = 0; j < num_buses; j++)
    {
        // bus &current_bus = bus_list[j];
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            pack_enter_motor_mode_cmd(bus_data + i * 8);
        }
    }
    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    printf("Enter motor mode sent \n");
}

void SpineBoard::zeroMotorCommand()
{
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);
    printf("Sending zero command ...\n");
    for (int j(0); j < num_buses; j++)
    {
        for (int i = 0; i < num_nodes; i++)
        {
            bus_list[j].command.j[i].v_des = 0.0f;
            bus_list[j].command.j[i].p_des = 0.0f;
            bus_list[j].command.j[i].kp = 0.0f;
            bus_list[j].command.j[i].kd = 0.0f;
            bus_list[j].command.j[i].t_ff = 0.0f;
        }
    }

    // Pack the data to send
    for (int j = 0; j < num_buses; j++)
    {
        bus &current_bus = bus_list[j];
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            pack_cmd_private_o2(bus_data + i * 8, current_bus, i);
        }
    }

    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    printf("Zero Command sent \n");
}

void SpineBoard::zeroEncoders()
{
    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

    bool any_recalibrate = false;
    for (int j = 0; j < num_buses; j++)
    {
        uint8_t *bus_data = data_to_send.data() + j * num_nodes * 8;

        for (int i = 0; i < num_nodes; i++)
        {
            if (bus_list[j].params[i].recalibrate)
            {
                pack_zero_encoder(bus_data + i * 8);
                any_recalibrate = true;
                printf("Zeroing encoder for bus %d, node %d\n", j, i);
            }
            else
            {
                // Send zero command for actuators not being recalibrated
                pack_cmd_private_o2(bus_data + i * 8, bus_list[j], i);
            }
        }
    }

    if (any_recalibrate)
    {
        send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
        std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        printf("Zero encoder sent\n");
    }
}

// -----------------------------------------------------------------------------
// start: launch receive and send threads
// -----------------------------------------------------------------------------
void SpineBoard::start()
{
    std::cout << "UDP server listening on " << server_socket.local_endpoint() << std::endl;

    // -------- Receive thread: handle incoming UDP from Teensy --------
    receive_thread = std::thread([&]()
                                 {
            while (true) {
                std::vector<uint8_t> recv_buffer(TEENSY_PAYLOAD_BYTES + 16);
                asio::ip::udp::endpoint client_endpoint;
                size_t bytes_received = server_socket.receive_from(asio::buffer(recv_buffer), client_endpoint);
                std::vector<uint8_t> received_data(recv_buffer.begin(), recv_buffer.begin() + bytes_received);
                handle_udp_packet(client_endpoint, received_data);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            } });

    // -------- Send thread: init once, then loop (pack + send to Teensy every 200 µs) --------
    send_thread = std::thread([&]()
                              {
            bool first_time = true;
            unsigned int send_count = 0;
            while (true) {
                if (first_time)
                {

                    initBoard();
                    zeroEncoders();
                    boardInitialized = true;

                    first_time = false;
                    continue;
                }

                // When false, do not send any packets so Teensy keeps last (e.g. Type 3). Avoids
                // sending Type 1 right after final zero, which was causing the motor to spin.
                if (!allow_command_send_) {
                    std::this_thread::sleep_for(std::chrono::microseconds(200));
                    continue;
                }

                update_command();

                // Generate example data to send
                std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

                // Keep-alive: re-send Enter Motor Mode (Type 3) every 500 ms so motor stays enabled.
                bool send_enter = (++send_count % 2500 == 0);
                for (int j = 0; j < num_buses; j++) {
                    bus& current_bus = bus_list[j];
                    uint8_t* bus_data = data_to_send.data() + j * num_nodes * 8;
                    for (int i = 0; i < num_nodes; i++) {
                        if (send_enter && j == 0 && i == 0)
                            pack_enter_motor_mode_cmd(bus_data + i * 8);
                        else if (use_mit_pack_)
                            pack_cmd(bus_data + i * 8, current_bus, i);
                        else
                            pack_cmd_private_o2(bus_data + i * 8, current_bus, i);
                    }
                }

                send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            } });
}

// -----------------------------------------------------------------------------
// O2 parameter write (Type 18): build 48-byte packet [0x12, 8-byte payload, zeros], send to Teensy
// -----------------------------------------------------------------------------
void SpineBoard::sendParameterWriteU8(uint16_t param_id, uint8_t value)
{
    std::vector<uint8_t> payload(TEENSY_PAYLOAD_BYTES, 0);
    payload[0] = 0x12;  // magic: Teensy sends as Type 18
    payload[1] = (uint8_t)(param_id & 0xFF);
    payload[2] = (uint8_t)(param_id >> 8);
    payload[3] = 0;
    payload[4] = 0;
    payload[5] = value;  // U8 at byte index 4 in O2 Type 18 param write
    payload[6] = 0;
    payload[7] = 0;
    send_data_to_teensy(payload, TEENSY_PAYLOAD_BYTES);
}

// -----------------------------------------------------------------------------
// UDP send: build 48-byte payload + CRC, send to Teensy
// -----------------------------------------------------------------------------
void SpineBoard::send_data_to_teensy(const std::vector<uint8_t> &data, const int data_size)
{
    // Teensy expects 48-byte payload (2 buses * 3 nodes * 8). Convert our 24-byte logical packet to 48-byte layout.
    std::vector<uint8_t> payload;
    if (data_size == TEENSY_PAYLOAD_BYTES && data[0] == 0x12) {
        // Parameter write (Type 18): 48 bytes [0x12, 8-byte Type18 payload, zeros]; no layout conversion.
        payload.assign(data.begin(), data.begin() + TEENSY_PAYLOAD_BYTES);
    } else if (data_size == TEENSY_PAYLOAD_BYTES) {
        // Full 48-byte packet (e.g. 2 buses × 3 nodes): send as-is to match Teensy layout
        payload.assign(data.begin(), data.begin() + TEENSY_PAYLOAD_BYTES);
    } else if (data_size == num_buses * num_nodes * 8 && data_size <= TEENSY_PAYLOAD_BYTES) {
        payload.resize(TEENSY_PAYLOAD_BYTES);
        to_teensy_48(data.data(), payload.data());
    } else {
        payload.assign(data.begin(), data.end());
        payload.resize(data_size, 0);
    }

    // Calculate CRC-8 for the payload
    uint8_t crc_value = calculate_crc8(payload.data(), payload.size());

    // Append the CRC value to the payload
    std::vector<uint8_t> packet(payload);
    packet.push_back(crc_value);

    // Send the data to the Teensy
    sock_send.send_to(asio::buffer(packet), udp::endpoint(asio::ip::make_address(teensy_ip), teensy_port));

    // check if crc != 85
    // if ((int)crc_value != 133) // zero command crc value is 133
    // {
    //     printf("crc_value: %d\n", crc_value);

    //     // print out the packet in a human-readable format
    //     std::cout << "Board: " << board_name << std::endl;
    //     std::cout << "Sent packet: ";
    //     for (int i = 0; i < packet.size(); i++)
    //     {
    //         std::cout << std::hex << (int)packet[i] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

// -----------------------------------------------------------------------------
// process_data: unpack Teensy feedback into bus_list state
// -----------------------------------------------------------------------------
void SpineBoard::process_data(const std::vector<uint8_t> &data_list)
{
    std::lock_guard<std::mutex> lock(bus_list_mutex);

    if (data_list.size() >= TEENSY_PAYLOAD_BYTES) {
        // Teensy sends 48 bytes: bus0 nodes 0,1,2 at 0-23; bus1 nodes 0,1,2 at 24-47
        for (int j = 0; j < num_buses; j++) {
            size_t bus_offset = j * num_nodes * 8;
            if (bus_offset + num_nodes * 8 > data_list.size())
                break;
            for (int i = 0; i < num_nodes; i++) {
                std::vector<uint8_t> node_data(data_list.begin() + bus_offset + i * 8,
                                               data_list.begin() + bus_offset + (i + 1) * 8);
                unpack_reply_o2_manual(node_data, bus_list[j], i);
            }
        }
        return;
    }

    for (int j(0); j < num_buses; j++)
    {
        size_t bus_offset = j * num_nodes * 8;
        if (bus_offset + num_nodes * 8 > data_list.size())
            break;
        std::vector<uint8_t> bus_data(data_list.begin() + bus_offset, data_list.begin() + bus_offset + num_nodes * 8);
        for (int i = 0; i < num_nodes; i++)
        {
            std::vector<uint8_t> node_data(bus_data.begin() + i * 8, bus_data.begin() + (i + 1) * 8);
            unpack_reply_o2_manual(node_data, bus_list[j], i);
        }
    }
}

void SpineBoard::update_command()
{
}

// -----------------------------------------------------------------------------
// handle_udp_packet: entry point for received UDP from Teensy
// -----------------------------------------------------------------------------
void SpineBoard::handle_udp_packet(const udp::endpoint &client_endpoint, const std::vector<uint8_t> &data)
{
    static std::chrono::time_point<std::chrono::steady_clock> last_time = std::chrono::steady_clock::now();
    // Unpack the received data
    std::vector<uint8_t> data_list(data.begin(), data.end());

    // Process the received data
    if (boardInitialized)
        process_data(data_list);
#ifdef DEBUG_MODE
    for (int j(0); j < num_buses; j++)
    {
        printf("===============================BUS %d=====================================\n", j);
        print_bus_state(bus_list[j], num_nodes);
        printf("=========================================================================\n");
    }
#endif
    if (!first_state_received)
    {
        first_state_received = true;
    }
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    last_time = current_time;
#ifdef DEBUG_MODE
    std::cout << "Elapsed time since last call: " << elapsed_time.count() * 1000 << " ms" << " i.e. " << 1 / elapsed_time.count() << " Hz" << std::endl;
#endif
}

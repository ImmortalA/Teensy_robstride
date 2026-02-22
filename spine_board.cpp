#include "spine_board.h"
#include <ifaddrs.h>
#include <netinet/in.h>
#include <arpa/inet.h>

SpineBoard::SpineBoard(const std::string &ip, const std::string &interface, int port, int nodes, int buses, std::string _board_name)
    : num_nodes(nodes), num_buses(buses), teensy_ip(ip), teensy_port(port),
      first_state_received(false), bus_list(buses),
      sock_send(io_context), server_socket(io_context), board_name(_board_name)
{
    // Find the enp1s0 interface IP address
    asio::ip::address_v4 enp1s0_address;
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];

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
            enp1s0_address = asio::ip::make_address_v4(host);
            break;
        }
    }

    freeifaddrs(ifaddr);

    if (enp1s0_address.is_unspecified())
    {
        std::cerr << "Failed to find the " << interface << " interface IP address" << std::endl;
        throw std::runtime_error("Failed to find the enp1s0 interface IP address");
    }

    // Bind the sending socket to the enp1s0 interface
    sock_send.open(asio::ip::udp::v4());
    sock_send.bind(asio::ip::udp::endpoint(enp1s0_address, 0));

    // Bind the server socket to the enp1s0 interface
    server_socket.open(asio::ip::udp::v4());
    server_socket.bind(asio::ip::udp::endpoint(enp1s0_address, teensy_port));

    std::cout << "Server bound to " << server_socket.local_endpoint() << std::endl;

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
void SpineBoard::initBoard()
{

    // make sure actuator type is set
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
    printf("num_buses: %d\n", num_buses);

    std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);
    // Send exit motor mode command

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
            pack_cmd(bus_data + i * 8, current_bus, i);
        }
    }

    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    printf("Zero Command sent \n");

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

    for (int i = 0; i < 1; i++)
    {

        send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::microseconds(2000000));

    printf("Enter motor mode sent \n");

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
            pack_cmd(bus_data + i * 8, current_bus, i);
        }
    }

    send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
    std::this_thread::sleep_for(std::chrono::microseconds(1000000));
    printf("Zero Command sent After motor mode \n");
}

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
            pack_cmd(bus_data + i * 8, current_bus, i);
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
                pack_cmd(bus_data + i * 8, bus_list[j], i);
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

void SpineBoard::start()
{
    std::cout << "UDP server listening on " << server_socket.local_endpoint() << std::endl;

    // Start the server in a separate thread
    receive_thread = std::thread([&]()
                                 {
            while (true) {
                std::vector<uint8_t> recv_buffer(num_nodes * 8 * num_buses + num_buses * num_nodes);
                asio::ip::udp::endpoint client_endpoint;
                size_t bytes_received = server_socket.receive_from(asio::buffer(recv_buffer), client_endpoint);
                std::vector<uint8_t> received_data(recv_buffer.begin(), recv_buffer.begin() + bytes_received);
                handle_udp_packet(client_endpoint, received_data);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            } });

    send_thread = std::thread([&]()
                              {
            bool first_time = true;
            while (true) {
                if (first_time)
                {

                    initBoard();
                    zeroEncoders();
                    boardInitialized = true;

                    first_time = false;
                    continue;
                }

                update_command();

                // Generate example data to send
                std::vector<uint8_t> data_to_send(num_nodes * 8 * num_buses);

                // Pack the data to send
                for (int j = 0; j < num_buses; j++) {
                    
                    bus& current_bus = bus_list[j];
                    uint8_t* bus_data = data_to_send.data() + j * num_nodes * 8;

                    for (int i = 0; i < num_nodes; i++) {
                        pack_cmd(bus_data + i * 8, current_bus, i);
                    }
                }

                send_data_to_teensy(data_to_send, num_buses * num_nodes * 8);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            } });
}
void SpineBoard::send_data_to_teensy(const std::vector<uint8_t> &data, const int data_size)
{
    // Pad or truncate the data to match the expected size (num_nodes * 8)
    std::vector<uint8_t> padded_data(data);
    padded_data.resize(data_size, 0);

    // Calculate CRC-8 for the payload
    uint8_t crc_value = calculate_crc8(padded_data.data(), padded_data.size());

    // Append the CRC value to the payload
    std::vector<uint8_t> packet(padded_data);
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

void SpineBoard::process_data(const std::vector<uint8_t> &data_list)
{

    for (int j(0); j < num_buses; j++)
    {
        std::vector<uint8_t> bus_data(data_list.begin() + j * num_nodes * 8, data_list.begin() + (j + 1) * num_nodes * 8);
        for (int i = 0; i < num_nodes; i++)
        {
            std::vector<uint8_t> node_data(bus_data.begin() + i * 8, bus_data.begin() + (i + 1) * 8);

            std::lock_guard<std::mutex> lock(bus_list_mutex);
            unpack_reply(node_data, bus_list[j], i);
        }
    }
}
void SpineBoard::update_command()
{
}
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
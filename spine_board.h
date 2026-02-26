#ifndef SPINE_BOARD_H
#define SPINE_BOARD_H
#include <atomic>
#include <mutex>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include <asio.hpp>
#include "utils.h"
#include <cmath>
// #define DEBUG_MODE
#define ZERO_ENCODERS // remove this line to disable zeroing of encoders

using asio::ip::udp;
class SpineBoard
{
private:
    const int num_nodes;
    const int num_buses;
    std::string teensy_ip;
    int teensy_port;
    bool first_state_received;
    std::vector<bus> bus_list;

    asio::io_context io_context;
    asio::ip::udp::socket sock_send;
    asio::ip::udp::socket server_socket;

    bool actuator_params_set = false;
    mutable std::mutex bus_list_mutex;
    std::thread receive_thread;
    std::thread send_thread;
    std::string board_name;
    std::atomic<bool> allow_command_send_{false};  // when false, send thread does not send Type 1 after init
    std::atomic<bool> use_mit_pack_{false};        // when true, send thread uses pack_cmd (MIT) instead of pack_cmd_private_o2

public:
    SpineBoard(const std::string &ip, const std::string &interface, int port, int nodes, int buses, std::string board_name = "board_1");

    void setActuatorParams(const std::vector<std::vector<ActuatorParams>> &params)
    {
        if (static_cast<int>(params.size()) != num_buses || static_cast<int>(params[0].size()) != num_nodes)
        {
            std::cout << "Error: Invalid parameters size. Expected " << num_buses << " buses and " << num_nodes << " nodes per bus." << std::endl;
            return;
        }

        for (int j = 0; j < num_buses; j++)
        {
            for (int i = 0; i < num_nodes; i++)
            {
                bus_list[j].params[i] = params[j][i];
            }
            bus_list[j].params_vec = params[j];
        }
        actuator_params_set = true;
    }
    std::vector<std::vector<ActuatorParams>> getBoardActuatorParams() const
    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        std::vector<std::vector<ActuatorParams>> params;
        for (int j = 0; j < num_buses; j++)
        {
            std::vector<ActuatorParams> bus_params;
            for (int i = 0; i < num_nodes; i++)
            {
                bus_params.push_back(bus_list[j].params[i]);
            }
            params.push_back(bus_params);
        }
        return params;
    }

    std::vector<bus> getBusList() const
    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        return bus_list;
    }

    void setBusList(const std::vector<bus> &new_bus_list)
    {
        std::lock_guard<std::mutex> lock(bus_list_mutex);
        bus_list = new_bus_list;
    }
    void setAllowCommandSend(bool allow) { allow_command_send_ = allow; }
    void setUseMitPack(bool use) { use_mit_pack_ = use; }
    ~SpineBoard()
    {
        for (int j = 0; j < num_buses; j++)
        {
            delete[] bus_list[j].state.j;
            delete[] bus_list[j].command.j;
            delete[] bus_list[j].params;
        }
    }
    void process_data(const std::vector<uint8_t> &data_list);
    void send_data_to_teensy(const std::vector<uint8_t> &data, const int data_size);
    void handle_udp_packet(const asio::ip::udp::endpoint &client_endpoint, const std::vector<uint8_t> &data);
    void update_command();
    void initBoard();
    void start();
    void end()
    {
        join();
        closeSockets();
    }
    void closeSockets()
    {
        try
        {
            sock_send.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing sock_send: " << e.what() << std::endl;
        }

        // Close the server socket
        try
        {
            server_socket.close();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error closing server_socket: " << e.what() << std::endl;
        }
    }
    void setThreadAffinityAndPriority(int coreId, int priority=49)
    {

        // make sure the thread is running and joinable
        if (!receive_thread.joinable())
        {
            std::cerr << "Error: receive_thread is not joinable\n";
            return;
        }

        if (!send_thread.joinable())
        {
            std::cerr << "Error: RC send_thread is not joinable\n";
            return;
        }
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(coreId, &cpuset);
        int rc = pthread_setaffinity_np(receive_thread.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc != 0)
        {
            std::cerr << "Error calling pthread_setaffinity_np on RC thread: " << rc << "\n";
        }

        rc = pthread_setaffinity_np(send_thread.native_handle(), sizeof(cpu_set_t), &cpuset);
        if (rc != 0)
        {
            std::cerr << "Error calling pthread_setaffinity_np on IMU thread: " << rc << "\n";
        }

        // set priority
        sched_param param;
        param.sched_priority = priority;
        pthread_setschedparam(receive_thread.native_handle(), SCHED_FIFO, &param);
        pthread_setschedparam(send_thread.native_handle(), SCHED_FIFO, &param);
    }
    void join()
    {
        if (receive_thread.joinable())
        {
            receive_thread.join();
        }
        if (send_thread.joinable())
        {
            send_thread.join();
        }
    }
    void restBoard();
    void zeroEncoders();
    void exitMotorMode();
    void enterMotorMode();
    void zeroMotorCommand();
    bool boardInitialized = false;
};
#endif // SPINE_BOARD_H
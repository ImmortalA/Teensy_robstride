#include "spine_board.h"
struct ActuatorInfo
{
    int board;
    int bus;
    int node;
};
int main()
{

    constexpr static int NUM_ACTUATOR_TEENSY = 2;
    constexpr static int ACTUATOR_MAX_NUM_CAN_BUSES = 3;
    constexpr static int ACTUATOR_MAX_NUM_NODES_PER_BUS = 1;

    static std::string BOARD_INTERFACE_NAME = "eno1";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[NUM_ACTUATOR_TEENSY] = {"172.17.0.5", "172.17.0.6"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[NUM_ACTUATOR_TEENSY] = {8003, 8004};

    constexpr static int IMU_PORT = 8000;
    constexpr static int RC_PORT = 8001;

    // actuator | board | bus|
    // hip roll | 1 | 3|
    // hip pitch |2 | 1|
    // knee | 2 | 3|
    // ankle in |1 | 1|
    // ankle out | 1 | 2|
    // toe | 2 | 2|
    static std::vector<ActuatorInfo> ACTUATOR_INFO_MAP = {
        // board, bus, node
        {0, 2, 0},  // hip roll
        {1, 0, 0},  // hip pitch
        {1, 2, 0},  // knee
        {0, 0, 0},  // ankle in
        {0, 1, 0},  // ankle out
        {1, 1, 0}}; // toe

    static std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = {// Robostride O2 on all buses
                                                                                    {
                                                                                        // board 0
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                    },
                                                                                    {
                                                                                        // board 1
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                        {
                                                                                            getActuatorParams(ActuatorType::ROBOSTRIDE_O2),
                                                                                        },
                                                                                    }};

    std::vector<std::unique_ptr<SpineBoard>> _spine_boards;
    _spine_boards.clear();
    for (size_t i = 0; i < NUM_ACTUATOR_TEENSY; i++)
    {
        _spine_boards.push_back(
            std::make_unique<SpineBoard>(ACTUATOR_TEENSY_BOARD_IPS[i],
                                         BOARD_INTERFACE_NAME,
                                         ACTUATOR_TEENSY_BOARD_PORTS[i],
                                         ACTUATOR_MAX_NUM_NODES_PER_BUS,
                                         ACTUATOR_MAX_NUM_CAN_BUSES, "board_" + std::to_string(i)));
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
    {
        auto &board = _spine_boards[id];
        board->setActuatorParams(ACTUATOR_PARAMS[id]);            
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        board->start();
        board->setThreadAffinityAndPriority(id);
    }
    int dt_us = 10000;
    float dt = dt_us / 1e6;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool all_initialized = true;
        for (size_t id = 0; id < _spine_boards.size(); ++id)
        {
            auto &board = _spine_boards[id];
            all_initialized = all_initialized && board->boardInitialized;
        }

        if (all_initialized)
        {
            break;
        }
    }

    std::cout << "Running SpineBoard..." << std::endl;

    while (true)
    {
        static int iter = 0;

        for (size_t i = 0; i < 6; i++)
        {
            auto info = ACTUATOR_INFO_MAP[i];
            auto bus_list = _spine_boards[info.board]->getBusList();
            std::cout << "Actuator: " << i << " " << bus_list[info.bus].state.j[info.node].p << std::endl;
            // bus_list[info.bus].command.j[info.node].p_des = 0.0f;
            // bus_list[info.bus].command.j[info.node].v_des = 0.0f;
            // bus_list[info.bus].command.j[info.node].kp = 0.0f;
            // bus_list[info.bus].command.j[info.node].kd = 0.0f;
            // bus_list[info.bus].command.j[info.node].t_ff = 0.0f;
            // if (i >= 2)
            // {
            //     auto p_cmd = 0.5 * sin(0.5 * iter * dt * 2 * M_PI);
            //     bus_list[info.bus].command.j[info.node].p_des = p_cmd;
            // }
            // else
            // {
            //     auto p_cmd = 0.3 * sin(0.5 * iter * dt * 2 * M_PI);
            //     bus_list[info.bus].command.j[info.node].p_des = p_cmd;
            // }

            // _spine_boards[info.board]->setBusList(bus_list);

            if (i==4)
            {
                // bus_list[info.bus].command.j[info.node].kp = 100.0f;
                // bus_list[info.bus].command.j[info.node].kd = 2.0f;
                bus_list[info.bus].command.j[info.node].t_ff = 0.3 * sin(0.5 * iter * dt * 2 * M_PI);
            }
            // else
            // {
            //     bus_list[info.bus].command.j[info.node].kp = 100.0f;
            //     bus_list[info.bus].command.j[info.node].kd = 2.0f;
            // }
        }
        std::cout << "=====================" << std::endl;
        iter++;
        std::this_thread::sleep_for(std::chrono::microseconds(dt_us));
    }
    for (size_t id = 0; id < _spine_boards.size(); ++id)
    {
        auto &board = _spine_boards[id];
        board->end();
    }
}
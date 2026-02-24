#include "spine_board.h"
struct ActuatorInfo
{
    int board;
    int bus;
    int node;
};
int main()
{

    // Single Teensy, 1 motor on board 0 CAN 1 (Can1 = bus 1)
    constexpr static int NUM_ACTUATOR_TEENSY = 1;
    constexpr static int ACTUATOR_MAX_NUM_CAN_BUSES = 3;
    constexpr static int ACTUATOR_MAX_NUM_NODES_PER_BUS = 1;

    // Ethernet: PC = 192.168.0.100 (enp8s0), Teensy = 192.168.0.101
    static std::string BOARD_INTERFACE_NAME = "enp8s0";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[NUM_ACTUATOR_TEENSY] = {"192.168.0.101"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[NUM_ACTUATOR_TEENSY] = {8003};

    constexpr static int IMU_PORT = 8000;
    constexpr static int RC_PORT = 8001;

    // One motor: board 0, bus 1 (Can1), node 0
    static std::vector<ActuatorInfo> ACTUATOR_INFO_MAP = {
        {0, 1, 0},  // 0: single motor on board 0 CAN 1
    };

    static std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = {
        {
            // board 0: bus 0 (Can0), bus 1 (Can1), bus 2 (unused)
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
        },
    };

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

    // Command fields (see joint_control in utils.h): p_des, v_des, kp, kd, t_ff
    // Robostride O2 limits (utils.h): p [-12.5, 12.5] rad, v [-45, 45], kp [0, 500], kd [0, 5], t_ff [-15, 15] Nm

    while (true)
    {
        static int iter = 0;

        // Get a copy of each board's bus list once per cycle
        std::vector<std::vector<bus>> boards_bus_lists(NUM_ACTUATOR_TEENSY);
        for (size_t b = 0; b < NUM_ACTUATOR_TEENSY; b++)
            boards_bus_lists[b] = _spine_boards[b]->getBusList();

        for (size_t i = 0; i < ACTUATOR_INFO_MAP.size(); i++)
        {
            auto info = ACTUATOR_INFO_MAP[i];
            auto &bus_list = boards_bus_lists[info.board];
            std::cout << "Motor (board 0 CAN 1): p=" << bus_list[info.bus].state.j[info.node].p << std::endl;

            // Position control: try constant 0.5 rad first to see if motor moves; then use sine
            float p_target = 0.5f;  // constant: move to 0.5 rad (try this first); or: 0.2f * sin(0.3f * iter * dt * 2 * M_PI)
            bus_list[info.bus].command.j[info.node].p_des = p_target;
            bus_list[info.bus].command.j[info.node].v_des = 0.0f;
            bus_list[info.bus].command.j[info.node].kp = 80.0f;
            bus_list[info.bus].command.j[info.node].kd = 2.0f;
            bus_list[info.bus].command.j[info.node].t_ff = 0.0f;
        }

        // Push updated commands back so the send thread forwards them to the Teensy
        for (size_t b = 0; b < NUM_ACTUATOR_TEENSY; b++)
            _spine_boards[b]->setBusList(boards_bus_lists[b]);

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
#include "spine_board.h"
#include <cmath>
#include <limits>

struct ActuatorInfo
{
    int board;
    int bus;
    int node;
};

int main()
{
    // -------- Hardware / topology --------
    // Single Teensy, 1 motor on board 0 CAN 0 (Can0 = first bus)
    constexpr static int NUM_ACTUATOR_TEENSY = 1;
    constexpr static int ACTUATOR_MAX_NUM_CAN_BUSES = 3;
    constexpr static int ACTUATOR_MAX_NUM_NODES_PER_BUS = 1;

    // -------- Network --------
    // PC = 192.168.0.100 (enp8s0), Teensy = 192.168.0.101
    static std::string BOARD_INTERFACE_NAME = "enp8s0";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[NUM_ACTUATOR_TEENSY] = {"192.168.0.101"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[NUM_ACTUATOR_TEENSY] = {8003};

    constexpr static int IMU_PORT = 8000;
    constexpr static int RC_PORT = 8001;

    // -------- Actuator map and params --------
    // One motor: board 0, bus 0 (Can0), node 0
    static std::vector<ActuatorInfo> ACTUATOR_INFO_MAP = {
        {0, 0, 0},  // single motor on board 0 CAN 0 (first port)
    };

    static std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = {
        {
            // board 0: bus 0 (Can0), bus 1 (Can1), bus 2 (unused)
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
            {getActuatorParams(ActuatorType::ROBOSTRIDE_O2)},
        },
    };

    // -------- Create and start boards --------
    std::vector<std::unique_ptr<SpineBoard>> _spine_boards;
    _spine_boards.clear();
    for (size_t i = 0; i < NUM_ACTUATOR_TEENSY; i++)
    {
        _spine_boards.push_back(
            std::make_unique<SpineBoard>(ACTUATOR_TEENSY_BOARD_IPS[i],
                                         BOARD_INTERFACE_NAME,
                                         ACTUATOR_TEENSY_BOARD_PORTS[i],
                                         ACTUATOR_MAX_NUM_NODES_PER_BUS,
                                         ACTUATOR_MAX_NUM_CAN_BUSES,
                                         "board_" + std::to_string(i)));
    }

    for (size_t id = 0; id < _spine_boards.size(); ++id)
    {
        auto &board = _spine_boards[id];
        board->setActuatorParams(ACTUATOR_PARAMS[id]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        board->start();
        board->setThreadAffinityAndPriority(id);
    }

    // -------- Wait until init (and user prompts) are done --------
    int dt_us = 10000;
    float dt = dt_us / 1e6;
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool all_initialized = true;
        for (size_t id = 0; id < _spine_boards.size(); ++id)
            all_initialized = all_initialized && _spine_boards[id]->boardInitialized;
        if (all_initialized)
            break;
    }

    // -------- Select test mode --------
    std::cout << "Select test mode (all use Type 1 operation control unless noted):\n";
    std::cout << "  0: Enable only (no Type 1 commands)\n";
    std::cout << "  1: Private – position step to 0.2 rad\n";
    std::cout << "  2: Private – position sine (amp=0.2 rad, 1 rad/s)\n";
    std::cout << "  3: MIT – same as 1 but MIT payload (16b p + 12b v,kp,kd,torque), t_ff=0.5 Nm\n";
    std::cout << "  4: Private – velocity hold v_des=0.2 rad/s\n";
    std::cout << "Mode: " << std::flush;

    int mode = 0;
    if (!(std::cin >> mode))
        mode = 0;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (mode == 0)
    {
        std::cout << "Running SpineBoard... Motor enabled. No position commands sent." << std::endl;
        std::cout << "Press Enter to exit." << std::flush;
        std::cin.get();
    }
    else
    {
        bool use_mit = (mode == 3);
        for (auto &board : _spine_boards)
        {
            board->setAllowCommandSend(true);
            board->setUseMitPack(use_mit);
        }

        std::cout << "Running control test (Ctrl+C to stop)...\n";

        int iter = 0;
        while (true)
        {
            std::vector<std::vector<bus>> boards_bus_lists(NUM_ACTUATOR_TEENSY);
            for (size_t b = 0; b < NUM_ACTUATOR_TEENSY; b++)
                boards_bus_lists[b] = _spine_boards[b]->getBusList();

            for (size_t i = 0; i < ACTUATOR_INFO_MAP.size(); i++)
            {
                auto info = ACTUATOR_INFO_MAP[i];
                auto &bus_list = boards_bus_lists[info.board];

                float p_target = 0.0f;
                float v_target = 0.0f;
                float t_ff = 0.0f;

                if (mode == 1 || mode == 3)
                {
                    p_target = 0.2f;
                    if (mode == 3)
                        t_ff = 0.5f;
                }
                else if (mode == 2)
                {
                    float t = iter * dt;
                    p_target = 0.2f * std::sin(t);
                }
                else if (mode == 4)
                {
                    v_target = 0.2f;
                }

                // Type 1 limits (PROTOCOL_REFERENCE): Kp 0–500, Kd 0–5. Ki not in frame (set via param 0x2015 if needed).
                bus_list[info.bus].command.j[info.node].p_des = p_target;
                bus_list[info.bus].command.j[info.node].v_des = v_target;
                bus_list[info.bus].command.j[info.node].kp = 25.0f;   // typical 25–30 from docs; max 500
                bus_list[info.bus].command.j[info.node].kd = 2.5f;    // damping 2–5 typical; max 5
                bus_list[info.bus].command.j[info.node].t_ff = t_ff;

                if (iter % 100 == 0)
                {
                    std::cout << "p=" << bus_list[info.bus].state.j[info.node].p
                              << " p_des=" << p_target << " v_des=" << v_target
                              << " t_ff=" << t_ff << std::endl;
                }
            }

            for (size_t b = 0; b < NUM_ACTUATOR_TEENSY; b++)
                _spine_boards[b]->setBusList(boards_bus_lists[b]);

            iter++;
            std::this_thread::sleep_for(std::chrono::microseconds(dt_us));
        }
    }

    // -------- Cleanup --------
    for (size_t id = 0; id < _spine_boards.size(); ++id)
        _spine_boards[id]->end();

    return 0;
}

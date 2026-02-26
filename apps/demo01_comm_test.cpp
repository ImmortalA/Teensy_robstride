#include "spine_board.h"

int main()
{
    // -------- Hardware / topology --------
    constexpr static int NUM_ACTUATOR_TEENSY = 1;
    constexpr static int ACTUATOR_MAX_NUM_CAN_BUSES = 3;
    constexpr static int ACTUATOR_MAX_NUM_NODES_PER_BUS = 1;

    // -------- Network --------
    // PC = 192.168.0.100 (enp8s0), Teensy = 192.168.0.101
    static std::string BOARD_INTERFACE_NAME = "enp8s0";
    static std::string ACTUATOR_TEENSY_BOARD_IPS[NUM_ACTUATOR_TEENSY] = {"192.168.0.101"};
    static int ACTUATOR_TEENSY_BOARD_PORTS[NUM_ACTUATOR_TEENSY] = {8003};

    // -------- Actuator params --------
    static std::vector<std::vector<std::vector<ActuatorParams>>> ACTUATOR_PARAMS = {
        {
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

    // -------- Wait until init is done --------
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        bool all_initialized = true;
        for (size_t id = 0; id < _spine_boards.size(); ++id)
            all_initialized = all_initialized && _spine_boards[id]->boardInitialized;
        if (all_initialized)
            break;
    }

    // -------- Run: enable only, wait for user to exit --------
    std::cout << "Running SpineBoard communication test... Motor enabled. No position commands sent." << std::endl;
    std::cout << "Press Enter to exit." << std::flush;
    std::cin.get();

    // -------- Cleanup --------
    for (size_t id = 0; id < _spine_boards.size(); ++id)
        _spine_boards[id]->end();

    return 0;
}


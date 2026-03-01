## Teensy-Spine-Board

Teensy 4.1 Ethernet–UDP to CAN bus bridge and C++ host demos for Robstride O2 actuators (extended 29‑bit CAN protocol).

### Repository layout

| Path | Description |
|------|--------------|
| **`apps/`** | PC/host executables (built by CMake) |
| `apps/main.cpp` | Main control demo (modes 0–4, private + MIT). |
| `apps/demo01_comm_test.cpp` | Basic comms test between PC and Teensy. |
| `apps/demo02_MIT.cpp` | MIT‑only control demo. |
| `apps/demo03_daisy_chain_2_motors.cpp` | Two motors on Can0 (daisy chain). |
| **`teensy/teensy.ino`** | Teensy 4.1 firmware (Ethernet ↔ CAN bridge, Robstride O2). Sends CAN commands, waits for Type 2 from expected motor(s) (see `NUM_DAISY_MOTORS`), then accepts next UDP batch. |
| **`examples_teensy/`** | Arduino/Teensy examples (not built by CMake). |
| `examples_teensy/E01_test_CAN/` | CAN bus test. |
| `examples_teensy/E02_enable_motor_only/` | Enable motor only. |
| `examples_teensy/E03_pc_teensy_udp_bridge/` | PC–Teensy UDP bridge. |
| `examples_teensy/E04_single_motor_feedback_timing/` | Single-motor feedback timing. |
| `examples_teensy/E05_daisy_chain_2_motors/` | Daisy-chain 2 motors (Teensy sketch). |
| `examples_teensy/E06_daisy_chain_feedback_timing/` | Daisy chain + E04-style timing: send→response (Teensy CAN send → receive Type 2), "Daisy Can0 timing" summary. |
| **`robstride_o2/`** | C++ helper library for O2 private protocol (ID packing, Type 1/2/18 frames, MIT packing). |
| **`spine_board.cpp`, `spine_board.h`** | Host UDP client, threading, and bus/state management. |
| **`utils.h`** | Shared utilities. |
| **`ETHERNET_SETUP.md`** | Network and IP setup (PC ↔ Teensy). |

Optional (gitignored): **`extra_materials/`** – vendor docs and reference code.

### Requirements

- CMake ≥ 3.10, C++14 compiler, pthreads (Linux).
- Asio headers (`asio.hpp`) available to CMake (standalone or Boost).
- Teensy 4.1 with:
  - `teensy/teensy.ino` flashed.
  - Static IP `192.168.0.101` (or update IPs in `apps/*.cpp` and `ETHERNET_SETUP.md`).
  - Robstride O2 motor(s) on Can0, ID(s) matching `MOTOR_ID` (and `MOTOR_ID+1` for 2-motor daisy). Set `NUM_DAISY_MOTORS` to 1 (single motor) or 2 (daisy) in `teensy/teensy.ino` and E06. For 2 motors, if only one node appears in timing, set `O2_FEEDBACK_ID_MIDDLE_BYTE 1`.

### Building

```bash
cd Teensy-Spine-Board-main
mkdir -p build
cd build
cmake ..
make -j4
```

Executables (in `build/`):

| Binary | Source | Description |
|--------|--------|-------------|
| `test_spine` | `apps/main.cpp` | Main demo, mode menu (0–4). |
| `test_spine_mit` | `apps/demo02_MIT.cpp` | MIT-only sine control. |
| `comm_test` | `apps/demo01_comm_test.cpp` | PC–Teensy comm test. |
| `daisy_chain_2_motors` | `apps/demo03_daisy_chain_2_motors.cpp` | Two motors on Can0. |

### Running the main demo

On the PC (after flashing `teensy/teensy.ino` and wiring CAN + power):

```bash
cd build
./test_spine
```

Choose a mode (0–4). Mode 3 uses the MIT payload; others use the private Type 1 control frame.

Network (editable in `apps/main.cpp`, `apps/demo02_MIT.cpp`, `apps/demo03_daisy_chain_2_motors.cpp`):

- PC: `192.168.0.100` on interface `enp8s0`.
- Teensy: `192.168.0.101`, UDP port `8003`. See `ETHERNET_SETUP.md`.

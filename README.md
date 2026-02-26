## Teensy-Spine-Board

Teensy 4.1 Ethernet–UDP to CAN bus bridge and C++ host demos for Robstride O2 actuators (extended 29‑bit CAN protocol).

### Repository layout

- **`apps/`** – PC/host examples  
  - `main.cpp` – main control demo (modes 0–4, private + MIT).  
  - `main_MIT.cpp` – MIT‑focused variant.  
  - `main_comm_test.cpp` – basic comms test between PC and Teensy.
- **`teensy/teensy.ino`** – Teensy 4.1 firmware (Ethernet ↔ CAN bridge, Robstride O2 type mapping).
- **`robstride_o2/`** – C++ helper library for the O2 private protocol (ID packing, Type 1/2/18 frames, MIT packing).
- **`spine_board.cpp/.h`** – host UDP client, threading, and bus/state management.
- **`examples_Teensy/`** – original Arduino examples from Robstride (not built by CMake).
- **`extra_materials/`** – vendor docs and reference code (ignored by git).

### Requirements

- CMake ≥ 3.10, a C++14 compiler, and pthreads (Linux).  
- Boost.Asio or standalone Asio headers available to CMake as `asio.hpp`.  
- Teensy 4.1 with:
  - `teensy/teensy.ino` flashed.  
  - Static IP `192.168.0.101` (or update IPs in `apps/*.cpp` and `ETHERNET_SETUP.md`).  
  - Robstride O2 motor on CAN0, ID matching `MOTOR_ID` in `teensy.ino`.

### Building

```bash
cd Teensy-Spine-Board-main
mkdir -p build
cd build
cmake ..
make -j4
```

This produces:

- `test_spine` – main host demo.  
- `test_spine_mit` – MIT‑focused demo.  
- `comm_test` – comms test.

### Running the main demo

On the PC (after flashing the Teensy firmware and wiring CAN + power):

```bash
cd build
./test_spine
```

You will be prompted to select a mode (0–4). Mode 3 uses the MIT payload; other modes use the private Type 1 control frame.

Network assumptions (editable in `apps/main.cpp` and `apps/main_MIT.cpp`):

- PC: `192.168.0.100` on interface `enp8s0`.  
- Teensy: `192.168.0.101`, UDP port `8003` (see `ETHERNET_SETUP.md`).
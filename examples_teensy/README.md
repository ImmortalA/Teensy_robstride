# Teensy examples

Arduino/Teensy sketches for the Spine Board (Teensy 4.1 + Ethernet + CAN). These are **not** built by the repo’s CMake; open and upload them from the Arduino IDE or PlatformIO.

Host apps (PC) live in **`../apps/`** and are built by CMake. After building, run the executables from `../build/`.

---

## Which host app to use with each example

| Teensy example | Host app (in `apps/`) | Build output | Notes |
|----------------|------------------------|--------------|--------|
| **E01_test_CAN** | — | — | **Standalone.** CAN bus test only (no Ethernet). No PC app. |
| **E02_enable_motor_only** | — | — | **Standalone.** Sends enable sequence (Type 4 → 6 → 3) to the motor. No UDP, no PC app. |
| **E03_pc_teensy_udp_bridge** | `main.cpp`, `demo01_comm_test.cpp`, `demo02_MIT.cpp`, `demo03_daisy_chain_2_motors.cpp` | `test_spine`, `comm_test`, `test_spine_mit`, `daisy_chain_2_motors` | UDP ↔ CAN bridge. Use with any of these apps for single or two motors. |
| **E04_single_motor_feedback_timing** | `main.cpp`, `demo01_comm_test.cpp`, `demo02_MIT.cpp` | `test_spine`, `comm_test`, `test_spine_mit` | Same as E03 with timing/feedback stats. Best with **single motor**. |
| **E05_daisy_chain_2_motors** | `demo03_daisy_chain_2_motors.cpp` | `daisy_chain_2_motors` | Bridge with 2 motors enabled at startup. Use **`daisy_chain_2_motors`** for two motors on Can0. Also works with `test_spine`, `test_spine_mit`, `comm_test`. |

---

## Quick reference

- **Only test CAN / only enable motor:** flash **E01** or **E02**. No host app.
- **Single motor, full control from PC:** flash **E03** or **E04**, then run `./test_spine` or `./test_spine_mit` or `./comm_test`.
- **Two motors (daisy chain on Can0):** flash **E05**, then run `./daisy_chain_2_motors`.

Network: PC = 192.168.0.100, Teensy = 192.168.0.101, port 8003. See **`../ETHERNET_SETUP.md`**.

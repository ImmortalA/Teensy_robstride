# Teensy examples (legacy)

Arduino/Teensy sketches for the **Spine board** workflow (Teensy + Ethernet + CAN). These are **not** built by CMake; open them in the Arduino IDE.

**RobStride Serial/CAN controller:** use the sketch at the **repository root**: **`Robstride_Controller/`** (moved out of this folder).

Host PC apps for E03–E06 live under **`../apps/`** and are built from **`../CMakeLists.txt`** (run CMake from **`legacy/build`** as described in **`../README.md`** in this legacy tree).

---

## Which host app to use with each example

| Teensy example | Host app (in `../apps/`) | Build output | Notes |
|----------------|--------------------------|--------------|--------|
| **E01_test_CAN** | — | — | **Standalone.** CAN only. |
| **E02_enable_motor_only** | — | — | **Standalone.** Enable sequence. |
| **E03_pc_teensy_udp_bridge** | `main.cpp`, `demo01_comm_test.cpp`, `demo02_MIT.cpp`, `demo03_daisy_chain_2_motors.cpp` | `test_spine`, `comm_test`, `test_spine_mit`, `daisy_chain_2_motors` | UDP ↔ CAN bridge. |
| **E04_single_motor_feedback_timing** | same as E03 subset | `test_spine`, `comm_test`, `test_spine_mit` | Timing / single motor. |
| **E05_daisy_chain_2_motors** | `demo03_daisy_chain_2_motors.cpp` | `daisy_chain_2_motors` | Two motors on Can0. |
| **E06_daisy_chain_feedback_timing** | `demo03_daisy_chain_2_motors.cpp` | `daisy_chain_2_motors` | Daisy timing. |
| **E07_teensy_only_mit_sine** | — | — | **Standalone.** Enable + Type 18 RUN_MODE + MIT Type 1 sine; choose a RobStride motor model in the sketch. |

Network: see **`../ETHERNET_SETUP.md`**.

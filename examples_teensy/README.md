# Teensy examples

Arduino/Teensy sketches for the Spine Board (Teensy 4.1 + Ethernet + CAN). These are **not** built by the repo’s CMake; open and upload them from the Arduino IDE or PlatformIO.

Host apps (PC) live in **`../apps/`** and are built by CMake. After building, run the executables from `../build/`.

---

## Which host app to use with each example

| Teensy example | Host app (in `apps/`) | Build output | Notes |
|----------------|------------------------|--------------|--------|
| **E01_test_CAN** | — | — | **Standalone.** CAN bus test only (no Ethernet). No PC app. |
| **E02_enable_motor_only** | — | — | **Standalone.** Sends enable sequence (Type 4 → 6 → 3) to the motor. No UDP, no PC app. |
| **E03_pc_teensy_udp_bridge** | `main.cpp`, `demo01_comm_test.cpp`, `demo02_MIT.cpp`, `demo03_daisy_chain_2_motors.cpp` | `test_spine`, `comm_test`, `test_spine_mit`, `daisy_chain_2_motors` | UDP ↔ CAN bridge. **Send → wait for Type 2 (1 motor) → next batch.** Set `NUM_DAISY_MOTORS` to 2 in sketch for daisy. |
| **E04_single_motor_feedback_timing** | `main.cpp`, `demo01_comm_test.cpp`, `demo02_MIT.cpp` | `test_spine`, `comm_test`, `test_spine_mit` | Timing: **send→response** (µs/Hz), avg cmd→fb. **Wait for 1 motor’s Type 2 before next batch.** Single motor only. |
| **E05_daisy_chain_2_motors** | `demo03_daisy_chain_2_motors.cpp` | `daisy_chain_2_motors` | Bridge with 2 motors enabled at startup. Use **`daisy_chain_2_motors`** for two motors on Can0. |
| **E06_daisy_chain_feedback_timing** | `demo03_daisy_chain_2_motors.cpp` | `daisy_chain_2_motors` | Timing for daisy: **send→response** per motor, "Daisy Can0 timing" when both report. **Wait for both motors’ Type 2 before next batch.** Set `NUM_DAISY_MOTORS` to 1 or 2; set `O2_FEEDBACK_ID_MIDDLE_BYTE 1` if only node 1 appears. |

---

## Quick reference

- **Only test CAN / only enable motor:** flash **E01** or **E02**. No host app.
- **Single motor, full control from PC:** flash **E03** or **E04**, then run `./test_spine` or `./test_spine_mit` or `./comm_test`.
- **Two motors (daisy chain on Can0):** flash **E05**, then run `./daisy_chain_2_motors`.
- **Two motors + timing (like E04 for daisy):** flash **E06**, run `./daisy_chain_2_motors`, watch Serial for `send->response` (µs/Hz) and "Daisy Can0 timing". E06 waits for both motors’ Type 2 before accepting the next UDP batch.

**Wait-for-response:** E03, E04, E06 (and main teensy.ino) accept the next UDP control packet only after receiving Type 2 from the expected motor(s)—send batch → wait response(s) → next batch. E03/E04 use 1 motor; teensy.ino and E06 use `NUM_DAISY_MOTORS` (1 or 2).

Network: PC = 192.168.0.100, Teensy = 192.168.0.101, port 8003. See **`../ETHERNET_SETUP.md`**.

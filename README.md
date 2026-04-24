# RobStride Teensy CAN controller

Teensy 4.1 firmware for **RobStride** actuators over **CAN** (FlexCAN_T4, extended IDs). Control and diagnostics use a **USB Serial** command line (no Ethernet / no PC bridge in this workflow).

<p align="center">
  <img src="setup.jpeg" alt="Bench setup with motors and controller" width="800"/>
</p>

## What to open in the Arduino IDE

Open the sketch folder:

| Path | Contents |
|------|-----------|
| **`Robstride_Controller/`** | `Robstride_Controller.ino` + `RobStrideMotor.cpp` / `RobStrideMotor.h` |

1. Install **Teensyduino** (or your Teensy board package) and the **FlexCAN_T4** library.  
2. Select your Teensy board (e.g. Teensy 4.1) and the correct USB/COM port.  
3. Edit **`USER CONFIG`** at the top of `Robstride_Controller.ino` (CAN bitrate, motor IDs, models, watchdog, stream defaults).  
4. Upload, open Serial Monitor at **115200**, type **`help`** for the full command list.

Typical bring-up: **`reset`** → **`enable 1`** (use your motor CAN id) → **`mode 1 velocity`** → **`vel 1 0.5`** (rad/s), or **`all enable`** / **`sync vel …`** for multiple motors.

## Hardware

- Teensy 4.1 (or as supported by your board selection) on CAN1 (see sketch: `FlexCAN_T4<CAN1, …>`).  
- RobStride drives on the same CAN bus; IDs must match **`MOTOR_IDS[]`** in the sketch.

## Legacy (not required for this project)

Older **Spine board** material (Ethernet ↔ UDP ↔ CAN bridge, CMake PC apps, and Teensy examples **E01–E06**) is kept under **`legacy/`** for reference only. See **`legacy/README.md`** if you need that stack.

# Xiaomi Motor ESP32 Control

ESP32 CAN control for Xiaomi motors: multiple modes and real-time status.

## Overview

- CAN communication (1 Mbps)
- Modes: speed, position, current, operation control
- Real-time status over serial
- Serial command interface

## Hardware

### Main
- ESP32 board
- CAN module (1 Mbps)
- Xiaomi motor (CAN ID = 1)
- 12–24 V DC supply

### Wiring

**ESP32 ↔ CAN module**
```
ESP32    CAN module
GPIO5 ←→ TX
GPIO4 ←→ RX
3.3V  ←→ VCC
GND   ←→ GND
```

**CAN module ↔ Motor**
```
CAN module   Motor
CAN_H     ←→ CAN_H
CAN_L     ←→ CAN_L
12–24V    ←→ power+
GND       ←→ GND
```

## Software

### Arduino IDE
1. Add ESP32 board URL: `https://dl.espressif.com/dl/package_esp32_index.json`
2. Install board: **Espressif Systems ESP32**
3. Board: **ESP32 Arduino → ESP32 Dev Module** (or ESP32S3 Dev Module as in project)

### Library
- **ESP32-TWAI-CAN** (from Library Manager)

## Quick Start

1. Open `mi_motor_control.ino`
2. Select board (e.g. ESP32S3 Dev Module) and port
3. Upload
4. Serial Monitor 115200
5. Watch init and real-time data

## Features

### Auto test
On start, speed steps: 0 → 2 → -2 → 1 → -1 → 0 rad/s every 5 s.

### Serial commands

| Command       | Action              | Example    |
|---------------|---------------------|------------|
| `stop`        | Stop motor          | `stop`     |
| `enable`      | Enable motor        | `enable`   |
| `disable`     | Disable motor       | `disable`  |
| `speed X`     | Set speed (rad/s)   | `speed 3.5`|
| `pos X`       | Position mode (rad) | `pos 1.57` |
| `speed_mode`  | Switch to speed mode| `speed_mode`|
| `zero`        | Set zero            | `zero`     |
| `help`        | Help                | `help`     |

### Real-time output
```
M1: master_id, motor_id, error_state, HALL_err, encoder_err, temp_err, cur_err, volt_err, mode, angle, speed, torque, temp
```

## Control Modes

1. **Speed (SPEED_MODE):** Target angular velocity -30 to +30 rad/s.
2. **Position (POS_MODE):** Target angle about -12.5 to +12.5 rad; max speed limit.
3. **Current (CUR_MODE):** Target current -23 to +23 A.
4. **Operation (CTRL_MODE):** Torque, position, velocity, PD in one frame.

## CAN / Data

- **29-bit extended ID:** type(5 bit) + data2(16 bit) + address(8 bit)
- **8-byte payload:** control or feedback

Feedback scaling (concept):
- Angle: raw → about -4π to 4π rad
- Speed: raw → about -30 to 30 rad/s
- Torque: raw → about -12 to 12 Nm
- Temperature: raw/10 °C

## Safety

- **Power:** 12–24 V only; do not exceed rating
- **CAN:** Correct CAN_H/CAN_L; avoid reverse
- **Ground:** Good common ground
- **Parameters:** Stay within limits; enable after config; monitor errors and temperature
- **Mechanical:** Secure mounting; clear range; have emergency stop; test with small values first

## Parameters (example)

```cpp
#define MOTER_1_ID  1
#define MASTER_ID   0
#define P_MIN       -12.5f
#define P_MAX       12.5f
#define V_MIN       -30.0f
#define V_MAX       30.0f
#define T_MIN       -12.0f
#define T_MAX       12.0f
```

CAN: TX=5, RX=4; baud 1 Mbps (in code).

## Troubleshooting

- **CAN fail:** Check wiring, 1 Mbps, 120 Ω termination
- **Motor no response:** Check power, motor ID, enable command
- **Bad data:** Increase timeout; check bus noise; verify parsing
- **Build error:** Check ESP32 board package and library version

## Extensions (ideas)

- Multi-motor control
- Trajectory planning
- PID auto-tuning
- WiFi/Web control
- SD logging
- Fault diagnosis

---

**Version:** v1.0 · 2024 · ESP32S3 + Xiaomi motor · Arduino IDE

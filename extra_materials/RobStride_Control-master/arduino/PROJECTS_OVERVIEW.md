# ESP32 Xiaomi Motor Project Collection

## Overview

This repo contains several Arduino projects for controlling Xiaomi motors from ESP32, from basic to advanced.

---

## Project Structure

```
mi_arduino/
├── mi_motor_control/          # Basic motor control
├── joint_position_control/    # Joint position control
├── simple_joint_control/      # Simplified joint control
├── ANTI_VIBRATION_GUIDE.md    # Vibration solutions
└── PROJECTS_OVERVIEW.md       # This file
```

---

## Project Comparison

| Project                  | Complexity | Vibration | Use case           | Notes              |
|--------------------------|------------|-----------|--------------------|--------------------|
| **mi_motor_control**     | ⭐⭐⭐       | Basic     | Multi-mode demo    | Full feature set   |
| **joint_position_control**| ⭐⭐⭐⭐      | Good      | Joint control      | Serial commands, monitoring |
| **simple_joint_control** | ⭐         | **Best**  | Quick prototype    | Simplest, low vibration |

---

## Which One to Use

### Quick start → **simple_joint_control**
```
Pros: Easiest; just type numbers to control
Use: 1.57 or 90 to move to that position
```

### Professional use → **joint_position_control**
```
Pros: Full joint control features
Use: pos 1.57, angle 90, status, etc.
```

### Feature demo → **mi_motor_control**
```
Pros: Shows all control modes
Use: Auto demo of speed, position, operation control
```

---

## Project Details

### 1. mi_motor_control – Basic motor control

**Purpose:** Learn basic Xiaomi motor control.

**Features:**
- 4 control modes
- Auto test sequence
- Serial commands
- Real-time monitoring

**Commands:**
```bash
speed 2.0     # Set speed
pos 1.57      # Set position
enable        # Enable motor
help          # Help
```

**Use:** Learning, testing, demos.

---

### 2. joint_position_control – Joint position control

**Purpose:** Dedicated joint motor control.

**Features:**
- Position control mode
- Serial command interface
- Smooth motion
- Status monitoring
- Error detection

**Commands:**
```bash
pos 1.57       # 1.57 rad
angle 90       # 90°
speed 2.0      # Max speed
status         # Status
```

**Use:** Robot joints, arms, precise positioning.

---

### 3. simple_joint_control – Simplified joint control **(recommended)**

**Purpose:** Simplest joint control with minimal vibration.

**Features:**
- Very simple operation
- Tuned anti-vibration parameters
- Direct number input
- Auto radian/angle detection

**Commands:**
```bash
1.57           # 1.57 rad (90°)
90             # 90°
-45            # -45°
stop           # Stop
```

**Use:** Quick prototypes, teaching, simple apps.

---

## Hardware

### General
- ESP32 board
- CAN module (1 Mbps)
- Xiaomi motor (CAN ID=1)
- 12–24 V power

### Wiring
```
ESP32      CAN module    Xiaomi motor
GPIO5  ←→  TX       ←→  CAN_H
GPIO4  ←→  RX       ←→  CAN_L
3.3V   ←→  VCC      ←→  12-24V
GND    ←→  GND      ←→  GND
```

---

## Quick Start

### 1. Setup
```bash
1. Arduino IDE + ESP32 board support
2. Library: ESP32-TWAI-CAN
3. Board: ESP32 Dev Module
```

### 2. Choose project
```bash
Beginner:     simple_joint_control
Application:  joint_position_control
Learning:     mi_motor_control
```

### 3. Upload and run
```bash
1. Open the project .ino
2. Connect ESP32, select port
3. Upload
4. Serial Monitor (115200)
5. Send commands to control motor
```

---

## Control Modes

| Mode        | Code       | Use             | Vibration | Supported in   |
|-------------|------------|-----------------|-----------|----------------|
| **Position**| POS_MODE   | Precise position| Lowest    | All            |
| **Velocity**| SPEED_MODE | Continuous turn | Medium    | mi_motor_control |
| **Operation**| CTRL_MODE | Complex control | Higher    | mi_motor_control |
| **Current** | CUR_MODE   | Force control   | Higher    | mi_motor_control |

**Best for joints: position mode + tuned parameters.**

---

## Vibration

### Causes
1. Gains too high
2. Speed changes too fast
3. Abrupt mode changes

### What to do
1. **Use tuned sketch:** `simple_joint_control`
2. **Lower max speed:** e.g. ≤ 2.0 rad/s
3. **Lower gains:** e.g. KP 15–25
4. **Add delay:** e.g. 20 ms between commands

**Details:** `ANTI_VIBRATION_GUIDE.md`

---

## Performance

| Project                  | Response | Smoothness | Ease of use | Features   |
|--------------------------|----------|------------|-------------|------------|
| mi_motor_control         | Fast     | Medium     | Medium      | ⭐⭐⭐⭐⭐     |
| joint_position_control   | Medium   | Good       | Good        | ⭐⭐⭐⭐      |
| simple_joint_control     | Medium   | **Best**   | **Best**    | ⭐⭐⭐       |

---

## Troubleshooting

### Common issues
1. **Build error:** Check ESP32 board and libraries
2. **No CAN:** Check wiring and baud rate
3. **Motor not moving:** Check power and enable
4. **Vibration:** Prefer `simple_joint_control`

### Debug order
1. Test basics with `mi_motor_control`
2. Check smooth motion with `simple_joint_control`
3. Build application with `joint_position_control`

---

## Recommendations

### Development flow
```bash
1. Test:    mi_motor_control
2. Verify:  simple_joint_control
3. Deploy:  joint_position_control
```

### Learning path
```bash
Beginner:  simple_joint_control → joint_position_control
Developer: mi_motor_control → simple_joint_control → joint_position_control
```

### Project choice
```bash
Quick check  → simple_joint_control
Joint application → joint_position_control
Learning modes   → mi_motor_control
```

---

## Support

- Vibration: `ANTI_VIBRATION_GUIDE.md`
- Hardware: each project README
- Software: project docs

---

**Last updated:** 2024  
**Version:** v2.0 ESP32 optimized

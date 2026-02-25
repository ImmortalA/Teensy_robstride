# Joint Position Control

**Position control tuned for joint motors**

## Features

- Joint-oriented position control
- Serial commands (position, angle, speed)
- Smooth motion, reduced vibration
- Real-time status
- Error detection

## Serial Commands

### Position
```bash
pos 1.57       # go to 1.57 rad
angle 90       # go to 90°
speed 2.0      # set max speed 2.0 rad/s
```

### Control
```bash
stop           # stop
zero           # set current position as zero
enable         # enable motor
disable        # disable motor
status         # detailed status
help           # help
```

## Hardware

```
ESP32      CAN module    Motor
GPIO5  ←→ TX       ←→ CAN_H
GPIO4  ←→ RX       ←→ CAN_L
3.3V   ←→ VCC      ←→ 12-24V
GND    ←→ GND      ←→ GND
```

## Steps

1. **IDE:** Board = ESP32 Dev Module; install **ESP32-TWAI-CAN**
2. **Upload:** Open `joint_position_control.ino`, select port, upload
3. **Control:** Serial Monitor 115200, send position commands

## Parameters

- **Max speed:** 3.0 rad/s (configurable)
- **Position KP:** 30.0 (low vibration)
- **Velocity KP:** 10.0
- **Position tolerance:** 0.05 rad

## Status

Shows in real time:
- Position (rad and deg)
- Velocity
- Torque
- Temperature
- Error state

## Use Cases

- Robot joint control
- Arm position control
- Servo replacement
- Precise angle control

See parent folder’s `ANTI_VIBRATION_GUIDE.md` for vibration tuning.

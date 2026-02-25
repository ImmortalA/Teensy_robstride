# Simple Joint Control

**Minimal joint position control with 0–360° angle input and reduced vibration**

## Features

- **0–360° angle input** (recommended), very intuitive
- **Default motor ID = 11**, changeable at runtime
- **Dynamic motor ID config** without recompiling
- Config saved automatically, survives power loss
- Very simple, core functionality only
- Low vibration, tuned parameters
- Auto-detect angle vs radian input
- Plug and play

## Default Config

```cpp
Motor ID: 11
Master ID: 0
Max speed: 2.0 rad/s
Position KP: 30.0
Torque limit: 12.0 Nm
```

## Commands

### Angle input (0–360°, recommended)
```bash
0              # 0° (home)
90             # 90° CW
180            # 180° CW
270            # 270° CW
360            # back to 0°
45             # 45° CW
135            # 135° CW
```

### Radian input
```bash
1.57           # π/2 rad (90°)
3.14           # π rad (180°)
-1.57          # -π/2 rad (-90°)
0              # zero
```

### Control
```bash
stop           # stop immediately
zero           # set current position as zero
id X           # set motor ID to X (1–127), default 11
config         # show current config
restart        # restart motor (after changing ID)
```

**Note:** Default motor ID is 11. If your motor has a different ID, use `id X` to change it.

## Angle System

- **0°** = home
- **90°** = 90° CW
- **180°** = 180° CW
- **270°** = 270° CW
- **360°** = same as 0°

Input rules:
- **0–360** → treated as degrees
- Small values → treated as radians
- Values > 360° → normalized to 0–360°

## Tuned Parameters

```cpp
Max speed: 2.0 rad/s   // less vibration
Position KP: 20.0       // conservative
Speed KP: 10.0          // stable
```

## Quick Start

1. **Upload:** Open `simple_joint_control.ino`, select ESP32, upload.
2. **Control:** Open Serial Monitor (115200), type position values, watch smooth motion.

## Real-time Feedback

```
Motor 11: position 1.570 rad (90.0°), speed: 0.15 rad/s
Motor 11: position 1.571 rad (90.1°), speed: 0.05 rad/s
Motor 11: position 1.571 rad (90.1°), speed: 0.00 rad/s ✓ arrived
```

## Examples

### Angle test
```bash
0       # home
90      # 90°
180     # 180°
270     # 270°
360     # back to 0°
45      # 45°
stop    # stop
```

### Single-motor ID config
```bash
config          # current config (default ID=11)
id 2            # set motor ID to 2
config          # confirm
restart         # apply
90              # test new ID
```

### If motor ID is not 11
```bash
config          # current ID
id 7            # set to 7
restart         # apply
90              # test
```

## Safety

- Position limited to about -12.5–12.5 rad
- Angle > 360° normalized
- ID range 1–127
- Config validated
- `stop` for emergency stop
- Real-time status shows current motor ID

## Use Cases

- Robot joint control
- Angle positioning
- Quick prototyping
- Teaching / demos
- Testing

## Troubleshooting

- **Motor not moving:** Check CAN, power, init messages, and motor ID (`config`). If motor ID ≠ 11, use `id X` then `restart`.
- **ID change not applied:** Run `restart` after `id X`; confirm ID 1–127 and check with `config`.
- **Config lost:** First run uses defaults; EEPROM saves after you set values.
- **Vibration:** Sketch is tuned for low vibration; check mechanical mounting.
- **Position off:** Use `zero` to redefine zero; check backlash and correct motor ID.

For more features, use the `joint_position_control` project.

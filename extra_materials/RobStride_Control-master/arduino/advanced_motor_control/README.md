# Advanced Motor Control

**Full control system based on the official Xiaomi motor function-code table**

## Features

- **Full function-code support:** All parameters from the official table
- **Real-time status:** Temperature, voltage, faults, position, velocity
- **Online PID tuning:** Adjust all PID parameters
- **Safety:** Auto monitoring and emergency stop
- **Persistent config:** Saved to EEPROM
- **Fault diagnosis:** Fault code analysis and handling

## Function-Code Mapping

### Config
```cpp
0x200a // CAN_ID - node ID (1-127)
0x200b // CAN_MASTER - host ID (0-127)
0x200c // CAN_TIMEOUT
0x200d // motorOverTemp
0x200f // GearRatio
```

### Control
```cpp
0x2012 // cur_kp
0x2013 // cur_ki
0x2014 // spd_kp
0x2015 // spd_ki
0x2016 // loc_kp
```

### Limits
```cpp
0x2007 // limit_torque (0-12 Nm)
0x2018 // limit_spd (0-200 rad/s)
0x2019 // limit_cur (0-23 A)
```

### Status
```cpp
0x3014 // rotation
0x3015 // modPos
0x3016 // mechPos ★
0x3017 // mechVel ★
0x3006 // motorTemp
0x300c // vBus
0x3022 // faultSta
0x3023 // warnSta
```

## Commands

### Basic
```bash
pos 1.57          # position 1.57 rad (90°)
spd 5.0           # speed 5.0 rad/s
stop              # emergency stop
clear             # clear fault
```

### Status
```bash
status            # full status
params            # all tunable parameters
info              # device info
```

### Tuning (inside `tune` mode)
```bash
tune              # enter tune mode
  kp 25.0         # position KP
  kd 5.0          # velocity KP
  ki 0.02         # velocity KI
  test            # step response test
  save            # save to EEPROM
  exit            # exit
```

### Safety
```bash
monitor           # continuous safety monitor
emergency         # emergency stop
```

## Safety

- **Temperature:** Auto stop above 85°C
- **Voltage:** Warn if outside 18–32 V
- **Position:** Warn if outside ±12 rad
- **Fault:** Stop on fault detection

## Quick Start

1. **Hardware:** ESP32 + CAN module, Xiaomi motor + 12–24 V, CAN wired.
2. **Software:** Upload `advanced_motor_control.ino`, Serial Monitor 115200.
3. **Test:** `status` → `pos 1.57` → `pos 0`.

## Troubleshooting

- **No response:** Check CAN, baud 1 Mbps, power 12–24 V, motor ID.
- **Bad PID:** Use `tune`, start small, use `test` after each change.
- **Overheat:** Reduce load/speed, improve cooling.
- **Set failed:** Check function code and parameter range.

## Tips

- Tune in order: position KP → velocity KP (kd) → velocity KI (ki).
- Parameters are saved and reloaded on restart.
- Run `status` and `monitor` before critical operations.

---

**Summary:** Full Xiaomi motor control via function codes for professional use.

# Dual Motor Control

**Control two Xiaomi motors with independent and synchronous modes**

## Features

- **Independent control:** Motor 1 and Motor 2 separately
- **Sync mode:** Both move to target positions together
- **0–360° input:** Angle or radian
- **Dynamic ID:** Change motor IDs at runtime
- **Per-motor PID:** Separate KP for each motor
- **Config saved:** Survives power loss
- **Real-time status:** Position and speed for both motors

## Commands

### Dual control (recommended)
```bash
90,180           # Motor 1 → 90°, Motor 2 → 180°
0,0              # both to 0°
90,360           # Motor 1 → 90°, Motor 2 → 360° (back to 0°)
45,135           # 45° and 135°
1.57,3.14        # rad: π/2 and π
```

### Single motor
```bash
1-90             # Motor 1 → 90°
2-180            # Motor 2 → 180°
1-0              # Motor 1 → 0°
2-270            # Motor 2 → 270°
```

### Legacy sync (compatible)
```bash
sync 90 180      # Motor 1 → 90°, Motor 2 → 180°
sync 0 0         # both to 0°
sync 45 135      # 45° and 135°
```

### Config
```bash
id1 3            # Motor 1 ID = 3
id2 4            # Motor 2 ID = 4
config           # show config
restart          # restart both motors
```

### Tuning
```bash
kp1 25           # Motor 1 position KP
kp2 30           # Motor 2 position KP
speed 3.0        # max speed (both)
```

### Status
```bash
status           # detailed status
stop             # stop both
zero             # set zero (both)
help             # help
```

## Default Config

```cpp
Motor 1 ID: 1
Motor 2 ID: 2
Master ID: 0
Max speed: 2.0 rad/s
Motor 1 position KP: 30.0
Motor 2 position KP: 30.0
Torque limit: 12.0 Nm
```

## Changing Motor IDs

```bash
id1 11           # if Motor 1 is actually ID 11
id2 12           # if Motor 2 is actually ID 12
restart          # apply
```

## Use Cases

- Two-joint robot: `sync 0 0`, `sync 90 45`, `sync 180 90`
- Pan-tilt: `1-90` (pan), `2-45` (tilt), `sync 180 90`
- Symmetric mechanism: `sync 30 -30`, `sync 0 0`
- Two wheels: `1-90`, `2-90`, `1-0`, `2-0`

## Safety

- Independent control; range limited to about ±12 rad; angle/radian auto-detected; `stop` stops both; one fault does not disable the other.

## Troubleshooting

- **No response:** Check CAN and power; confirm IDs with `config`; test each motor alone.
- **Out of sync:** Tune KP so both respond similarly; check load; use `status` for position difference.
- **Config lost:** First run uses defaults; re-set and it will save.
- **Vibration:** Lower KP and speed; check mounting.

---

**Summary:** Full dual-motor control with independent and sync modes for coordinated motion.

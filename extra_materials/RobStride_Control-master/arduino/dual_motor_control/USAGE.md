# Dual Motor Control – Quick Usage

## Quick Start

### 1. Upload
- Open `dual_motor_control.ino`
- Select ESP32 board
- Upload

### 2. Serial Monitor
- Baud: 115200
- You should see: `=== Dual motor control ===`

### 3. Basic test
```bash
1-90              # Motor 1 → 90°
2-180             # Motor 2 → 180°
sync 0 0          # both to 0°
```

## Main Commands

### Single motor
```bash
1-90              # Motor 1 → 90°
2-180             # Motor 2 → 180°
1-0               # Motor 1 → 0°
2-270             # Motor 2 → 270°
```

### Sync
```bash
sync 90 180       # Motor 1 → 90°, Motor 2 → 180°
sync 0 0          # both to 0°
sync 45 135       # 45° and 135°
```

### Config
```bash
config            # show config
id1 3             # Motor 1 ID = 3
id2 4             # Motor 2 ID = 4
restart           # restart both
```

### Tuning
```bash
kp1 25            # Motor 1 KP
kp2 30            # Motor 2 KP
speed 3.0         # max speed
```

### Status
```bash
status            # status
stop              # stop both
zero              # set zero
help              # help
```

## Recommended Flow

### First use
```bash
config            # check config
1-90              # test Motor 1
2-90              # test Motor 2
sync 45 135       # test sync
```

### If motor IDs are not 1 and 2
```bash
id1 11            # Motor 1 actual ID
id2 12            # Motor 2 actual ID
restart           # apply
1-90              # test Motor 1
2-90              # test Motor 2
```

### Two-joint example
```bash
sync 0 0          # home
sync 90 45        # joint 1 → 90°, joint 2 → 45°
sync 180 90       # joint 1 → 180°, joint 2 → 90°
sync 0 0          # home
```

## Notes

1. **Default IDs:** 1 and 2  
2. **ID range:** 1–127  
3. **Angle:** 0–360°  
4. **Radians:** Small values supported  
5. **Sync:** Both motors move together  

## Troubleshooting

- **No response:** Check CAN and power; use `config` to verify IDs; try each motor alone.
- **Wrong IDs:** Use `config`, then `id1 X`, `id2 Y`, then `restart`.

---

**Short:** `1-90` (Motor 1), `2-180` (Motor 2), `sync 90 45` (sync).

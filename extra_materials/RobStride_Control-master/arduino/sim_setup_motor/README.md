# Motor ID Setup Program

**Change Xiaomi motor IDs over CAN for multi-motor setups**

## Features

- **Scan:** Find all motors on the bus
- **Set ID:** Change motor ID over CAN
- **Batch:** Set a sequence of IDs quickly
- **Verify:** Confirm IDs after changes
- **Safety:** Avoid conflicts and mistakes

## Serial Commands

### Basic
```bash
help                   # help
scan                   # scan motors (IDs 1–127)
scan 1 20              # scan IDs 1–20 only
verify                 # verify found motors
```

### Set ID (only one motor on bus)
```bash
set 1 2                # change ID 1 → 2
set 5 10               # change ID 5 → 10
reset 3                # reset motor 3
```

### Batch
```bash
batch 5                # batch set starting from ID 5
```

## Usage

### Option 1: One motor at a time
```bash
# 1. Connect first motor
scan                   # see current ID
set 1 2                # set to ID 2
verify                 # confirm

# 2. Disconnect, connect next motor
scan                   # see its ID
set 1 3                # set to ID 3
verify                 # confirm

# 3. Repeat for all motors
```

### Option 2: Batch (recommended)
```bash
batch 5                # start batch from ID 5
# Prompts: connect motor 1, press key → set ID 5
#          connect motor 2, press key → set ID 6
#          ...
verify                 # verify all
```

### Option 3: Planned IDs
```bash
# Plan: Motor 1→ID 1, Motor 2→ID 2, Motor 3→ID 3
scan 1 10
set 7 1                # found ID 7 → set to 1
# disconnect, connect next
set 7 2                # set to 2
# ...
verify                 # final check
```

## Protocol (SET_MOTOR_CAN_ID)

- **Mode:** 7 (SET_MOTOR_CAN_ID)
- **Data:** First byte = new motor ID
- **Arbitration ID:** (7<<24) | (MASTER_ID<<8) | OLD_MOTOR_ID
- **Check:** Ping motor with new ID after change

## Use Cases

### Multi-joint robot (e.g. 6-DOF arm)
```bash
batch 6                # IDs 1–6
# ID 1: base, 2: shoulder, 3: elbow, 4: wrist, 5: wrist2, 6: gripper
```

### Multi-wheel
```bash
set 1 1                # front left (keep 1)
set 1 2                # front right
set 1 3                # rear left
set 1 4                # rear right
```

### Pan-tilt
```bash
set 1 10               # pan
set 1 11               # tilt
```

## Important

### Safety
1. **Only one motor on bus when changing ID** to avoid multiple responses.
2. Write down which physical motor has which ID.
3. Keep power stable during ID change.

### ID planning
- Use consecutive IDs (1, 2, 3…) when possible.
- Group by function (joints, wheels, etc.).
- Leave some IDs for future use.

### Troubleshooting
```bash
# Set failed?
reset <id>             # reset motor
set <old> <new>         # retry
verify                 # confirm

# No motor in scan?
- Check CAN wiring and power
- Baud 1 Mbps, 120 Ω termination

# ID conflict?
scan 1 127             # find all
- Reassign unique IDs
```

## CAN Frame

- **Arbitration ID:** (7<<24) | (0<<8) | old_motor_id
- **Data:** [new_motor_id, 0, 0, 0, 0, 0, 0, 0]
- **Motor ID:** 1–127
- **Baud:** 1 Mbps, extended frame, 8 bytes

## Tips

- Use an ID plan (e.g. 1–10 joints, 11–20 wheels).
- Run `verify` after changes.
- If one motor fails, use `reset <id>`; if IDs conflict, `scan 1 127` and reassign.

---

**Summary:** Use this program to assign and verify motor IDs over CAN for multi-motor systems. Batch mode is fastest for many motors.

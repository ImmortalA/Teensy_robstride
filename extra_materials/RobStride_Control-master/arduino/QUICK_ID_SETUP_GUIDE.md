# Quick Motor ID Setup Guide

## 5-minute setup for multiple motor IDs

### Preparation
1. **Hardware**
   - ESP32 + CAN module
   - Xiaomi motor + 12–24 V supply
   - CAN bus connected

2. **Software**
   - Upload `sim_setup_motor.ino` to ESP32
   - Open Serial Monitor (115200)

3. **First check**
   ```bash
   test              # Test CAN communication
   ```

---

## Method 1: Batch setup (recommended)

### Step 1: Start batch
```bash
batch 1
```
Prompt: "Starting batch setup from ID=1"

### Step 2: Connect motors one by one
```
=== Motor #1 ===
Connect motor to bus, then press any key to continue...
Or type 'stop' to end batch
```
- Connect motor 1 → press key → ID set to 1
- Connect motor 2 → press key → ID set to 2
- Connect motor 3 → press key → ID set to 3
- Repeat until done

### Step 3: Verify
```bash
verify
```
Example:
```
Found 3 motors
Motor IDs: 1, 2, 3
✅ Enough motors for multi-motor control
```

---

## Method 2: Single motor setup

### Single motor
```bash
# 1. Connect one motor
test              # Test link
scan              # See current ID, e.g. "Found motor ID: 7"

# 2. Change ID
set 7 1           # Change ID 7 → 1

# 3. Verify
verify
```

### Batch change
```bash
# 6-joint arm
test
set 1 1           # Joint 1 → ID 1 (then swap to motor 2)
set 1 2           # Joint 2 → ID 2
set 1 3           # Joint 3 → ID 3
set 1 4           # Joint 4 → ID 4
set 1 5           # Joint 5 → ID 5
set 1 6           # Joint 6 → ID 6
verify
```

---

## Example setups

### 3-joint arm
```bash
test
batch 3           # Batch from ID 1
# Connect 3 motors in turn
verify            # IDs: 1, 2, 3
```

### 4-wheel drive
```bash
test
set 1 10          # Front left
set 1 11          # Front right
set 1 12          # Rear left
set 1 13          # Rear right
verify
```

### Pan-tilt (2 axes)
```bash
test
set 1 20          # Pan
set 1 21          # Tilt
scan 20 25        # Check IDs 20–25
```

---

## Troubleshooting

### test fails
```
❌ CAN communication failed!
```
- Check CAN_H/CAN_L
- Check motor power (12–24 V)
- Check baud rate (1 Mbps)
- Check 120 Ω termination

### scan finds no motor
```
No motors found
```
- Run `test` first
- Connect only one motor
- Power-cycle motor

### set fails
```
❌ Set failed, check connection or retry
```
- Only one motor on bus
- Run `test` again
- Try `reset 1` then `set` again

### verify shows wrong count
```
Fewer motors than expected
```
- Possible ID conflict; re-assign IDs
- Test each motor
- Check power for all motors

---

## Important

### ❌ Do not
```bash
# Wrong: change ID with multiple motors on bus
# Several motors respond and IDs get mixed
```

### ✅ Do
```bash
# Right: one motor at a time
# Set ID, disconnect, connect next, repeat
# Then connect all and verify
```

### If scan finds nothing
```bash
# Checklist:
1. CAN wiring correct
2. Motor power on (12–24 V)
3. 120 Ω termination
4. 1 Mbps baud
```

### If set fails
```bash
reset 1               # Reset motor
set 1 2               # Retry
verify
```

### If several motors respond
```bash
# Fix:
1. Disconnect all motors
2. Connect only target motor
3. Set ID again
4. Verify before connecting next
```

---

## Tips

### 1. Fast scan
```bash
scan 1 5              # Scan IDs 1–5 only
```

### 2. ID plan
```bash
# Suggested ranges:
1–10   : Joints
11–20  : Wheels
21–30  : Actuators
31–50  : Spare
```

### 3. Safe workflow
```bash
set 1 5
verify
# Confirm before next step
```

### 4. Keep a table
```
Motor ID list:
ID=1 : Base joint
ID=2 : Shoulder
ID=3 : Elbow
ID=4 : Wrist
```

---

## Success

You’re done when you see:
```
✓ Motor ID set to: X
Found N motors
Motor IDs: 1, 2, 3, ...
✅ Enough motors for multi-motor control
```

---

**Related files:**
- `sim_setup_motor.ino` – ID setup program
- `README.md` – Full usage
- `simple_joint_control.ino` – Use after IDs are set

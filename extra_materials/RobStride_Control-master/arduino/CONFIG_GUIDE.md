# Motor ID Configuration Guide

## Overview

`simple_joint_control` supports **dynamic motor ID configuration**: you can control motors with different IDs without recompiling.

## Quick Use

### 1. View Current Config
```bash
config
```
**Example output:**
```
=== Current configuration ===
Motor ID: 1
Master ID: 0
Max speed: 2.0 rad/s
Position KP: 20.0
==================
```

### 2. Change Motor ID
```bash
id 2
```
**Example output:**
```
Motor ID set to: 2
Enter 'restart' to restart motor and apply new ID
```

### 3. Restart to Apply New ID
```bash
restart
```

### 4. Test New ID
```bash
90      # Test if motor 2 responds
```

## Full Configuration Flow

### Scenario 1: First use with motor ID=2
```bash
config          # Confirm current ID=1
id 2            # Set to ID=2
config          # Confirm ID changed to 2
restart         # Apply new config
90              # Motor 2 should go to 90°
0               # Motor 2 should return to 0°
```

### Scenario 2: Switch between motors
```bash
# Currently controlling motor 3, switch to motor 5
config          # Current ID=3
id 5            # Set to ID=5
restart         # Restart motor
180             # Test motor 5
```

### Scenario 3: Restore defaults
```bash
# Back to default motor ID=1
id 1            # Set to ID=1
restart         # Restart
```

## Configuration Storage

### Auto-save
- Changes are **saved automatically** to ESP32 EEPROM
- **Survives power loss**; loaded on next boot
- First run uses default config (ID=1)

### Stored structure
```cpp
struct MotorConfig {
  uint8_t motor_id;        // Motor ID (1-127)
  uint8_t master_id;       // Master ID (fixed 0)
  float max_speed;         // Max speed (2.0 rad/s)
  float position_kp;       // Position KP (20.0)
  bool initialized;        // Init flag
};
```

## Technical Details

### ID range
- **Motor ID**: 1–127 (standard range)
- **Master ID**: 0 (fixed by protocol)
- **Validation**: Out-of-range values are rejected

### Validation examples
```bash
id 0            # ❌ Error: motor ID must be 1-127
id 128          # ❌ Error: motor ID must be 1-127
id 5            # ✅ OK: within range
```

### Live feedback
- Status shows the motor ID being controlled
```bash
Motor 2: position 1.571 rad (90.0°), speed: 0.15 rad/s
         ↑
       Current motor ID
```

## Example Uses

### 1. Multi-joint robot
```bash
# Joint 1 (motor ID=1)
id 1
restart
0
90

# Joint 2 (motor ID=2)
id 2
restart
0
45

# Joint 3 (motor ID=3)
id 3
restart
0
180
```

### 2. Multi-device test
```bash
id 1 && restart && 90
id 2 && restart && 90
id 3 && restart && 90
```

### 3. Maintenance
```bash
config          # View config
id 10           # Switch to test motor
restart         # Apply
90              # Test
```

## Safety

### Protection
- **Validation**: Config checked for validity
- **Backup**: Default config on first run
- **Recovery**: Invalid config falls back to defaults

### Usage
- **Feedback**: Clear success/failure for each action
- **Confirm**: ID change requires restart to take effect
- **Status**: Current motor ID shown in real time

## Troubleshooting

### Config change has no effect
```bash
# Symptom: after "id 2" motor still responds as ID=1
# Fix: must enter "restart"
id 2
restart
```

### Config lost after reboot
```bash
# Symptom: config back to default after reboot
# Fix: set again; EEPROM will save
id 2
restart
```

### Invalid ID accepted
```bash
# Symptom: id 0 or id 128 accepted
# Fix: program rejects and shows error
❌ Motor ID must be 1-127
```

### EEPROM error
```bash
# Symptom: config error on first run
# Fix: program uses default and saves
First run or invalid config, using default
```

## Best Practices

### 1. Check config regularly
```bash
config          # Before important operations
```

### 2. Test after ID change
```bash
id X
restart
90      # Test that new ID responds
```

### 3. Keep an ID map
```
Motor ID assignment:
ID=1 : Left arm joint
ID=2 : Right arm joint
ID=3 : Head joint
...
```

### 4. Backup important config
```bash
# Note current config
Motor ID: 1
Max speed: 2.0
Position KP: 20.0
```

---

**Summary:** You can control different motor IDs without recompiling. Config is saved automatically.

**File:** `/mi_arduino/simple_joint_control/simple_joint_control.ino`

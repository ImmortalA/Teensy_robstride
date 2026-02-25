# 0–360° Angle Control Quick Guide

## Getting Started with Angle Control

### Angle Input System

The `simple_joint_control` project supports **0–360° angle input** for more intuitive control.

### Angle Reference

```
Angle    Position description
0°     = Start / straight ahead
45°    = Northeast
90°    = Right
135°   = Southeast
180°   = Behind
225°   = Southwest
270°   = Left
315°   = Northwest
360°   = Back to start (0°)
```

## Quick Command Tests

### Basic Angle Test
```bash
# In Serial Monitor, enter:
0        # Back to start
90       # 90° (right)
180      # 180° (behind)
270      # 270° (left)
0        # Back to 0°
stop     # Stop
```

### Precise Angle Test
```bash
22.5     # 22.5°
67.5     # 67.5°
112.5    # 112.5°
157.5    # 157.5°
0        # Back to start
```

### Large Angle Test
```bash
450      # 450° → normalized to 90°
720      # 720° → normalized to 0°
-90      # -90° → treated as radians
```

## Input Recognition Rules

The program interprets input as follows:

| Input range | Interpretation | Example        |
|-------------|----------------|----------------|
| **0–360**   | Angle ✅       | `90` → 90°    |
| **Small rad values** | Radians | `1.57` → π/2 rad |
| **>360°**   | Normalized angle | `450` → 90° |
| **Negative**| Radians       | `-1.57` → -π/2 rad |

## Real-Time Feedback Format

The program shows both angle formats:
```
Position: 1.571 rad (90.0° → 0–360° display: 90.0°), speed: 0.15 rad/s
           │      │                 └─ 0–360° angle (intuitive)
           │      └─ Computed angle
           └─ Radians (internal)
```

## Usage Steps

### 1. Upload Program
```bash
1. Arduino IDE: open simple_joint_control.ino
2. Board: ESP32 Dev Module
3. Upload
```

### 2. Start Controlling
```bash
1. Open Serial Monitor (115200)
2. Wait for "Ready! 0–360° angle input supported"
3. Enter angle values to control
```

### 3. Angle Control Example
```bash
# Four cardinal directions:
0      # Ahead
90     # Right
180    # Behind
270    # Left
0      # Back ahead
```

## Advanced Usage

### 1. Fine Positioning
```bash
# Use decimals for precision:
33.75   # 33.75°
146.25  # 146.25°
292.5   # 292.5°
```

### 2. Continuous Motion
```bash
# Sequence: 0 → 45 → 90 → 135 → 180 → 225 → 270 → 315 → 0
# Enter one angle every 2 seconds
```

### 3. Range Test
```bash
# Test full range:
0 → 360    # Full turn
180 → 0    # Half turn back
90 → 270   # Half turn
```

## Safety

### Automatic Protection
- Angles out of range are limited
- Values >360° are normalized
- Enter "stop" anytime for emergency stop

### Recommendations
- Start with small angles
- Check that motion is smooth
- Ensure mechanical mounting is secure

## Example Applications

### 1. Robot Joint
```bash
# Joint motion:
0      # Initial
45     # Lift 45°
90     # Lift 90°
45     # Lower 45°
0      # Back to initial
```

### 2. Camera Pan-Tilt
```bash
# Pan-tilt directions:
0      # Straight ahead
45     # 45° right
90     # Right
315    # 45° left
0      # Back ahead
```

### 3. Pointer / Indicator
```bash
# Dial pointer:
0      # 0°
90     # 3 o’clock
180    # 6 o’clock
270    # 9 o’clock
0      # 12 o’clock
```

## Troubleshooting

### Inaccurate angle
```bash
1. Re-zero: enter "zero"
2. Check mechanical zero
3. Test 0° and 180° for accuracy
```

### Motor vibration
```bash
This program is already tuned for low vibration.
If it still vibrates:
1. Check mechanical mounting
2. Check power supply
3. Enter "stop" to stop test
```

### Wrong angle/radian interpretation
```bash
If radians are taken as angle:
1. Use decimal radians: 1.57, 3.14
2. Or explicit: 1.5708 for π/2
3. Check Serial output for interpretation
```

---

**Summary:** You can control the motor directly with 0–360° angles. Prefer angle input when possible.

**File:** `/mi_arduino/simple_joint_control/simple_joint_control.ino`

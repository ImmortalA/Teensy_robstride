# Motor Vibration Solutions Guide

## Vibration Cause Analysis

### Common Causes

1. **Control parameters too high**
   - KP (proportional gain) too high → oscillation
   - KD (derivative gain) inappropriate → sluggish or oscillatory response

2. **Speed changes too fast**
   - Max speed set too high
   - No acceleration limit

3. **Control mode switching not smooth**
   - Direct mode switching
   - Parameter re-set timing wrong

4. **Mechanical factors**
   - Load mismatch
   - Loose mounting

## Solutions

### Solution 1: Use Simplified Joint Control (Recommended)

**File: `simple_joint_control.ino`**

Features:
- Very simple, position control only
- Optimized control parameters to reduce vibration
- Supports radian and angle input
- Automatic range limiting

**Usage:**
```cpp
// Direct position input
1.57          # 1.57 rad (90°)
90            # 90°
-45           # -45°
stop          # Stop
zero          # Set zero
```

### Solution 2: Use Advanced Joint Control

**File: `joint_position_control.ino`**

Features:
- Full position control system
- Detailed parameter configuration
- Real-time status monitoring
- Error detection and handling

## Key Parameter Tuning

### Vibration-Reduction Settings

```cpp
// 1. Limit max speed (important!)
max_speed = 2.0;        // Reduce from 30 to 2.0 rad/s

// 2. Lower position control gain
position_kp = 20.0;     // Reduce from 30 to 20

// 3. Set suitable velocity control parameters
speed_kp = 10.0;        // Velocity proportional gain
speed_ki = 0.1;         // Velocity integral gain

// 4. Limit current
current_limit = 5.0;    // Limit max current
```

### Parameter Effects on Vibration

| Parameter   | Too high    | Too low     | Recommended (joint) |
|-------------|-------------|-------------|----------------------|
| Max speed   | High vib.   | Slow        | 2.0 rad/s           |
| Position KP | Oscillation | Poor accuracy | 15-25             |
| Speed KP    | Overshoot   | Slow        | 5-15                |
| Speed KI    | Oscillation | Steady error| 0.05-0.2            |

## Control Mode Comparison

| Mode        | Use           | Vibration   | Application      |
|-------------|----------------|-------------|------------------|
| **Position** ⭐ | Precise positioning | Minimum  | Joint control    |
| **Velocity**   | Continuous rotation | Medium   | Wheel control    |
| **Operation**  | Complex control     | Higher   | Advanced apps    |
| **Current**    | Force control       | High     | Force control    |

**Best practice for joints: Position mode + tuned parameters**

## Quick Test Steps

### 1. Test with Simplified Program
```cpp
// Upload simple_joint_control.ino
// Serial input:
1.0    # Should move smoothly to 1.0 rad
-1.0   # Smooth return
stop   # Stop immediately
```

### 2. Observe Vibration
- Smooth, no vibration → parameters OK
- Slight vibration → reduce speed
- Obvious vibration → reduce KP

### 3. Further Tuning
```cpp
// If still vibrating, reduce step by step:
max_speed = 1.0;      // Down to 1.0 rad/s
position_kp = 15.0;   // Down to 15
```

## Vibration Diagnosis Flow

### Step 1: Check Basic Setup
```cpp
1. Confirm position mode
2. Check max speed
3. Verify mechanical mounting
```

### Step 2: Parameter Tuning
```cpp
// Start from conservative values
max_speed = 1.0;
position_kp = 10.0;

// Increase gradually to find best point
```

### Step 3: Real-Time Monitoring
```cpp
// Watch:
- Overshoot < 5%
- Settling time < 2 s
- No sustained oscillation
```

## Code Tips

### 1. Set Parameters in Steps
```cpp
// Wrong: set all at once
motor.Set_SingleParameter(LOC_REF, position);

// Right: set in steps
motor.Set_SingleParameter(LIMIT_SPD, max_speed);  // Limit speed first
delay(20);
motor.Set_SingleParameter(LOC_REF, position);      // Then set position
```

### 2. Add Delay Between Commands
```cpp
motor.Set_SingleParameter(LOC_KP, 20.0);
delay(50);  // Important: give motor time to process
```

### 3. Gradual Position Changes
```cpp
// Step large moves
float start_pos = current_position;
float target_pos = desired_position;
float steps = 10;

for (int i = 1; i <= steps; i++) {
    float intermediate = start_pos + (target_pos - start_pos) * i / steps;
    motor.Set_SingleParameter(LOC_REF, intermediate);
    delay(100);
}
```

## Test Cases

### Basic Position Test
```cpp
// Sequence: 1.57 → 0 → -1.57 → 0
// Expect: smooth arrival at each position, no oscillation
```

### Small-Step Test
```cpp
// 0.1 rad steps: 0.0 → 0.1 → 0.2 → ... → 1.0
// Expect: each step smooth, no jump
```

### Response Time Test
```cpp
// Large move: 0.0 → 3.14
// Expect: smooth arrival in 2-3 s, overshoot < 5%
```

## Troubleshooting

### Still strong vibration?
1. Reduce speed further: `max_speed = 0.5`
2. Reduce gains: `position_kp = 10`
3. Check mechanical mounting
4. Check power: stable voltage and current

### Response too slow?
1. Increase KP: `position_kp = 25`
2. Increase max speed: `max_speed = 3.0`
3. Reduce delays between commands

### Not accurate enough?
1. Increase KP (watch for vibration)
2. Tighten position tolerance
3. Check encoder feedback

---

**Summary:** Start with `simple_joint_control.ino`, which is already tuned for vibration. If issues remain, reduce max speed and KP step by step.

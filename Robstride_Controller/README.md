# RobStride Controller (Teensy)

Open **`Robstride_Controller.ino`** in the Arduino IDE (Teensyduino + **FlexCAN_T4**).

- Current configuration: one RobStride RS06 motor, CAN id `1`, CAN bitrate `1 Mbps`.
- Serial Monitor: **115200** baud; type **`help`** after upload.
- Motion uses RobStride private **Type-1 operation-control** frames. The old native RAM-mode `pos`, `vel`, and `cur` commands are not used.

## Quick Start

```text
reset
enable 1
stream 1 100
nudge 1 0.2 10 1
```

## Commands

Safe motion commands:

```text
move 1 <position_rad> [kp] [kd]
nudge 1 <delta_rad> [kp] [kd]
jog 1 <rad/s> [kp] [kd]
jog 1 0
torque 1 <Nm>
motion 1 <p> <v> <kp> <kd> <tq>
```

Diagnostics and recovery:

```text
status 1
fault 1
diag
stream 0
stop 1
estop
reset
enable 1
```

After `estop`, run `reset` and then `enable 1` again.

Parent repo **`README.md`** has the overview.

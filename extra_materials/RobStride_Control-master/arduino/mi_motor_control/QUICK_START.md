# Quick Start

## 5-minute setup

### 1. Hardware
```
ESP32      CAN module    Motor
GPIO5  ←→ TX       ←→ CAN_H
GPIO4  ←→ RX       ←→ CAN_L
3.3V   ←→ VCC      ←→ 12-24V
GND    ←→ GND      ←→ GND
```

### 2. Arduino IDE
1. Install ESP32 board support
2. Board: **ESP32 Dev Module** (or ESP32S3 as in project)
3. Library: **ESP32-TWAI-CAN**

### 3. Upload
1. Open `mi_motor_control.ino`
2. Select port
3. Upload

### 4. Run
1. Serial Monitor **115200**
2. Watch init messages
3. Motor will start automatic speed test

## Serial commands

```
stop          # stop
enable        # enable
speed 2.0     # 2 rad/s
pos 1.57      # 90°
help          # help
```

## Expected output

```
=== Xiaomi motor control ESP32 ===
Initializing...
1. CAN init...
2. Motor (ID=1) init...
3. Set mechanical zero...
4. Set speed mode...
5. Enable motor...
=== Init done ===
Motor enabled, running...

M1: 0,1,0,0,0,0,0,0,2, angle:0.00, speed:0.00, torque:0.00, temp:25.0
```

## If something’s wrong

1. **Upload fails:** Check ESP32 driver and port
2. **Motor no response:** Check CAN and power
3. **Compile error:** Confirm ESP32 board and library installed

See `README.md` for full details.

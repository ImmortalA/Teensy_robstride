# RobStride Arduino Control

Arduino-based RobStride motor control library for ESP32 and Arduino microcontrollers.

## Supported Boards

- **ESP32**: Recommended; built-in TWAI/CAN controller
- **Arduino Mega 2560**: Requires external CAN module (MCP2515)
- **Arduino Uno**: Requires external CAN module (MCP2515)

## Project Structure

```
arduino/
├── README.md                    # Arduino documentation
├── simple_joint_control/        # Simple joint control
│   ├── simple_joint_control.ino # Main program
│   ├── TWAI_CAN_MI_Motor.h      # CAN driver header
│   └── TWAI_CAN_MI_Motor.cpp    # CAN driver implementation
├── joint_position_control/      # Position control
├── dual_motor_control/          # Dual motor control
├── advanced_motor_control/      # Advanced control
├── mi_motor_control/            # Basic control
└── sim_setup_motor/             # Motor setup
```

## Hardware Connection

### ESP32 Direct Connection
```
ESP32           CAN Transceiver      RobStride Motor
-----           ---------------       ---------------
GPIO5    <----> TX       <----> CAN_H
GPIO4    <----> RX       <----> CAN_L
3.3V     -----> VCC
GND      -----> GND
```

### Arduino + MCP2515 Module
```
Arduino        MCP2515            CAN Transceiver      RobStride Motor
--------       --------           ---------------      ---------------
D10(SPI_SS)   CS
D11(SPI_MOSI) SI/MOSI
D12(SPI_MISO) SO/MISO
D13(SPI_SCK)  SCK/SCLK
5V            VCC
GND           GND
MCP2515 TX  -> CAN_H -> Motor
MCP2515 RX  -> CAN_L -> Motor
```

## Quick Start

### 1. Environment Setup

#### Arduino IDE
1. Install Arduino IDE 1.8.19+
2. Install ESP32 board support
3. Tools → Board → ESP32 Arduino → ESP32 Dev Module

#### PlatformIO
```ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino
```

### 2. Build and Upload

```bash
# Using Arduino IDE
1. Open the corresponding .ino file
2. Select the correct board
3. Upload the program

# Using PlatformIO
pio run --target upload
```

## Control Examples

### Basic Position Control

```cpp
#include "TWAI_CAN_MI_Motor.h"

TWAI_CAN_MI_Motor motor(11);  // Motor ID=11

void setup() {
    Serial.begin(115200);
    motor.init(CAN_SPEED_1000KBPS);
    motor.enable_motor();
}

void loop() {
    // Set position to 90 degrees
    motor.send_mit_command(PI/2, 30.0, 0.5);
    delay(2000);

    // Return to origin
    motor.send_mit_command(0, 30.0, 0.5);
    delay(2000);
}
```

### Parameter Tuning

```cpp
// Position gain (0-500.0)
motor.set_kp(30.0);

// Damping gain (0-100.0)
motor.set_kd(0.5);

// Velocity limit (rad/s)
motor.set_velocity_limit(20.0);

// Torque limit (Nm)
motor.set_torque_limit(30.0);
```

## Control Modes

### MIT Mode (Mode 0)
Direct control of position, velocity, and torque.

```cpp
// MIT control command
void send_mit_command(float position, float kp, float kd);
```

### Position Mode (Mode 1)
Internal PID position control.

```cpp
// Set position target
void set_position_target(float position);

// Set PID parameters
void set_position_kp(float kp);
void set_position_kd(float kd);
```

### Velocity Mode (Mode 2)
Velocity closed-loop control.

```cpp
// Set velocity target
void set_velocity_target(float velocity);

// Set velocity PID
void set_velocity_kp(float kp);
void set_velocity_ki(float ki);
```

## Communication Protocol

### CAN Frame Format
- **Baud rate**: 1000 kbps
- **Data length**: 8 bytes
- **Extended ID**: 29-bit standard CAN extended frame

### Command Types
- **0x00**: Get device ID
- **0x01**: Operation control (MIT mode)
- **0x02**: Operation status feedback
- **0x03**: Enable motor
- **0x04**: Disable motor
- **0x18**: Write parameter

## Project Descriptions

### simple_joint_control
Simplest joint control example; basic MIT control.
- Single motor control
- Interactive parameter tuning
- Real-time status feedback

### joint_position_control
Dedicated position control example.
- Smooth trajectory planning
- Multi-segment motion
- Position error monitoring

### dual_motor_control
Dual-motor coordinated control.
- Synchronized motion
- Master-slave control
- Coordinated trajectories

### advanced_motor_control
Advanced control features.
- Adaptive control
- Vibration suppression
- Temperature monitoring

## Troubleshooting

### Communication Issues
1. **CAN bus errors**
   - Check baud rate (1000 kbps)
   - Verify termination resistor (120Ω)
   - Confirm CAN_H/CAN_L wiring

2. **Motor not responding**
   - Check motor ID setting
   - Verify power supply (12-48V)
   - Confirm enable command was sent successfully

### Build Issues
1. **ESP32 build errors**
   - Update ESP32 board package
   - Check Arduino IDE version
   - Clear build cache

2. **Library dependency issues**
   - Ensure all header files are included
   - Check TWAI library version
   - Verify SPI connection (MCP2515)

## Performance

| Board         | Control Rate | Latency  | Motors | Notes                    |
|---------------|--------------|----------|--------|--------------------------|
| ESP32         | 100-200 Hz   | 2-5 ms   | 1-8    | Built-in CAN, high perf  |
| Arduino Mega  | 50-100 Hz    | 5-10 ms  | 1-4    | Rich I/O, stable         |
| Arduino Uno   | 30-50 Hz     | 10-20 ms | 1-2    | Low cost, entry level     |

## License

MIT License — see [../LICENSE](../LICENSE)

## Support

- Source: https://github.com/tianrking/robstride-control
- Issues: https://github.com/tianrking/robstride-control/issues

#ifndef ROBSTRIDE_O2_MOTOR_H
#define ROBSTRIDE_O2_MOTOR_H

#include <cstdint>
#include "utils.h"

// Robostride O2 command types in the Private protocol (extended 29-bit CAN ID).
// Manual Section 4: Type in bits 28–24, data in 23–8, motor ID in 7–0.
enum class O2CommandType : uint8_t
{
    GetDeviceId      = 0,   // Get device ID
    OperationControl = 1,   // Motor control (Type 1)
    Feedback         = 2,   // Motor feedback (RX only)
    EnableMotor      = 3,   // Enter motor mode
    StopMotor        = 4,   // Exit motor mode
    SetZero          = 6,   // Set mechanical zero
    SetMotorCanId    = 7,   // Set motor CAN ID
    ParamRead        = 17,  // Parameter read
    ParamWrite       = 18,  // Parameter write
    Fault            = 21,  // Fault (vendor-specific)
    Save             = 22,  // Save parameters
    Baud             = 23,  // Baud rate
    Report           = 24,  // Report
    Protocol         = 25,  // Protocol
    Version          = 26   // Version
};

// Common parameter IDs (from RobStride Arduino / manual 4.1.11). Use with Type 17/18.
namespace O2ParamId
{
    constexpr uint16_t RUN_MODE   = 0x7005;  // uint8: 0=operation, 1=position, 2=velocity, 3=current
    constexpr uint16_t IQ_REF     = 0x7006;  // float: current mode Iq, A
    constexpr uint16_t SPD_REF    = 0x700A;  // float: velocity mode setpoint, rad/s
    constexpr uint16_t LOC_REF    = 0x7016;  // float: position mode setpoint, rad
    constexpr uint16_t LIMIT_SPD  = 0x7017;  // float: position mode speed limit, rad/s
}

// Motor control modes (manual Section 3.4 / 4.3).
// OperationControl uses Type 1. Velocity/Current/PP/CSP require parameter write (Type 18)
// to switch mode; register index not in our reference — use manufacturer docs if needed.
enum class O2ControlMode : uint8_t
{
    OperationControl = 0,  // Type 1: position, velocity, Kp, Kd (and torque in MIT)
    CurrentMode     = 1,   // Iq command — switch via Type 18
    VelocityMode    = 2,   // Target speed — switch via Type 18
    PositionPP      = 3,   // Profile position — switch via Type 18
    PositionCSP     = 4,   // Cyclic sync position — switch via Type 18
    Stop            = 5    // Type 4 / 0xFD
};

struct O2Feedback
{
    float position = 0.0f;  // unwrapped radians
    float velocity = 0.0f;  // rad/s
    float torque   = 0.0f;  // Nm
    float temp     = 0.0f;  // °C
};

// Small helper around the Robostride O2 Private protocol.
// This class only builds/parses CAN payloads and IDs; it does not perform any I/O.
class RobstrideO2Motor
{
public:
    explicit RobstrideO2Motor(uint8_t can_id);

    uint8_t canId() const { return can_id_; }

    // Build the 29-bit extended CAN ID used by the Teensy firmware:
    // bits 28–24: type, bits 7–0: motor CAN ID, middle bits are zero.
    uint32_t makeId(O2CommandType type) const;

    // MIT protocol: 11-bit CAN ID. Bits 10–8 = mode type, bits 7–0 = motor ID.
    // Use standard (non-extended) frames when the motor is in MIT protocol.
    uint16_t makeMitId(uint8_t mode_type) const;

    // Magic-byte commands (payload: 0xFF×7 + magic).
    void buildEnableFrame(uint8_t out[8]) const;
    void buildStopFrame(uint8_t out[8]) const;
    void buildZeroFrame(uint8_t out[8]) const;

    // Type 0: Get device ID. Payload typically zeros; motor responds with ID.
    void buildGetDeviceIdFrame(uint8_t out[8]) const;

    // Type 7: Set motor CAN ID. out[0] = new_id (0–255), out[1..7] = 0.
    void buildSetCanIdFrame(uint8_t new_can_id, uint8_t out[8]) const;

    // Type 17: Parameter read. Byte0–1 = param_id (little-endian), byte2–7 = 0.
    void buildParamReadFrame(uint16_t param_id, uint8_t out[8]) const;

    // Type 18: Parameter write. Byte0–1 = param_id, byte2–3 = 0, byte4–7 = value (float LE).
    void buildParamWriteFrame(uint16_t param_id, float value, uint8_t out[8]) const;

    // Type 18: Parameter write (single byte, e.g. RUN_MODE). Byte0–1 = param_id, byte2–3 = 0, byte4 = value.
    void buildParamWriteU8(uint16_t param_id, uint8_t value, uint8_t out[8]) const;

    // Types 21–26: fault, save, baud, report, protocol, version. Payload vendor-specific; here zeros.
    void buildGenericTypeFrame(O2CommandType type, uint8_t out[8]) const;

    // Type 1 "operation control" command in Private protocol layout:
    // four 16-bit values (position, velocity, Kp, Kd), high byte first.
    // Values are clamped using the provided ActuatorParams.
    void buildOperationFrame(float pos_rad,
                             float vel_rad_s,
                             float kp,
                             float kd,
                             const ActuatorParams &limits,
                             uint8_t out[8]) const;

    // MIT protocol Command 3: 16b position + 12b velocity + 12b Kp + 12b Kd + 12b torque.
    // Use with makeMitId(3) and standard 11-bit CAN frames when motor is in MIT protocol.
    void buildMitOperationFrame(float pos_rad,
                                float vel_rad_s,
                                float kp,
                                float kd,
                                float torque_ff,
                                const ActuatorParams &limits,
                                uint8_t out[8]) const;

    // Type 2 feedback in the manual's 16-bit layout.
    // prev_unwrapped_pos is the previous unwrapped position, used for wrap handling.
    O2Feedback decodeFeedback(const uint8_t in[8],
                              const ActuatorParams &limits,
                              float prev_unwrapped_pos) const;

private:
    uint8_t can_id_;
};

#endif // ROBSTRIDE_O2_MOTOR_H


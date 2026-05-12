#ifndef SPINE_UTILS_H
#define SPINE_UTILS_H
#include <cmath>
#include <vector>
#include <cstdio>
#include <cstdint>

// RobStride O2 parameter IDs (used with SpineBoard::sendParameterWriteU8). See vendor docs / Robstride_Controller.
namespace O2ParamId
{
constexpr uint16_t RUN_MODE = 0x7005;
}

enum class O2ControlMode : uint8_t
{
    OperationControl = 0,
};

// Robostride O2 (extended CAN protocol: id|data|mode in 29-bit ID)
#define P_MIN_ROBOSTRIDE_O2 -12.5f
#define P_MAX_ROBOSTRIDE_O2 12.5f
#define V_MIN_ROBOSTRIDE_O2 -45.0f
#define V_MAX_ROBOSTRIDE_O2 45.0f
#define KP_MIN_ROBOSTRIDE_O2 0.0f
#define KP_MAX_ROBOSTRIDE_O2 500.0f
#define KD_MIN_ROBOSTRIDE_O2 0.0f
#define KD_MAX_ROBOSTRIDE_O2 5.0f
#define T_MIN_ROBOSTRIDE_O2 -15.0f
#define T_MAX_ROBOSTRIDE_O2 15.0f

const float WRAP_RANGE = 25.0f; // -12.5 to 12.5, total range is 25
const float WRAP_MIN = -12.5f;
const float WRAP_MAX = 12.5f;

// CRC-8 polynomial (Dallas/Maxim)
const uint8_t CRC8_POLYNOMIAL = 0x31;

enum class ActuatorType
{
    ROBOSTRIDE_O2,
};

// Struct for actuator parameters
struct ActuatorParams
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
    bool recalibrate = false;  // If true, zero encoder on init
};

ActuatorParams getActuatorParams(ActuatorType type)
{
    switch (type)
    {
    case ActuatorType::ROBOSTRIDE_O2:
        return {
            P_MIN_ROBOSTRIDE_O2, P_MAX_ROBOSTRIDE_O2,
            V_MIN_ROBOSTRIDE_O2, V_MAX_ROBOSTRIDE_O2,
            KP_MIN_ROBOSTRIDE_O2, KP_MAX_ROBOSTRIDE_O2,
            KD_MIN_ROBOSTRIDE_O2, KD_MAX_ROBOSTRIDE_O2,
            T_MIN_ROBOSTRIDE_O2, T_MAX_ROBOSTRIDE_O2};
    default:
        return {};
    }
}

struct joint_state
{
    float p = 0.0, v = 0.0, t = 0.0, temp = 0.0, p_orig = 0.0;
    uint8_t error_code;
};
struct joint_control
{
    float p_des = 0.0f, v_des = 0.0f, kp = 0.0f, kd = 0.0f, t_ff = 0.0f;
};

struct bus_state_data
{
    joint_state *j;
};
struct bus_command_data
{
    joint_control *j;
};

struct bus
{
    bus_state_data state;
    bus_command_data command;
    ActuatorParams *params;
    std::vector<ActuatorParams> params_vec;
};
// Calculate CRC-8 checksum
uint8_t calculate_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float sb_fmaxf(float x, float y)
{
    /// Returns maximum of x, y ///
    return (((x) > (y)) ? (x) : (y));
}

float sb_fminf(float x, float y)
{
    /// Returns minimum of x, y ///
    return (((x) < (y)) ? (x) : (y));
}

float sb_fmaxf3(float x, float y, float z)
{
    /// Returns maximum of x, y, z ///
    return (x > y ? (x > z ? x : z) : (y > z ? y : z));
}

float sb_fminf3(float x, float y, float z)
{
    /// Returns minimum of x, y, z ///
    return (x < y ? (x < z ? x : z) : (y < z ? y : z));
}

float sb_fmod(float a, float b)
{
    return a - b * floorf(a / b);
}

float wrap_angle(float angle) {
    return fmodf(angle - WRAP_MIN, WRAP_RANGE) + WRAP_MIN;
}

// MIT-style: 16b p + 12b v + 12b Kp + 12b Kd + 12b torque (used if motor is in MIT protocol)
void pack_cmd(uint8_t *msg, bus &bus, const int node_id)
{
    auto &joint = bus.command.j[node_id];
    auto &params = bus.params[node_id];

    float p_des = sb_fminf(sb_fmaxf(params.p_min, joint.p_des), params.p_max);
    float v_des = sb_fminf(sb_fmaxf(params.v_min, joint.v_des), params.v_max);
    float kp = sb_fminf(sb_fmaxf(params.kp_min, joint.kp), params.kp_max);
    float kd = sb_fminf(sb_fmaxf(params.kd_min, joint.kd), params.kd_max);
    float t_ff = sb_fminf(sb_fmaxf(params.t_min, joint.t_ff), params.t_max);

    int p_int = float_to_uint(p_des, params.p_min, params.p_max, 16);
    int v_int = float_to_uint(v_des, params.v_min, params.v_max, 12);
    int kp_int = float_to_uint(kp, params.kp_min, params.kp_max, 12);
    int kd_int = float_to_uint(kd, params.kd_min, params.kd_max, 12);
    int t_int = float_to_uint(t_ff, params.t_min, params.t_max, 12);

    msg[0] = p_int >> 8;
    msg[1] = p_int & 0xFF;
    msg[2] = v_int >> 4;
    msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    msg[4] = kp_int & 0xFF;
    msg[5] = kd_int >> 4;
    msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    msg[7] = t_int & 0xFF;
}

// Robostride O2 Private protocol Type 1 (manual 4.1.2): 4×16-bit (p, v, Kp, Kd), no torque. High byte first.
// Use this when the motor is in Private protocol (extended 29-bit ID).
void pack_cmd_private_o2(uint8_t *msg, bus &bus, const int node_id)
{
    auto &joint = bus.command.j[node_id];
    auto &params = bus.params[node_id];

    float p_des = sb_fminf(sb_fmaxf(params.p_min, joint.p_des), params.p_max);
    float v_des = sb_fminf(sb_fmaxf(params.v_min, joint.v_des), params.v_max);
    float kp = sb_fminf(sb_fmaxf(params.kp_min, joint.kp), params.kp_max);
    float kd = sb_fminf(sb_fmaxf(params.kd_min, joint.kd), params.kd_max);

    int p_int = float_to_uint(p_des, params.p_min, params.p_max, 16);
    int v_int = float_to_uint(v_des, params.v_min, params.v_max, 16);
    int kp_int = float_to_uint(kp, params.kp_min, params.kp_max, 16);
    int kd_int = float_to_uint(kd, params.kd_min, params.kd_max, 16);

    msg[0] = (p_int >> 8) & 0xFF;
    msg[1] = p_int & 0xFF;
    msg[2] = (v_int >> 8) & 0xFF;
    msg[3] = v_int & 0xFF;
    msg[4] = (kp_int >> 8) & 0xFF;
    msg[5] = kp_int & 0xFF;
    msg[6] = (kd_int >> 8) & 0xFF;
    msg[7] = kd_int & 0xFF;
}

void pack_exit_motor_mode_cmd(uint8_t *frame)
{

    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    frame[3] = 0xFF;
    frame[4] = 0xFF;
    frame[5] = 0xFF;
    frame[6] = 0xFF;
    frame[7] = 0xFD;
}

void pack_enter_motor_mode_cmd(uint8_t *frame)
{

    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    frame[3] = 0xFF;
    frame[4] = 0xFF;
    frame[5] = 0xFF;
    frame[6] = 0xFF;
    frame[7] = 0xFC;
}

void pack_zero_encoder(uint8_t *frame)
{

    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = 0xFF;
    frame[3] = 0xFF;
    frame[4] = 0xFF;
    frame[5] = 0xFF;
    frame[6] = 0xFF;
    frame[7] = 0xFE;
}

// void unpack_reply(const std::vector<uint8_t> &buf, bus &bus, const int node_id)
// {
//     auto &params = bus.params[node_id];

//     int id = buf[0];
//     int p_int = (buf[1] << 8) | buf[2];
//     int v_int = (buf[3] << 4) | (buf[4] >> 4);
//     int i_int = ((buf[4] & 0xF) << 8) | buf[5];

//     // Convert uints to floats
//     float p = uint_to_float(p_int, params.p_min, params.p_max, 16);
//     float v = uint_to_float(v_int, params.v_min, params.v_max, 12);
//     float t = uint_to_float(i_int, params.t_min, params.t_max, 12);

//     bus.state.j[node_id].p = p;
//     bus.state.j[node_id].v = v;
//     bus.state.j[node_id].t = t;
// }
void unpack_reply(const std::vector<uint8_t> &buf, bus &bus, const int node_id)
{
    auto &params = bus.params[node_id];

    int id = buf[0];
    int p_int = (buf[1] << 8) | buf[2];
    int v_int = (buf[3] << 4) | (buf[4] >> 4);
    int i_int = ((buf[4] & 0xF) << 8) | buf[5];

    // Convert uints to floats
    float p_wrapped = uint_to_float(p_int, params.p_min, params.p_max, 16);
    float v = uint_to_float(v_int, params.v_min, params.v_max, 12);
    float t = uint_to_float(i_int, params.t_min, params.t_max, 12);

    // Store the original wrapped value
    bus.state.j[node_id].p_orig = p_wrapped;

    // Unwrap the angle
    float prev_unwrapped = bus.state.j[node_id].p;
    // float prev_wrapped = fmodf(prev_unwrapped - WRAP_MIN, WRAP_RANGE) + WRAP_MIN;
    float prev_wrapped = wrap_angle(prev_unwrapped);
    float diff = p_wrapped - prev_wrapped;

    // Adjust for wraparound
    if (diff > WRAP_RANGE / 2)
    {
        diff -= WRAP_RANGE;
    }
    else if (diff < -WRAP_RANGE / 2)
    {
        diff += WRAP_RANGE;
    }

    // Update the unwrapped angle
    float unwrapped = prev_unwrapped + diff;

    bus.state.j[node_id].p = unwrapped; // Store the unwrapped value
    bus.state.j[node_id].v = v;
    bus.state.j[node_id].t = t;
}

// Robostride O2 Private protocol Type 2 (manual): 8 bytes = 16b p, 16b v, 16b torque, 16b temp (×10 °C). High byte first.
// Use when the motor sends feedback in manual format.
void unpack_reply_o2_manual(const std::vector<uint8_t> &buf, bus &bus, const int node_id)
{
    auto &params = bus.params[node_id];
    if (buf.size() < 8)
        return;

    int p_int = (buf[0] << 8) | buf[1];
    int v_int = (buf[2] << 8) | buf[3];
    int t_int = (buf[4] << 8) | buf[5];
    int temp_int = (buf[6] << 8) | buf[7];

    float p_wrapped = uint_to_float(p_int, params.p_min, params.p_max, 16);
    float v = uint_to_float(v_int, params.v_min, params.v_max, 16);
    float t = uint_to_float(t_int, params.t_min, params.t_max, 16);
    float temp_c = (float)temp_int / 10.0f;

    bus.state.j[node_id].p_orig = p_wrapped;

    float prev_unwrapped = bus.state.j[node_id].p;
    float prev_wrapped = wrap_angle(prev_unwrapped);
    float diff = p_wrapped - prev_wrapped;
    if (diff > WRAP_RANGE / 2)
        diff -= WRAP_RANGE;
    else if (diff < -WRAP_RANGE / 2)
        diff += WRAP_RANGE;
    float unwrapped = prev_unwrapped + diff;

    bus.state.j[node_id].p = unwrapped;
    bus.state.j[node_id].v = v;
    bus.state.j[node_id].t = t;
    bus.state.j[node_id].temp = temp_c;
}

void print_bus_state(const bus &bus, const int num_nodes)
{
    for (int i = 0; i < num_nodes; i++)
    {
        printf("Node %d: P: %f, V: %f, T: %f status %d\n", i, bus.state.j[i].p, bus.state.j[i].v, bus.state.j[i].t, bus.state.j[i].error_code);
    }
}
#endif // SPINE_UTILS_H
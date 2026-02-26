#include "robstride_o2_motor.h"
#include <cstring>

namespace
{
    inline void fill_magic(uint8_t out[8], uint8_t magic)
    {
        for (int i = 0; i < 7; ++i)
        {
            out[i] = 0xFF;
        }
        out[7] = magic;
    }

    inline void fill_zeros(uint8_t out[8])
    {
        std::memset(out, 0, 8);
    }
}

RobstrideO2Motor::RobstrideO2Motor(uint8_t can_id)
    : can_id_(can_id)
{
}

uint32_t RobstrideO2Motor::makeId(O2CommandType type) const
{
    uint8_t t = static_cast<uint8_t>(type);
    return (static_cast<uint32_t>(t & 0x1F) << 24) |
           static_cast<uint32_t>(can_id_ & 0xFF);
}

uint16_t RobstrideO2Motor::makeMitId(uint8_t mode_type) const
{
    return (static_cast<uint16_t>(mode_type & 0x07) << 8) |
           static_cast<uint16_t>(can_id_ & 0xFF);
}

void RobstrideO2Motor::buildEnableFrame(uint8_t out[8]) const
{
    fill_magic(out, 0xFC);
}

void RobstrideO2Motor::buildStopFrame(uint8_t out[8]) const
{
    fill_magic(out, 0xFD);
}

void RobstrideO2Motor::buildZeroFrame(uint8_t out[8]) const
{
    fill_magic(out, 0xFE);
}

void RobstrideO2Motor::buildGetDeviceIdFrame(uint8_t out[8]) const
{
    fill_zeros(out);
}

void RobstrideO2Motor::buildSetCanIdFrame(uint8_t new_can_id, uint8_t out[8]) const
{
    fill_zeros(out);
    out[0] = new_can_id;
}

void RobstrideO2Motor::buildParamReadFrame(uint16_t param_id, uint8_t out[8]) const
{
    fill_zeros(out);
    out[0] = static_cast<uint8_t>(param_id & 0xFF);
    out[1] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
}

void RobstrideO2Motor::buildParamWriteFrame(uint16_t param_id, float value, uint8_t out[8]) const
{
    fill_zeros(out);
    out[0] = static_cast<uint8_t>(param_id & 0xFF);
    out[1] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
    std::memcpy(out + 4, &value, 4);
}

void RobstrideO2Motor::buildParamWriteU8(uint16_t param_id, uint8_t value, uint8_t out[8]) const
{
    fill_zeros(out);
    out[0] = static_cast<uint8_t>(param_id & 0xFF);
    out[1] = static_cast<uint8_t>((param_id >> 8) & 0xFF);
    out[4] = value;
}

void RobstrideO2Motor::buildGenericTypeFrame(O2CommandType /*type*/, uint8_t out[8]) const
{
    fill_zeros(out);
}

void RobstrideO2Motor::buildOperationFrame(float pos_rad,
                                           float vel_rad_s,
                                           float kp,
                                           float kd,
                                           const ActuatorParams &limits,
                                           uint8_t out[8]) const
{
    float p_des = sb_fminf(sb_fmaxf(limits.p_min, pos_rad), limits.p_max);
    float v_des = sb_fminf(sb_fmaxf(limits.v_min, vel_rad_s), limits.v_max);
    float kp_clamped = sb_fminf(sb_fmaxf(limits.kp_min, kp), limits.kp_max);
    float kd_clamped = sb_fminf(sb_fmaxf(limits.kd_min, kd), limits.kd_max);

    int p_int = float_to_uint(p_des, limits.p_min, limits.p_max, 16);
    int v_int = float_to_uint(v_des, limits.v_min, limits.v_max, 16);
    int kp_int = float_to_uint(kp_clamped, limits.kp_min, limits.kp_max, 16);
    int kd_int = float_to_uint(kd_clamped, limits.kd_min, limits.kd_max, 16);

    out[0] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_int & 0xFF);
    out[2] = static_cast<uint8_t>((v_int >> 8) & 0xFF);
    out[3] = static_cast<uint8_t>(v_int & 0xFF);
    out[4] = static_cast<uint8_t>((kp_int >> 8) & 0xFF);
    out[5] = static_cast<uint8_t>(kp_int & 0xFF);
    out[6] = static_cast<uint8_t>((kd_int >> 8) & 0xFF);
    out[7] = static_cast<uint8_t>(kd_int & 0xFF);
}

void RobstrideO2Motor::buildMitOperationFrame(float pos_rad,
                                              float vel_rad_s,
                                              float kp,
                                              float kd,
                                              float torque_ff,
                                              const ActuatorParams &limits,
                                              uint8_t out[8]) const
{
    float p_des = sb_fminf(sb_fmaxf(limits.p_min, pos_rad), limits.p_max);
    float v_des = sb_fminf(sb_fmaxf(limits.v_min, vel_rad_s), limits.v_max);
    float kp_clamped = sb_fminf(sb_fmaxf(limits.kp_min, kp), limits.kp_max);
    float kd_clamped = sb_fminf(sb_fmaxf(limits.kd_min, kd), limits.kd_max);
    float t_clamped = sb_fminf(sb_fmaxf(limits.t_min, torque_ff), limits.t_max);

    int p_int = float_to_uint(p_des, limits.p_min, limits.p_max, 16);
    int v_int = float_to_uint(v_des, limits.v_min, limits.v_max, 12);
    int kp_int = float_to_uint(kp_clamped, limits.kp_min, limits.kp_max, 12);
    int kd_int = float_to_uint(kd_clamped, limits.kd_min, limits.kd_max, 12);
    int t_int = float_to_uint(t_clamped, limits.t_min, limits.t_max, 12);

    out[0] = static_cast<uint8_t>((p_int >> 8) & 0xFF);
    out[1] = static_cast<uint8_t>(p_int & 0xFF);
    out[2] = static_cast<uint8_t>((v_int >> 4) & 0xFF);
    out[3] = static_cast<uint8_t>(((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
    out[4] = static_cast<uint8_t>(kp_int & 0xFF);
    out[5] = static_cast<uint8_t>((kd_int >> 4) & 0xFF);
    out[6] = static_cast<uint8_t>(((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F));
    out[7] = static_cast<uint8_t>(t_int & 0xFF);
}

O2Feedback RobstrideO2Motor::decodeFeedback(const uint8_t in[8],
                                            const ActuatorParams &limits,
                                            float prev_unwrapped_pos) const
{
    O2Feedback fb;

    int p_int = (static_cast<int>(in[0]) << 8) | static_cast<int>(in[1]);
    int v_int = (static_cast<int>(in[2]) << 8) | static_cast<int>(in[3]);
    int t_int = (static_cast<int>(in[4]) << 8) | static_cast<int>(in[5]);
    int temp_int = (static_cast<int>(in[6]) << 8) | static_cast<int>(in[7]);

    float p_wrapped = uint_to_float(p_int, limits.p_min, limits.p_max, 16);
    float v = uint_to_float(v_int, limits.v_min, limits.v_max, 16);
    float t = uint_to_float(t_int, limits.t_min, limits.t_max, 16);
    float temp_c = static_cast<float>(temp_int) / 10.0f;

    float prev_wrapped = wrap_angle(prev_unwrapped_pos);
    float diff = p_wrapped - prev_wrapped;

    if (diff > WRAP_RANGE / 2)
    {
        diff -= WRAP_RANGE;
    }
    else if (diff < -WRAP_RANGE / 2)
    {
        diff += WRAP_RANGE;
    }

    float unwrapped = prev_unwrapped_pos + diff;

    fb.position = unwrapped;
    fb.velocity = v;
    fb.torque = t;
    fb.temp = temp_c;

    return fb;
}


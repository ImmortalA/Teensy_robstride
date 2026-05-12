#include "RobStrideMotor.h"
#include <math.h>
#include <string.h>

extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

static constexpr uint8_t kCmdRamWrite = 18;
static constexpr uint8_t kCmdMotion = 1;
static constexpr uint8_t kCmdEnable = 3;
static constexpr uint8_t kCmdStop = 4;
static constexpr uint8_t kCmdZero = 6;
static constexpr uint8_t kCmdGetStatus = 15;
static constexpr uint16_t kA_run = 0x7005;
static constexpr uint32_t kRunModeOperationControl = 0;

bool RobStrideMotor::s_global_estop = false;
uint32_t RobStrideMotor::s_last_bus_send_us_ = 0;
uint32_t RobStrideMotor::s_can_tx_fails = 0;

#ifndef MIN_CMD_INTERVAL_US
#define MIN_CMD_INTERVAL_US 2000u
#endif
#ifndef MIN_GLOBAL_INTER_SEND_US
#define MIN_GLOBAL_INTER_SEND_US 300u
#endif
#ifndef JUMP_MAX_VEL
#define JUMP_MAX_VEL 200.0f
#endif
static constexpr float kJumpMaxPos = 1.0f;
static constexpr float kPi = 3.1415926f;
static const MotorParams kP00 = {-4 * kPi, 4 * kPi, -33, 33, -14, 14, 0, 500, 0, 5};
static const MotorParams kP01 = {-4 * kPi, 4 * kPi, -44, 44, -17, 17, 0, 500, 0, 5};
static const MotorParams kP02 = {-4 * kPi, 4 * kPi, -44, 44, -17, 17, 0, 500, 0, 5};
static const MotorParams kP03 = {-4 * kPi, 4 * kPi, -20, 20, -60, 60, 0, 500, 0, 5};
static const MotorParams kP04 = {-4 * kPi, 4 * kPi, -15, 15, -120, 120, 0, 500, 0, 5};
static const MotorParams kP06 = {-4 * kPi, 4 * kPi, -50, 50, -36, 36, 0, 5000, 0, 100};
static constexpr uint8_t kCmdStatus = 2;

static uint16_t floatToUint_s(float x, float x_min, float x_max, int bits) {
  if (bits > 16) bits = 16;
  if (x > x_max) x = x_max;
  if (x < x_min) x = x_min;
  float span = x_max - x_min;
  float scaled = (x - x_min) * ((float)((1u << bits) - 1u)) / span;
  return (uint16_t)scaled;
}
static float uintToFloat_s(uint16_t x, float x_min, float x_max) {
  return ((float)x / 65535.0f) * (x_max - x_min) + x_min;
}
static uint32_t packExtId_(uint8_t mid, uint8_t cmd, uint16_t opt) {
  return (uint32_t)(cmd & 0xFFu) << 24 | (uint32_t)(opt & 0xFFFFu) << 8 | (uint32_t)(mid & 0xFFu);
}
static bool canTxFrame(uint32_t id, const uint8_t* d) {
  CAN_message_t m;
  m.flags.extended = 1;
  m.id = id;
  m.len = 8;
  memcpy(m.buf, d, 8);
  return (Can0.write(m) > 0);
}

static uint32_t g_dec_uninited_ms[8];

void RobStrideMotor::setGlobalEstop(bool a) { s_global_estop = a; }
bool RobStrideMotor::globalEstopActive() { return s_global_estop; }

RobStrideMotor::RobStrideMotor()
    : id_(0),
      master_id_(0),
      mode_status_(0),
      inited_(false),
      state_(MotorState::UNINIT),
      last_pos_fb_(0),
      last_vel_fb_(0),
      have_last_fb_(false),
      last_feedback_ms_(0),
      last_send_us_(0),
      metrics_(),
      pend_type_(PEND_NONE) {}

void RobStrideMotor::notInited_() { metrics_.not_inited_cmds++; }
void RobStrideMotor::stateBlock_() { metrics_.state_blocked_cmds++; }
void RobStrideMotor::estopBlock_() { metrics_.estop_blocked_sends++; }

bool RobStrideMotor::isfiniteCmd_(float x) const { return (isfinite(x) != 0); }

float RobStrideMotor::clamp(float v, float lo, float hi) const {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool RobStrideMotor::busWouldBlock_(bool is_stop) const {
  if (is_stop) return false;
  uint32_t t = micros();
  if (s_last_bus_send_us_ != 0 && (uint32_t)(t - s_last_bus_send_us_) < MIN_GLOBAL_INTER_SEND_US) return true;
  if (last_send_us_ != 0 && (uint32_t)(t - last_send_us_) < MIN_CMD_INTERVAL_US) return true;
  return false;
}

void RobStrideMotor::markBusTx_() {
  uint32_t t = micros();
  s_last_bus_send_us_ = t;
  last_send_us_ = t;
}

void RobStrideMotor::shortSpinForBus_() {
  for (uint8_t k = 0; k < 30; k++) {
    if (!busWouldBlock_(false)) return;
    delayMicroseconds(200);
  }
  if (busWouldBlock_(false)) metrics_.throttle_drops++;
}

bool RobStrideMotor::sendNow_(uint8_t cmd, uint16_t opt, const uint8_t* d, bool is_stop) {
  if (!inited_) {
    notInited_();
    return false;
  }
  if (s_global_estop && !is_stop) {
    estopBlock_();
    return false;
  }
  if (busWouldBlock_(is_stop) && !is_stop) {
    metrics_.throttle_drops++;
    return false;
  }
  if (!canTxFrame(packExtId_(id_, cmd, opt), d)) {
    s_can_tx_fails++;
    return false;
  }
  markBusTx_();
  return true;
}

bool RobStrideMotor::sendOneRamU32_(uint16_t addr, uint32_t raw) {
  uint8_t b[8] = {0};
  b[4] = (uint8_t)raw;
  b[5] = (uint8_t)(raw >> 8);
  b[6] = (uint8_t)(raw >> 16);
  b[7] = (uint8_t)(raw >> 24);
  return sendNow_(kCmdRamWrite, addr, b, false);
}

void RobStrideMotor::buildMotion8_(uint8_t o[8], uint16_t* t_opt, float pos, float vel, float kp, float kd, float tq) {
  float p = clamp(pos, params_.P_MIN, params_.P_MAX);
  float v = clamp(vel, params_.V_MIN, params_.V_MAX);
  float kpp = clamp(kp, params_.KP_MIN, params_.KP_MAX);
  float kdd = clamp(kd, params_.KD_MIN, params_.KD_MAX);
  float t = clamp(tq, params_.T_MIN, params_.T_MAX);
  uint16_t pu = floatToUint_s(p, params_.P_MIN, params_.P_MAX, 16);
  uint16_t vu = floatToUint_s(v, params_.V_MIN, params_.V_MAX, 16);
  uint16_t ku = floatToUint_s(kpp, params_.KP_MIN, params_.KP_MAX, 16);
  uint16_t du = floatToUint_s(kdd, params_.KD_MIN, params_.KD_MAX, 16);
  uint16_t tu = floatToUint_s(t, params_.T_MIN, params_.T_MAX, 16);
  o[0] = (uint8_t)(pu >> 8);
  o[1] = (uint8_t)pu;
  o[2] = (uint8_t)(vu >> 8);
  o[3] = (uint8_t)vu;
  o[4] = (uint8_t)(ku >> 8);
  o[5] = (uint8_t)ku;
  o[6] = (uint8_t)(du >> 8);
  o[7] = (uint8_t)du;
  *t_opt = tu;
}

void RobStrideMotor::sendOrQueueMotion_(float pos, float vel, float kp, float kd, float tq) {
  if (!inited_) {
    notInited_();
    return;
  }
  if (s_global_estop) {
    estopBlock_();
    return;
  }
  if (state_ != MotorState::RUNNING) {
    stateBlock_();
    return;
  }
  if (!isfiniteCmd_(pos) || !isfiniteCmd_(vel) || !isfiniteCmd_(kp) || !isfiniteCmd_(kd) || !isfiniteCmd_(tq)) {
    metrics_.invalid_float_in++;
    return;
  }
  if (busWouldBlock_(false)) {
    pend_type_ = PEND_MOTION;
    pend0_ = pos;
    pend1_ = vel;
    pend2_ = kp;
    pend3_ = kd;
    pend4_ = tq;
    metrics_.throttle_coalesced++;
    return;
  }
  uint8_t o[8];
  uint16_t tu;
  buildMotion8_(o, &tu, pos, vel, kp, kd, tq);
  if (!sendNow_(kCmdMotion, tu, o, false)) {
    pend_type_ = PEND_MOTION;
    pend0_ = pos;
    pend1_ = vel;
    pend2_ = kp;
    pend3_ = kd;
    pend4_ = tq;
  }
}

void RobStrideMotor::tryFlushPending_() {
  if (pend_type_ == PEND_NONE) return;
  if (busWouldBlock_(false)) return;
  uint8_t pt = pend_type_;
  if (pt == PEND_MOTION) {
    uint8_t o[8];
    uint16_t tu;
    buildMotion8_(o, &tu, pend0_, pend1_, pend2_, pend3_, pend4_);
    if (sendNow_(kCmdMotion, tu, o, false)) pend_type_ = PEND_NONE;
    return;
  }
}

void RobStrideMotor::servicePending() { tryFlushPending_(); }

void RobStrideMotor::pollAllHealth(RobStrideMotor* m, int n, uint32_t to_ms) {
  for (int i = 0; i < n; i++) m[i].pollHealth(to_ms);
}

bool RobStrideMotor::begin(uint8_t id, RobStrideMotorModel model) {
  if (id == 0) return false;
  id_ = id;
  master_id_ = 0;
  mode_status_ = 0;
  status_.valid = false;
  fault_ = FaultFlags{};
  have_last_fb_ = false;
  last_feedback_ms_ = 0;
  last_send_us_ = 0;
  metrics_ = MotorRuntimeMetrics{};
  pend_type_ = PEND_NONE;
  if (!applyModel(model)) {
    inited_ = false;
    state_ = MotorState::UNINIT;
    return false;
  }
  inited_ = true;
  state_ = MotorState::READY;
  return true;
}

bool RobStrideMotor::applyModel(RobStrideMotorModel model) {
  switch (model) {
    case RobStrideMotorModel::RS00: params_ = kP00; return true;
    case RobStrideMotorModel::RS01: params_ = kP01; return true;
    case RobStrideMotorModel::RS02: params_ = kP02; return true;
    case RobStrideMotorModel::RS03: params_ = kP03; return true;
    case RobStrideMotorModel::RS04: params_ = kP04; return true;
    case RobStrideMotorModel::RS06: params_ = kP06; return true;
    default: return false;
  }
}

void RobStrideMotor::setMasterId(uint8_t m) {
  if (!inited_) {
    notInited_();
    return;
  }
  master_id_ = m;
}

void RobStrideMotor::onHostEstop() {
  if (!inited_) return;
  state_ = MotorState::ESTOP;
}

void RobStrideMotor::onHostEstopRelease() {
  if (!inited_) return;
  if (state_ != MotorState::ESTOP) return;
  if (s_global_estop) return;
  if (fault_.has_fault) state_ = MotorState::FAULT;
  else state_ = MotorState::READY;
}

void RobStrideMotor::onWatchdogStop() { onHostEstop(); }

bool RobStrideMotor::tryRecoverFaultToReady() {
  if (state_ != MotorState::FAULT) return false;
  if (fault_.has_fault) return false;
  state_ = MotorState::READY;
  return true;
}

void RobStrideMotor::enable() {
  if (!inited_) {
    notInited_();
    return;
  }
  if (s_global_estop) {
    estopBlock_();
    return;
  }
  if (state_ == MotorState::FAULT || state_ == MotorState::ESTOP) {
    stateBlock_();
    return;
  }
  if (state_ != MotorState::READY) {
    stateBlock_();
    return;
  }
  uint8_t z[8] = {0};

  // Match the known-good RobStride private startup sequence:
  // Type 4 stop/reset, Type 18 RUN_MODE=operation, then Type 3 enable.
  if (!sendNow_(kCmdStop, master_id_, z, true)) return;
  delay(50);
  last_send_us_ = 0;
  s_last_bus_send_us_ = 0;

  if (!sendOneRamU32_(kA_run, kRunModeOperationControl)) return;
  delay(50);
  last_send_us_ = 0;
  s_last_bus_send_us_ = 0;

  if (!sendNow_(kCmdEnable, master_id_, z, false)) return;
  delay(50);
  if (s_global_estop) return;

  uint32_t feedbackStampBefore = status_.stamp_us;
  requestStatus();
  for (uint8_t i = 0; i < 5; i++) {
    delay(10);
    Can0.events();
    if (status_.valid && status_.stamp_us != feedbackStampBefore) break;
  }

  if (status_.valid && status_.stamp_us != feedbackStampBefore && !fault_.has_fault) {
    state_ = MotorState::RUNNING;
  } else if (fault_.has_fault) {
    state_ = MotorState::FAULT;
  } else {
    state_ = MotorState::READY;
  }
}

void RobStrideMotor::stop() {
  if (!inited_) {
    notInited_();
    return;
  }
  uint8_t z[8] = {0};
  if (!sendNow_(kCmdStop, master_id_, z, true)) return;
  if (state_ == MotorState::ESTOP) return;
  if (s_global_estop) {
    state_ = MotorState::ESTOP;
    return;
  }
  if (state_ == MotorState::RUNNING || state_ == MotorState::READY) state_ = MotorState::READY;
}

void RobStrideMotor::zero() {
  if (!inited_) {
    notInited_();
    return;
  }
  if (s_global_estop) {
    estopBlock_();
    return;
  }
  if (state_ == MotorState::FAULT || state_ == MotorState::ESTOP) {
    stateBlock_();
    return;
  }
  if (state_ != MotorState::READY && state_ != MotorState::RUNNING) {
    stateBlock_();
    return;
  }
  shortSpinForBus_();
  uint8_t d[8] = {0};
  d[0] = 1;
  (void)sendNow_(kCmdZero, master_id_, d, false);
}

void RobStrideMotor::motion(float pos, float vel, float kp, float kd, float tq) { sendOrQueueMotion_(pos, vel, kp, kd, tq); }

void RobStrideMotor::requestStatus() {
  if (!inited_) {
    notInited_();
    return;
  }
  shortSpinForBus_();
  uint8_t z[8] = {0};
  (void)sendNow_(kCmdGetStatus, master_id_, z, false);
}

bool RobStrideMotor::checkFeedbackJump_(float p, float v) {
  if (!isfinite(p) || !isfinite(v)) return false;
  if (!have_last_fb_) return true;
  if (fabsf(p - last_pos_fb_) > kJumpMaxPos) return false;
  if (fabsf(v - last_vel_fb_) > JUMP_MAX_VEL) return false;
  return true;
}

void RobStrideMotor::applyDecodeFault_() {
  if (s_global_estop) return;
  if (fault_.has_fault) state_ = MotorState::FAULT;
}

bool RobStrideMotor::decodeFeedback(const CAN_message_t& msg) {
  if (!inited_) {
    uint32_t m = millis();
    uint8_t s = (uint8_t)(id_ & 7u);
    if (g_dec_uninited_ms[s] == 0u || (uint32_t)(m - g_dec_uninited_ms[s]) > 1000u) {
      g_dec_uninited_ms[s] = m;
      metrics_.decode_err_uninited++;
    }
    return false;
  }
  if (!msg.flags.extended || msg.len != 8) return false;
  uint8_t cmd = (uint8_t)((msg.id >> 24) & 0xFFu);
  uint8_t mid = (uint8_t)((msg.id >> 8) & 0xFFu);
  if (mid != id_ || cmd != kCmdStatus) return false;
  uint16_t opt = (uint16_t)((msg.id >> 8) & 0xFFFFu);
  uint8_t oh = (uint8_t)(opt >> 8);
  mode_status_ = (oh >> 6) & 3;
  uint8_t fb = oh & 0x3F;
  fault_.has_fault = (fb >> 5) & 1;
  fault_.encoder_not_calibrated = (fb >> 4) & 1;
  fault_.gridlock_overload = (fb >> 3) & 1;
  fault_.magnetic_coding = (fb >> 2) & 1;
  fault_.overtemperature = (fb >> 1) & 1;
  fault_.three_phase_overcurrent = fb & 1;
  uint16_t rp = (uint16_t)msg.buf[1] | (uint16_t)msg.buf[0] << 8;
  uint16_t rv = (uint16_t)msg.buf[3] | (uint16_t)msg.buf[2] << 8;
  uint16_t rt = (uint16_t)msg.buf[5] | (uint16_t)msg.buf[4] << 8;
  uint16_t rT = (uint16_t)msg.buf[7] | (uint16_t)msg.buf[6] << 8;
  float np = uintToFloat_s(rp, params_.P_MIN, params_.P_MAX);
  float nv = uintToFloat_s(rv, params_.V_MIN, params_.V_MAX);
  float nt = uintToFloat_s(rt, params_.T_MIN, params_.T_MAX);
  float nT = (float)rT / 10.0f;
  if (!isfinite(np) || !isfinite(nv) || !isfinite(nt) || !isfinite(nT)) return false;
  if (!checkFeedbackJump_(np, nv)) {
    metrics_.jump_rejected++;
    return false;
  }
  last_pos_fb_ = np;
  last_vel_fb_ = nv;
  have_last_fb_ = true;
  status_.valid = true;
  status_.stamp_us = micros();
  last_feedback_ms_ = millis();
  status_.position = np;
  status_.velocity = nv;
  status_.torque = nt;
  status_.temperature_c = nT;
  applyDecodeFault_();
  return true;
}

void RobStrideMotor::pollHealth(uint32_t feedback_timeout_ms) {
  if (!inited_ || !status_.valid || feedback_timeout_ms == 0) return;
  uint32_t now = millis();
  if ((uint32_t)(now - last_feedback_ms_) > feedback_timeout_ms) status_.valid = false;
}

void RobStrideMotor::getRuntimeMetrics(MotorRuntimeMetrics& out) const { out = metrics_; }

void RobStrideMotor::clearRuntimeMetrics() { metrics_ = MotorRuntimeMetrics{}; }

uint8_t RobStrideMotor::id() const { return id_; }
Status RobStrideMotor::getStatus() const { return status_; }
FaultFlags RobStrideMotor::getFault() const { return fault_; }
uint8_t RobStrideMotor::getModeStatus() const { return mode_status_; }
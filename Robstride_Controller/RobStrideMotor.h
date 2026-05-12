#ifndef ROBSTRIDE_MOTOR_H
#define ROBSTRIDE_MOTOR_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

enum class RobStrideMotorModel : uint8_t {
  RS00 = 0, RS01 = 1, RS02 = 2, RS03 = 3, RS04 = 4, RS06 = 6
};

/** Host-side lifecycle; drives command gating (not the drive’s internal run mode). */
enum class MotorState : uint8_t { UNINIT, READY, RUNNING, FAULT, ESTOP };

struct MotorParams {
  float P_MIN, P_MAX, V_MIN, V_MAX, T_MIN, T_MAX;
  float KP_MIN, KP_MAX, KD_MIN, KD_MAX;
};

struct Status {
  bool valid = false;
  uint32_t stamp_us = 0;
  float position = 0, velocity = 0, torque = 0, temperature_c = 0;
};

struct FaultFlags {
  bool has_fault = false, encoder_not_calibrated = false, gridlock_overload = false;
  bool magnetic_coding = false, overtemperature = false, three_phase_overcurrent = false;
};

/** Cumulative; safe to read from any context; do not use Serial in ISR. */
struct MotorRuntimeMetrics {
  uint32_t not_inited_cmds;
  uint32_t invalid_float_in;
  uint32_t throttle_drops;
  uint32_t throttle_coalesced;
  uint32_t estop_blocked_sends;
  uint32_t state_blocked_cmds;
  uint32_t decode_err_uninited;
  uint32_t jump_rejected;
};

class RobStrideMotor {
public:
  RobStrideMotor();

  bool begin(uint8_t id, RobStrideMotorModel model);

  static void setGlobalEstop(bool active);
  static bool globalEstopActive();

  /** Host estop/reset: sets drive ESTOP and updates state (call on estop/reset). */
  void onHostEstop();
  void onHostEstopRelease();
  void onWatchdogStop();

  void setMasterId(uint8_t master_id);
  void enable();
  void stop();
  void zero();
  void motion(float pos, float vel, float kp, float kd, float torque_ff);

  void requestStatus();
  bool decodeFeedback(const CAN_message_t& msg);

  void pollHealth(uint32_t feedback_timeout_ms);

  /** Run after decode + Serial; flushes throttled / coalesced motion RAM commands. */
  void servicePending();

  static void pollAllHealth(RobStrideMotor* motors, int n, uint32_t feedback_timeout_ms);

  uint8_t id() const;
  MotorState state() const { return state_; }
  Status getStatus() const;
  FaultFlags getFault() const;
  uint8_t getModeStatus() const;

  void getRuntimeMetrics(MotorRuntimeMetrics& out) const;
  void clearRuntimeMetrics();
  static uint32_t canTxFailCount() { return s_can_tx_fails; }

  uint32_t throttleDropCount() const { return metrics_.throttle_drops; }

  /** If in FAULT and drive reports no fault, move to READY. */
  bool tryRecoverFaultToReady();

private:
  uint8_t id_;
  uint8_t master_id_;
  uint8_t mode_status_;
  bool inited_;
  MotorState state_;
  MotorParams params_;
  Status status_;
  FaultFlags fault_;
  float last_pos_fb_;
  float last_vel_fb_;
  bool have_last_fb_;

  uint32_t last_feedback_ms_;
  uint32_t last_send_us_;
  MotorRuntimeMetrics metrics_;

  static bool s_global_estop;
  static uint32_t s_last_bus_send_us_;
  static uint32_t s_can_tx_fails;

  enum : uint8_t { PEND_NONE, PEND_MOTION };
  uint8_t pend_type_;
  float pend0_, pend1_, pend2_, pend3_, pend4_;

  bool applyModel(RobStrideMotorModel model);
  float clamp(float v, float lo, float hi) const;
  bool busWouldBlock_(bool is_stop) const;
  void markBusTx_();
  void shortSpinForBus_();
  void sendOrQueueMotion_(float pos, float vel, float kp, float kd, float tq);
  void tryFlushPending_();
  bool sendOneRamU32_(uint16_t addr, uint32_t raw);
  void buildMotion8_(uint8_t out8[8], uint16_t* t_opt, float p, float v, float kp, float kd, float tq);
  bool sendNow_(uint8_t cmd, uint16_t opt, const uint8_t* data, bool is_stop);
  void notInited_();
  void stateBlock_();
  void estopBlock_();
  bool isfiniteCmd_(float x) const;
  void applyDecodeFault_();
  bool checkFeedbackJump_(float p, float v);
};

#endif

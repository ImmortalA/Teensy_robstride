#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <string.h>

// ================================================================
// USER CONFIG
// ================================================================

static constexpr uint32_t CAN_BAUD = 1000000;
static constexpr uint8_t MOTOR_ID = 1;
static constexpr uint8_t MASTER_ID = 0;

// Select your RobStride motor model (affects scaling/clamping ranges).
// Values align with `Robstride_Controller/RobStrideMotor.h`.
enum class RobStrideMotorModel : uint8_t {
  RS00 = 0,
  RS01 = 1,
  RS02 = 2,
  RS03 = 3,
  RS04 = 4,
  RS06 = 6,
};

static constexpr RobStrideMotorModel SELECTED_MOTOR = RobStrideMotorModel::RS06;

// Change CAN1 to CAN2/CAN3 if your wiring uses another Teensy CAN port.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// RobStride RS06 communication types
#define CANCOM_MOTOR_CTRL   1
#define CANCOM_MOTOR_ENABLE 3
#define CANCOM_MOTOR_RESET  4
#define CANCOM_PARAM_WRITE  18

// Run mode parameter
static constexpr uint16_t RUN_MODE = 0x7005;
static constexpr uint8_t RUN_MODE_OPERATION = 0;

// Default limits = RS06 / O6 (keeps current behavior unless SELECTED_MOTOR changed)
static float P_MIN = -12.5664f;  // -4pi
static float P_MAX = 12.5664f;   //  4pi
static float V_MIN = -50.0f;
static float V_MAX = 50.0f;
static float T_MIN = -36.0f;
static float T_MAX = 36.0f;

static float KP_MIN = 0.0f;
static float KP_MAX = 5000.0f;  // RS06 manual mapping
static float KD_MIN = 0.0f;
static float KD_MAX = 100.0f;   // RS06 manual mapping

static const char* motorModelLabel(RobStrideMotorModel m) {
  switch (m) {
    case RobStrideMotorModel::RS00: return "RS00";
    case RobStrideMotorModel::RS01: return "RS01";
    case RobStrideMotorModel::RS02: return "RS02";
    case RobStrideMotorModel::RS03: return "RS03";
    case RobStrideMotorModel::RS04: return "RS04";
    case RobStrideMotorModel::RS06: return "RS06";
    default: return "UNKNOWN";
  }
}

static void applyMotorModel(RobStrideMotorModel m) {
  // P/V/T ranges match `Robstride_Controller/RobStrideMotor.cpp` (kP00..kP06).
  // RS06 keeps the existing KP/KD scaling from your manual.
  const float kPi = 3.14159265f;
  switch (m) {
    case RobStrideMotorModel::RS00:
      P_MIN = -4 * kPi; P_MAX = 4 * kPi; V_MIN = -33.0f; V_MAX = 33.0f; T_MIN = -14.0f; T_MAX = 14.0f;
      KP_MIN = 0.0f; KP_MAX = 500.0f; KD_MIN = 0.0f; KD_MAX = 5.0f;
      break;
    case RobStrideMotorModel::RS01:
      P_MIN = -4 * kPi; P_MAX = 4 * kPi; V_MIN = -44.0f; V_MAX = 44.0f; T_MIN = -17.0f; T_MAX = 17.0f;
      KP_MIN = 0.0f; KP_MAX = 500.0f; KD_MIN = 0.0f; KD_MAX = 5.0f;
      break;
    case RobStrideMotorModel::RS02:
      P_MIN = -4 * kPi; P_MAX = 4 * kPi; V_MIN = -44.0f; V_MAX = 44.0f; T_MIN = -17.0f; T_MAX = 17.0f;
      KP_MIN = 0.0f; KP_MAX = 500.0f; KD_MIN = 0.0f; KD_MAX = 5.0f;
      break;
    case RobStrideMotorModel::RS03:
      P_MIN = -4 * kPi; P_MAX = 4 * kPi; V_MIN = -20.0f; V_MAX = 20.0f; T_MIN = -60.0f; T_MAX = 60.0f;
      KP_MIN = 0.0f; KP_MAX = 500.0f; KD_MIN = 0.0f; KD_MAX = 5.0f;
      break;
    case RobStrideMotorModel::RS04:
      P_MIN = -4 * kPi; P_MAX = 4 * kPi; V_MIN = -15.0f; V_MAX = 15.0f; T_MIN = -120.0f; T_MAX = 120.0f;
      KP_MIN = 0.0f; KP_MAX = 500.0f; KD_MIN = 0.0f; KD_MAX = 5.0f;
      break;
    case RobStrideMotorModel::RS06:
    default:
      P_MIN = -12.5664f; P_MAX = 12.5664f; V_MIN = -50.0f; V_MAX = 50.0f; T_MIN = -36.0f; T_MAX = 36.0f;
      KP_MIN = 0.0f; KP_MAX = 5000.0f; KD_MIN = 0.0f; KD_MAX = 100.0f;
      break;
  }
}

// Safe sine command
static constexpr float SINE_AMP = 2.0f;     // rad
static constexpr float SINE_OMEGA = 1.0f;    // rad/s
static constexpr float KP = 25.0f;
static constexpr float KD = 2.5f;
static constexpr float T_FF = 0.0f;

static constexpr uint32_t CTRL_PERIOD_MS = 10;

// ================================================================
// GLOBAL STATE
// ================================================================

static bool center_ready = false;
static float p_center = 0.0f;

static uint32_t t0_ms = 0;
static uint32_t last_ctrl_ms = 0;

// ================================================================
// UTILS
// ================================================================

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static uint16_t float_to_uint16(float x, float x_min, float x_max) {
  x = clampf(x, x_min, x_max);
  return (uint16_t)((x - x_min) * 65535.0f / (x_max - x_min));
}

static float uint16_to_float(uint16_t x, float x_min, float x_max) {
  return ((float)x) * (x_max - x_min) / 65535.0f + x_min;
}

// 29-bit extended ID:
// bit28~24 = communication type
// bit23~8  = data field 2
// bit7~0   = motor id
static inline uint32_t makeExtId(uint8_t motor_id, uint8_t type) {
  return ((uint32_t)(type & 0x1F) << 24) |
         ((uint32_t)(MASTER_ID & 0xFF) << 8) |
         ((uint32_t)(motor_id & 0xFF));
}

static inline uint32_t makeExtIdWithData(uint8_t motor_id, uint8_t type, uint16_t data16) {
  return ((uint32_t)(type & 0x1F) << 24) |
         ((uint32_t)(data16 & 0xFFFF) << 8) |
         ((uint32_t)(motor_id & 0xFF));
}

// ================================================================
// RS06 REAL TYPE 1 PACKING
// ================================================================
// Manual Type 1:
// ID data field = torque: 0~65535 maps to -36Nm~36Nm
// Byte0~1 = target angle: -4pi~4pi
// Byte2~3 = target velocity: -50~50 rad/s
// Byte4~5 = Kp: 0~5000
// Byte6~7 = Kd: 0~100
// High byte first.

static void packRS06Type1(uint8_t *msg, float p, float v, float kp, float kd) {
  uint16_t p_int  = float_to_uint16(p,  P_MIN,  P_MAX);
  uint16_t v_int  = float_to_uint16(v,  V_MIN,  V_MAX);
  uint16_t kp_int = float_to_uint16(kp, KP_MIN, KP_MAX);
  uint16_t kd_int = float_to_uint16(kd, KD_MIN, KD_MAX);

  msg[0] = (p_int >> 8) & 0xFF;
  msg[1] = p_int & 0xFF;

  msg[2] = (v_int >> 8) & 0xFF;
  msg[3] = v_int & 0xFF;

  msg[4] = (kp_int >> 8) & 0xFF;
  msg[5] = kp_int & 0xFF;

  msg[6] = (kd_int >> 8) & 0xFF;
  msg[7] = kd_int & 0xFF;
}

// ================================================================
// CAN SEND
// ================================================================

static bool sendFrame(uint8_t type, const uint8_t payload[8], bool print_tx = false) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.len = 8;
  memcpy(msg.buf, payload, 8);

  if (type == CANCOM_MOTOR_CTRL) {
    uint16_t torque_int = float_to_uint16(T_FF, T_MIN, T_MAX);
    msg.id = makeExtIdWithData(MOTOR_ID, type, torque_int);
  } else if (type == CANCOM_PARAM_WRITE) {
    msg.id = makeExtId(MOTOR_ID, type);
  } else {
    msg.id = makeExtId(MOTOR_ID, type);
  }

  int ok = Can0.write(msg);

  if (print_tx) {
    Serial.printf("TX type=%u id=0x%08lX write=%d data=",
                  type, (unsigned long)msg.id, ok);
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", msg.buf[i]);
    Serial.println();
  }

  return ok > 0;
}

static void sendZeros(uint8_t type) {
  uint8_t z[8] = {0};
  sendFrame(type, z, false);
}

static void setRunModeOperation() {
  uint8_t d[8] = {0};

  d[0] = RUN_MODE & 0xFF;
  d[1] = RUN_MODE >> 8;
  d[2] = 0x00;
  d[3] = 0x00;
  d[4] = RUN_MODE_OPERATION;
  d[5] = 0x00;
  d[6] = 0x00;
  d[7] = 0x00;

  Serial.println("Set RUN_MODE = operation control mode");
  sendFrame(CANCOM_PARAM_WRITE, d, false);
  delay(300);
}

static void enableMotor() {
  Serial.println("Stop/reset Type 4");
  sendZeros(CANCOM_MOTOR_RESET);
  delay(300);

  Serial.println("Enable Type 3");
  sendZeros(CANCOM_MOTOR_ENABLE);
  delay(300);
}

// ================================================================
// CAN RX
// ================================================================

static void readRxAndLockCenter() {
  CAN_message_t rx;

  while (Can0.read(rx)) {
    uint8_t type = (rx.id >> 24) & 0x1F;

    if (type == 2 && rx.len >= 8) {
      uint16_t p_raw = ((uint16_t)rx.buf[0] << 8) | rx.buf[1];
      uint16_t v_raw = ((uint16_t)rx.buf[2] << 8) | rx.buf[3];
      uint16_t t_raw = ((uint16_t)rx.buf[4] << 8) | rx.buf[5];

      float p = uint16_to_float(p_raw, P_MIN, P_MAX);
      float v = uint16_to_float(v_raw, V_MIN, V_MAX);
      float tq = uint16_to_float(t_raw, T_MIN, T_MAX);

      if (!center_ready) {
        p_center = p;
        center_ready = true;
        Serial.printf("Locked center position: %.4f rad\r\n", p_center);
      }

      // static int n = 0;
      // if (++n % 100 == 0) {
      //   Serial.printf("RX p=%.4f v=%.4f tq=%.3f\r\n", p, v, tq);
      // }
    }
  }
}

// ================================================================
// SETUP / LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println();
  Serial.println("RS06/O6 REAL Type 1 operation-control sine test");
  applyMotorModel(SELECTED_MOTOR);
  Serial.printf("Selected motor model: %s\r\n", motorModelLabel(SELECTED_MOTOR));

  Can0.begin();
  Can0.setClock(CLK_60MHz);
  Can0.setBaudRate(CAN_BAUD);
  Can0.setMaxMB(16);
  Can0.enableFIFO();

  delay(500);

  setRunModeOperation();
  enableMotor();

  t0_ms = millis();
  last_ctrl_ms = millis();

  Serial.println("Waiting for feedback to lock center...");
}

void loop() {
  Can0.events();
  readRxAndLockCenter();

  if (!center_ready) {
    return;
  }

  uint32_t now = millis();

  if (now - last_ctrl_ms >= CTRL_PERIOD_MS) {
    last_ctrl_ms = now;

    float t = (now - t0_ms) * 0.001f;

    float p_des = p_center + SINE_AMP * sinf(SINE_OMEGA * t);
    float v_des = SINE_AMP * SINE_OMEGA * cosf(SINE_OMEGA * t);

    uint8_t buf[8];
    packRS06Type1(buf, p_des, v_des, KP, KD);

    static int n = 0;
    bool print_tx = (++n % 100 == 0);

    sendFrame(CANCOM_MOTOR_CTRL, buf, false);
  }
}
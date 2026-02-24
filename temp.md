/*
  RobStride RS02 / RS02-IP67 control via Teensy 4.1 CAN (Private protocol, extended 29-bit)

  Protocol (Manual Section 4):
    29-bit extended ID:
      bits 28..24 = communication type
      bits 7..0   = motor CAN ID
      (bits 23..8 often unused/0 in simple usage)

    Type 3 = enable motor
    Type 4 = stop motor
    Type 6 = set mechanical zero
    Type 1 = operation control: 8 bytes, 4x uint16 big-endian:
        [pos][vel][kp][kd]
    Type 2 = feedback: 8 bytes, 4x uint16 big-endian:
        [pos][vel][torque][temp_x10C]

  Scaling (per manual ranges):
    pos   : uint16 -> (-4π .. 4π) rad
    vel   : uint16 -> (-44 .. 44) rad/s
    kp    : uint16 -> (0 .. 500)
    kd    : uint16 -> (0 .. 5)
    torque: uint16 -> (-17 .. 17) Nm
    temp  : uint16 -> temperature * 10 (°C)
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ---------- CAN setup ----------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;

static const uint32_t CAN_BAUD = 1000000;  // 1 Mbps

// ---------- Motor config ----------
static const uint8_t MOTOR_ID = 1;         // set to your motor CAN ID (0..127 in tool, typically 1..)
static const uint16_t SEND_HZ = 200;       // control command rate (e.g. 200 Hz)
static const uint32_t SEND_PERIOD_US = 1000000UL / SEND_HZ;

// ---------- Manual ranges ----------
static constexpr float POS_MIN = -4.0f * PI;
static constexpr float POS_MAX =  4.0f * PI;

static constexpr float VEL_MIN = -44.0f;
static constexpr float VEL_MAX =  44.0f;

static constexpr float KP_MIN  = 0.0f;
static constexpr float KP_MAX  = 500.0f;

static constexpr float KD_MIN  = 0.0f;
static constexpr float KD_MAX  = 5.0f;

static constexpr float TQ_MIN  = -17.0f;
static constexpr float TQ_MAX  =  17.0f;

// ---------- Helpers ----------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline uint16_t float_to_u16(float x, float x_min, float x_max) {
  x = clampf(x, x_min, x_max);
  float span = x_max - x_min;
  float norm = (x - x_min) / span;                 // 0..1
  uint32_t v = (uint32_t) lroundf(norm * 65535.0f);
  if (v > 65535UL) v = 65535UL;
  return (uint16_t)v;
}

static inline float u16_to_float(uint16_t x, float x_min, float x_max) {
  float norm = ((float)x) / 65535.0f;
  return x_min + norm * (x_max - x_min);
}

static inline void put_u16_be(uint8_t *buf, int idx, uint16_t v) {
  buf[idx + 0] = (uint8_t)(v >> 8);
  buf[idx + 1] = (uint8_t)(v & 0xFF);
}

static inline uint16_t get_u16_be(const uint8_t *buf, int idx) {
  return (uint16_t)((buf[idx] << 8) | buf[idx + 1]);
}

// Build 29-bit extended ID: type in bits 28..24, motor id in bits 7..0
static inline uint32_t make_ext_id(uint8_t type, uint8_t motor_id) {
  return ((uint32_t)(type & 0x1F) << 24) | (uint32_t)motor_id;
}

// ---------- Sending frames ----------
bool send_type_frame(uint8_t type, uint8_t motor_id, const uint8_t payload[8]) {
  CAN_message_t msg;
  msg.id = make_ext_id(type, motor_id);
  msg.len = 8;
  msg.flags.extended = 1;

  for (int i = 0; i < 8; i++) msg.buf[i] = payload ? payload[i] : 0;

  return Can1.write(msg);
}

bool enable_motor(uint8_t motor_id) {
  uint8_t p[8] = {0};
  return send_type_frame(3, motor_id, p);
}

bool stop_motor(uint8_t motor_id) {
  uint8_t p[8] = {0};
  return send_type_frame(4, motor_id, p);
}

bool set_zero(uint8_t motor_id) {
  uint8_t p[8] = {0};
  return send_type_frame(6, motor_id, p);
}

// Type 1 operation control: [pos][vel][kp][kd] as uint16 big-endian
bool send_operation_control(uint8_t motor_id, float pos_rad, float vel_rads, float kp, float kd) {
  uint16_t pos_u = float_to_u16(pos_rad, POS_MIN, POS_MAX);
  uint16_t vel_u = float_to_u16(vel_rads, VEL_MIN, VEL_MAX);
  uint16_t kp_u  = float_to_u16(kp,      KP_MIN,  KP_MAX);
  uint16_t kd_u  = float_to_u16(kd,      KD_MIN,  KD_MAX);

  uint8_t p[8];
  put_u16_be(p, 0, pos_u);
  put_u16_be(p, 2, vel_u);
  put_u16_be(p, 4, kp_u);
  put_u16_be(p, 6, kd_u);

  return send_type_frame(1, motor_id, p);
}

// ---------- Feedback parsing ----------
struct MotorFeedback {
  uint8_t motor_id = 0;
  float pos_rad = 0;
  float vel_rads = 0;
  float torque_nm = 0;
  float temp_c = 0;
  uint32_t raw_id = 0;
};

bool parse_feedback_type2(const CAN_message_t &msg, MotorFeedback &fb) {
  // type is bits 28..24 in our encoding
  uint8_t type = (uint8_t)((msg.id >> 24) & 0x1F);
  if (type != 2) return false;
  if (msg.len < 8) return false;

  uint8_t mid = (uint8_t)(msg.id & 0xFF);

  uint16_t pos_u = get_u16_be(msg.buf, 0);
  uint16_t vel_u = get_u16_be(msg.buf, 2);
  uint16_t tq_u  = get_u16_be(msg.buf, 4);
  uint16_t tp_u  = get_u16_be(msg.buf, 6);

  fb.motor_id  = mid;
  fb.pos_rad   = u16_to_float(pos_u, POS_MIN, POS_MAX);
  fb.vel_rads  = u16_to_float(vel_u, VEL_MIN, VEL_MAX);
  fb.torque_nm = u16_to_float(tq_u,  TQ_MIN,  TQ_MAX);
  fb.temp_c    = ((float)tp_u) / 10.0f;
  fb.raw_id    = msg.id;

  return true;
}

// ---------- Demo motion ----------
static uint32_t last_send_us = 0;
static uint32_t last_print_ms = 0;

float demo_pos_command(float t_sec) {
  // simple sine within +/- 1 rad
  return 1.0f * sinf(2.0f * PI * 0.2f * t_sec);  // 0.2 Hz
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("RobStride RS02 Teensy CAN controller starting...");

  // Start CAN
  Can1.begin();
  Can1.setBaudRate(CAN_BAUD);

  // Optional: enable FIFO and accept all frames
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.enableFIFOInterrupt();   // not required; we will poll in loop()

  delay(200);

  Serial.println("Sending enable...");
  if (!enable_motor(MOTOR_ID)) {
    Serial.println("Enable send failed (CAN write). Check wiring/transceiver.");
  }
  delay(50);

  // Optionally set zero at startup (be careful: only do when safe)
  // Serial.println("Setting zero...");
  // set_zero(MOTOR_ID);
  // delay(50);

  Serial.println("Running. Send Type1 at fixed rate; reading Type2 feedback.");
  Serial.println("Commands:");
  Serial.println("  e = enable, s = stop, z = set zero");
}

void handle_serial_commands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'e') {
      Serial.println("Enable motor");
      enable_motor(MOTOR_ID);
    } else if (c == 's') {
      Serial.println("Stop motor");
      stop_motor(MOTOR_ID);
    } else if (c == 'z') {
      Serial.println("Set zero");
      set_zero(MOTOR_ID);
    }
  }
}

void loop() {
  handle_serial_commands();

  // Send control at SEND_HZ
  uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_send_us) >= SEND_PERIOD_US) {
    last_send_us = now_us;

    float t = millis() / 1000.0f;
    float pos = demo_pos_command(t);
    float vel = 0.0f;

    // Tune these carefully. Start small.
    float kp = 20.0f;
    float kd = 0.5f;

    send_operation_control(MOTOR_ID, pos, vel, kp, kd);
  }

  // Read all pending CAN frames (poll)
  CAN_message_t msg;
  while (Can1.read(msg)) {
    MotorFeedback fb;
    if (parse_feedback_type2(msg, fb)) {
      // Print at ~20 Hz
      if (millis() - last_print_ms >= 50) {
        last_print_ms = millis();

        Serial.print("ID=");
        Serial.print((int)fb.motor_id);
        Serial.print("  pos(rad)=");
        Serial.print(fb.pos_rad, 4);
        Serial.print("  vel(rad/s)=");
        Serial.print(fb.vel_rads, 4);
        Serial.print("  tq(Nm)=");
        Serial.print(fb.torque_nm, 3);
        Serial.print("  temp(C)=");
        Serial.println(fb.temp_c, 1);
      }
    }
  }
}
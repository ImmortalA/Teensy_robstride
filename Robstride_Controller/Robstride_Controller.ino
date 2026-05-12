// ================================================================
// 1. INCLUDES
// ================================================================

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <string.h>
#include "RobStrideMotor.h"

// ================================================================
// 2. USER CONFIG (EDIT HERE ONLY)
// ================================================================

static constexpr uint32_t CAN_BAUD = 1000000;
static constexpr uint8_t MASTER_ID = 0;

/** Comm idle watchdog ([WARN] wd). Set false for bench use / leaving motors idle without Serial traffic. */
static constexpr bool WDOG_EN = false;
/** Max gap without a *completed* command that calls armWatchdog(). Increase for slow Serial Monitor use. */
static constexpr uint32_t WDOG_MS = 5000;
/** While typing a line, refresh the deadline on each byte so slow typing does not trip the watchdog. */
static constexpr bool WDOG_EXTEND_ON_SERIAL_RX = false;

static constexpr bool STREAM_DEF = false;
static constexpr uint32_t STREAM_MS_DEF = 50;

static constexpr uint32_t FEEDBACK_TIMEOUT_MS = 500;
static constexpr uint32_t HEARTBEAT_MS = 0;
static constexpr bool SERIAL_CMD_ACK = true;

static constexpr uint32_t SYNC_STAGGER_US = 200;
static constexpr uint32_t FOR_EACH_GAP_US = 200;
static constexpr float JOG_MAX_TARGET_ERROR_RAD = 0.5f;

static constexpr uint8_t MOTOR_IDS[] = {1};
static constexpr RobStrideMotorModel MOTOR_MODELS[] = {RobStrideMotorModel::RS06};

// ================================================================
// 3. MOTOR OBJECTS / GLOBAL STATE
// ================================================================

constexpr int NUM_MOTORS = (int)(sizeof(MOTOR_IDS) / sizeof(MOTOR_IDS[0]));
static_assert(NUM_MOTORS > 0, "MOTOR_IDS");
static_assert((int)(sizeof(MOTOR_MODELS) / sizeof(MOTOR_MODELS[0])) == NUM_MOTORS, "models");

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
static RobStrideMotor motors[NUM_MOTORS];

typedef void (RobStrideMotor::*MotorFn)();

static uint32_t tCmd, tStream, tHeartbeat;
static bool estop = false;
static bool streamOn = STREAM_DEF;
static uint32_t streamMs = STREAM_MS_DEF;
static bool watchdogTripped = false;
static uint32_t last_throttle_log_drops;
static uint32_t lastLogThrottleMs;

static char lineBuf[192];
static int nbuf = 0;
static bool lineOverflow = false;

static bool jogOn = false;
static float jogVel = 0.0f;
static float jogTarget = 0.0f;
static float jogKp = 10.0f;
static float jogKd = 1.0f;
static uint32_t lastJogUs = 0;

// ================================================================
// 4. FUNCTION DECLARATIONS (PROTOTYPES)
// ================================================================

// --- Public sketch API
void initSerial();
void initCan();
void initMotors();
void readSerial();
void handleCommand(const String& line);
void serviceMotors();
void checkWatchdog();
void streamStatus();
void serviceJog();
void printHelp();

// --- Internal
void onCan(const CAN_message_t& msg);
void forEach(MotorFn fn);
void armWatchdog();
void refreshCommWatchdogDeadlineOnSerialRx();
void logThrottleDropsIfNeeded();
void printStartupBanner();
void printDiagnostics();
void validateNoDuplicateMotorIds();
void extraStagger();
void parseCommandLine(char* line);
void emitHeartbeatIfNeeded(uint32_t now);

static bool extraArgs(char** ctx);
static bool pu32(const char* s, uint32_t& o);
static bool pf(const char* s, float& o);
static bool tok5(char** ctx, float& p, float& v, float& kp, float& kd, float& tq);
static RobStrideMotor* motorById(uint8_t id);
static bool feedbackForCommand(RobStrideMotor* motor, Status& status);

// ================================================================
// 5. SETUP + LOOP (TOP, EASY TO SEE)
// ================================================================

void setup() {
  initSerial();
  validateNoDuplicateMotorIds();
  initCan();
  initMotors();
  printHelp();
  armWatchdog();
}

void loop() {
  Can0.events();

  const uint32_t now = millis();
  RobStrideMotor::pollAllHealth(motors, NUM_MOTORS, FEEDBACK_TIMEOUT_MS);
  serviceMotors();
  serviceJog();
  logThrottleDropsIfNeeded();
  readSerial();
  checkWatchdog();
  emitHeartbeatIfNeeded(now);
  streamStatus();
}

// ================================================================
// 6. IMPLEMENTATION (ALL FUNCTIONS BELOW)
// ================================================================

// --- initSerial
void initSerial() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
}

// --- initCan
void initCan() {
  Can0.begin();
  Can0.setBaudRate(CAN_BAUD);
  Can0.setMaxMB(16);
  Can0.enableMBInterrupts();
  Can0.onReceive(onCan);
}

// --- validate / init motors
void validateNoDuplicateMotorIds() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    for (int j = i + 1; j < NUM_MOTORS; j++) {
      if (MOTOR_IDS[i] == MOTOR_IDS[j]) {
        Serial.println(F("[ERR] duplicate MOTOR_IDS"));
        while (true) delay(1000);
      }
    }
  }
}

void printStartupBanner() {
  Serial.println(F("RobStride ready"));
  Serial.printf("NUM_MOTORS = %d\r\nIDs:", NUM_MOTORS);
  for (int i = 0; i < NUM_MOTORS; i++) Serial.printf(" %u", (unsigned)MOTOR_IDS[i]);
  Serial.println();
  Serial.println(F("Tip: type  help  — this sketch is configured for motor id 1."));
}

void initMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!motors[i].begin(MOTOR_IDS[i], MOTOR_MODELS[i])) {
      Serial.print(F("[ERR] motor init failed: "));
      Serial.println(i);
      while (true) delay(1000);
    }
    motors[i].setMasterId(MASTER_ID);
  }
  tCmd = millis();
  tStream = tCmd;
  tHeartbeat = tCmd;
  last_throttle_log_drops = 0;
  lastLogThrottleMs = tCmd;
  for (int i = 0; i < NUM_MOTORS; i++) last_throttle_log_drops += motors[i].throttleDropCount();
  printStartupBanner();
}

// --- printHelp
void printHelp() {
  Serial.println(F("=== RobStride one-motor control (id=1) ==="));
  Serial.println(F(""));
  Serial.println(F("--- Safe start ---"));
  Serial.println(F("  reset                    clear host estop"));
  Serial.println(F("  enable 1                 enable drive in operation-control mode"));
  Serial.println(F("  stream 1 100             print feedback every 100 ms"));
  Serial.println(F(""));
  Serial.println(F("--- Motion commands (all use working Type-1 path) ---"));
  Serial.println(F("  move 1 <rad> [kp] [kd]      absolute position, defaults kp=10 kd=1"));
  Serial.println(F("  nudge 1 <rad> [kp] [kd]     relative move from current feedback"));
  Serial.println(F("  jog 1 <rad/s> [kp] [kd]     continuous velocity, defaults kp=10 kd=1"));
  Serial.println(F("  jog 1 0                     stop jogging"));
  Serial.println(F("  torque 1 <Nm>               torque feedforward at current position"));
  Serial.println(F("  motion 1 <p> <v> <kp> <kd> <tq>   raw Type-1 command"));
  Serial.println(F(""));
  Serial.println(F("--- Good first tests ---"));
  Serial.println(F("  nudge 1 0.2 10 1"));
  Serial.println(F("  jog 1 0.5"));
  Serial.println(F("  jog 1 0"));
  Serial.println(F("  torque 1 0.3"));
  Serial.println(F(""));
  Serial.println(F("--- Stop / recovery ---"));
  Serial.println(F("  stop 1                    stop motor output"));
  Serial.println(F("  estop                     host estop + stop"));
  Serial.println(F("  reset                     clear host estop; run enable 1 after estop"));
  Serial.println(F("  recover 1                 host recover if fault flag cleared"));
  Serial.println(F(""));
  Serial.println(F("--- Feedback / diagnostics ---"));
  Serial.println(F("  status 1                  request one feedback frame"));
  Serial.println(F("  fault 1                   print decoded fault bits"));
  Serial.println(F("  diag                      print host state and counters"));
  Serial.println(F("  stream 0                  stop streaming"));
  Serial.println(F(""));
  Serial.println(F("Notes: old pos/vel/cur RAM-mode commands are intentionally removed."));
}

// --- CAN
void onCan(const CAN_message_t& msg) {
  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].decodeFeedback(msg)) return;
}

// --- service / forEach
void forEach(MotorFn fn) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    (motors[i].*fn)();
    delayMicroseconds(FOR_EACH_GAP_US);
  }
}

void serviceMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) motors[i].servicePending();
}

// --- Watchdog
void armWatchdog() {
  tCmd = millis();
  watchdogTripped = false;
}

void refreshCommWatchdogDeadlineOnSerialRx() {
  if (!WDOG_EXTEND_ON_SERIAL_RX || watchdogTripped) return;
  tCmd = millis();
}

void checkWatchdog() {
  const uint32_t now = millis();
  if (WDOG_EN && !estop && !watchdogTripped && (uint32_t)(now - tCmd) > WDOG_MS) {
    RobStrideMotor::setGlobalEstop(true);
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].onWatchdogStop();
    forEach(&RobStrideMotor::stop);
    watchdogTripped = true;
    Serial.println(F("[WARN] wd"));
  }
}

void emitHeartbeatIfNeeded(uint32_t now) {
  if (HEARTBEAT_MS > 0 && (uint32_t)(now - tHeartbeat) >= HEARTBEAT_MS) {
    tHeartbeat = now;
    Serial.print(F("[hb] g_estop="));
    Serial.print(RobStrideMotor::globalEstopActive() ? 1 : 0);
    Serial.print(F(" can_fail="));
    Serial.println((unsigned long)RobStrideMotor::canTxFailCount());
  }
}

// --- Stream
void streamStatus() {
  const uint32_t now = millis();
  if (streamOn && (uint32_t)(now - tStream) >= streamMs) {
    tStream = now;
    for (int i = 0; i < NUM_MOTORS; i++) {
      motors[i].requestStatus();
      extraStagger();
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
      Status s = motors[i].getStatus();
      uint32_t age_us = s.valid ? (uint32_t)(micros() - s.stamp_us) : 0u;
      Serial.printf("%u st=%u v=%u a_us=%lu p=%.4f vel=%.4f t=%.3f T=%.1f\r\n", (unsigned)motors[i].id(),
                    (unsigned)(uint8_t)motors[i].state(), s.valid ? 1u : 0u, (unsigned long)age_us, (double)s.position,
                    (double)s.velocity, (double)s.torque, (double)s.temperature_c);
    }
  }
}

void serviceJog() {
  if (!jogOn || estop || NUM_MOTORS != 1) return;

  uint32_t nowUs = micros();
  if (lastJogUs == 0) {
    lastJogUs = nowUs;
    return;
  }

  uint32_t dtUs = nowUs - lastJogUs;
  if (dtUs < 10000) return;
  lastJogUs = nowUs;

  float dt = (float)dtUs * 1.0e-6f;
  jogTarget += jogVel * dt;

  Status s = motors[0].getStatus();
  if (s.valid) {
    float err = jogTarget - s.position;
    if (err > JOG_MAX_TARGET_ERROR_RAD) jogTarget = s.position + JOG_MAX_TARGET_ERROR_RAD;
    else if (err < -JOG_MAX_TARGET_ERROR_RAD) jogTarget = s.position - JOG_MAX_TARGET_ERROR_RAD;
  }

  motors[0].motion(jogTarget, jogVel, jogKp, jogKd, 0.0f);
}

// --- Serial line input
void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    refreshCommWatchdogDeadlineOnSerialRx();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        Serial.println(F("[WARN] line_overflow"));
        lineOverflow = false;
      } else if (nbuf > 0) {
        lineBuf[nbuf] = 0;
        handleCommand(String(lineBuf));
      }
      nbuf = 0;
    } else if (nbuf < (int)sizeof(lineBuf) - 1) {
      lineBuf[nbuf++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void handleCommand(const String& line) {
  char work[192];
  line.toCharArray(work, (unsigned int)sizeof(work));
  parseCommandLine(work);
}

// --- Throttle + diag
void logThrottleDropsIfNeeded() {
  uint32_t total = 0;
  for (int i = 0; i < NUM_MOTORS; i++) total += motors[i].throttleDropCount();
  if (total < last_throttle_log_drops + 32) return;
  uint32_t m = millis();
  if ((uint32_t)(m - lastLogThrottleMs) < 2000) return;
  lastLogThrottleMs = m;
  last_throttle_log_drops = total;
  Serial.print(F("[WARN] throttle_drops>="));
  Serial.println((unsigned long)total);
}

void printDiagnostics() {
  Serial.print(F("[diag] global_estop="));
  Serial.print(RobStrideMotor::globalEstopActive() ? 1 : 0);
  Serial.print(F(" can_tx_fail="));
  Serial.println((unsigned long)RobStrideMotor::canTxFailCount());
  for (int i = 0; i < NUM_MOTORS; i++) {
    MotorRuntimeMetrics m;
    motors[i].getRuntimeMetrics(m);
    Serial.print(F("[diag] id="));
    Serial.print((unsigned)motors[i].id());
    Serial.print(F(" st="));
    Serial.print((unsigned)(uint8_t)motors[i].state());
    Serial.print(F(" td="));
    Serial.print((unsigned long)m.throttle_drops);
    Serial.print(F(" tco="));
    Serial.print((unsigned long)m.throttle_coalesced);
    Serial.print(F(" sblk="));
    Serial.print((unsigned long)m.state_blocked_cmds);
    Serial.print(F(" eblk="));
    Serial.print((unsigned long)m.estop_blocked_sends);
    Serial.print(F(" ni="));
    Serial.print((unsigned long)m.not_inited_cmds);
    Serial.print(F(" inv="));
    Serial.print((unsigned long)m.invalid_float_in);
    Status s = motors[i].getStatus();
    Serial.print(F(" fb="));
    Serial.print(s.valid ? 1 : 0);
    Serial.print(F(" p="));
    Serial.print(s.position, 4);
    Serial.print(F(" vel="));
    Serial.println(s.velocity, 4);
  }
}

// --- Parsers
static bool extraArgs(char** ctx) { return strtok_r(nullptr, " \t\r\n", ctx) != nullptr; }

static bool pu32(const char* s, uint32_t& o) {
  if (!s || !*s) return false;
  char* e = nullptr;
  unsigned long v = strtoul(s, &e, 0);
  if (e == s) return false;
  o = (uint32_t)v;
  return true;
}

static bool pf(const char* s, float& o) {
  if (!s || !*s) return false;
  char* e = nullptr;
  o = strtof(s, &e);
  return (e != s) && isfinite(o);
}

static bool tok5(char** ctx, float& p, float& v, float& kp, float& kd, float& tq) {
  char *a, *b, *c, *d, *e;
  a = strtok_r(nullptr, " \t\r\n", ctx);
  b = strtok_r(nullptr, " \t\r\n", ctx);
  c = strtok_r(nullptr, " \t\r\n", ctx);
  d = strtok_r(nullptr, " \t\r\n", ctx);
  e = strtok_r(nullptr, " \t\r\n", ctx);
  return a && b && c && d && e && pf(a, p) && pf(b, v) && pf(c, kp) && pf(d, kd) && pf(e, tq);
}

static RobStrideMotor* motorById(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].id() == id) return &motors[i];
  return nullptr;
}

static bool feedbackForCommand(RobStrideMotor* motor, Status& status) {
  status = motor->getStatus();
  if (status.valid) return true;

  motor->requestStatus();
  delay(10);
  Can0.events();

  status = motor->getStatus();
  return status.valid || status.stamp_us != 0;
}

void extraStagger() { delayMicroseconds(SYNC_STAGGER_US); }

// --- Command line (strtok) — same syntax as before
void parseCommandLine(char* line) {
  char* ctx = nullptr;
  char* cmd = strtok_r(line, " \t\r\n", &ctx);
  if (!cmd) return;

  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    printHelp();
    return;
  }

  if (!strcmp(cmd, "diag")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    printDiagnostics();
    return;
  }

  if (!strcmp(cmd, "reset") || !strcmp(cmd, "estop_reset")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    jogOn = false;
    estop = false;
    RobStrideMotor::setGlobalEstop(false);
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].onHostEstopRelease();
    Serial.println(F("reset host; run enable 1 after estop"));
    return;
  }

  if (!strcmp(cmd, "estop")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    jogOn = false;
    estop = true;
    RobStrideMotor::setGlobalEstop(true);
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].onHostEstop();
    forEach(&RobStrideMotor::stop);
    Serial.println(F("estop"));
    return;
  }

  if (!strcmp(cmd, "stream")) {
    char* a = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!a) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    uint32_t on;
    if (!pu32(a, on)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    streamOn = (on != 0);
    char* b = strtok_r(nullptr, " \t\r\n", &ctx);
    if (b) {
      uint32_t ms;
      if (!pu32(b, ms) || ms == 0) {
        Serial.println(F("[ERR] bad_args"));
        return;
      }
      streamMs = ms;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    return;
  }

  char* ids = strtok_r(nullptr, " \t\r\n", &ctx);
  if (!ids) {
    Serial.println(F("[ERR] bad_args (missing motor id or extra words?)"));
    Serial.println(F("hint: this sketch is configured for one motor: use id 1, e.g. enable 1"));
    return;
  }
  uint32_t idu;
  if (!pu32(ids, idu)) {
    Serial.println(F("[ERR] bad_args"));
    return;
  }
  RobStrideMotor* m = motorById((uint8_t)idu);
  if (!m) {
    Serial.println(F("[ERR] unknown id"));
    return;
  }

  if (!strcmp(cmd, "recover")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (m->tryRecoverFaultToReady()) Serial.println(F("recover ok"));
    else Serial.println(F("recover n/a"));
    return;
  }

  if (!strcmp(cmd, "enable")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (!estop) m->enable();
    if (!estop) armWatchdog();
    return;
  }
  if (!strcmp(cmd, "stop")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    jogOn = false;
    m->stop();
    armWatchdog();
    return;
  }
  if (!strcmp(cmd, "zero")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (!estop) m->zero();
    if (!estop) armWatchdog();
    return;
  }

  if (!strcmp(cmd, "move")) {
    char* ptxt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!ptxt) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    float p;
    if (!pf(ptxt, p)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    float kp = 10.0f;
    float kd = 1.0f;
    char* kpt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kpt && !pf(kpt, kp)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    char* kdt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kdt && !pf(kdt, kd)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;
    jogOn = false;
    m->motion(p, 0.0f, kp, kd, 0.0f);
    m->servicePending();
    armWatchdog();
    if (SERIAL_CMD_ACK) {
      Serial.print(F("[OK] move target="));
      Serial.println(p, 5);
    }
    return;
  }

  if (!strcmp(cmd, "torque")) {
    char* t = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!t) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    float tq;
    if (!pf(t, tq)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;

    Status s;
    if (!feedbackForCommand(m, s)) {
      Serial.println(F("[ERR] no_feedback; run stream 1 100 or status 1, then retry"));
      return;
    }

    jogOn = false;
    m->motion(s.position, 0.0f, 0.0f, 0.0f, tq);
    m->servicePending();
    armWatchdog();
    if (SERIAL_CMD_ACK) {
      Serial.print(F("[OK] torque Nm="));
      Serial.println(tq, 5);
    }
    return;
  }

  if (!strcmp(cmd, "jog")) {
    char* v = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!v) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    float vel;
    if (!pf(v, vel)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    float kp = 10.0f;
    float kd = 1.0f;
    char* kpt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kpt && !pf(kpt, kp)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    char* kdt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kdt && !pf(kdt, kd)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;

    if (fabsf(vel) < 1.0e-6f) {
      jogOn = false;
      m->motion(jogTarget, 0.0f, kp, kd, 0.0f);
      m->servicePending();
      armWatchdog();
      if (SERIAL_CMD_ACK) Serial.println(F("[OK] jog stop"));
      return;
    }

    Status s;
    if (!feedbackForCommand(m, s)) {
      Serial.println(F("[ERR] no_feedback; run stream 1 100 or status 1, then retry"));
      return;
    }

    jogVel = vel;
    jogTarget = s.position;
    jogKp = kp;
    jogKd = kd;
    lastJogUs = micros();
    jogOn = true;
    armWatchdog();
    if (SERIAL_CMD_ACK) {
      Serial.print(F("[OK] jog vel="));
      Serial.println(jogVel, 5);
    }
    return;
  }

  if (!strcmp(cmd, "nudge")) {
    char* d = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!d) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    float delta;
    if (!pf(d, delta)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    float kp = 10.0f;
    float kd = 1.0f;
    char* kpt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kpt && !pf(kpt, kp)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    char* kdt = strtok_r(nullptr, " \t\r\n", &ctx);
    if (kdt && !pf(kdt, kd)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;

    Status s;
    if (!feedbackForCommand(m, s)) {
      Serial.println(F("[ERR] no_feedback; run stream 1 100 or status 1, then retry"));
      return;
    }

    jogOn = false;
    m->motion(s.position + delta, 0.0f, kp, kd, 0.0f);
    m->servicePending();
    armWatchdog();
    if (SERIAL_CMD_ACK) {
      Serial.print(F("[OK] nudge target="));
      Serial.println(s.position + delta, 5);
    }
    return;
  }

  if (!strcmp(cmd, "motion")) {
    float p, v, kp, kd, tq;
    if (!tok5(&ctx, p, v, kp, kd, tq)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;
    jogOn = false;
    m->motion(p, v, kp, kd, tq);
    m->servicePending();
    armWatchdog();
    if (SERIAL_CMD_ACK) Serial.println(F("[OK] motion"));
    return;
  }

  if (!strcmp(cmd, "status")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    m->requestStatus();
    return;
  }

  if (!strcmp(cmd, "fault")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    FaultFlags f = m->getFault();
    Serial.printf("%u ms=%u f=%u%u%u%u%u%u st=%u\r\n", (unsigned)m->id(), (unsigned)m->getModeStatus(),
                  f.has_fault, f.encoder_not_calibrated, f.gridlock_overload, f.magnetic_coding,
                  f.overtemperature, f.three_phase_overcurrent, (unsigned)(uint8_t)m->state());
    return;
  }

  Serial.println(F("[ERR] unknown_cmd"));
}

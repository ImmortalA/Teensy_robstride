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

static constexpr uint8_t MOTOR_IDS[] = {1, 2};
static constexpr RobStrideMotorModel MOTOR_MODELS[] = {
    RobStrideMotorModel::RS06, RobStrideMotorModel::RS06};

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
static int8_t parseMode(const char* s);
static bool tok5(char** ctx, float& p, float& v, float& kp, float& kd, float& tq);
static bool allFiniteSyncScalars(const float* vals, int count);
static RobStrideMotor* motorById(uint8_t id);

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
  Can0.enableFIFO();
  Can0.setClock(CLK_60MHz);
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
  Serial.println(F("Tip: type  help  — per-motor commands need an id (e.g.  enable 1  or  all enable )."));
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
  Serial.println(F("=== RobStride serial commands ==="));
  Serial.println(F(""));
  Serial.println(F("--- Quick start (one motor, id=1) ---"));
  Serial.println(F("  reset"));
  Serial.println(F("  enable 1"));
  Serial.println(F("  mode 1 velocity          <- word must be: velocity | position | current | motion"));
  Serial.println(F("  vel 1 0.5                <- setpoint in rad/s (not RPM)"));
  Serial.println(F("  stream 1 100              <- optional: prints feedback incl. velocity"));
  Serial.println(F(""));
  Serial.println(F("--- Velocity (per motor) ---"));
  Serial.println(F("  1) enable <id>     2) mode <id> velocity     3) vel <id> <rad/s>"));
  Serial.println(F("  Use full word  velocity  in mode (  mode 1 vel  is wrong)."));
  Serial.println(F("  Stop:  vel <id> 0   or   stop <id>"));
  Serial.println(F(""));
  Serial.println(F("--- Velocity (all motors, same order as MOTOR_IDS[]) ---"));
  Serial.println(F("  all enable"));
  Serial.println(F("  mode 1 velocity          (repeat for each id if needed)"));
  Serial.println(F("  sync vel <m0> <m1> ...     one rad/s per motor, e.g. two motors:  sync vel 0.2 -0.2"));
  Serial.println(F(""));
  Serial.println(F("--- Position / torque (per motor) ---"));
  Serial.println(F("  mode 1 position          then  pos 1 <rad>"));
  Serial.println(F("  mode 1 current           then  cur 1 <A>"));
  Serial.println(F("  mode 1 motion            then  motion 1 <p> <v> <kp> <kd> <tq>"));
  Serial.println(F(""));
  Serial.println(F("--- Per-motor (replace 1 with CAN id from MOTOR_IDS) ---"));
  Serial.println(F("  enable 1   stop 1   zero 1   status 1   fault 1   recover 1"));
  Serial.println(F(""));
  Serial.println(F("--- All motors (no id on these lines) ---"));
  Serial.println(F("  all enable   all stop   all zero   all status"));
  Serial.println(F(""));
  Serial.println(F("--- Other ---"));
  Serial.println(F("  sync pos|vel|cur <values...>     sync motion <5 floats per motor...>"));
  Serial.println(F("  stream 0|1 [ms]   diag   estop   reset | estop_reset   help"));
  Serial.println(F(""));
  Serial.println(F("If estop or [WARN] wd:  reset  then  enable  again."));
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
    Serial.println((unsigned long)m.invalid_float_in);
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

static int8_t parseMode(const char* s) {
  if (!strcmp(s, "motion")) return 0;
  if (!strcmp(s, "position")) return 1;
  if (!strcmp(s, "velocity")) return 2;
  if (!strcmp(s, "current")) return 3;
  return -1;
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

static bool allFiniteSyncScalars(const float* vals, int count) {
  for (int i = 0; i < count; i++)
    if (!isfinite(vals[i])) return false;
  return true;
}

static RobStrideMotor* motorById(uint8_t id) {
  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].id() == id) return &motors[i];
  return nullptr;
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
    estop = false;
    RobStrideMotor::setGlobalEstop(false);
    for (int i = 0; i < NUM_MOTORS; i++) motors[i].onHostEstopRelease();
    Serial.println(F("reset host"));
    return;
  }

  if (!strcmp(cmd, "estop")) {
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
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

  if (!strcmp(cmd, "all")) {
    char* a = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!a) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (!strcmp(a, "enable")) {
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] bad_args"));
        return;
      }
      if (!estop) {
        for (int i = 0; i < NUM_MOTORS; i++) {
          motors[i].enable();
          delayMicroseconds(FOR_EACH_GAP_US);
        }
        armWatchdog();
      }
      return;
    }
    if (!strcmp(a, "stop")) {
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] bad_args"));
        return;
      }
      forEach(&RobStrideMotor::stop);
      armWatchdog();
      return;
    }
    if (!strcmp(a, "zero")) {
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] bad_args"));
        return;
      }
      if (!estop) {
        for (int i = 0; i < NUM_MOTORS; i++) {
          motors[i].zero();
          extraStagger();
        }
        armWatchdog();
      }
      return;
    }
    if (!strcmp(a, "status")) {
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] bad_args"));
        return;
      }
      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].requestStatus();
        extraStagger();
      }
      return;
    }
    Serial.println(F("[ERR] bad_cmd"));
    return;
  }

  if (!strcmp(cmd, "sync")) {
    char* sub = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!sub) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;

    if (!strcmp(sub, "pos") || !strcmp(sub, "vel") || !strcmp(sub, "cur")) {
      float vals[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) {
        char* t = strtok_r(nullptr, " \t\r\n", &ctx);
        if (!t || !pf(t, vals[i])) {
          Serial.println(F("[ERR] sync_err"));
          return;
        }
      }
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] sync_extra"));
        return;
      }
      if (!allFiniteSyncScalars(vals, NUM_MOTORS)) {
        Serial.println(F("[ERR] invalid number"));
        return;
      }
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (!strcmp(sub, "pos")) motors[i].setPosition(vals[i]);
        else if (!strcmp(sub, "vel")) motors[i].setVelocity(vals[i]);
        else motors[i].setCurrent(vals[i]);
        extraStagger();
      }
      serviceMotors();
      armWatchdog();
      return;
    }

    if (!strcmp(sub, "motion")) {
      float p[NUM_MOTORS], v[NUM_MOTORS], kp[NUM_MOTORS], kd[NUM_MOTORS], tq[NUM_MOTORS];
      for (int i = 0; i < NUM_MOTORS; i++) {
        if (!tok5(&ctx, p[i], v[i], kp[i], kd[i], tq[i])) {
          Serial.println(F("[ERR] sync_err"));
          return;
        }
      }
      if (extraArgs(&ctx)) {
        Serial.println(F("[ERR] sync_extra"));
        return;
      }
      if (!allFiniteSyncScalars(p, NUM_MOTORS) || !allFiniteSyncScalars(v, NUM_MOTORS) || !allFiniteSyncScalars(kp, NUM_MOTORS) ||
          !allFiniteSyncScalars(kd, NUM_MOTORS) || !allFiniteSyncScalars(tq, NUM_MOTORS)) {
        Serial.println(F("[ERR] invalid number"));
        return;
      }
      for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].motion(p[i], v[i], kp[i], kd[i], tq[i]);
        extraStagger();
      }
      serviceMotors();
      armWatchdog();
      return;
    }

    Serial.println(F("[ERR] bad_cmd"));
    return;
  }

  char* ids = strtok_r(nullptr, " \t\r\n", &ctx);
  if (!ids) {
    Serial.println(F("[ERR] bad_args (missing motor id or extra words?)"));
    Serial.println(F("hint:  enable 1   not just \"enable\" — id is the drive CAN node (see MOTOR_IDS)."));
    Serial.println(F("       or use:  all enable   for every motor."));
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

  if (!strcmp(cmd, "mode")) {
    char* mo = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!mo) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    int8_t md = parseMode(mo);
    if (md < 0) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (!estop) m->setRunMode((RunMode)md);
    if (!estop) armWatchdog();
    return;
  }

  if (!strcmp(cmd, "pos") || !strcmp(cmd, "vel") || !strcmp(cmd, "cur")) {
    char* t = strtok_r(nullptr, " \t\r\n", &ctx);
    if (!t) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    float x;
    if (!pf(t, x)) {
      Serial.println(F("[ERR] invalid number"));
      return;
    }
    if (extraArgs(&ctx)) {
      Serial.println(F("[ERR] bad_args"));
      return;
    }
    if (estop) return;
    if (!strcmp(cmd, "pos")) m->setPosition(x);
    else if (!strcmp(cmd, "vel")) m->setVelocity(x);
    else m->setCurrent(x);
    m->servicePending();
    armWatchdog();
    if (SERIAL_CMD_ACK) {
      Serial.print(F("[OK] "));
      Serial.print(cmd);
      Serial.print(' ');
      Serial.print((unsigned)idu);
      Serial.print(' ');
      Serial.println(x, 5);
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

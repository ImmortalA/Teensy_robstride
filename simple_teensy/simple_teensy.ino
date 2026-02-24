/*
 * simple_teensy.ino — Send motor control directly to Robostride O2 via CAN.
 * No Ethernet, no PC. Teensy → CAN → motor only. Serial (115200) optional for feedback print.
 *
 * 1) setup(): init CAN, then send Exit → Zero → Enter motor mode to the motor.
 * 2) loop(): every 10 ms send Type‑1 position control frames (p_des, v_des, Kp, Kd).
 *
 * Debug-friendly: we send on BOTH CAN ports (CAN1 & CAN2) and accept ANY O2 feedback
 * from any motor ID, so we can see if *anything* is talking on the bus.
 *
 * Hardware: Teensy 4.1, CAN1 & CAN2 pins. Protocol: O2 Private, 1 Mbps, extended ID (mode<<24)|motor_id.
 */
#include <FlexCAN_T4.h>

// Try these IDs; many O2 motors default to 0 or 1.
static const uint8_t kTestMotorIds[] = {0, 1};
static const int kNumTestMotorIds = 2;

// O2 CAN types (bits 28-24 of extended ID)
#define O2_TYPE_CTRL          1
#define O2_TYPE_FEEDBACK      2
#define O2_TYPE_ENTER         3
#define O2_TYPE_EXIT          4
#define O2_TYPE_ZERO          6

// O2 limits (from robstride_usb_to_can_python-main/mapping.py)
// Angle:  -4π .. +4π   → 0 .. 65535
// Vel:    -50 .. +50   → 0 .. 65535
// Torque: -6  .. +6 Nm → 0 .. 65535 (used only for feedback here)
#define P_MIN                 (-4.0f * 3.14159265f)
#define P_MAX                 ( 4.0f * 3.14159265f)
#define V_MIN                 (-50.0f)
#define V_MAX                 ( 50.0f)
#define KP_MIN                (0.0f)
#define KP_MAX                (500.0f)
#define KD_MIN                (0.0f)
#define KD_MAX                (5.0f)

// Two CAN ports: CAN1 and CAN2 (Teensy 4.1 has both).
FlexCAN_T4<CAN1, RX_SIZE_16, TX_SIZE_4> Can0;  // First CAN port
FlexCAN_T4<CAN2, RX_SIZE_16, TX_SIZE_4> Can1;  // Second CAN port (same as "Can1" in main project)

uint8_t tx_buf[8];
uint8_t rx_buf[8];
volatile bool rx_ready = false;
volatile bool fb_seen = false;
volatile uint8_t fb_motor_id = 0;
volatile uint8_t fb_bus = 0; // 0 = CAN0, 1 = CAN1
volatile uint32_t fb_ms = 0;

// Process one received CAN message (call from loop or from onReceive callbacks).
static void process_rx(const CAN_message_t& msg, uint8_t bus) {
    if (msg.len != 8 || !msg.flags.extended) return;
    uint8_t type = (msg.id >> 24) & 0x1F;
    if (type != O2_TYPE_FEEDBACK) return;
    uint8_t mid = msg.id & 0xFF;
    memcpy(rx_buf, msg.buf, 8);
    rx_ready = true;
    fb_seen = true;
    fb_motor_id = mid;
    fb_bus = bus;
    fb_ms = millis();
}

static uint32_t make_id(uint8_t motor_id, uint8_t type) {
    return ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)(motor_id & 0xFF));
}

static int float_to_uint16(float x, float x_min, float x_max) {
    float t = (x - x_min) / (x_max - x_min);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return (int)(t * 65535.0f + 0.5f);
}

static float uint16_to_float(int u, float x_min, float x_max) {
    return x_min + (x_max - x_min) * ((float)u / 65535.0f);
}

// Pack O2 Type 1 control: 4×16-bit (p, v, Kp, Kd), high byte first.
static void pack_control(float p_des, float v_des, float kp, float kd, uint8_t* buf) {
    int p_int = float_to_uint16(p_des, P_MIN, P_MAX);
    int v_int = float_to_uint16(v_des, V_MIN, V_MAX);
    int kp_int = float_to_uint16(kp, KP_MIN, KP_MAX);
    int kd_int = float_to_uint16(kd, KD_MIN, KD_MAX);
    buf[0] = (p_int >> 8) & 0xFF;
    buf[1] = p_int & 0xFF;
    buf[2] = (v_int >> 8) & 0xFF;
    buf[3] = v_int & 0xFF;
    buf[4] = (kp_int >> 8) & 0xFF;
    buf[5] = kp_int & 0xFF;
    buf[6] = (kd_int >> 8) & 0xFF;
    buf[7] = kd_int & 0xFF;
}

// Send one frame on CAN0 (first bus).
static void send_frame0(uint32_t id, const uint8_t* buf) {
    CAN_message_t msg;
    msg.id = id;
    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(msg.buf, buf, 8);
    Can0.write(msg);
}

// Send one frame on CAN1 (second bus).
static void send_frame1(uint32_t id, const uint8_t* buf) {
    CAN_message_t msg;
    msg.id = id;
    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(msg.buf, buf, 8);
    Can1.write(msg);
}

void onCanReceive0(const CAN_message_t& msg) {
    process_rx(msg, 0);
}

void onCanReceive1(const CAN_message_t& msg) {
    process_rx(msg, 1);
}

static void poll_can_buses() {
    CAN_message_t msg;
    while (Can0.read(msg)) process_rx(msg, 0);
    while (Can1.read(msg)) process_rx(msg, 1);
}

static bool wait_feedback_match(uint8_t expected_bus, uint8_t expected_id, uint32_t timeout_ms) {
    uint32_t t0 = millis();
    while ((millis() - t0) < timeout_ms) {
        poll_can_buses();
        if (fb_seen && fb_bus == expected_bus && fb_motor_id == expected_id) {
            return true;
        }
        delay(2);
    }
    return false;
}

// Actively probe if a motor is alive on each bus/id pair.
static void probe_motor_alive() {
    Serial.println("=== Motor alive probe start ===");
    bool found_any = false;
    for (uint8_t bus = 0; bus < 2; ++bus) {
        for (int k = 0; k < kNumTestMotorIds; ++k) {
            uint8_t mid = kTestMotorIds[k];
            uint32_t id_enter = make_id(mid, O2_TYPE_ENTER);
            uint32_t id_ctrl = make_id(mid, O2_TYPE_CTRL);

            // Send enter once
            memset(tx_buf, 0xFF, 7);
            tx_buf[7] = 0xFC;
            if (bus == 0) send_frame0(id_enter, tx_buf);
            else send_frame1(id_enter, tx_buf);
            delay(20);

            // Send a few control frames and wait for matching feedback
            pack_control(0.3f, 0.0f, 80.0f, 2.0f, tx_buf);
            for (int i = 0; i < 5; ++i) {
                if (bus == 0) send_frame0(id_ctrl, tx_buf);
                else send_frame1(id_ctrl, tx_buf);
                delay(5);
            }

            if (wait_feedback_match(bus, mid, 250)) {
                Serial.printf("ALIVE: bus=%u id=%u (feedback seen)\n", bus, mid);
                found_any = true;
            } else {
                Serial.printf("NO FB: bus=%u id=%u\n", bus, mid);
            }
        }
    }
    if (!found_any) {
        Serial.println("Probe result: no motor feedback found on tested bus/id pairs.");
    }
    Serial.println("=== Motor alive probe end ===");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // Optional: wait for Serial for debug (comment out to run without Serial Monitor)
    // while (!Serial && millis() < 3000) {}

    Serial.println("Simple Teensy - motor control direct via CAN (both CAN1 & CAN2)");

    // Setup CAN0 (first bus)
    Can0.begin();
    Can0.setBaudRate(1000000);
    Can0.setMaxMB(8);
    Can0.setMBFilter(MB1, 0, 0x0); // accept all IDs
    Can0.enhanceFilter(MB1);
    Can0.distribute();
    Can0.enableMBInterrupts();
    Can0.onReceive(onCanReceive0);
    Can0.setClock(CLK_60MHz);

    // Setup CAN1 (second bus)
    Can1.begin();
    Can1.setBaudRate(1000000);
    Can1.setMaxMB(8);
    Can1.setMBFilter(MB1, 0, 0x0); // accept all IDs
    Can1.enhanceFilter(MB1);
    Can1.distribute();
    Can1.enableMBInterrupts();
    Can1.onReceive(onCanReceive1);
    Can1.setClock(CLK_60MHz);

    Can0.mailboxStatus();
    Can1.mailboxStatus();
    Serial.println("CAN0 & CAN1 @ 1 Mbps, extended RX");

    // ---- Init sequence on both buses, for both candidate IDs ----
    Serial.println("Exit motor mode (all IDs, both buses)...");
    memset(tx_buf, 0xFF, 7);
    tx_buf[7] = 0xFD;
    for (int k = 0; k < kNumTestMotorIds; ++k) {
        uint8_t mid = kTestMotorIds[k];
        uint32_t id = make_id(mid, O2_TYPE_EXIT);
        send_frame0(id, tx_buf);
        send_frame1(id, tx_buf);
    }
    delay(1000);

    Serial.println("Zero encoder (all IDs, both buses)...");
    tx_buf[7] = 0xFE;
    for (int k = 0; k < kNumTestMotorIds; ++k) {
        uint8_t mid = kTestMotorIds[k];
        uint32_t id = make_id(mid, O2_TYPE_ZERO);
        send_frame0(id, tx_buf);
        send_frame1(id, tx_buf);
    }
    delay(1000);

    Serial.println("Enter motor mode (all IDs, both buses)...");
    tx_buf[7] = 0xFC;
    for (int k = 0; k < kNumTestMotorIds; ++k) {
        uint8_t mid = kTestMotorIds[k];
        uint32_t id = make_id(mid, O2_TYPE_ENTER);
        send_frame0(id, tx_buf);
        send_frame1(id, tx_buf);
    }
    delay(500);

    Serial.println("Ready. Sending position commands + printing feedback.");
    probe_motor_alive();
}

void loop() {
    static uint32_t iter = 0;
    const float dt = 0.01f;  // 10 ms loop

    // ----- Send motor control directly (one Type-1 frame per loop) -----
    float p_des = 0.2f * sin(0.3f * iter * dt * 2.0f * 3.14159f);  // slow sine ±0.2 rad
    // float p_des = 0.5f;   // or constant position
    float v_des = 0.0f;
    float kp = 80.0f;
    float kd = 2.0f;
    pack_control(p_des, v_des, kp, kd, tx_buf);
    // Send same command to all candidate IDs on both buses
    for (int k = 0; k < kNumTestMotorIds; ++k) {
        uint8_t mid = kTestMotorIds[k];
        uint32_t id = make_id(mid, O2_TYPE_CTRL);
        send_frame0(id, tx_buf);
        send_frame1(id, tx_buf);
    }

    // Poll for feedback (in case interrupt path doesn't fire) on both buses
    poll_can_buses();

    if (rx_ready) {
        rx_ready = false;
        int p_int = (rx_buf[0] << 8) | rx_buf[1];
        int v_int = (rx_buf[2] << 8) | rx_buf[3];
        int t_int = (rx_buf[4] << 8) | rx_buf[5];
        int temp_int = (rx_buf[6] << 8) | rx_buf[7];
        float p = uint16_to_float(p_int, P_MIN, P_MAX);
        float v = uint16_to_float(v_int, V_MIN, V_MAX);
        // Torque range per mapping.py: -6 .. +6 Nm
        float t = uint16_to_float(t_int, -6.0f, 6.0f);
        float temp_c = (float)temp_int / 10.0f;
        Serial.printf("FB: p=%.3f v=%.3f t=%.3f T=%.1fC\n", p, v, t, temp_c);
    }

    iter++;
    delay(10);
}

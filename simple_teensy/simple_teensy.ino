/*
 * simple_teensy.ino — Send motor control directly to Robostride O2 via CAN.
 * No Ethernet, no PC. Teensy → CAN → motor only. Serial (115200) optional for feedback print.
 *
 * 1) setup(): init CAN, then send Exit → Zero → Enter motor mode to the motor.
 * 2) loop(): every 10 ms send one Type-1 position control frame (p_des, v_des, Kp, Kd) to the motor.
 *
 * Hardware: Teensy 4.1, CAN2 pins. Protocol: O2 Private, 1 Mbps, extended ID (mode<<24)|motor_id.
 */
#include <FlexCAN_T4.h>

// Motor CAN ID: 0 or 1 to match motor (try 1 if motor does not move).
#define MOTOR_CAN_ID          1

// O2 CAN types (bits 28-24 of extended ID)
#define O2_TYPE_CTRL          1
#define O2_TYPE_FEEDBACK      2
#define O2_TYPE_ENTER         3
#define O2_TYPE_EXIT          4
#define O2_TYPE_ZERO          6

// O2 limits (manual)
#define P_MIN                 (-12.5f)
#define P_MAX                 (12.5f)
#define V_MIN                 (-45.0f)
#define V_MAX                 (45.0f)
#define KP_MIN                (0.0f)
#define KP_MAX                (500.0f)
#define KD_MIN                (0.0f)
#define KD_MAX                (5.0f)

FlexCAN_T4<CAN2, RX_SIZE_16, TX_SIZE_4> Can1;  // Second CAN port (same as "Can1" in main project)

uint8_t tx_buf[8];
uint8_t rx_buf[8];
volatile bool rx_ready = false;

// Process one received CAN message (call from loop or from onCanReceive).
static void process_rx(const CAN_message_t& msg) {
    if (msg.len != 8 || !msg.flags.extended) return;
    uint8_t type = (msg.id >> 24) & 0x1F;
    if (type != O2_TYPE_FEEDBACK) return;
    uint8_t mid = msg.id & 0xFF;
    if (mid != MOTOR_CAN_ID) return;
    memcpy(rx_buf, msg.buf, 8);
    rx_ready = true;
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

static void send_frame(uint32_t id, const uint8_t* buf) {
    CAN_message_t msg;
    msg.id = id;
    msg.flags.extended = 1;
    msg.len = 8;
    memcpy(msg.buf, buf, 8);
    Can1.write(msg);
}

void onCanReceive(const CAN_message_t& msg) {
    process_rx(msg);
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    // Optional: wait for Serial for debug (comment out to run without Serial Monitor)
    // while (!Serial && millis() < 3000) {}

    Serial.println("Simple Teensy - motor control direct via CAN");
    Can1.begin();
    Can1.setBaudRate(1000000);
    Can1.setMaxMB(8);
    Can1.setMBFilter(MB1, 0, 0x0); // accept all IDs (extended)
    Can1.enhanceFilter(MB1);
    Can1.distribute();
    Can1.enableMBInterrupts();
    Can1.onReceive(onCanReceive);
    Can1.setClock(CLK_60MHz);
    Can1.mailboxStatus();
    Serial.println("CAN1 (CAN2 port) @ 1 Mbps, extended RX");

    // ---- Init sequence ----
    Serial.println("Exit motor mode...");
    memset(tx_buf, 0xFF, 7);
    tx_buf[7] = 0xFD;
    send_frame(make_id(MOTOR_CAN_ID, O2_TYPE_EXIT), tx_buf);
    delay(1000);

    Serial.println("Zero encoder...");
    tx_buf[7] = 0xFE;
    send_frame(make_id(MOTOR_CAN_ID, O2_TYPE_ZERO), tx_buf);
    delay(1000);

    Serial.println("Enter motor mode...");
    tx_buf[7] = 0xFC;
    send_frame(make_id(MOTOR_CAN_ID, O2_TYPE_ENTER), tx_buf);
    delay(500);

    Serial.println("Ready. Sending position commands + printing feedback.");
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
    send_frame(make_id(MOTOR_CAN_ID, O2_TYPE_CTRL), tx_buf);

    // Poll for feedback (in case interrupt path doesn't fire)
    CAN_message_t msg;
    while (Can1.read(msg)) process_rx(msg);

    if (rx_ready) {
        rx_ready = false;
        int p_int = (rx_buf[0] << 8) | rx_buf[1];
        int v_int = (rx_buf[2] << 8) | rx_buf[3];
        int t_int = (rx_buf[4] << 8) | rx_buf[5];
        int temp_int = (rx_buf[6] << 8) | rx_buf[7];
        float p = uint16_to_float(p_int, P_MIN, P_MAX);
        float v = uint16_to_float(v_int, V_MIN, V_MAX);
        float t = uint16_to_float(t_int, -17.0f, 17.0f);
        float temp_c = (float)temp_int / 10.0f;
        Serial.printf("p=%.3f v=%.3f t=%.3f T=%.1fC\n", p, v, t, temp_c);
    }

    iter++;
    delay(10);
}

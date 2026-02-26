/*
 * Simple Teensy: enable one motor only (RobStride O2, 29-bit extended CAN).
 * Sends: stop (Type 4) -> zero (Type 6) -> enable (Type 3) on both buses.
 * Set MOTOR_ID to your motor's CAN ID (e.g. 0 or 1).
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

#define MOTOR_ID  1

#define CANCOM_MOTOR_ENABLE   3
#define CANCOM_MOTOR_RESET    4
#define CANCOM_MOTOR_ZERO     6

static inline uint32_t makeId(uint8_t motor_id, uint8_t type) {
  return ((uint32_t)(type & 0x1F) << 24) | ((uint32_t)(motor_id & 0xFF));
}

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

static void sendBoth(uint32_t id, const uint8_t *data) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = data[i];
  Can0.write(msg);
  Can1.write(msg);
  Serial.printf("  TX id=0x%08lX data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               (unsigned long)id,
               data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.setClock(CLK_60MHz);

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(16);
  Can1.enableFIFO();
  Can1.setClock(CLK_60MHz);

  uint8_t payload[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  Serial.println("Stop (Type 4)...");
  sendBoth(makeId(MOTOR_ID, CANCOM_MOTOR_RESET), payload);
  delay(200);
  Serial.println("Zero (Type 6)...");
  sendBoth(makeId(MOTOR_ID, CANCOM_MOTOR_ZERO), payload);
  delay(200);
  Serial.println("Enable (Type 3)...");
  sendBoth(makeId(MOTOR_ID, CANCOM_MOTOR_ENABLE), payload);
  delay(200);

  Serial.println("Enable sent.");
}

void loop() {
  Can0.events();
  Can1.events();


  
  delay(10);
}


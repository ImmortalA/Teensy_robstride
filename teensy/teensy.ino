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

// No templates here: Arduino's .ino prototype pass breaks on `template <typename ...>`.
static void sendExtended0(uint32_t id, const uint8_t *data) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = data[i];
  Can0.write(msg);
}

static void sendExtended1(uint32_t id, const uint8_t *data) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = id;
  msg.len = 8;
  for (int i = 0; i < 8; i++) msg.buf[i] = data[i];
  Can1.write(msg);
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
   {
     const uint32_t id = makeId(MOTOR_ID, CANCOM_MOTOR_RESET);
     sendExtended0(id, payload);
     sendExtended1(id, payload);
     Serial.printf("  TX id=0x%08lX data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)id,
                   payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]);
   }
   delay(200);
   Serial.println("Zero (Type 6)...");
   {
     const uint32_t id = makeId(MOTOR_ID, CANCOM_MOTOR_ZERO);
     sendExtended0(id, payload);
     sendExtended1(id, payload);
     Serial.printf("  TX id=0x%08lX data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)id,
                   payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]);
   }
   delay(200);
   Serial.println("Enable (Type 3)...");
   {
     const uint32_t id = makeId(MOTOR_ID, CANCOM_MOTOR_ENABLE);
     sendExtended0(id, payload);
     sendExtended1(id, payload);
     Serial.printf("  TX id=0x%08lX data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                   (unsigned long)id,
                   payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]);
   }
   delay(200);
 
   Serial.println("Enable sent.");
 }
 
 void loop() {
   Can0.events();
   Can1.events();
 
 
   
   delay(10);
 }
 
 
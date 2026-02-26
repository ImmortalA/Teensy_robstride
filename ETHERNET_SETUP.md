# Ethernet setup (PC ↔ Teensy)

Same subnet: **192.168.0.x** / 255.255.255.0.

| Role        | IP             | Interface / note      |
|------------|----------------|------------------------|
| PC (host)  | 192.168.0.100  | enp8s0                 |
| First Teensy (board 0) | 192.168.0.101 | port 8003 (static in sketch) |
| Second Teensy (board 1) | 192.168.0.102 | port 8004 (if used)   |

---

## 1. PC (Linux)

Set static IP on Ethernet:

```bash
sudo ip addr add 192.168.0.100/24 dev enp8s0
```

Check: `ifconfig enp8s0` → `inet 192.168.0.100`.

---

## 2. Host apps (apps/*.cpp)

- **BOARD_INTERFACE_NAME** = `"enp8s0"` (Ethernet).
- **ACTUATOR_TEENSY_BOARD_IPS** = `{"192.168.0.101", "192.168.0.102"}` — first Teensy = 192.168.0.101, second = 192.168.0.102. Set in `apps/main.cpp`, `apps/demo02_MIT.cpp`, `apps/demo03_daisy_chain_2_motors.cpp` as needed. Must match the Teensy firmware.

---

## 3. Teensy firmware (teensy/teensy.ino)

**First Teensy (board 0) = 192.168.0.101, port 8003**

### Static IP on Teensy

1. In **teensy/teensy.ino** set:
   - `#define USE_STATIC_IP 1`
   - `IPAddress teensyIP(192, 168, 0, 101);`  — board 0; use 102 for board 1
   - `IPAddress teensySubnet(255, 255, 255, 0);`
   - `IPAddress teensyGateway(192, 168, 0, 1);`  — optional if no router on the link
2. Build and upload the sketch. The Teensy will come up with that IP (no DHCP).
3. For DHCP instead, set `USE_STATIC_IP` to `0`.

Other settings:

- **kPort** = 8003 (board 0). Use 8004 for a second Teensy (board 1).
- **udp.send("192.168.0.100", kPort, ...)** → PC IP; host receives on 8003/8004.

---

## 4. Checklist

- [ ] PC: enp8s0 = 192.168.0.100
- [ ] apps (e.g. main.cpp): BOARD_INTERFACE_NAME = "enp8s0", ACTUATOR_TEENSY_BOARD_IPS[0] = "192.168.0.101"
- [ ] teensy.ino: USE_STATIC_IP 1, teensyIP = 192.168.0.101, kPort 8003, udp.send("192.168.0.100", kPort, ...)
- [ ] Rebuild host (`make`), re-upload Teensy, run `./test_spine`

---

## 5. Motor not moving (feedback OK, no motion)

If you see `Motor (board 0 CAN 1): p=-12.5` (or other values) but the motor does not move:

1. **Motor CAN ID** – In **teensy/teensy.ino** set `MOTOR_ID` to match the motor. Default in the sketch is `1`. If the motor uses another ID in the official Robstride motor tool, set `#define MOTOR_ID <id>` and re-upload the Teensy.
2. **Confirm ID in motor tool** – Connect the motor with the official Windows motor_tool (USB-CAN). Note the “CAN_ID” or “id” shown when the motor is detected; use that value for `MOTOR_ID`.
3. **Debug** – Re-upload teensy.ino, open Serial Monitor (115200). Confirm the CAN ID in transmitted frames matches the motor.
4. **Test position** – The host app (e.g. test_spine) sends position/velocity commands. If the motor is at -12.5 rad (raw 0), it should try to move toward the commanded position. Try a larger amplitude or step in the app to see if it responds.

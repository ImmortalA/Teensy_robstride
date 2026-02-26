# Robostride O2 protocol (from User Manual)

Summary from **RS02User Manual260112.pdf** and **RS02-IP67User Manual260112.pdf**.

---

## Two protocols in the manual

The motor can run in:

1. **Private protocol (default)** – Section 4. **Extended 29-bit CAN.** Communication **type** in ID bits 28–24.
2. **MIT protocol** – Section 6. **11-bit CAN ID** (Bit 10–8 = mode type, Bit 7–0 = motor ID). Same magic bytes 0xFC/0xFD/0xFE in **payload**.

Our code uses **extended 29-bit** frames with type in the ID → **Private protocol** format. If the motor is set to **MIT protocol** (protocol switch F_CMD=2), it expects **11-bit** frames; then our current link would be wrong.

---

## Private protocol (Section 4) – what we use

- **CAN:** 2.0, 1 Mbps, **extended frame**, 29-bit ID.
- **ID layout:** Bit28–24 = Communication type, Bit23–8 = data area, Bit7–0 = destination (motor CAN ID).

| Type (decimal) | Name              | Our use |
|----------------|-------------------|--------|
| 0              | Get device ID     | No |
| 1              | Operation control (motor control) | Yes – normal commands |
| 2              | Motor feedback    | Yes – RX only |
| 3              | Motor enabled to run | Yes – enter motor mode |
| 4              | Motor stops running  | Yes – exit motor mode |
| 6              | Set motor mechanical zero | Yes – zero encoder |
| 7              | Set motor CAN ID  | No |
| 17, 18, 21, 22, 23, 24, 25, 26 | Parameter read/write, fault, save, baud, report, protocol, version | No |

**Magic bytes (manual Section 6 – MIT):** Payload `FF FF FF FF FF FF FF xx`:  
- **0xFC** = Enable motor (Command 1) → we map to Type 3.  
- **0xFD** = Stop motor (Command 2) → we map to Type 4.  
- **0xFE** = Set zero (Command 4) → we map to Type 6.

So our mapping (0xFD→4, 0xFC→3, 0xFE→6) matches the manual’s intent.

---

## Type 1 (operation control) – data format in manual

Manual 4.1.2: **All 16-bit** (high byte first):

- Byte0–1: Target angle [0–65535] → (-4π ~ 4π) rad  
- Byte2–3: Target angular velocity [0–65535] → (-44 ~ 44) rad/s  
- Byte4–5: Kp [0–65535] → (0.0 ~ 500.0)  
- Byte6–7: Kd [0–65535] → (0.0 ~ 5.0)  

**No torque** in this Type 1 description.

**Kp / Kd / Ki note:** Type 1 carries only **Kp** and **Kd**. **Ki** is not in the frame; it is a motor-internal (velocity loop) parameter. Set Ki via parameter write (Type 18), e.g. velocity KI at 0x2015 (typical 0.002–0.02). Recommended tuning order (from RobStride examples): position Kp → velocity Kp (our Kd) → velocity Ki (parameter only). Typical values: Kp 25–30 (up to 500), Kd 2–5.

Our **utils.h** `pack_cmd()` uses **MIT-style** packing: 16b position + 12b velocity + 12b Kp + 12b Kd + 12b torque (packed in 8 bytes). So:

- **If the motor is in Private protocol** and expects Type 1 as above → our payload format is **different** (we use 12-bit fields and include torque). You may need a separate pack path for “Private Type 1” (16-bit each, no torque) if the motor ignores or misreads our current payload.
- **If the motor is in MIT protocol** → it expects **11-bit** ID and our payload layout matches **MIT Command 3** (Section 6.5); but we send **extended** frames, so the link would still be wrong until we send 11-bit IDs for MIT mode.

---

## Type 2 (motor feedback) – manual

- Byte0–1: Current angle [0–65535] → (-4π~4π)  
- Byte2–3: Current angular velocity [0–65535] → (-44~44) rad/s  
- Byte4–5: Current torque [0–65535] → (-17~17) Nm  
- Byte6–7: Temperature (×10 °C), high byte first  

Our `unpack_reply()` uses 16b position, 12b velocity, 12b torque. So feedback parsing is **similar but not identical** to the manual (they use 16-bit for v and torque). If feedback looks wrong, align to the manual’s 16-bit layout.

---

## Control modes (manual Section 3.4 / 4.3)

- **Operation control mode** – five parameters (position, velocity, Kp, Kd; and effectively torque in MIT).  
- **Current mode** – Iq command.  
- **Velocity mode** – target speed.  
- **Position mode (PP)** – profile position.  
- **Position mode (CSP)** – cyclic sync position.  
- **Stop** – Type 4 / 0xFD.

We only use **operation control mode** (Type 1) and enter/exit/zero (Type 3/4/6). Other modes would need extra commands (e.g. parameter write Type 18 to switch mode).

---

## Match checklist (after reading manual)

- [x] Extended 29-bit ID with type in high bits; we use (id<<21)|(data<<5)|mode (same idea).
- [x] Type 3 = enable, Type 4 = stop, Type 6 = zero; we send 0xFC→3, 0xFD→4, 0xFE→6.
- [x] Type 1 = control, Type 2 = feedback; we use both.
- [x] **Payload format Type 1:** Manual = four 16-bit (p, v, Kp, Kd), no torque. We now use `pack_cmd_private_o2()` for O2 (4×16-bit, no torque).
- [x] **Feedback format Type 2:** Manual = 16-bit for p, v, torque, temp. We now use `unpack_reply_o2_manual()` for O2 (16b each + temp °C).
- [ ] **Protocol mode:** Confirm motor is in **Private protocol** (extended frame). If it is in **MIT protocol**, we must send 11-bit ID and MIT Command 3 payload instead.

---

## Motor Studio (motor_toolV13) – extracted from official tool

The Robostride **motor_toolV13** folder contains only the built Windows app (`motor_tool.exe`) and Qt DLLs (no source). The following was extracted from the executable with `strings` for use in our project.

### How the tool talks to the motor

- **USB-CAN adapter:** The tool uses a **USB-CAN** module (serial port). It opens a **COM port** and sends **AT commands** to configure the CAN adapter, then sends/receives CAN frames over serial.
- **Relevant DLL:** `Qt5SerialPort.dll` → serial (COM) communication to the adapter.
- So the chain is: **PC ↔ serial (COM) ↔ USB-CAN adapter ↔ CAN bus ↔ motor**. Our project replaces “PC serial + USB-CAN” with **PC Ethernet ↔ Teensy ↔ CAN**.

### AT commands (USB-CAN module configuration)

These strings appear in the official tool; they configure the **adapter**, not the motor directly. Our Teensy does not use AT commands; it talks CAN directly. They still confirm the intended CAN setup:

| String in exe | Likely meaning |
|---------------|----------------|
| `AT+CAN_MODE=0` | CAN mode (0 = normal?) |
| `AT+CAN_FRAMEFORMAT=1,0,1,0` | Frame format: **1** = extended frame (0 = standard). Confirms **extended 29-bit** for O2. |
| `AT+CAN_BAUD=1000000` | **1 Mbps** CAN bit rate (matches manual and our firmware). |
| `AT+CAN_BAUD=500000` | Tool also supports **500 kbps** (message: “The module baud rate is 500kbps” / “1Mkbps”). |
| `AT+USART_PARAM=` | Serial port params for PC–adapter link. |
| `AT+CAN_FILTER0=1,0,0,0` | CAN acceptance filter (filter 0). |
| `,8,1,0` | Possibly related to frame format or filter (e.g. 8 bytes, extended, …). |

**Takeaway:** Official tool uses **1 Mbps**, **extended frame**. Our Teensy and PROTOCOL_REFERENCE are aligned with that.

### UI / workflow (confirmed strings)

- **CAN:** “Extended Frame” / “Standard Frame” option; “USB-CAN”; “CAN Module configuration successful/failed”.
- **Motor identity:** “SetID”, “CAN_ID”, “SetCAN_ID--device”, “Set motor type **0** successfully”, “Set motor type **1** successfully” → at least two **motor types** (0 and 1). Could be protocol (Private vs MIT) or product variant.
- **Control:** “lineEditSetKp”, “lineEditSetKd”, “lineEditSetTorque” → Kp, Kd, and torque are exposed (consistent with operation control / Type 1 and our `pack_cmd`).
- **Modes:** “Set motor mode--device”, “Set MOTOR mode successfully!”, “Set the mechanical zero position successfully!” → same concepts we use: **motor mode**, **mechanical zero**.
- **Detection:** “Detected equipment, mcuId:0X … canId: … CAN: … id:” → tool detects device by **mcuId** and **canId** (CAN ID).

### Error / protocol messages

- “Type and data frame do not match”, “Missing frames”, “Number missing frame”, “Type missing frame” → multi-frame or type-byte protocol; our single-frame Type 1/2 usage is consistent with “type” in the ID.
- “Frame Error”, “Parameter mismatch”, “Read parameter 0X …”, “Write parameter 0X …” → parameter read/write by some **index/address (0x…)**; we don’t use parameter R/W yet.
- “The module frame type has been modified successfully. Re-plugging and unplugging will take effect” → frame type (standard vs extended) is a module setting; we use extended on Teensy.

### What we can use for our project

1. **CAN:** Use **1 Mbps**, **extended frame** (we already do).
2. **Frame format:** No change; AT `CAN_FRAMEFORMAT=1,0,1,0` supports our 29-bit extended ID.
3. **Motor types 0 and 1:** If the motor doesn’t respond, try ensuring it is in the same “motor type” as in the tool (e.g. type 0 = Private extended); we don’t know the exact register yet (likely a parameter write, Type 17/18/21–26 in the manual).
4. **Kp / Kd / Torque:** The tool exposes all three; our `pack_cmd` (p, v, Kp, Kd, t_ff) matches this. If the motor expects **Private Type 1** (four 16-bit, no torque), we still need a separate pack path as in the checklist above.
5. **Debugging:** If you have the USB-CAN adapter, you can run motor_tool on Windows, set 1 Mbps + extended frame, and capture traffic to compare CAN IDs and payloads with what the Teensy sends.

---

## Comparison with RobStride Arduino (`extra_materials/RobStride_Control-master/arduino/`)

Differences that can cause “motor rotates once then stops” and what we did:

| Item | Arduino | Ours (before fix) | Fix |
|------|---------|-------------------|-----|
| **29-bit ID for Type 1** | `(mode<<24)\|(data<<8)\|id` with **data = torque** (16-bit, -12..+12 Nm) | `(type<<24)\|motor_id` (data = 0) | Teensy now uses `makeO2ExtendedIdWithData(motor_id, 1, 32768)` for Type 1 (0 Nm in data field). |
| **Enter Motor Mode (Type 3)** | Sent **once** at init after Change_Mode and params | Re-sent every ~2 s in the send loop | Removed periodic re-send; send Type 3 only at init. |
| **Change_Mode (Type 18)** | Called **before** Motor_Enable; then Set_SingleParameter(LIMIT_SPD, LOC_KP, …) | We send RUN_MODE **after** enable, when user picks mode | We send RUN_MODE=0 (operation) before the control loop so the motor accepts Type 1. Use **RUN_MODE=0 for all test modes** (1–4); do not use RUN_MODE=2 for “velocity” or the motor may ignore Type 1 and stop after one move. |
| **Init order (Arduino)** | Zero → Change_Mode → Set params (LIMIT_SPD, LOC_KP, SPD_KP, SPD_KI, LIMIT_CUR) → Motor_Enable | Stop → Zero → Enable at boot; RUN_MODE when user selects mode | We do not set LIMIT_SPD/LOC_KP/etc. after Change_Mode; add if needed for stability. |

What we still do **not** do (optional from Arduino):

- **Set_SingleParameter** after Change_Mode: LIMIT_SPD (0x7017), LOC_KP (0x701E), SPD_KP, SPD_KI, LIMIT_CUR. Add if the motor is stiff or oscillates.
- **Torque in ID from PC:** We use a constant 0 Nm (32768) in the Type 1 ID. To send real feedforward torque from the PC would require a 10-byte-per-node packet (2 bytes data + 8 bytes p,v,kp,kd) and Teensy support.
- **Position mode (POS_MODE):** Arduino examples often use RUN_MODE=1 and set LOC_REF (0x7016) instead of streaming Type 1. We use operation mode (RUN_MODE=0) and stream Type 1.

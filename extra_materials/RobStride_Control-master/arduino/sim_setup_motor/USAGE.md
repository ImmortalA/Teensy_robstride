# Motor ID Setup – Usage

## Quick Start

### 1. Upload
- Open `sim_setup_motor.ino`
- Select ESP32 board
- Upload

### 2. Serial Monitor
- Baud: 115200
- You should see: `=== Xiaomi motor ID setup ===`

### 3. Test link
```bash
test
```
If you see `✅ CAN OK!` you can continue.

## Main Commands

### Test and scan
```bash
test              # test CAN
scan              # scan IDs 1–127
scan 1 20         # scan 1–20
verify            # verify set motors
help              # help
```

### Set ID
```bash
set 1 2           # change ID 1 → 2
batch 5           # batch from ID 5
reset 3           # reset motor 3
```

## Recommended Flow

### Scenario 1: Set 3 motors to ID 1, 2, 3
```bash
test              # 1. test link
batch 3           # 2. batch set
# Connect motor 1, Enter → ID 1
# Connect motor 2, Enter → ID 2
# Connect motor 3, Enter → ID 3
verify            # 3. verify
```

### Scenario 2: Set one motor to a specific ID
```bash
test              # 1. test link
scan              # 2. see current ID
set 7 1           # 3. change ID 7 → 1
verify            # 4. verify
```

## Important

1. **When changing ID:** Only **one motor** on the bus.
2. **After each change:** Disconnect that motor, connect the next, then change.
3. **When all are set:** Connect all motors to the bus.
4. **ID range:** 1–127.

## Troubleshooting

- **`test` fails:** Check CAN wiring, motor power (12–24 V), baud (1 Mbps).
- **`scan` finds nothing:** Run `test` first; try with a single motor.
- **`set` fails:** Ensure only one motor on bus; run `test` to confirm link.

---

**Short flow:** test → batch/scan/set → verify ✅

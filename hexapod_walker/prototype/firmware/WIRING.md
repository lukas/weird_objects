# Hexapod prototype — wiring & bench bring-up

One-page checklist for wiring the electronics and testing them **on a
bench power supply first**, before the LiPo is ever connected. Pair
this with the firmware in `prototype_servo_bridge/` and the client in
`../pi_control/servo_bridge_client.py`.

Your first built arm is **leg 0 = joints 0 / 1 / 2**, all on PCA9685
`0x40`, channels 0 / 1 / 2, stock DS3225 pigtails (no extensions). This
doc gets that leg moving safely.

**You do not need the Raspberry Pi for any of this.** The whole servo
bench test runs on just the **Arduino Mega + your laptop** (USB serial).
The Pi is only involved in the very last step (the IMU), which is on a
separate I²C bus — see Stage F.

---

## 0. Why bench power before the battery

A charged 3S LiPo is a ~12 V, many-amp source with no current limit —
a wiring mistake dumps all of it into a short (smoke, burnt traces, or
a slammed servo). A bench supply lets you **set a current limit** so a
mistake trips the supply instead of cooking hardware. Only move to the
LiPo once everything below passes on the bench.

If you don't own a bench supply: at minimum put an **inline fuse**
(~5 A) on the servo rail and keep first power-ons short, but a
current-limited supply is strongly preferred for bring-up.

---

## 1. Two power domains (share ground only)

```
   BENCH SUPPLY (set 5.5V, limit 2A)        ← swap in LiPo+BEC later
            │
   ┌────────┴────────┐
   │                 │
 PCA 0x40 V+      (PCA 0x41 V+)             ← servo power, NOT from Arduino
 servo rail        servo rail

   Arduino Mega  ── 5V / GND / SDA / SCL ──► PCA 0x40 (+ PCA 0x41 later)
        │   (Mega is the I²C master for the PCA boards)
       USB ──► laptop (logic power + serial during bench test)

   MPU-6050 IMU ── separate bus ──► Raspberry Pi GPIO I²C  (Stage F only)
```

- **Logic** (Mega, PCA9685 VCC) is powered from the Mega's USB on the
  bench — clean and current-limited by the laptop port. No Pi needed.
- **Servo V+** comes only from the bench supply (later: the BECs) into
  the PCA9685 **screw terminals**. Never power DS3225s from the Mega
  5V pin (firmware header, lines 39–41).
- **All grounds common**: bench-supply −, PCA V+ terminal GND, PCA
  logic GND, Mega GND. Verify with a continuity beep.
- **The IMU is NOT on the Mega's bus.** It rides the Pi's I²C bus and
  is read by the Pi directly — the servo firmware never touches it. It
  plays no part in the arm test; see Stage F.

---

## 2. I²C control bus — Mega ↔ PCA9685 (Mega is the master)

The **Arduino Mega is the I²C master** for the PCA boards (the firmware
does `Wire.begin()` / `pwm.begin()`). The PCAs hang off the Mega's pins
20/21 — **not** the Pi or laptop. That's why you verify them with a
scanner *sketch* on the Mega, not with `i2cdetect` (which only scans a
Linux host's own bus, and your laptop doesn't have one).

| Mega pin        | PCA9685 #1 (0x40) | PCA9685 #2 (0x41, later) |
|-----------------|-------------------|--------------------------|
| `SDA` (pin 20)  | SDA               | SDA (chain)              |
| `SCL` (pin 21)  | SCL               | SCL (chain)              |
| `5V`            | VCC (logic)       | VCC (logic)              |
| `GND`           | GND (logic)       | GND (logic)              |

- For the single-arm bench test you only wire **board `0x40`**. Board
  `0x41` comes later (it only carries joints 16 & 17 = leg 5 hip+knee).
- Daisy-chain board #2 from board #1's pass-through header, or run
  separate jumpers from the Mega.
- **Solder the `A0` pad on board #2** so it enumerates as `0x41`.
  Board #1 stays unjumpered (`0x40`).
- VCC (logic, ~5 V low current) is **separate** from the V+ servo
  terminal block — don't bridge them.

**Verify the Mega sees the board(s)** with this scanner sketch, read in
the Arduino serial monitor @ `115200`, then re-flash the real firmware:

```cpp
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Scanning I2C...");
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("found 0x"); Serial.println(a, HEX);
    }
  }
}
void loop() {}
```

Expect `0x40` (and `0x41` once that board's `A0` jumper is set).

---

## 2b. Where the Raspberry Pi fits in (USB, **not** I²C)

Short answer: **you never connect the Pi to the PCA's SDA/SCL.** The
PCA boards stay on the Arduino's I²C bus, and the Arduino is their only
master. The Pi connects to the **Arduino over USB** and sends the same
text commands you typed by hand (`C`, `J 0 20`, …). The Arduino is a
"servo bridge": Pi decides *what* pose, Arduino does the real-time PWM.

```
  Raspberry Pi ──USB cable──► Arduino Mega ──I²C (pins 20/21)──► PCA 0x40 / 0x41 ──► servos
       │           (serial,           (Mega = I²C master)
       │            115200)
       └──I²C (GPIO2/3, its OWN bus)──► MPU-6050 IMU
```

Two separate I²C buses, one per processor — they are **not** joined:

- **Arduino's bus** (Mega pins 20/21): the two PCA9685 boards. Mega is
  master.
- **Pi's bus** (Pi GPIO2 `SDA` / GPIO3 `SCL`): the MPU-6050 only. Pi is
  master.

> ⚠ **Do not wire the Pi's SDA/SCL to the Arduino's SDA/SCL.** That
> would put two masters on one bus and they'd fight. The Pi↔Arduino
> link is USB serial, full stop.

### How to talk to the Arduino from the Pi

It's the *exact same serial protocol* you've been typing — the Pi just
sends those lines over the USB cable instead of you typing them. The
repo's client already does this:

```bash
# On the Pi (after: python -m pip install pyserial):
python hexapod_walker/prototype/pi_control/servo_bridge_client.py \
    --port /dev/ttyACM0 centre
python hexapod_walker/prototype/pi_control/servo_bridge_client.py \
    --port /dev/ttyACM0 joint 1 20 --sweep
```

- The Arduino enumerates on the Pi as **`/dev/ttyACM0`** (or
  `/dev/ttyUSB0` on some clones). Find it with `ls /dev/ttyACM* /dev/ttyUSB*`.
- This is the only wire between Pi and Arduino: the **USB-A → USB-B
  cable** from the BOM. Power-wise the Pi can power the Mega over that
  cable, but on the robot give each its own supply (see §5).
- Everything in Stages C–E works identically whether the serial host is
  your laptop (bench) or the Pi (on-robot) — only the `--port` changes.

### Division of labor (why two processors)

- **Arduino Mega**: real-time servo PWM via the PCAs, hard angle-limit
  clamping, trim. Deterministic, never blocked by an OS.
- **Raspberry Pi**: high-level brain — gait planner, vision, Wi-Fi, and
  reading the IMU on its own bus. Streams pose vectors (`A <18 floats>`)
  to the Arduino over USB.

---

## 3. Servos — leg 0 (your built arm)

All three land on PCA `0x40`. Source of truth:
`python -m hexapod_walker.prototype.pi_control.wire_harness_plan`.

| Joint | Axis      | Board | Ch | Cable          | Limit (deg) |
|------:|-----------|-------|---:|----------------|-------------|
| 0     | yaw       | 0x40  | 0  | stock pigtail  | ±35         |
| 1     | hip pitch | 0x40  | 1  | stock pigtail  | −80 … +30   |
| 2     | knee      | 0x40  | 2  | stock pigtail  | −20 … +80   |

3-pin connector orientation on each channel column:

- **brown/black → GND** (outer-rim row, `–`)
- **red → V+** (middle)
- **orange/yellow → PWM** (signal, nearest the chip)

Reversing the connector is the #1 bring-up mistake — match the
silkscreen `GND/V+/PWM` rows exactly.

> When you wire all 6 legs later: **only joints 16 & 17 (leg 5 hip +
> knee) cross to board `0x41`.** Everything else is on `0x40`.

---

## 4. Bench bring-up sequence

**You drive the servos by typing commands into the Arduino IDE Serial
Monitor** — no Python, no Pi. The firmware parses newline-terminated
commands (see `prototype_servo_bridge.ino`):

| Type this        | Does                                              |
|------------------|---------------------------------------------------|
| `?`              | print help                                        |
| `C`              | centre all joints (0°)                             |
| `J <joint> <deg>`| move one joint, e.g. `J 0 20` = joint 0 to +20°   |
| `T <joint> <deg>`| set a trim offset (±30°), e.g. `T 1 -5`           |
| `P`              | print the trim table                              |

Serial Monitor settings: **baud `115200`**, line ending **"Newline"**
(so each command is terminated with `\n`). Each command echoes `OK ...`.

> The firmware clamps every command to the safe per-axis limits (yaw
> ±35, hip −80…+30, knee −20…+80), so you can't drive past them even by
> typing a bigger number.

The Python client in `../pi_control/servo_bridge_client.py` is an
*optional* convenience (it adds smooth `--sweep` ramps and a `wiggle`
macro) — but it is **not needed** for any stage below.

### Stage A — Rails only, nothing else connected
Bench supply set to **5.5 V, current limit 2.0 A**. No PCA, no Mega.

- [ ] Output reads ~5.5 V on the meter.
- [ ] No short between the V+ and GND leads (continuity = open).
- [ ] Supply OFF.

### Stage B — Mega + PCA9685 logic, no servos, no servo rail
Mega powered by **USB only** (laptop); bench supply still
OFF/disconnected. Just the Mega and PCA `0x40` — no Pi.

- [ ] Upload the **I²C scanner sketch** (§2); serial monitor @ `115200`
      lists `0x40` (and `0x41` if that board is wired + jumpered). If a
      board is missing, fix wiring / the `A0` jumper now.
- [ ] Re-flash `prototype_servo_bridge.ino` (needs the
      "Adafruit PWM Servo Driver Library").
- [ ] Serial @ `115200` prints `OK prototype_servo_bridge` + help.
- [ ] PCA + Mega regulators stay cool after a minute.

### Stage C — One spare servo, off the arm
Plug a **spare** DS3225 into `0x40` ch 0. Connect bench supply to the
PCA V+ terminal. Supply ON (still 5.5 V / 2 A limit). In the Serial
Monitor, type these one line at a time:

```
C            ← centre (joint 0 → 0°)
J 0 15       ← nudge +15°
J 0 0
J 0 -15      ← nudge −15°
J 0 0
J 0 30
J 0 -30
J 0 0
```

- [ ] Centres without a violent jump; each step is smooth both ways.
- [ ] Idle current is small; moving current well under 2 A (the supply
      shouldn't be hitting its limit / going into CV→CC).
- [ ] No continuous buzz (= stall), no overheating.

### Stage D — Leg 0 servos, one joint at a time
Plug the arm's 3 servos into `0x40` ch 0/1/2. **Clamp the arm, foot in
the air.** Raise the current limit to **~3 A** (three DS3225s).

> ⚠ On serial-open the Mega resets and `setup()` calls `centreAll()` →
> every joint jumps to 0°. Confirm each horn's 0° is near mid-travel
> *before* opening the Serial Monitor, or the arm can drive into a hard
> stop the instant you connect.

Step each joint out in small increments, typing one line at a time and
watching the joint after every step:

```
J 0 10       ← yaw toward +; then 20, then -20 (limit ±35)
J 1 -10      ← hip; then -25 (the stance value); stay within −80…+30
J 2 30       ← knee; then 60 (stance value); stay within −20…+80
```

- [ ] Each joint moves the **expected direction**.
- [ ] No mechanical binding before the firmware limit.
- [ ] Fix offsets with trim instead of re-bolting the horn, e.g. `T 1 -5`
      (trim is clamped ±30° and re-applies immediately).

### Stage E — Coordinated stance + load
Bring all three joints to the neutral standing pose by typing them in
sequence:

```
J 0 0
J 1 -25
J 2 60
```

- [ ] The leg settles into a stable stance pose.
- [ ] Hold 30–60 s: servos warm at most, supply not pinned at its limit.
- [ ] Let the foot lightly touch; small hip/knee moves feel sane.

> Optional: the `stance` command in the Python client sends all 18
> joints at once, but typing the three `J` lines above does the same
> thing for leg 0 with just the Serial Monitor.

### Stage F — IMU (requires the Pi, separate from the arm test)
The MPU-6050 is **not** on the Mega's bus — wire it to the **Pi's**
GPIO I²C (`SDA`=GPIO2/pin 3, `SCL`=GPIO3/pin 5, `3V3`, `GND`), `AD0`
low → `0x68`. This is the one place `i2cdetect` is the right tool, and
it runs **on the Pi**:

```bash
i2cdetect -y 1    # on the Pi → expect 0x68
```

- [ ] `i2cdetect -y 1` on the Pi shows `0x68`.
- [ ] Read the MPU-6050 in Python; gyro/accel values change as you tilt
      the chassis.

---

## 5. Switching from bench supply to LiPo

Only after Stages A–F pass:

1. Bench supply OFF and disconnected.
2. Wire: `LiPo XT60 → anti-spark switch → split → BEC #1 (→ PCA 0x40)`
   and `BEC #2 (→ PCA 0x41)`.
3. **Set both BEC outputs to the same voltage (5.0–6.0 V) with the
   meter before connecting them to the PCAs.**
4. Re-verify common ground across LiPo −, both BECs, both PCAs, and the
   Mega. (The IMU shares ground with the Pi on its own bus.)
5. The **anti-spark switch is now your e-stop** — keep a hand near it
   for the first powered run.

Per the harness plan, **leg 0 needs no extension cables** — stock
pigtails reach ch 0/1/2 directly. Don't add slack you'll have to manage.

---

## Per-joint log (fill in for all 6 legs)

| Joint | Ch | Board | Neutral | Safe min | Safe max | Dir | Binds at | Notes |
|------:|---:|-------|--------:|---------:|---------:|-----|----------|-------|
| 0 yaw | 0  | 0x40  | 0       |          |          |     |          |       |
| 1 hip | 1  | 0x40  | −25     |          |          |     |          |       |
| 2 knee| 2  | 0x40  | +60     |          |          |     |          |       |

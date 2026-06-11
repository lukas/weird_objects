# Hexapod prototype — wiring & bench bring-up (FEETECH bus servos)

One-page checklist for wiring the electronics and testing them **on a
bench power supply first**, before the LiPo is ever connected. Pair this
with the Pi-side driver in `../pi_control/feetech_bus.py`.

**June 2026 redesign.** The prototype now uses **18× FEETECH STS3215**
(ST-3215-C018, 12 V / 30 kg·cm) **serial-bus** smart servos. Each servo
has a built-in 12-bit magnetic encoder and reports **position, load,
voltage, current and temperature** back over the bus, so the robot has
closed-loop joint feedback with **no external sensors**. This also
deletes the old stack: **no Arduino Mega, no PCA9685 PWM boards, no
servo BECs, no AS5600 encoders.** The Raspberry Pi (or your laptop)
talks the STS protocol **directly** over one USB→TTL bus adapter.

Your first built arm is **leg 0 = joints 0 / 1 / 2 = servo IDs 1 / 2 /
3**. This doc gets that leg moving safely, then scales to all 18.

---

## 0. Why bench power before the battery

A charged 3S LiPo is a ~12 V, many-amp source with no current limit — a
wiring mistake dumps all of it into a short (smoke, burnt traces, or a
slammed servo). A bench supply lets you **set a current limit** so a
mistake trips the supply instead of cooking hardware. Only move to the
LiPo once everything below passes on the bench.

If you don't own a bench supply: at minimum put an **inline fuse**
(~5 A for one leg, ~15–20 A for all 18) on the servo V+ rail and keep
first power-ons short — but a current-limited supply is strongly
preferred for bring-up.

> **Voltage note:** the STS3215-30kg is the **12 V** variant (range
> 6–12.6 V). Run the bus at **12 V** (bench supply, later a 3S LiPo at
> 11.1 V nominal). There is **no servo BEC** any more — 12 V goes
> straight to the servos. The Pi still needs its own 5 V (a 5 V BEC or
> USB supply); it is **never** powered from the 12 V servo rail.

---

## 1. Two power domains (share ground only)

```
   BENCH SUPPLY (set 12.0 V, limit 2 A for one leg)   ← swap in 3S LiPo later
            │  12 V servo rail
   ┌────────┴─────────────────────────────────┐
   │  V+ injected on the bus power rail        │
 Bus adapter ──TTL bus──► servo 1 ─► servo 2 ─► … ─► servo 18
 (FE-URT-1 /             (3-wire daisy chain: V+ / GND / Signal)
  Waveshare)
   │ USB
   ▼
 Raspberry Pi (or laptop)  ── 5 V of its own (USB / 5 V BEC) ──┐
   │                                                           │
   └──I²C (GPIO2/3, its OWN bus)──► MPU-6050 IMU  (Stage F)     │
                                                               ground common
```

- **Servo power** (12 V) is a single rail injected onto the bus. The
  STS3215 connector is **3-pin**: `V+ / GND / Signal`. Servos pass power
  and signal through to the next servo, so the whole robot is one chain.
- **The bus adapter** (FE-URT-1 or Waveshare Bus Servo Adapter) converts
  USB↔half-duplex TTL and passes the 12 V through to the first servo's
  power pins (most adapters have a DC barrel jack for the servo rail).
- **All grounds common**: bench-supply −, bus-rail GND, adapter GND, Pi
  GND. Verify with a continuity beep.
- **The IMU is on the Pi's I²C bus**, totally separate from the servo
  bus. It plays no part in the arm test; see Stage F.

---

## 2. The serial bus — Pi ↔ adapter ↔ servos

There is exactly **one** data wire pair on the robot: the half-duplex
TTL bus, daisy-chained servo to servo. No I²C, no per-servo PWM lines.

```
  Raspberry Pi ──USB──► Bus adapter ──TTL bus──► ID1 ─► ID2 ─► ID3 ─► … ─► ID18
       │   (serial, 1 Mbps)          (V+ / GND / Signal, chained)
       └──I²C (GPIO2/3)──► MPU-6050 IMU   (separate bus)
```

- The adapter enumerates on the Pi as **`/dev/ttyUSB0`** (or
  `/dev/ttyACM0`). Find it with `ls /dev/ttyUSB* /dev/ttyACM*`.
- **12 V must be present on the bus** for the adapter to talk to servos
  (the STS logic is powered from V+). Power the rail before scanning.
- Default servo baud is **1 Mbps**; `feetech_bus.py` uses that.

### Install the driver (once, on the Pi/laptop)

```bash
python -m pip install feetech-servo-sdk pyserial
```

### Assign servo IDs (once per servo — CRITICAL)

Every servo ships as **ID 1**. A bus needs unique IDs, so you set each
one **with only that single servo connected** (otherwise every ID-1
servo answers at once and you brick the address):

```bash
# Connect ONE new servo. It is ID 1 from the factory. Give it its ID:
python ../pi_control/feetech_bus.py --port /dev/ttyUSB0 setid --from 1 --to 3   # this is joint 2 (knee), leg 0
```

Logical joint → servo ID is simply **ID = joint + 1**:

| Leg | yaw (axis0) | hip (axis1) | knee (axis2) |
|----:|:-----------:|:-----------:|:------------:|
| 0   | ID 1        | ID 2        | ID 3         |
| 1   | ID 4        | ID 5        | ID 6         |
| 2   | ID 7        | ID 8        | ID 9         |
| 3   | ID 10       | ID 11       | ID 12        |
| 4   | ID 13       | ID 14       | ID 15        |
| 5   | ID 16       | ID 17       | ID 18        |

Label each servo as you ID it. After all are chained, verify:

```bash
python ../pi_control/feetech_bus.py --port /dev/ttyUSB0 scan   # expect IDs 1..18
```

---

## 2b. Where the Raspberry Pi fits in

The Pi **is** the controller now — there is no microcontroller in the
loop. It runs `feetech_bus.py`, which:

- maps the 18 logical joints to servo IDs 1..18,
- enforces the same safe per-axis angle limits in software,
- applies per-joint **trims** (now stored in
  `../pi_control/feetech_trims.json`, since there's no EEPROM),
- sync-writes goal positions and reads back live feedback.

Everything below works identically whether the host is your laptop
(bench) or the Pi (on-robot) — only `--port` changes.

---

## 3. Servos — leg 0 (your built arm)

| Joint | Axis      | Servo ID | Limit (deg) |
|------:|-----------|:--------:|-------------|
| 0     | yaw       | 1        | ±35         |
| 1     | hip pitch | 2        | −80 … +30   |
| 2     | knee      | 3        | −20 … +80   |

STS3215 3-pin bus connector (both ports are identical — chain in or
out of either):

- **black → GND**
- **red → V+ (12 V)**
- **white/yellow → Signal**

Match the connector keying; the two ports on each servo are wired in
parallel, so "in" vs "out" doesn't matter electrically — pick whichever
makes the harness tidy.

---

## 3b. The zero state — how the leg must sit at 0 / 0 / 0

`centre` (and `joint <j> 0`) drives every joint to **0°**. The horns
must be bolted so that **0° corresponds to this pose**, or every angle
you command afterwards is off by the mounting error.

**At 0 / 0 / 0 the whole leg is dead straight and horizontal, pointing
straight out from the body**, with coxa, femur, tibia and foot colinear:

```
   zero state (side view, body at left):

   body | ==coxa== (Y)yaw  ==femur== (H)hip  ==tibia== (K)knee  ==> ● foot
                    axis ↕               axis ↺              axis ↺
        all three links colinear, parallel to the table, foot straight ahead
```

- **yaw = 0** → leg points straight out along its mounting azimuth.
- **hip = 0** → femur horizontal, in line with the coxa.
- **knee = 0** → tibia straight, in line with the femur.

### Which way each axis moves (so you mount horns the right way round)

| Axis (joint)   | `+` angle moves the link…        | `−` angle moves it… | Range      | Stance |
|----------------|----------------------------------|---------------------|------------|-------:|
| yaw (j0)       | swings horizontally one way      | the other way       | ±35°       | 0°     |
| hip pitch (j1) | femur tip **down**               | femur tip **up**    | −80 … +30° | −25°   |
| knee pitch (j2)| tibia tip **down** (folds under) | tibia tip **up**    | −20 … +80° | +60°   |

So the **standing stance (0, −25, +60)** is "femur lifted ~25°, knee
folded ~60°," planting the foot below the body.

### Mounting each horn against zero

1. `python ../pi_control/feetech_bus.py --port /dev/ttyUSB0 centre` —
   the servo holds at count 2048 (its 0° centre).
2. With the servo powered and held at 0°, fit the FEETECH POM horn /
   link so the link sits in the straight-out zero pose above, then bolt
   it (4× M2.5 on the 9.9 mm square pattern + the central M3 horn screw).
3. Get within one spline tooth, bolt it, then null out the rest with a
   software trim: `feetech_bus.py … trim <joint> <deg>` (clamped ±30°).
   e.g. if hip sits 4° high at "0": `trim 1 -4`.
4. Re-run `centre` and confirm all three links are straight and level.

> Trims are saved to **`feetech_trims.json`** and re-applied on every
> command, so you calibrate each joint **once**.

> Unlike the old PWM servos, the STS3215 is **absolute**: it knows its
> angle the instant it powers on (12-bit magnetic encoder), so there is
> no blind centre-on-boot slam — you can `relax` it, pose it by hand,
> and `feedback` will read exactly where it is.

---

## 4. Bench bring-up sequence

Drive everything from `feetech_bus.py`. Common commands:

| Command                              | Does                                            |
|--------------------------------------|-------------------------------------------------|
| `scan`                               | list servo IDs on the bus                       |
| `setid --from 1 --to N`              | re-ID the **only** servo on the bus             |
| `centre`                             | all joints → 0°                                 |
| `joint <j> <deg> [--sweep]`          | move one joint (eased with `--sweep`)           |
| `wiggle --joint <j>`                 | direction/range check macro                     |
| `stance`                             | full standing pose on all 18 joints             |
| `relax [--joint j]`                  | torque **off** (limp) so you can pose by hand   |
| `hold [--joint j]`                   | torque **on** (hold position)                   |
| `feedback [--watch]`                 | position / load / volt / temp / current table   |
| `trim <j> <deg>`                     | set + save a trim offset (±30°)                 |

Software clamps every command to the safe per-axis limits (yaw ±35, hip
−80…+30, knee −20…+80).

### Stage A — Rail only, nothing else connected
Bench supply set to **12.0 V, current limit 2.0 A**. No adapter, no servos.

- [ ] Output reads ~12.0 V on the meter.
- [ ] No short between V+ and GND leads (continuity = open).
- [ ] Supply OFF.

### Stage B — Adapter + Pi, no servos
Bus adapter on USB; 12 V rail still OFF. 

- [ ] `ls /dev/ttyUSB* /dev/ttyACM*` shows the adapter.
- [ ] `scan` returns nothing yet (no servos / no power) — that's fine; it
      confirms the port opens.

### Stage C — One spare servo, off the arm
Connect a **spare** STS3215 to the adapter and the 12 V rail. Supply ON
(12 V / 2 A limit).

```bash
feetech_bus.py --port /dev/ttyUSB0 scan          # expect [1] (factory ID)
feetech_bus.py --port /dev/ttyUSB0 setid --from 1 --to 1
feetech_bus.py --port /dev/ttyUSB0 joint 0 15
feetech_bus.py --port /dev/ttyUSB0 joint 0 -15
feetech_bus.py --port /dev/ttyUSB0 feedback      # reads back ~ -15 deg, volts, temp
```

- [ ] Servo answers `scan`; moves smoothly both ways.
- [ ] `feedback` shows sane volts (~12 V), low load, reasonable temp.
- [ ] Idle/move current well under the 2 A limit.

### Stage D — Leg 0 servos (IDs 1/2/3), one joint at a time
ID the three leg-0 servos (1, 2, 3), chain them, plug into the rail.
**Clamp the arm, foot in the air.** Current limit ~**3 A**.

```bash
feetech_bus.py --port /dev/ttyUSB0 scan          # expect [1, 2, 3]
feetech_bus.py --port /dev/ttyUSB0 joint 0 20 --sweep    # yaw
feetech_bus.py --port /dev/ttyUSB0 joint 1 -25 --sweep   # hip toward stance
feetech_bus.py --port /dev/ttyUSB0 joint 2 60 --sweep    # knee toward stance
```

- [ ] Each joint moves the **expected direction** (flip `JOINT_SIGN[j]`
      in `feetech_bus.py` if reversed).
- [ ] No mechanical binding before the software limit.
- [ ] Fix offsets with `trim`, not by re-bolting the horn.

### Stage E — Coordinated stance + load

```bash
feetech_bus.py --port /dev/ttyUSB0 stance
feetech_bus.py --port /dev/ttyUSB0 feedback --watch
```

- [ ] Leg settles into a stable stance pose.
- [ ] Hold 30–60 s watching `feedback`: temp climbs modestly, load
      sane, supply not pinned at its limit.

### Stage F — IMU (Pi I²C, separate from the servo bus)
The MPU-6050 is **4 wires** from the GY-521 breakout to the **Pi's
40-pin header** — nothing to do with the servo bus.

| GY-521 pin | → Pi header pin           | What it is        |
|------------|---------------------------|-------------------|
| `VCC`      | **pin 1** (3V3 power)     | power — 3V3, **not** 5V |
| `GND`      | **pin 6** (Ground)        | ground            |
| `SCL`      | **pin 5** (GPIO3 / SCL1)  | I²C clock         |
| `SDA`      | **pin 3** (GPIO2 / SDA1)  | I²C data          |
| `AD0`      | leave unconnected         | sets address `0x68` |
| `XDA`,`XCL`,`INT` | leave unconnected  | not used          |

> ⚠ **Power the GY-521 from 3V3 (pin 1), never 5V** — its SDA/SCL
> pull-ups go to VCC and 5V would over-drive the Pi's 3.3 V I²C lines.

```bash
sudo raspi-config        # Interface Options → I2C → Enable (once)
sudo apt install -y i2c-tools
i2cdetect -y 1           # expect 0x68
```

---

## 5. Switching from bench supply to LiPo

Only after Stages A–F pass:

1. Bench supply OFF and disconnected.
2. Wire: `3S LiPo XT60 → anti-spark switch → 12 V bus rail` (with a
   ~15–20 A fuse for all 18 servos), plus a **separate 5 V BEC** tapped
   off the LiPo for the Pi.
3. **Verify common ground** across LiPo −, bus rail GND, adapter, and Pi.
4. The **anti-spark switch is your e-stop** — keep a hand near it for
   the first powered run.

> With 18 servos the bus carries real current at stall (2.7 A each).
> Don't rely on the adapter's thin pass-through for all of it — inject
> 12 V on a **separate power rail** (thick wire / distribution) and let
> the bus connectors carry it leg-to-leg, or split the chain into 2–3
> branches each fed from the rail.

---

## Per-joint log (fill in for all 6 legs)

| Joint | ID | Neutral | Safe min | Safe max | Dir (`JOINT_SIGN`) | Binds at | Notes |
|------:|---:|--------:|---------:|---------:|--------------------|----------|-------|
| 0 yaw | 1  | 0       |          |          |                    |          |       |
| 1 hip | 2  | −25     |          |          |                    |          |       |
| 2 knee| 3  | +60     |          |          |                    |          |       |

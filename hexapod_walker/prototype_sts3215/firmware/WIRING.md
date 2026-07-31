# Hexapod prototype — wiring & bench bring-up (FEETECH bus servos)

Buildable wiring plan for the **18× FEETECH STS3215** serial-bus
hexapod, plus a bench bring-up checklist so you test everything **on a
current-limited bench supply first**, before the LiPo is ever
connected. Pair this with the host-side driver in
`../pi_control/feetech_bus.py`.

> **Rendered harness diagram:** the full connector-by-connector WireViz
> diagram (with wire colors, gauges and the auto-generated BOM) lives at
> [`harness.svg`](harness.svg) / `harness.png` / `harness.html` next to
> this file. Regenerate it (after editing this doc's topology or the
> geometry in `../pi_control/wire_harness_plan.py`) with
> `../scripts/render_harness_diagram.py` (repo venv).

**June 2026 redesign.** The prototype uses **18× FEETECH STS3215**
(ST-3215-C018, 12 V / 30 kg·cm) **serial-bus** smart servos (6 legs ×
yaw/hip/knee). Each servo has a built-in 12-bit magnetic encoder and
reports **position, load, voltage, current and temperature** back over
the bus, so the robot has closed-loop joint feedback with **no external
sensors**. This deletes most of the old stack: **no Arduino Mega, no
PCA9685 PWM boards, no servo BECs, and no AS5600 encoders.**

The controller is now an **Arduino Uno Q** (on-board Linux SoC + MCU).
It runs the Python gait/RL/teleop on its **Linux** side and drives the
half-duplex STS3215 TTL bus through a small **USB bus-servo adapter**
plugged into the Uno Q's USB-C — it replaces the Raspberry Pi. The
adapter does the half-duplex direction-switching in hardware and
enumerates on Linux as **`/dev/ttyUSB0`**, so `feetech_bus.py` runs
unchanged (only `--port` changes). A **XINGYHENG 12 V→5 V buck** on the
upper deck powers the Uno Q's logic from the 3S rail.

> **Buy one USB bus-servo adapter** — either:
> - **FEETECH FE-URT-2** (`FE-URT2-C001`) — the Type-C successor to the
>   FE-URT-1; native **USB-C**, a **TTL-BUS** port for STS/SCS servos, a
>   separate RS485 port for SMS servos, a **3.3 V / 5 V logic-level
>   slide switch** (set **5 V** — see §2), and a 5.08 screw terminal for
>   servo power. Because the Uno Q is USB-C, this is the tidiest pick: a
>   single **USB-C-to-C** cable. *(The older FE-URT-1 also works but is
>   mini-USB, so you'd need a C-to-mini cable / OTG adapter.)*
> - **Waveshare Bus Servo Adapter (A)** (SKU **25514**) — also native
>   USB-C, **D/V/G** servo header, `9–12.6 V` power input; instead of a
>   voltage switch it has an **A/B mode jumper** (A = UART to an MCU,
>   **B = USB** to a host — use **B**). CH343 USB-serial chip.
>
> **USB-C host note:** the Uno Q's *only* USB is a single Type-C port,
> so the adapter hangs off it in **USB-host / OTG** mode. Use a USB-C
> OTG adapter or a small **USB-C hub** (ideally one with power
> passthrough so the Uno Q can still be powered while hosting the
> adapter).

> **Do NOT wire the servo bus to the Uno Q's D0/D1 header.** That UART
> is the Uno Q's **STM32 MCU** side (`Serial1`, **3.3 V**, sketch-owned)
> and is **not** exposed to the Linux side as a `/dev/tty*`, so the
> Python driver cannot reach it. The servo bus reaches Linux **only**
> through the USB adapter above.

Your first built arm is **leg 0 = joints 0 / 1 / 2 = servo IDs 1 / 2 /
3**. This doc gets that leg moving safely, then scales to all 18 with a
**distributed-power harness** (§6) that keeps any one connector from
melting.

---

## 0. Why bench power before the battery

A charged 3S LiPo is a ~12 V, many-amp source with no current limit — a
wiring mistake dumps all of it into a short (smoke, burnt traces, or a
slammed servo). A bench supply lets you **set a current limit** so a
mistake trips the supply instead of cooking hardware. Only move to the
LiPo once everything below passes on the bench.

If you don't own a bench supply: at minimum put an **inline fuse**
(~5 A for one leg, the **15–20 A main fuse** of §6 for all 18) on the
servo V+ rail and keep first power-ons short — but a current-limited
supply is strongly preferred for bring-up.

> **Voltage note:** the STS3215-30kg is the **12 V** variant (range
> 6–12.6 V). Run the bus at **12 V** (bench supply, later a 3S LiPo at
> 11.1 V nominal / 12.6 V full). There is **no servo BEC** — the raw 3S
> rail goes straight to the servos. The Uno Q logic runs on **5 V from
> the buck**; it is **never** powered from the 12 V servo rail.

---

## 1. Two power domains (share ground only)

```
   BENCH SUPPLY (set 12.0 V, limit 2 A for one leg)   ← swap in 3S LiPo later
            │  12 V servo rail
            ▼
   POWER DISTRIBUTION BUS BAR  (see §6 for the full LiPo harness)
            │  per-leg branches (16–18 AWG silicone)
            ├─► leg 0 V+/GND ──┐
            ├─► leg 1 V+/GND   │  (V+/GND injected per leg, NOT chained
            └─► …              │   leg-to-leg through the servo pins)
                               │
 Uno Q ─USB-C─► USB bus-servo adapter ─TTL signal─► ID1 ─► ID2 ─► … ─► ID18
   │           (FE-URT-2 / Waveshare A;          (signal+GND chained;
   │            half-duplex dir-switch, 1 Mbps)   V+ fed per leg)
   │
   └──I²C (its OWN bus)──► MPU-6050 IMU  (Stage F)
                               │
                          ground common (LiPo −, bus-bar GND, Uno Q GND,
                                         adapter GND)
```

- **Servo power** (12 V) comes from a **power distribution bus bar**,
  split into **per-leg branches** in heavy silicone wire. Power is
  **not** passed leg-to-leg through the thin servo connector pins
  (see §6 for why that melts the upstream connector).
- **Data** is one half-duplex TTL signal, daisy-chained servo to servo.
  The Uno Q drives it over USB through the **USB bus-servo adapter** at
  **1 Mbps**; the adapter handles the half-duplex direction switching.
- **All grounds common**: bench-supply − (or LiPo −), bus-bar GND,
  Uno Q GND, adapter GND, IMU GND. Verify with a continuity beep.
- The adapter carries **DATA** (its `D`/`G` pins → bus signal + GND).
  Servo **12 V** comes from the rail, not the Uno Q and not "5 V/3 V":
  on the bench (one leg) you may feed 12 V into the adapter's power
  terminal; on the full robot the adapter is **data-only** and each leg
  gets 12 V per-leg from the bus bar (see §2 and §6).
- **The IMU is on the Uno Q's I²C bus**, totally separate from the
  servo bus. It plays no part in the arm test; see Stage F.

---

## 2. The serial bus — Uno Q ↔ USB adapter ↔ servos (DATA)

There is exactly **one** data signal on the robot: the half-duplex TTL
bus, daisy-chained servo to servo. It reaches the Uno Q's Linux side
through a **USB bus-servo adapter** (§ intro). No I²C on the servos, no
per-servo PWM lines.

```
  Uno Q ─USB-C─► USB adapter ─TTL (1 Mbps)─► ID1 ─► ID2 ─► ID3 ─► … ─► ID18
       │  (USB-C OTG/hub)   half-duplex, single signal line + common GND
       └──I²C (separate bus)──► MPU-6050 IMU
```

- The STS3215 bus is **half-duplex** (one signal wire, TX and RX
  shared). You do **not** hand-wire TX/RX: the USB adapter does the
  direction switching in hardware. You only connect the adapter's servo
  header to the bus — **`D` → Signal, `G` → GND** (and `V` = servo 12 V,
  but read the power note below before wiring `V`).
- **Adapter mode/level settings** (this is the "5 V or 3 V?" question):
  - **FE-URT-2 / FE-URT-1** have a **3.3 V / 5 V logic-level slide
    switch** for the TTL signal. STS/SCS TTL servos use a **5 V** bus —
    set the switch to **5 V**. Plug the servo chain into the **TTL-BUS**
    port (not the RS485 port, which is for SMS servos).
  - **Waveshare Bus Servo Adapter (A)** has no voltage switch; it has an
    **A/B mode jumper** — set it to **B (USB)** since the host is the
    Uno Q over USB.
  - **Neither switch sets the servo supply.** The 3.3/5 V logic level is
    the *signal* voltage only. Servo **power** is a separate **12 V**
    rail (matched to the STS3215's 9–12.6 V range), **never** 5 V or 3 V.
- **How servo power reaches the bus** (keep §6 distributed-power intact):
  - **Bench, one leg** (Stages C–E): simplest to feed **12 V** into the
    adapter's screw terminal and let `V` pass to that one leg through the
    servo header (one leg ≈ 1.5–3.7 A, under the adapter's ~5–6 A limit).
  - **Full 18-servo robot** (§6): do **not** route servo power through
    the adapter — it can't carry 18 servos. Use the adapter for **`D` +
    `G` only**; inject each leg's 12 V per-leg from the **bus bar**.
    Leave the adapter's power terminal unpowered (or a low-current 12 V
    reference tap). Either way, common ground is mandatory.
- The host port enumerates as **`/dev/ttyUSB0`** on the Uno Q's Linux
  side (CH340-based FE-URT-1). A CH343-based board (FE-URT-2 / Waveshare)
  may instead show as `/dev/ttyUSB0` or `/dev/ttyCH343USB0` depending on
  the kernel driver — just find it with `ls /dev/ttyUSB* /dev/ttyACM*`
  and pass that to `--port`.
- **12 V must be present on the bus** for the servos to talk (the STS
  logic is powered from V+). Power the rail before scanning.
- Default servo baud is **1 Mbps**; `feetech_bus.py` uses that.

### Install the driver (once, on the Uno Q / laptop)

```bash
python -m pip install feetech-servo-sdk pyserial
```

### Assign servo IDs (once per servo — CRITICAL)

Every servo ships as **ID 1**. A bus needs unique IDs, so you set each
one **with only that single servo connected** (otherwise every ID-1
servo answers at once and you brick the address):

```bash
# Connect ONE new servo. It is ID 1 from the factory. Give it its ID:
python ../pi_control/feetech_bus.py --port /dev/ttyUSB0 setid --from 1 --to 3   # joint 2 (knee), leg 0
```

Logical joint → servo ID is simply **ID = joint + 1**
(`joint = leg*3 + axis`, axis 0=yaw, 1=hip, 2=knee):

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

## 2b. Where the Arduino Uno Q fits in

The Uno Q **is** the controller — there is no separate microcontroller
in the loop. The only thing between it and the servos is the passive
**USB bus-servo adapter** (a USB↔half-duplex-TTL converter, no CPU). Its
Linux side runs `feetech_bus.py`, which:

- maps the 18 logical joints to servo IDs 1..18,
- enforces the same safe per-axis angle limits in software,
- applies per-joint **trims** (stored in
  `../pi_control/feetech_trims.json`, since there's no EEPROM),
- sync-writes goal positions and reads back live feedback.

Everything below works identically whether the host is your laptop
(bench) or the Uno Q (on-robot): both talk to the **same USB bus-servo
adapter**, so only `--port` changes.

---

## 3. Servos — leg 0 (your built arm)

| Joint | Axis      | Servo ID | Limit (deg) |
|------:|-----------|:--------:|-------------|
| 0     | yaw       | 1        | ±35         |
| 1     | hip pitch | 2        | −80 … +30   |
| 2     | knee      | 3        | −20 … +80   |

STS3215 3-pin bus connector (both ports identical — chain in or out of
either). The connector is a **Molex 5264 / Mini-SPOX 2.5 mm**, 3-pin:

- **black → GND**
- **red → V+ (12 V)**
- **white/yellow → Signal**

Match the connector keying; the two ports on each servo are wired in
parallel, so "in" vs "out" doesn't matter electrically — pick whichever
makes the harness tidy. **But note (§6): the red V+ pin is rated only
~3 A, so do not let it carry more than one leg's current.**

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
Bench supply set to **12.0 V, current limit 2.0 A**. No servos.

- [ ] Output reads ~12.0 V on the meter.
- [ ] No short between V+ and GND leads (continuity = open).
- [ ] Supply OFF.

### Stage B — Uno Q + USB adapter on the bus, no servos
Uno Q powered (USB-C or buck); **USB bus-servo adapter plugged into the
Uno Q's USB-C** (mode set per §2: FE-URT-2 level switch → 5 V, or
Waveshare jumper → B); 12 V rail still OFF.

- [ ] `ls /dev/ttyUSB* /dev/ttyACM*` shows the adapter's port (note it,
      that's your `--port`).
- [ ] `scan` returns nothing yet (no servos / no power) — that's fine; it
      confirms the port opens.

### Stage C — One spare servo, off the arm
Connect a **spare** STS3215 to the bus signal and the 12 V rail. Supply
ON (12 V / 2 A limit).

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
ID the three leg-0 servos (1, 2, 3), chain them, plug into one **per-leg
branch** of the rail. **Clamp the arm, foot in the air.** Current limit
~**3 A**.

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

### Stage F — IMU (Uno Q I²C, separate from the servo bus)
The MPU-6050 is **4 wires** from the GY-521 breakout to the Uno Q's
I²C pins — nothing to do with the servo bus.

| GY-521 pin | → Uno Q                | What it is        |
|------------|------------------------|-------------------|
| `VCC`      | **3V3** power          | power — 3V3, **not** 5V |
| `GND`      | **GND**                | ground            |
| `SCL`      | **I²C SCL**            | I²C clock         |
| `SDA`      | **I²C SDA**            | I²C data          |
| `AD0`      | leave unconnected      | sets address `0x68` |
| `XDA`,`XCL`,`INT` | leave unconnected | not used          |

> ⚠ **Power the GY-521 from 3V3, never 5V** — its SDA/SCL pull-ups go
> to VCC and 5V would over-drive the 3.3 V I²C lines.

```bash
sudo apt install -y i2c-tools
i2cdetect -y 1           # expect 0x68
```

---

## 5. Switching from bench supply to LiPo

Only after Stages A–F pass, build the §6 power harness and then:

1. Bench supply OFF and disconnected.
2. Wire the full §6 harness:
   `3S LiPo XT60 → anti-spark switch → main fuse (15–20 A) → bus bar →
   per-leg branches`, plus the **buck** for the Uno Q's 5 V.
3. **Verify common ground** across LiPo −, bus-bar GND, Uno Q, IMU.
4. The **anti-spark switch is your e-stop** — keep a hand near it for
   the first powered run.

---

## 6. Distributed-power harness (all 18 servos)

This is the heart of the full-robot wiring. **Read it before you chain
all 18 servos to one rail.**

> This whole section is also rendered as a WireViz diagram —
> [`harness.svg`](harness.svg) (regenerate with
> `../scripts/render_harness_diagram.py`). The per-leg branch lengths in
> it come from the geometry-derived `../pi_control/wire_harness_plan.py`.

### 6.1 Why you cannot daisy-chain power

The FEETECH 3-pin bus cable is **22–24 AWG with Molex 5264 /
Mini-SPOX 2.5 mm** connectors rated only **~3 A per pin**. Data is
fine over that (milliamps). **Power is not.**

If you chain all 18 servos' **power** through one bus, the **first
segment carries the sum of everything downstream**:

| Load case                       | Current the upstream pin sees |
|---------------------------------|------------------------------:|
| Idle (18 × ~0.2 A)              | ~3.6 A  — already over 3 A    |
| Walking (18 × ~0.5 A)           | ~9 A    — 3× the pin rating   |
| Several servos stalling at once | up to **~49 A** (18 × 2.7 A)  |

A 9 A continuous draw through a 3 A pin **melts the upstream 5264
connector**. So **DATA stays chained; POWER must be DISTRIBUTED** so no
single 5264 pin ever carries more than **one leg** (~1.5 A walking,
with stall headroom).

### 6.2 Architecture

```
  3S LiPo (11.1 V nom / 12.6 V full)
     │  XT60
     ▼
  ANTI-SPARK SWITCH  (precharge + on/off = e-stop)
     │  XT60 → 12–14 AWG silicone
     ▼
  MAIN FUSE  15–20 A  (blade/ANL in holder)
     │  12–14 AWG silicone
     ▼
  ┌──────────────  POWER DISTRIBUTION BUS BAR  ──────────────┐
  │  (V+ bar)                                       (GND bar) │
  │   ├─[opt 5–7 A]─► leg 0 branch  16–18 AWG ──► 5264 → leg 0 servos (V+/GND)
  │   ├─[opt 5–7 A]─► leg 1 branch  16–18 AWG ──► 5264 → leg 1 servos
  │   ├─[opt 5–7 A]─► leg 2 branch  16–18 AWG ──► 5264 → leg 2 servos
  │   ├─[opt 5–7 A]─► leg 3 branch  16–18 AWG ──► 5264 → leg 3 servos
  │   ├─[opt 5–7 A]─► leg 4 branch  16–18 AWG ──► 5264 → leg 4 servos
  │   ├─[opt 5–7 A]─► leg 5 branch  16–18 AWG ──► 5264 → leg 5 servos
  │   └──────────────► XINGYHENG 12→5 V BUCK ──► Uno Q 5 V (logic)
  └──────────────────────────────────────────────────────────┘

  DATA (separate, low current):
  Uno Q ─USB─► adapter ─sig+GND─► ID1─►ID2─►ID3 ─┊─► ID4─►ID5─►ID6 ─┊─► … ─► ID18
                                 └── leg 0 ──┘    ▲   └── leg 1 ──┘
                        leg-to-leg jumper = SIGNAL + GND only (no V+)
```

**The two rules that prevent the melt:**

1. **Within a leg** (3 servos): keep the stock 3-pin chain
   (V+/GND/Signal). The leg's first 5264 pin carries only that leg's
   3 servos (~1.5 A walking) — under the 3 A rating. **Inject that
   leg's V+/GND from its own bus-bar branch** at the leg's first servo
   (or, for extra margin, at the middle/hip servo so no internal pin
   carries more than 2 servos).
2. **Leg-to-leg**: the cable that continues the DATA chain to the next
   leg carries **Signal + GND only** — **cut/omit the V+ wire**. Each
   leg already has its own V+ feed from the bus bar, so power never
   bridges between legs and no pin ever carries more than one leg.

The "bus bar" is best implemented as a **drone power-distribution
board** — the spec'd part is the **Matek PDB-XT60** (36 × 50 mm, 11 g,
XT60 input, 6 output pad pairs at 15 A continuous each + 1 VCC/GND
pair for the buck feed; ~4× margin over the ~3.7 A worst-case branch).
A brass/copper bus bar, a fused distribution block, or paralleled
ring-terminal posts also work electrically but weigh 10–30× more
(Jul 2026 weight fix).  Ignore the PDB's small on-board 5 V/12 V BECs;
the Uno Q stays on the XINGYHENG buck.  The optional per-branch fuses
(5–7 A) protect each leg's thin harness against a multi-servo stall
in that leg.

### 6.3 Per-branch current budget

STS3215 @ 12 V: idle ~0.2 A, walking ~0.3–0.7 A/servo, peak/loaded
~1–2 A, stall 2.7 A.

| Branch                | Servos | Idle  | Walking (typ) | Walking (high) | 1 servo stall* | Wire        | Fuse        |
|-----------------------|:------:|------:|--------------:|---------------:|---------------:|-------------|-------------|
| Leg 0 (ID 1/2/3)      | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 1 (ID 4/5/6)      | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 2 (ID 7/8/9)      | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 3 (ID 10/11/12)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 4 (ID 13/14/15)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 5 (ID 16/17/18)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Logic (Uno Q via buck)| —      | ~0.3 A (5 V side) |     |                |                | per buck    | (buck-int.) |
| **Main trunk (sum)**  | **18** | ~3.6 A| **~9 A**      | **~12.6 A**    | (stalls add)   | 12–14 AWG   | **15–20 A** |

\* "1 servo stall" = one servo of the leg at 2.7 A stall while the
other two walk (~0.5 A each). A whole-leg 3-servo stall is ~8 A and is
what the optional 5–7 A branch fuse is there to catch on a sustained
jam. The 15–20 A main fuse protects the trunk; it is sized above the
~9–12.6 A walking draw and below a dead-short, not to ride out an
all-18 multi-stall (which the controller's torque limits should
prevent anyway).

### 6.4 Wire & connector summary

| Run                              | Gauge / part                              | Carries            |
|----------------------------------|-------------------------------------------|--------------------|
| LiPo → switch → fuse → bus bar   | **12–14 AWG silicone** (XT60 pigtails)    | full robot ~9–13 A |
| Bus bar → per-leg branch         | **16–18 AWG silicone**                    | one leg ~1.5–3.7 A |
| Branch → leg first servo         | **Molex 5264 3-pin pigtail** (crimped)    | one leg            |
| Within-leg servo-to-servo        | stock FEETECH 3-pin (22–24 AWG, 5264)     | one leg            |
| Leg-to-leg (DATA only)           | 2-wire signal+GND (22–24 AWG) — **no V+** | signal (mA)        |
| Uno Q USB-C → USB adapter        | USB-C-to-C cable + **USB-C OTG/hub**      | USB data + 5 V log.|
| USB adapter `D`/`G` → bus        | 2-wire signal+GND (22–24 AWG) — **no V+** | signal (mA)        |
| Bus bar → buck → Uno Q           | 20–22 AWG (low current)                   | logic (~0.3 A @5V) |

> **Common ground is mandatory.** The bus-bar GND, the Uno Q GND, the
> buck output GND, and the **USB adapter's servo-power GND** must all be
> bonded, or the half-duplex signal has no return reference and the bus
> goes silent/garbled. (USB already shares GND between the Uno Q and the
> adapter's logic side; the point here is the adapter's **12 V servo
> terminal GND** must also tie to the bus-bar GND.)

### 6.5 Simulated stand-up draw (MuJoCo, Jul 2026)

`../standup_current_sim.py` simulates the robot standing up from a
belly-down, legs-folded pose in the `mujoco_prototype.py` model and
converts net joint torques to bus current with the STS3215 electrical
model at 12 V (`I = 0.2 A idle + 2.5 A × |τ|/2.94 N·m`, capped at the
2.7 A stall).  Realistic-mass case = 2.89 kg total (1.30 kg chassis
subtree: plates + yaw servos + the 300 g / 138 × 46 × 24 mm LiPo +
PDB/buck/Uno Q deck), servo gains ×5 to approximate the real PID.

| Quantity | Gentle stand-up (1.5 s) | Fast stand-up (0.4 s) | Budget limit |
|---|---|---|---|
| Trunk peak (instantaneous) | 12.5 A | 16.7 A | — (≪100 ms spike; fuses ignore it) |
| Trunk peak (100 ms avg)    | 12.5 A | 12.2 A | 15–20 A main fuse |
| Standing hold (steady)     | 10.3 A | 10.3 A | ~9–13 A walking budget (§6.3) |
| Worst leg branch peak      | 2.1 A  | 3.0 A  | 5–7 A branch fuse, 15 A PDB pad |
| Worst single-servo torque  | 1.1 N·m| 2.1 N·m| 2.94 N·m stall |

Verdict (all within budget):

- The robot stands and stays upright in every mass case; the stiff-gain
  case reaches ~81 % of stance height (remaining sag is pure-P droop in
  the sim — the real servo's integral term closes it).
- Branch fuses, 16–18 AWG branch wire, and the Matek PDB pads all carry
  ≥2× margin over the worst simulated leg (3.0 A).
- **Fit a 20 A main fuse** (top of the §6.3 range): a fast stand-up
  spikes to 16.7 A for ~50 ms.  Blade fuses need sustained ~135 % of
  rating for seconds to open, so even 15 A survives it, but 20 A means
  a hard stand-up plus a stumble never nuisance-blows the trunk.
- Ramp the stand-up over ≥1 s in firmware: it halves the worst knee
  torque (2.1 → 1.1 N·m) and keeps every servo far from stall.
- Runtime at the ~10.3 A hold: ~13 min on a 2200 mAh 3S, ~29 min on a
  5000 mAh pack (the 138 × 46 × 24 mm envelope class).
- Caveat: the linear torque→current model is conservative for static
  holds — a geared STS3215 parked inside its deadband draws less, so
  real standing draw should land at or below these figures.

Re-run `PYTHONPATH=. python standup_current_sim.py --fast` and refresh
this table whenever masses, servo gains, or leg geometry change.

---

## Per-joint log (fill in for all 6 legs)

| Joint | ID | Neutral | Safe min | Safe max | Dir (`JOINT_SIGN`) | Binds at | Notes |
|------:|---:|--------:|---------:|---------:|--------------------|----------|-------|
| 0 yaw | 1  | 0       |          |          |                    |          |       |
| 1 hip | 2  | −25     |          |          |                    |          |       |
| 2 knee| 3  | +60     |          |          |                    |          |       |

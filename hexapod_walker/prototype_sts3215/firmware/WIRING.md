# Hexapod prototype — wiring & bench bring-up (FEETECH bus servos)

Buildable wiring plan for the **18× FEETECH STS3215** serial-bus
hexapod, plus a bench bring-up checklist so you test everything **on a
current-limited bench supply first**, before the LiPo is ever
connected. Pair this with the host-side driver in
`../motor_setup/feetech_bus.py`.

> **Rendered harness diagram:** the full connector-by-connector WireViz
> diagram (with wire colors, gauges and the auto-generated BOM) lives at
> [`harness.svg`](harness.svg) / `harness.png` / `harness.html` next to
> this file. Regenerate it (after editing this doc's topology or the
> geometry in `../motor_setup/wire_harness_plan.py`) with
> `../scripts/render_harness_diagram.py` (repo venv).

**June 2026 redesign.** The prototype uses **18× FEETECH STS3215**
(ST-3215-C018, 12 V / 30 kg·cm) **serial-bus** smart servos (6 legs ×
yaw/hip/knee). Each servo has a built-in 12-bit magnetic encoder and
reports **position, load, voltage, current and temperature** back over
the bus, so the robot has closed-loop joint feedback with **no external
sensors**. This deletes most of the old stack: **no Arduino Mega, no
PCA9685 PWM boards, no servo BECs, and no AS5600 encoders.**

The controller is an **Arduino Uno Q** (on-board Linux SoC + MCU).
Python gait / teleop / demos run on the **Linux** side.

**Preferred bus path (live):** the STM32 MCU sketch
[`feetech_bridge`](feetech_bridge/) owns **D0/D1 → FE-URT UART at 1 Mbps**.
Linux talks to the sketch over **`/dev/ttyHS1` @ 921600**
(`linux_control/mcu_feetech_bus.py`).

**Power (Aug 2026 as-built):** **no external buck, no PDB**. The 3S
battery feeds two domains that share ground only — `battery → trunk
Wagos → power Wagos → servos` (the whole distribution is Wago 221 lever
nuts) and `battery → Uno Q` (on-board regulator). Electronics deck =
magnet-held round Ø115 mount plate (Uno Q + breakout on top, 3.3 V Wago
underneath) + raised platform (screen
on top; MPU glued on chassis_top beside the central trunk Wagos).

**Fallback (bench only):** a **USB bus-servo adapter** (FE-URT-2 or
Waveshare) on the Uno Q USB-C OTG port, enumerated as `/dev/ttyUSB*`,
driven directly by `feetech_bus.py` at 1 Mbps — useful when the MCU
bridge is not flashed.

> ⚠ **The USB fallback does NOT work when the Uno Q is VIN-powered**
> (i.e. on-robot, battery → VIN).  Two documented Uno Q issues: with no
> USB-C cable at boot there is no CC negotiation, so the USB controller
> comes up in **device** mode (fixed in newer OS images; older ones need
> `echo host | sudo tee /sys/kernel/debug/usb/4e00000.usb/mode`, see
> [arduino/linux-qcom#2](https://github.com/arduino/linux-qcom/issues/2));
> and the board **never sources 5 V out of the USB-C jack from VIN** (a
> diode isolates VBUS from 5V_SYS), so the adapter stays unpowered
> unless it hangs off a powered PD hub.  This is why the robot uses the
> **D0/D1 UART path** — it is the only practical link under VIN power.

> **Buy one USB/TTL bus-servo adapter** (needed for either UART or USB mode):
> - **FEETECH FE-URT-2** (`FE-URT2-C001`) — Type-C, TTL-BUS for STS/SCS,
>   RS485 for SMS, **3.3 V / 5 V** logic switch (set **5 V** for USB host
>   mode; MCU UART path is 3.3 V-safe via the Uno Q pins — follow the
>   bridge wiring).  Screw terminal for servo power.
> - **Waveshare Bus Servo Adapter (A)** (SKU **25514**) — USB-C, D/V/G
>   header; jumper **A = UART to MCU**, **B = USB to host**.
>
> **USB-C host note:** when using the USB fallback, the Uno Q's single
> Type-C is in **OTG / host** mode — use a hub with power passthrough if
> you still need to power/debug the board over USB.

Your first built arm is **leg 0 = joints 0 / 1 / 2 = servo IDs 2 / 3 /
4**. This doc gets that leg moving safely, then scales to all 18 with a
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
> 11.1 V nominal / 12.6 V full). There is **no servo BEC** and **no
> external buck** — the raw 3S rail goes to the Wago→servo path, and a
> separate battery tap feeds the Uno Q (its on-board regulator).

---

## 1. Two power domains (share ground only)

Aug 2026 as-built: **no external buck, no PDB**. Battery → trunk Wagos →
servos, and battery → Uno Q, are separate feeds that share ground only.

```
   BENCH SUPPLY (set 12.0 V, limit 2 A for one leg)   ← swap in 3S LiPo later
            │
            ├─► central trunk Wagos (V+/GND pair of 5-port 221-415) ─► corner POWER Wago pairs (12V+G) ─► per-leg branches
            │         (V+/GND injected per leg, NOT chained leg-to-leg)
            │
            └─► Uno Q VIN (on-board regulator; own battery tap)

   DATA:  Uno Q ─sig+GND─► underside DATA Wagos (near yaw retainers)
              ─► yaw→hip→knee … (signal+GND chained; V+ fed per leg)

   DECK:  magnet hex plate (Uno Q + breakout)
          + raised platform (screen on top)
          + MPU glued on chassis_top (beside the trunk Wagos, near centre)
          MPU ──I²C──► Uno Q  (Stage F)

   ground common (LiPo −, trunk-Wago GND, Uno Q GND, adapter GND, IMU GND)
```

- **Servo power** (12 V) comes from the **central trunk Wago splice
  pair** — two **5-port Wago 221-415** side by side at the chassis_top
  centre (one V+, one GND) — then the **corner power Wago pairs** — one V+/GND pair of 3-port
  221-413 seated between tray walls printed into the chassis_bottom
  top face at each hex corner flat (between adjacent yaw cradles,
  entries facing inward; late-Aug 2026 — no separate tray part) —
  then **per-leg branches** in heavy
  silicone. Power is **not** passed leg-to-leg through the thin servo
  connector pins (see §6).
- **Uno Q power** is a **separate battery tap** (no buck on the servo
  rail). On the bench you can still USB-power the board.
- **Data** is one half-duplex TTL signal, daisy-chained servo to servo.
  On the robot, data jumpers land at **underside data Wagos** near the
  yaw retainers. Preferred path is the MCU `feetech_bridge`; USB adapter
  is the fallback (§ intro / §2).
- **All grounds common**: bench-supply − (or LiPo −), trunk-Wago GND,
  Uno Q GND, adapter GND, IMU GND. Verify with a continuity beep.
- On the bench (one leg) you may feed 12 V into the adapter's power
  terminal; on the full robot the adapter (if used) is **data-only**
  and each leg gets 12 V from the trunk / power Wagos (see §2 and §6).
- **The IMU is on the Uno Q's I²C bus**, glued on chassis_top just
  south of the central trunk Wagos (inboard, header row facing the
  open -X deck so its wires have room; not under the raised platform).
  Totally separate from the servo bus; see Stage F.

---

## 2. The serial bus — Uno Q ↔ bus-servo adapter ↔ servos (DATA)

There is exactly **one** data signal on the robot: the half-duplex TTL
bus, daisy-chained servo to servo. As-built it reaches the Uno Q over a
**3-wire TX/RX/GND pigtail** from the MCU's **D0/D1** pins to the
adapter's UART/MCU header (`feetech_bridge` sketch, § intro); the USB
OTG hookup is a bench-only fallback (and does not work on VIN power —
see the ⚠ note in § intro). No I²C on the servos, no per-servo PWM
lines.

```
  Uno Q ─D1/D0 TX/RX+GND─► adapter ─TTL (1 Mbps)─► ID1 ─► ID2 ─► … ─► ID18
       │  (feetech_bridge MCU UART)  half-duplex, single signal + common GND
       └──I²C (separate bus)──► MPU-6050 IMU

  (bench fallback: Uno Q ─USB-C OTG/powered hub─► adapter, jumper B)
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
    **A/B mode jumper** — as-built set it to **A (UART)**: the Uno Q's
    D0/D1 pigtail lands on the UART/MCU header.  **B (USB)** is only for
    the bench USB fallback.
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

Every servo ships as **ID 1**. A bus needs unique IDs, so you re-ID each
one as you add it. **ID 1 is never assigned to a robot joint** — leaving
the factory default free means a fresh servo can always join a live
daisy-chain without colliding. Prefer the naming wizard
(`motor_setup/urt2_motor_setup.py`) — plug in a servo, watch it wiggle,
then assign any open joint (`L2 knee`, menu pick, …).

```bash
# Free-form labeling (any order):
python ../motor_setup/urt2_motor_setup.py

# Or by hand: connect a factory servo (ID 1) and give it its robot ID:
python ../motor_setup/feetech_bus.py --port /dev/ttyUSB0 setid --from 1 --to 4   # joint 2 (knee), leg 0
```

Logical joint → servo ID is **ID = joint + 2** (IDs 2..19;
`joint = leg*3 + axis`, axis 0=yaw, 1=hip, 2=knee):

| Leg | yaw (axis0) | hip (axis1) | knee (axis2) |
|----:|:-----------:|:-----------:|:------------:|
| 0   | ID 2        | ID 3        | ID 4         |
| 1   | ID 5        | ID 6        | ID 7         |
| 2   | ID 8        | ID 9        | ID 10        |
| 3   | ID 11       | ID 12       | ID 13        |
| 4   | ID 14       | ID 15       | ID 16        |
| 5   | ID 17       | ID 18       | ID 19        |

Label each servo as you ID it. After all are chained, verify:

```bash
python ../motor_setup/feetech_bus.py --port /dev/ttyUSB0 scan   # expect IDs 2..19 (not 1)
```

---

## 2b. Where the Arduino Uno Q fits in

The Uno Q **is** the controller — there is no separate microcontroller
in the loop. The only thing between it and the servos is the passive
**USB bus-servo adapter** (a USB↔half-duplex-TTL converter, no CPU). Its
Linux side runs `feetech_bus.py`, which:

- maps the 18 logical joints to servo IDs 2..19 (ID 1 left free),
- enforces the same safe per-axis angle limits in software,
- applies per-joint **trims** (stored in
  `../motor_setup/feetech_trims.json`, since there's no EEPROM),
- sync-writes goal positions and reads back live feedback.

Everything below works identically whether the host is your laptop
(bench) or the Uno Q (on-robot): both talk to the **same USB bus-servo
adapter**, so only `--port` changes.

---

## 3. Servos — leg 0 (your built arm)

| Joint | Axis      | Servo ID | Limit (deg) |
|------:|-----------|:--------:|-------------|
| 0     | yaw       | 2        | ±35         |
| 1     | hip pitch | 3        | −80 … +30   |
| 2     | knee      | 4        | −20 … +80   |

### Where the wires leave an STS3215 (physical)

Feetech’s STS3215 is **not** a classic hobby-servo with a molded boot on
one short end (that was the old DS3225). On the real part (FEETECH STEP
`STS3215_03a`, Waveshare/SO-ARM mounts, and this project’s retainer /
test-cradle notes):

| Item | Fact |
|------|------|
| Ports | **Two** identical 3-pin bus sockets (daisy in / out) |
| Connector | **Molex 5264 / Mini-SPOX 2.5 mm** (often casually called “JST”) |
| Face | Recessed in the **BACK (idler) face** — opposite the output / disc-horn face |
| Side of that face | **Centre / −X half** of the back (away from the output-offset shaft at +X); the +X half of the back carries the idler hub |
| Exit direction | Cables plug into the back and leave **out the back** (when the servo hangs output-up under the chassis, that is **straight down** through the yaw retainer drop window) |
| Pinout | black → GND · red → V+ (12 V) · white/yellow → Signal |

Both ports are wired in parallel — “in” vs “out” is only harness
tidiness. **But note (§6): the red V+ pin is rated only ~3 A, so do not
let it carry more than one leg’s current.**

BuildViz / CAD note: `hexapod_prototype.WIRE_BOOT_*` and the little box on
`make_servo_body` are a **legacy DS3225-shaped stand-in** kept so some
cradle clearance channels still have a probe target. **Harness routes in
`tools/full_robot_viz_build.py` attach at the real back-face port cluster
above**, not at that +X stub.

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

| Axis (joint)   | `+` angle moves the link…        | `−` angle moves it… | Range      | Stand plant |
|----------------|----------------------------------|---------------------|------------|------------:|
| yaw (j0)       | swings horizontally one way      | the other way       | ±35°       | 0°          |
| hip pitch (j1) | femur tip **down**               | femur tip **up**    | −80 … +30° | **+20°**    |
| knee pitch (j2)| tibia tip **down** (folds under) | tibia tip **up**    | −20 … +80° | **+80°**    |

So the **default hardware stand plant (0, +20, +80)** is femur angled
toward the floor with tibia steep (or a learned plant from Calibrate →
Plant height).  CAD / MuJoCo / `sts` RL still use the older crouch
**(0, −25, +60)** — see `../RL_PLAN.md` Appendix A.

### Mounting each horn against zero

1. `python ../motor_setup/feetech_bus.py --port /dev/ttyUSB0 centre` —
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
Uno Q powered **over USB-C** (NOT VIN/battery tap — a VIN-powered Uno Q
gives no USB host mode and no VBUS, § intro ⚠ note); **USB bus-servo
adapter plugged into the Uno Q's USB-C** (mode set per §2: FE-URT-2
level switch → 5 V, or Waveshare jumper → B); 12 V rail still OFF.
On-robot / VIN power, skip the USB stages and use the D0/D1 UART bridge.

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

### Stage D — Leg 0 servos (IDs 2/3/4), one joint at a time
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

### Stage F — IMU (MCU Wire I²C on SDA/SCL, separate from the servo bus)
The MPU-6050 is **4 wires** from the GY-521 breakout to the Uno Q
**header SDA/SCL (D20/D21)** — STM32 `Wire`, **not** Linux SoC I²C.
Linux `i2cdetect` on `/dev/i2c-*` will **never** show `0x68` for this
wiring; the MCU sketch owns the bus. As-built (Aug 2026): the breakout
is **glued on `chassis_top`** just south of the central trunk Wago pair
(inboard, r = 43), right-angle header row facing the open -X deck so
the four jumpers have clear exit room; short run up to the Uno Q.

| GY-521 pin | → Uno Q                | What it is        |
|------------|------------------------|-------------------|
| `VCC`      | **3V3** power (via the 3.3 V Wago under the mount plate) | power — 3V3, **not** 5V |
| `GND`      | **GND**                | ground            |
| `SCL`      | **SCL (D21)**          | MCU Wire clock    |
| `SDA`      | **SDA (D20)**          | MCU Wire data     |
| `AD0`      | leave unconnected      | sets address `0x68` |
| `XDA`,`XCL`,`INT` | leave unconnected | not used          |

> ⚠ **Power the GY-521 from 3V3, never 5V** — its SDA/SCL pull-ups go
> to VCC and 5V would over-drive the 3.3 V I²C lines.
>
> As-built (Aug 2026): the 3.3 V loads splice at a **5-port Wago 221-415
> VHB'd under the round mount plate** (south rim, entries facing the
> rim), fed ONCE from the Uno Q 3V3 pin through the plate's east Ø8 wire
> port — one lever nut is the whole 3.3 V rail.  Current load = GY-521
> VCC only (3 spares): the **screen does NOT tap this Wago** — its whole
> 8-wire pigtail (VCC GND SCL SDA RES DC CS BLK) runs from the Uno Q up
> the az-330 platform leg and through the 24×5 wire slot in the
> platform's top plate; the panel is held by 4× M2 self-tappers in the
> top plate's corner pilot holes.  Screen pin map (Uno layout, SPI):
> VCC → **3V3**, GND → **GND** (power header), SCL → **D13** (SCK),
> SDA → **D11** (MOSI), CS → **D10**, DC → **D9**, RES → **D8**,
> BLK → **D7**.
>
> Do **not** use A4/A5 for this board's main `Wire` — on Uno Q those are
> not the labeled SDA/SCL header. Qwiic is a second MCU bus (`Wire1`).

Flash / keep [`feetech_bridge`](feetech_bridge/) (commands `I2CSCAN`,
`IMU`, `IMUR`), then on the Uno Q Linux side:

```bash
# after flash_feetech_bridge.sh — probe over /dev/ttyHS1
cd ~/linux_control   # or wherever mpu_probe.py + mcu_feetech_bus.py live
python3 mpu_probe.py
# expect: I2CSCAN → OK 0x68   IMU → OK 0x68   IMUR → OK <ax> ...
```

If `IMU` returns `ERR no_ack`: confirm VCC=**3V3**, GND common, and
SDA→SDA / SCL→SCL (not swapped, not on high-speed 1.8 V headers).

---

## 5. Switching from bench supply to LiPo

Only after Stages A–F pass, build the §6 power harness and then:

1. Bench supply OFF and disconnected.
2. Wire the full §6 harness:
   `3S LiPo XT60 → anti-spark switch → main fuse (15–20 A) → trunk
   Wagos → power Wagos → per-leg branches`, plus a **separate battery
   tap to the Uno Q** (no buck).
3. **Verify common ground** across LiPo −, trunk-Wago GND, Uno Q, IMU.
4. The **anti-spark switch is your e-stop** — keep a hand near it for
   the first powered run.

---

## 6. Distributed-power harness (all 18 servos)

This is the heart of the full-robot wiring. **Read it before you chain
all 18 servos to one rail.**

> This whole section is also rendered as a WireViz diagram —
> [`harness.svg`](harness.svg) (regenerate with
> `../scripts/render_harness_diagram.py`). The per-leg branch lengths in
> it come from the geometry-derived `../motor_setup/wire_harness_plan.py`.

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
     ├──────────────────────────────────────► Uno Q VIN  (own tap; no buck)
     ▼
  ANTI-SPARK SWITCH  (precharge + on/off = e-stop)
     │  XT60 → 12–14 AWG silicone
     ▼
  MAIN FUSE  15–20 A  (blade/ANL in holder)
     │  12–14 AWG silicone
     ▼
  ┌──────────  TRUNK WAGOS (V+ nut + GND nut, chassis_top)  ──────────┐
  │  (V+ splice)                                        (GND splice)  │
  │   ├─► chassis-top POWER Wago (12V+G) ─► leg 0 branch …
  │   ├─► chassis-top POWER Wago (12V+G) ─► leg 1 branch …
  │   ├─► … one Wago + branch per leg …
  │   └─► chassis-top POWER Wago (12V+G) ─► leg 5 branch …
  └────────────────────────────────────────────────────────────────────┘
  (as-built Aug 2026: no PDB — the whole distribution is Wago 221 lever nuts)

  DECK (magnet hex + raised platform):
      4× posts at CHASSIS_STANDOFF_HOLES_XY (±31.1):
         20 mm standoff + ~2.5 mm M3 thumb nut + Ø8×8 mm magnet
      → round_mount_plate_115 (Uno Q + breakout on top; 3.3 V Wago under)
      → hex_raised_platform_110 (screen on top)
      → MPU-6050 glued on chassis_top (beside trunk Wagos, near centre)

  DATA (separate, low current) — underside DATA Wagos near yaw retainers;
  **as-built entry is at each yaw**:
  Uno Q ─sig+GND─► data Wagos ─►
      ┌─► L0 yaw ─► L0 hip ─► L0 knee ─┐
      │   (leg 0 data through the leg)  │  leg-to-leg = SIGNAL+GND only
      ├─► L1 yaw ─► L1 hip ─► L1 knee ─┤  (no V+ between legs)
      ├─► …                            │
      └─► L5 yaw ─► L5 hip ─► L5 knee ─┘

  POWER (per-leg trunk-Wago branch) — **as-built inject at each hip**:
      power Wago ─V+/GND─► L? hip ─┬─► L? yaw
                                  └─► L? knee
      (hip is the power tee; yaw/knee take V+ from the hip, not from
       the chassis branch directly)

  LiPo velcro under chassis; data Wagos under chassis near yaw retainers.
```

**As-built per-leg topology (Aug 2026 bench build):**

| Rail | Enters the leg at | Then goes |
|------|-------------------|-----------|
| **DATA** (signal + GND) | **yaw** first | yaw → hip → knee (out through the leg), then leg-to-leg jumper to the next yaw |
| **POWER** (V+ / GND) | **hip** (bus-bar branch) | hip → yaw and hip → knee |

Diagnostic consequence: a yaw that **blinks (has V+)** but **does not answer** usually still has power via the hip tee, while its **DATA in** (chassis → yaw) or the yaw UART is dead. Hip/knee on that leg may still answer if they remain on the data chain past a deaf yaw — or drop off entirely if the chain is broken at that yaw.

**The two rules that prevent the melt:**

1. **Within a leg** (3 servos): keep the stock 3-pin chain for DATA
   (and local V+ after the hip inject). The leg's bus-bar branch
   injects **at the hip** so the chassis 5264 only feeds that tee;
   yaw and knee draw through the hip's second port / harness tee
   (still one leg's current on the branch — under the 3 A rating).
2. **Leg-to-leg**: the cable that continues the DATA chain to the next
   leg carries **Signal + GND only** — **cut/omit the V+ wire**. Each
   leg already has its own V+ feed from the trunk / power Wago at the hip,
   so power never bridges between legs and no pin ever carries more than
   one leg.

### 6.2.1 Fat power wires vs DATA (signal integrity)

The STS bus is **1 Mbps half-duplex TTL** — easy to upset. Thick V+/GND
leads are fine electrically for current, but if they **run parallel /
bundled** with the thin DATA line for more than a short hop they can
couple switching noise (motor PWM / load steps) into the signal and
make one servo look “deaf” (LED still blinks from V+, scan misses it)
or flaky.

Practical rules for this as-built topology:

- Keep the **hip power branch** (thick wire) **physically apart** from
  the **yaw data in** lead; cross at ~90° if they must meet, don’t
  zip-tie them side-by-side along the coxa.
- DATA should travel as **signal + its own GND pair** (twisted if you
  extend it). Do **not** rely on the fat power GND alone as the UART
  return over a long run — motor return current on that GND shifts the
  logic reference.
- Short stock 3-pin pigtails yaw↔hip↔knee are usually OK even with
  local V+ in the same plug; trouble shows up on **long chassis→yaw
  data runs** laid next to the bus-bar branch.
- Quick A/B test: temporarily route a **short, separate** signal+GND
  only into that yaw (power still from the hip). If it suddenly
  answers, the thick parallel power run was the culprit — not the
  servo.
- Optional: try `--baud 500000` (or 115200) once; if a “missing” yaw
  appears only at lower baud, treat it as noise/edge-rate until the
  layout is cleaned up, then return to 1 Mbps.

### 6.2.2 Adding legs one at a time (bring-up)

The bus often looks fine with 1–3 legs and then misbehaves when the
**4th** data stub goes on: extra cable capacitance + another yaw tap at
1 Mbps, or a bad **leg-to-leg** jumper (especially if V+ was left
connected on a “data-only” cable).

Bring-up rule:

1. Confirm scan with N legs.
2. Add leg N+1 **data** only after its power branch is sane; prefer not
   hot-plugging the data jumper under a heavy load.
3. If something that used to answer goes offline (e.g. L0 yaw LED blinks
   but scan misses it): **unplug the newest leg’s data jumper first**.
   If the old motors return, the newest stub/jumper is guilty — not the
   yaw servo itself.
4. Test the newest leg **alone** on the URT-2; then 3 good legs; then
   add the 4th.
5. Interactive checklist: `urt2_motor_setup.py --debug` → “new leg broke
   the bus” bring-up.

The "bus bar" is implemented as a **central trunk Wago splice pair**
(as-built Aug 2026 — an earlier revision spec'd a Matek PDB-XT60 drone
power-distribution board; the build dropped it): two **5-port Wago
221-415** side by side at the chassis_top centre — one splices the
fused V+ trunk into the branch feeds, the other splices GND (each nut:
1 trunk port in + 4 branch ports; share ports across the six branches
as wired).  **Polarity convention: V+ = the SOUTH nut (−Y, nearest the
switch), GND = the NORTH nut.**  At each corner tray the convention is
**V+ = the nut clockwise of the corner's outward ray, GND =
counterclockwise** (viewed from above; same at every corner, so a
red/black pair never crosses inside a tray).  A Wago 221 is rated 32 A / 4 mm² — well above the ~13 A
walking trunk.  From the trunk pair, land each motor branch on its leg's
**corner power Wago pair** (12 V+G; two 3-port 221-413 seated in the
tray walls printed into chassis_bottom at the hex corner) before the leg
harness.  The Uno Q
takes a **direct battery tap** (no external buck).  The optional
per-branch fuses (5–7 A) protect each leg's thin harness against a
multi-servo stall in that leg.

### 6.3 Per-branch current budget

STS3215 @ 12 V: idle ~0.2 A, walking ~0.3–0.7 A/servo, peak/loaded
~1–2 A, stall 2.7 A.

| Branch                | Servos | Idle  | Walking (typ) | Walking (high) | 1 servo stall* | Wire        | Fuse        |
|-----------------------|:------:|------:|--------------:|---------------:|---------------:|-------------|-------------|
| Leg 0 (ID 2/3/4)      | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 1 (ID 5/6/7)      | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 2 (ID 8/9/10)     | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 3 (ID 11/12/13)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 4 (ID 14/15/16)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Leg 5 (ID 17/18/19)   | 3      | 0.6 A | ~1.5 A        | ~2.1 A         | ~3.7 A         | 16–18 AWG   | opt 5–7 A   |
| Logic (Uno Q battery tap)| —   | ~0.3–0.5 A (board) |   |                |                | 20–22 AWG   | (board)     |
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
| LiPo → switch → fuse → trunk Wagos | **12–14 AWG silicone** (XT60 pigtails)  | full robot ~9–13 A |
| Trunk Wagos → power Wago → per-leg branch | **16–18 AWG silicone** + Wago 221 | one leg ~1.5–3.7 A |
| Branch → leg first servo         | **Molex 5264 3-pin pigtail** (crimped)    | one leg            |
| Within-leg servo-to-servo        | stock FEETECH 3-pin (22–24 AWG, 5264)     | one leg            |
| Leg-to-leg (DATA only)           | 2-wire signal+GND via underside data Wagos — **no V+** | signal (mA) |
| LiPo → Uno Q VIN                 | 20–22 AWG (separate battery tap)          | Uno Q (~0.3–0.5 A) |
| Uno Q USB-C → USB adapter (fallback) | USB-C-to-C + **USB-C OTG/hub**        | USB data           |
| USB adapter `D`/`G` → bus        | 2-wire signal+GND (22–24 AWG) — **no V+** | signal (mA)        |

> **Common ground is mandatory.** The trunk-Wago GND, the Uno Q GND, and
> the **USB adapter's servo-power GND** (if used) must all be bonded, or
> the half-duplex signal has no return reference and the bus goes
> silent/garbled. (USB already shares GND between the Uno Q and the
> adapter's logic side; the point here is the adapter's **12 V servo
> terminal GND** must also tie to the trunk-Wago GND.)

### 6.5 Simulated stand-up draw (MuJoCo, Jul 2026)

`../standup_current_sim.py` simulates the robot standing up from a
belly-down, legs-folded pose in the `mujoco_prototype.py` model and
converts net joint torques to bus current with the STS3215 electrical
model at 12 V (`I = 0.2 A idle + 2.5 A × |τ|/2.94 N·m`, capped at the
2.7 A stall).  Realistic-mass case = 2.89 kg total (1.30 kg chassis
subtree: plates + yaw servos + the battery — now 2 × 137 g shorty
LiPos under the belly, ~same mass as the retired 300 g bay pack —
+ Wagos/Uno Q/hex deck), servo gains ×5 to approximate the real PID.

| Quantity | Gentle stand-up (1.5 s) | Fast stand-up (0.4 s) | Budget limit |
|---|---|---|---|
| Trunk peak (instantaneous) | 12.5 A | 16.7 A | — (≪100 ms spike; fuses ignore it) |
| Trunk peak (100 ms avg)    | 12.5 A | 12.2 A | 15–20 A main fuse |
| Standing hold (steady)     | 10.3 A | 10.3 A | ~9–13 A walking budget (§6.3) |
| Worst leg branch peak      | 2.1 A  | 3.0 A  | 5–7 A branch fuse, 32 A Wago joint |
| Worst single-servo torque  | 1.1 N·m| 2.1 N·m| 2.94 N·m stall |

Verdict (all within budget):

- The robot stands and stays upright in every mass case; the stiff-gain
  case reaches ~81 % of stance height (remaining sag is pure-P droop in
  the sim — the real servo's integral term closes it).
- Branch fuses, 16–18 AWG branch wire, and the 32 A Wago 221 joints all
  carry ≥2× margin over the worst simulated leg (3.0 A).
- **Fit a 20 A main fuse** (top of the §6.3 range): a fast stand-up
  spikes to 16.7 A for ~50 ms.  Blade fuses need sustained ~135 % of
  rating for seconds to open, so even 15 A survives it, but 20 A means
  a hard stand-up plus a stumble never nuisance-blows the trunk.
- Ramp the stand-up over ≥1 s in firmware: it halves the worst knee
  torque (2.1 → 1.1 N·m) and keeps every servo far from stall.
- Runtime at the ~10.3 A hold: ~13 min on one 2200 mAh 3S, ~26 min on
  the as-built pair of paralleled 2200 mAh shorty packs (4400 mAh).
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

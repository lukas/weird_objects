# `rideable_v1` — Actuators, Drivetrain & the Knee-Brake Load-Holding Architecture

> How the 18 joints are actuated, and — the central feature of this
> redesign — how the machine **holds a rider's weight while standing with
> zero motor current** using a spring-applied, fail-safe electromagnetic
> brake on each knee. All torque, speed, and mass numbers trace back to
> the [design basis in `README.md` §2](README.md#2-design-basis-decided).

---

## 1. The problem with standing on QDD motors

Quasi-direct-drive (QDD) actuators — a large-diameter motor behind a
single low-ratio planetary stage — are wonderful for legged robots
because they are **backdrivable, fast, and torque-dense**. But that same
low ratio means the motor windings carry almost the full holding current
whenever the joint holds a static load. Holding torque at zero speed is
the worst-case thermal condition for a motor: no back-EMF, no cooling
from motion, `I²R` heat dumped straight into the windings.

A rideable hexapod spends most of its life **standing still** — mounting,
dismounting, stopped, holding a pose. If the hip and knee motors had to
hold the rider's weight on current, they would cook within minutes. So
the architecture is built around a simple rule:

> **The motors move the legs. Brakes hold the legs. The machine never
> stands on motor current.**

Two things follow from that rule: a **QDD + secondary-reduction** drive
that has huge peak torque for walking, and a **fail-safe brake** on every
holding joint that takes over the instant the leg is planted.

---

## 2. Actuator lineup (decided)

| Joint | Qty | Actuator | Rated τ | Peak τ | Bus | Internal ratio | Rated speed | Mass | Unit $ | Drive |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| **Hip-pitch** | 6 | MyActuator **RMD-X15-450** | 145 N·m | 450 N·m | 72 V | 20.25:1 | 98 rpm | 3.5 kg | ~$1,515 | + **6:1 secondary** |
| **Knee** | 6 | MyActuator **RMD-X15-450** | 145 N·m | 450 N·m | 72 V | 20.25:1 | 98 rpm | 3.5 kg | ~$1,515 | + **6:1 secondary** + **brake** |
| **Hip-yaw** | 6 | MyActuator **RMD-X8-120** | 43 N·m | 120 N·m | 48 V | 19.6:1 | 127 rpm | 1.4 kg | ~$645 | **direct** (no secondary) |

* The **hip-pitch and knee** joints are the high-torque joints and share a
  single part number (RMD-X15-450). Buying one SKU for the twelve
  hardest joints keeps spares simple.
* The **hip-yaw** joint only fights friction and lateral imbalance
  (~50–80 N·m), so a smaller RMD-X8-120 driven **directly** off its own
  20:1-ish internal reduction is plenty — 1.5× peak margin and a huge
  speed margin.
* **Bus split:** the twelve X15s run on the **72 V** main pack; the six
  X8s run on a **48 V** rail fed by a 72→48 V DC-DC (see
  [`POWER_SYSTEM.md`](POWER_SYSTEM.md)). Yaw draws little power, so the
  DC-DC is small.

---

## 3. The 6:1 secondary reduction

Each hip-pitch and knee actuator drives its joint through a **~6:1
secondary reduction** — the actuator output shaft is the *input* (fast
side), and the joint is the *output* (slow side). Three interchangeable
ways to build it:

| Option | Ratio | Notes |
|---|---:|---|
| **HTD-14M timing belt** | ~6:1 | Quiet, compliant, tolerant of shock, easy to tension. Big pulley on the joint, small on the actuator. Recommended default. |
| **#40 roller chain** | ~6:1 | Cheapest, most shock-tolerant, needs a guard + lubrication. |
| **Planetary stage** | 6:1 | Most compact and stiff, most expensive, lowest backlash. |

What the 6:1 buys at the joint, from the RMD-X15-450 (secondary
efficiency assumed **η ≈ 0.9**):

```
    Joint peak torque       = 450 N·m × 6 × 0.90  ≈  2,430 N·m
    Joint continuous torque = 145 N·m × 6 × 0.90  ≈    783 N·m
    Joint speed             = 98 rpm ÷ 6           =  16.3 rpm  ≈  98 °/s
```

### Torque margins (against the [design basis](README.md#2-design-basis-decided))

| Joint | Design load | Joint peak available | **Peak margin** | Nominal (static) | Joint continuous |
|---|---:|---:|---:|---:|---:|
| Hip-pitch | 1.2 kN·m | 2,430 N·m | **~2.0×** | ~0.8 kN·m | 783 N·m |
| Knee | ~0.85 kN·m | 2,430 N·m | **~2.9×** | ~0.85 kN·m | 783 N·m |
| Hip-yaw | ~0.08 kN·m | 120 N·m (direct) | **~1.5×** | ~0.05–0.08 kN·m | 43 N·m |

### Speed margin

The gait needs **~82 °/s** at the hip and knee for 0.4 m/s. The
drivetrain delivers **~98 °/s**, a ~20% margin. Yaw delivers 127 rpm
directly (~762 °/s) — effectively unlimited for this gait.

> **The continuous number is the whole point of the brakes.** Note that
> the ~783 N·m joint *continuous* rating is only about equal to the
> ~800 N·m nominal *static* hip torque. In dynamic walking the RMS torque
> per joint is well below continuous, so the motors are fine on the move.
> But holding the rider statically sits right at the thermal limit — which
> is exactly why the machine hands standing loads to the brakes ([§4](#4-the-knee-brake-load-holding-architecture)).

---

## 4. The knee-brake load-holding architecture

This is the central structural feature of `rideable_v1`.

### 4.1 What and where

On **each knee** (and, optionally, each hip-pitch), a **spring-applied,
electrically-released electromagnetic brake** is mounted on the
**actuator output shaft** — which is the **input (fast side) of the 6:1
secondary reduction**.

* **Spring-applied / electrically-released** means **power-OFF = engaged**.
  A friction disc is clamped by springs; energising the coil pulls the
  clamp back and releases it. Remove power and the springs re-clamp. This
  is the fail-safe direction: the machine defaults to *locked*, not
  *free*.
* **On the fast shaft**, the brake sees the joint torque *divided by the
  secondary ratio*, so it can be small and cheap.

### 4.2 The load path

```
    rider + vehicle weight
            │
            ▼   (foot force, up to 2.0 kN design per planted leg)
        [ foot ] ─ [ tibia ] ─────────────────────────────┐
                                                            │  knee joint
                                      knee JOINT OUTPUT (slow side) ◄── up to ~0.85 kN·m holding
                                                            │
                                      6:1 SECONDARY REDUCTION
                                                            │  torque ÷ 6 (× 1/η)
                                      brake-held INPUT SHAFT ◄── ~160 N·m held by the brake
                                                            │
                                      [ FAIL-SAFE BRAKE ] ── clamps the input shaft to the housing
                                                            │
                                      housing / leg frame ─ chassis ─ GROUND
```

Read it bottom-up: the leg frame is grounded through the chassis and the
planted feet; the brake clamps the actuator input shaft to that frame;
the 6:1 secondary multiplies the brake's grip up to the full joint
holding torque; and the tibia+foot deliver the rider's weight into it.
Nowhere in that chain does a motor winding carry current.

### 4.3 Brake sizing math

The knee holding torque when the leg is planted (see [§5](#5-standing-load-case-the-common-case)):

```
    τ_knee(hold)  ≈  0.85 kN·m  =  850 N·m   at the joint (slow side)
```

The brake sits on the input (fast) side of the 6:1 secondary, so the
torque it must hold is the joint torque reflected through the reduction,
accounting for efficiency:

```
    τ_brake  =  τ_knee(hold) / (N_secondary × η)
             =  850 N·m / (6 × 0.90)
             ≈  157 N·m   →  spec a ~160 N·m brake
```

A ~160 N·m spring-applied brake is a **compact, ~$150–250, ~0.5 kg**
component (many industrial servo / robot-joint fail-safe brakes land in
this class). Contrast with braking the joint *directly* on the slow side:
that brake would need to hold the full ~850 N·m (and ~1.2 kN·m if you
sized it to the hip design load), which is a much larger, heavier, and
pricier device. **Putting the brake on the fast shaft is what makes a
fail-safe brake on every joint affordable.**

> Apply a holding-torque service factor when you buy: pick a brake with a
> **static rated torque ≥ 1.5 × 160 N·m ≈ 240 N·m** so the 160 N·m
> working point sits comfortably inside the brake's rating with margin
> for disc wear and dynamic snatch loads.

### 4.4 Fail-safe behaviour

Because the brake is **power-off-engaged**:

| Event | Brake rail | Result |
|---|---|---|
| Normal walk | energised (released) | motors move the legs freely |
| Standing / hold | de-energised on planted legs | brakes hold; motors idle, cool |
| E-stop pressed | **power removed** | **all brakes engage → legs lock, rider supported** |
| Blown fuse / dead battery | **power removed** | **all brakes engage → legs lock** |
| Controller crash / watchdog timeout | **power removed** | **all brakes engage → legs lock** |

The rider's weight ends up on the mechanical brakes and the planted feet
in every failure mode — the machine freezes standing rather than folding.
The e-stop interlock and brake-release rail sequencing live in
[`POWER_SYSTEM.md`](POWER_SYSTEM.md).

> **Controller must land a valid tripod before de-energising.** A naive
> "cut power now" mid-swing would engage the brakes with a foot in the
> air and drop that corner ~0.2 m. The supervisor must sequence
> **freeze → plant a valid 3-foot tripod → engage brakes** for a
> controlled stop. Only a true emergency (already-detected fault) skips
> straight to power-off, accepting the drop.

### 4.5 Passive alternative — a self-locking worm secondary

There is a purely-mechanical alternative that needs no brake and no
electricity to hold: make the **secondary reduction a self-locking worm
gear** at the knee. A worm with a low lead angle is **non-backdrivable** —
the output (joint) cannot drive the input (motor), so a planted leg holds
itself with no power at all.

| | Fail-safe e-brake (recommended) | Self-locking worm secondary |
|---|---|---|
| Holds at zero power | yes (springs) | yes (geometry) |
| Backdrivable / compliant | yes (walks normally) | **no** — kills the QDD's backdrivability |
| Efficiency | ~0.9 (belt/chain/planetary) | **~0.5–0.7** (worm, high heat/wear) |
| Shock absorption on landing | good (compliant drive) | poor (rigid, loads teeth) |
| Cost/complexity per joint | +brake (~$200) | worm set (comparable) |

The worm's non-backdrivability defeats the main reason to use a QDD
actuator (soft, backdrivable, shock-tolerant legs), and its ~50–70%
efficiency wastes battery and adds heat. **The recommended design is the
fail-safe electromagnetic brake with an efficient (belt/chain/planetary)
secondary.** The worm is documented only as a fallback if the brake
supply chain or controls integration proves impractical.

---

## 5. Standing load case (the common case)

Mounting, dismounting, and every stop leave the machine **standing on a
tripod**. Each of the 3 support legs carries a third of the total weight:

```
    total weight            = 380 kg × 9.81  ≈  3.73 kN
    per support leg (of 3)  = 3.73 kN / 3    ≈  1.24 kN  ≈  1.3 kN nominal
    knee holding torque     ≈  1.3 kN × ~0.65 m arm  ≈  0.85 kN·m
```

The brake holds that **0.85 kN·m indefinitely at zero power and zero
heat** (via ~160 N·m on the fast shaft). Standing all day costs nothing
thermally or electrically. Only *walking* — where torque is dynamic and
the RMS load is well under the continuous rating — puts current through
the motors.

---

## 6. Per-joint summary table

| Joint (×6 each) | Actuator | Ratio (int × sec) | Peak τ | Cont. τ | Speed | Nominal load | Design load | Brake |
|---|---|---|---:|---:|---:|---:|---:|---|
| Hip-yaw | RMD-X8-120 | 19.6:1 (direct) | 120 N·m | 43 N·m | ~762 °/s | 50–80 N·m | ~80 N·m | none (optional) |
| Hip-pitch | RMD-X15-450 | 20.25:1 × 6:1 | 2,430 N·m | 783 N·m | ~98 °/s | ~0.8 kN·m | ~1.2 kN·m | optional ~160 N·m |
| Knee | RMD-X15-450 | 20.25:1 × 6:1 | 2,430 N·m | 783 N·m | ~98 °/s | ~0.85 kN·m | ~0.85 kN·m | **required ~160 N·m** |

Per-leg drivetrain: 1 × RMD-X8-120 (yaw, direct) + 2 × RMD-X15-450
(hip-pitch, knee, each + 6:1 secondary) + 1 (or 2) fail-safe brake(s).
Six legs → **18 actuators, 12 secondary reductions, 12 brakes**
(6 knee + 6 hip, taking the hip brakes as fitted). See [`BOM.md`](BOM.md).

---

## 7. Assumptions & open questions

1. **Secondary η ≈ 0.9 and ratio = 6:1** drive both the joint peak
   (2,430 N·m) and the brake size (160 N·m). A lossier or different-ratio
   secondary shifts both.
2. **Knee holding moment arm ≈ 0.65 m** gives the 0.85 kN·m used to size
   the brake; it depends on the final leg geometry ([`STRUCTURE.md`](STRUCTURE.md)).
3. **Hip brakes optional.** The knee is the mandatory holding joint;
   whether the hip also needs a brake depends on the tucked geometry's
   static hip torque at rest. The BOM carries 12 brakes (knees + hips) so
   the option is funded; drop to 6 (knees only) if analysis clears the hip.
4. **Brake static rating** should be bought at ≥ 1.5× the 160 N·m working
   torque (~240 N·m class) for wear and snatch margin.
5. **RMD-X15-450 / RMD-X8-120 specs** are taken from MyActuator's
   published figures; confirm torque/speed/thermal curves against a
   current datasheet before ordering.

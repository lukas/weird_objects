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

Each hip-pitch and knee actuator drives its joint through a **6:1
secondary reduction** — the actuator output shaft is the *input* (fast
side), and the joint is the *output* (slow side). The driven sprocket is
**coaxial with the joint and bolted to the moving link** (femur at the
hip, tibia at the knee); the actuator + brake are offset-mounted on the
parent link and reach it through the chain run.

| Option | Ratio | Notes |
|---|---:|---|
| **Duplex #40 roller chain, 12T→72T** | 6:1 | **Recommended default.** 72T driven sprocket is Ø291 mm PCD — fits the joint envelope. Two strands share load. Needs a guard + lubrication + periodic inspection. |
| **HTD-14M timing belt** | ~6:1 | **Not buildable single-stage at 6:1**: 14M pulleys need ≥ ~28 teeth, so 6:1 forces a 168T (~Ø750 mm) driven pulley. Only viable as two ~2.5:1 stages — more parts, more width. Rejected. |
| **Planetary stage** | 6:1 | Compact and stiff, but a 1.5–2 kN·m-class output planetary is a 10–20 kg, $1–2 k industrial gearbox per joint. Fallback if chain proves unserviceable. |

What the 6:1 buys at the joint, from the RMD-X15-450 (secondary
efficiency assumed **η ≈ 0.9**):

```
    Motor-limited peak      = 450 N·m × 6 × 0.90  ≈  2,430 N·m  (NOT deliverable — see below)
    Chain-limited peak      ≈ 1.6 kN·m            (working chain tension × Ø291 mm sprocket radius)
    Joint continuous torque = 145 N·m × 6 × 0.90  ≈    783 N·m
    Joint speed             = 98 rpm ÷ 6           =  16.3 rpm  ≈  98 °/s
```

**The chain, not the motor, sets the deliverable joint peak.** At the
72T sprocket's 145.5 mm pitch radius, torque = tension × 0.1455 m. A
duplex #40 chain (~27.8 kN minimum ultimate) run at a 2.5× dynamic
safety factor allows ~11.1 kN of tension → **~1.6 kN·m at the joint**.
The motor's theoretical 2,430 N·m would put ~16.7 kN through the chain
(60 % of ultimate) — so the controller **must software-limit commanded
joint torque to ~1.6 kN·m**. (This was the review finding that killed
the belt: the same 1.2 kN·m design torque means ~8.2 kN of strand
tension, several times what a 40 mm HTD-14M belt can carry.)

### Torque margins (against the [design basis](README.md#2-design-basis-decided))

| Joint | Design load | Joint peak available | **Peak margin** | Nominal (static) | Joint continuous |
|---|---:|---:|---:|---:|---:|
| Hip-pitch | 1.2 kN·m | ~1.6 kN·m (chain-limited) | **~1.3×** | ~0.7 kN·m (worst arm) | 783 N·m |
| Knee | ~0.85 kN·m | ~1.6 kN·m (chain-limited) | **~1.9×** | ~0.45–0.7 kN·m | 783 N·m |
| Hip-yaw | ~0.08 kN·m | 120 N·m (direct) | **~1.5×** | ~0.05–0.08 kN·m | 43 N·m |

The hip margin looks thin (1.3×) until you recall that the 1.2 kN·m
design value already carries ~1.8× conservatism over the worst
geometry-implied static torque (~0.68 kN·m at the 0.45 m worst-in-stride
arm) — the stacked margin to the real load is ~2.4×.

### Speed margin

The gait needs **~82 °/s** at the hip and knee for 0.4 m/s. The
drivetrain delivers **~98 °/s**, a ~20% margin. Yaw delivers 127 rpm
directly (~762 °/s) — effectively unlimited for this gait.

> **The continuous number is the whole point of the brakes.** Planted-leg
> joints carry ~0.45–0.7 kN·m static while walking — 60–90 % of the
> 783 N·m continuous rating at the worst instant of the stride. In
> dynamic walking the RMS torque is lower and the motors are fine on the
> move, but *holding* the rider statically parks the windings near the
> thermal limit — which is exactly why the machine hands standing loads
> to the brakes ([§4](#4-the-knee-brake-load-holding-architecture)).

---

## 4. The knee-brake load-holding architecture

This is the central structural feature of `rideable_v1`.

### 4.1 What and where

On **each knee and each hip-pitch** (12 total — mandatory, see §7), a
**spring-applied, electrically-released electromagnetic brake** is
mounted on the **actuator output shaft** — which is the **input (fast
side) of the 6:1 secondary reduction**.

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
            ▼   (foot force, up to 2.25 kN design per planted leg)
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

> **⚠ The chain is a single point of failure in this load path.** The
> brake grips the *fast* shaft, so everything between the brake and the
> joint — the 12T sprocket, the chain, the 72T sprocket and its bolts —
> is holding the rider. A snapped chain disconnects the brake and that
> knee folds. This is why the secondary is a **duplex** chain run at a
> conservative working tension (§3), why commanded torque is
> software-limited, and why chain/sprocket inspection is a scheduled
> maintenance item, not optional. (A joint-side brake would remove the
> failure mode but needs to hold ~0.85 kN·m directly — a much larger,
> heavier, costlier device; the duplex chain + inspection regime is the
> accepted trade.)

### 4.3 Brake sizing math

The knee holding torque when the leg is planted (see [§5](#5-standing-load-case-the-common-case))
is ~0.45 kN·m in the nominal parked tripod; we size the brake to a
**0.85 kN·m bound** that covers a stop at worst stride excursion with
uneven load share between the support legs:

```
    τ_knee(hold, bound)  ≈  0.85 kN·m  =  850 N·m   at the joint (slow side)
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
    total weight            = 460 kg × 9.81  ≈  4.5 kN
    per support leg (of 3)  = 4.5 kN / 3     ≈  1.5 kN nominal
    knee holding torque     ≈  1.5 kN × 0.29 m static arm      ≈  0.45 kN·m
    worst parked knee       ≈  1.5 kN × ~0.46 m worst arm      ≈  0.7 kN·m
                              (brake bound 0.85 kN·m also covers uneven load share)
```

(The 0.29 m arm is the horizontal knee-axis-to-foot distance of the
solved tucked pose — see `design_spec.yaml` / `tools/rideable_viz_build.py`;
an earlier draft used 0.65 m, which is not what the IK geometry gives.)

The brake holds its **0.85 kN·m bound indefinitely at zero power and
zero heat** (via ~160 N·m on the fast shaft). Standing all day costs
nothing thermally or electrically. Only *walking* — where torque is
dynamic and the RMS load is well under the continuous rating — puts
current through the motors.

---

## 6. Per-joint summary table

| Joint (×6 each) | Actuator | Ratio (int × sec) | Peak τ (deliverable) | Cont. τ | Speed | Nominal load | Design load | Brake |
|---|---|---|---:|---:|---:|---:|---:|---|
| Hip-yaw | RMD-X8-120 | 19.6:1 (direct) | 120 N·m | 43 N·m | ~762 °/s | 50–80 N·m | ~80 N·m | none |
| Hip-pitch | RMD-X15-450 | 20.25:1 × 6:1 | ~1.6 kN·m (chain-limited) | 783 N·m | ~98 °/s | ~0.7 kN·m | ~1.2 kN·m | **required ~160 N·m** |
| Knee | RMD-X15-450 | 20.25:1 × 6:1 | ~1.6 kN·m (chain-limited) | 783 N·m | ~98 °/s | ~0.45–0.7 kN·m | ~0.85 kN·m | **required ~160 N·m** |

Per-leg drivetrain: 1 × RMD-X8-120 (yaw, direct) + 2 × RMD-X15-450
(hip-pitch, knee, each + 6:1 duplex-#40 secondary + fail-safe brake).
Six legs → **18 actuators, 12 secondary reductions, 12 brakes**
(6 knee + 6 hip-pitch). See [`BOM.md`](BOM.md).

---

## 7. Assumptions & open questions

1. **Secondary η ≈ 0.9, ratio = 6:1, duplex #40 chain at ≥2.5× dynamic
   SF** drive the chain-limited joint peak (~1.6 kN·m, enforced in
   software) and the brake size (160 N·m). A lossier or different-ratio
   secondary shifts all of these. Bench-test one chain stage to failure
   before trusting the 2.5× figure.
2. **Knee holding moment arm**: the solved tucked geometry gives 0.29 m
   static / ~0.46 m worst parked; the brake is sized to a 0.85 kN·m
   bound on top of that ([`STRUCTURE.md`](STRUCTURE.md) for the leg
   geometry).
3. **Hip brakes are mandatory, not optional.** The hip-pitch static
   torque while standing (~0.45–0.7 kN·m at the worst parked arm) is a
   large fraction of the actuator's continuous rating — holding it on
   current would cook the motor exactly as at the knee. The BOM carries
   12 brakes (knees + hips), all fitted.
4. **Brake static rating** should be bought at ≥ 1.5× the 160 N·m working
   torque (~240 N·m class) for wear and snatch margin.
5. **RMD-X15-450 / RMD-X8-120 specs** are taken from MyActuator's
   published figures; confirm torque/speed/thermal curves against a
   current datasheet before ordering.
6. **Chain single-point-of-failure** (§4.2): duplex strands, software
   torque limit, and scheduled inspection are all part of the safety
   case. If any of the three is dropped, move the brake to the joint
   side and re-cost.

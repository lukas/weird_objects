# Rideable Hexapod Walker — `rideable_v1`

> A one-rider, six-legged walking vehicle sized to carry a ~100 kg adult
> at a slow, statically-stable walk (~0.4 m/s). This directory is a
> **fresh design iteration** distinct from the earlier full-size draft in
> [`../ASSEMBLY.md`](../ASSEMBLY.md): it swaps the $6–8 k harmonic-drive
> servomotors for a **quasi-direct-drive (QDD) actuator + secondary
> reduction** lineup, adopts a **tucked leg stance** that pulls the hip
> torque down into off-the-shelf actuator territory, and centres the
> whole machine on a **spring-applied, fail-safe electromagnetic brake at
> every knee** so the vehicle holds a rider's weight at **zero motor
> current**.
>
> **This is a serious, large-scale project.** A finished walker masses
> ~ 280 kg dry, ~ 380 kg with a rider, runs on a 72 V lithium bus, and
> can pinch or crush in ways that do not heal. Read [§9 Safety](#9-safety--reality-check)
> before building anything.

This design is the culmination of a series of engineering decisions that
are already made. The parameters in [§2](#2-design-basis-decided) are the
**source of truth**; every other doc in this directory derives from them.

---

## 1. Documents in this directory

| File | What it covers |
|---|---|
| [`README.md`](README.md) *(this file)* | Overview, design basis, tucked-stance rationale, subsystem integration, mass budget, cost roll-up. |
| [`DRIVETRAIN.md`](DRIVETRAIN.md) | Actuator lineup (RMD-X15 / RMD-X8), the 6:1 secondary reduction, and the **knee-brake load-holding architecture** — load path, brake sizing, fail-safe behaviour, per-joint torque/speed table. |
| [`BOM.md`](BOM.md) | Orderable bill of materials: 18 actuators, 12 brakes, 12 secondary-reduction sets, leg/chassis materials, battery, electronics — quantities, unit cost, line totals, grand total. |
| [`STRUCTURE.md`](STRUCTURE.md) | The welded 4130 chromoly space-frame legs: load estimates, why steel and not carbon-fibre rod, and stress-concentration handling at joints, bolt holes, foot pads, and welds. |
| [`POWER_SYSTEM.md`](POWER_SYSTEM.md) | The 72 V (20S) bus: pack sizing (~3 kWh), current budget (~2.5 kW peak walking), the 72→48 V yaw rail, fusing, contactor, and the fail-safe-brake power interlock. |

> **Status: mechanical-design draft. Not built or tested.** Every number
> below is a hand-calc with single-digit-percent tolerance. A real build
> needs professional FEA on the legs and chassis, a fatigue analysis on
> the welds, a thermal model of the actuator stack, and a controls-safety
> review. See [§9](#9-safety--reality-check).

---

## 2. Design basis (decided)

These are the fixed inputs. Do not silently change one without
re-deriving the rest — they propagate into every other doc.

| Parameter | Value | Notes |
|---|---|---|
| Purpose | Carry **1 adult rider (~100 kg)** | Private-property use only. |
| Cruise speed | **~0.4 m/s** (1.4 km/h, a slow walk) | Statically stable at all times. |
| Total operating mass | **~380 kg** | 100 kg rider + **~280 kg vehicle (dry)**. |
| Total weight on the ground | **~3.73 kN** | 380 kg × 9.81 m/s². |
| Gait | **Alternating tripod** | 3 legs support at all times → statically stable even on power loss. |
| DOF | **6 legs × 3 DOF = 18 actuators** | hip-yaw, hip-pitch, knee per leg. |
| Nominal per-leg foot load | **~1.3 kN** | 3.73 kN ÷ 3 support legs. |
| Design foot load | **~2.0 kN** | 1.3 kN × **1.5 safety factor**. |
| Leg stance | **Tucked** — foot ~0.5–0.6 m under the hip | *Not* sprawled ~1.2 m out. The key decision (see [§3](#3-the-tucked-stance-decision)). |
| Hip-pitch joint torque | **~0.8 kN·m nominal / ~1.2 kN·m design** | Tucked; the sprawled version needed ~1.5–1.7 kN·m. |
| Knee joint torque | **~500–850 N·m** | Holding-dominated (standing, not swinging). |
| Hip-yaw joint torque | **~50–80 N·m** | Only friction + lateral imbalance. Easy. |
| Gait joint speed requirement | **~82 °/s** at the hip/knee | For 0.4 m/s at the chosen stride. |

Everything downstream — actuator selection, brake sizing, structure,
battery — is dimensioned to hold the **design foot load of 2.0 kN** and
the **design hip torque of 1.2 kN·m**.

---

## 3. The tucked-stance decision

The single most important geometry choice in this design is that the foot
is kept **tucked ~0.5–0.6 m directly under the hip**, rather than
sprawled ~1.2 m outboard like a spider.

Hip-pitch torque is (roughly) `foot load × horizontal distance from the
hip axis to the foot`:

```
    SPRAWLED   τ_hip ≈ 1.3 kN × 1.20 m ≈ 1.56 kN·m  (design ~2.3 kN·m)
    TUCKED     τ_hip ≈ 1.3 kN × 0.60 m ≈ 0.78 kN·m  (design ~1.2 kN·m)
```

Halving the moment arm halves the hip torque. That is the difference
between needing a **$6–8 k harmonic-drive servo** at every hip (the old
[`../ASSEMBLY.md`](../ASSEMBLY.md) approach) and being able to use a
**~$1,515 QDD actuator behind a modest 6:1 reduction** (this design).
The knee benefits the same way: a near-vertical tibia keeps the foot's
horizontal offset from the knee small, so the knee is dominated by the
*holding* torque of a planted leg (~0.85 kN·m) rather than a large swing
torque.

Trade-offs of tucking:
* **Narrower support polygon.** Feet tucked under the hips give a smaller
  stability triangle than a sprawled stance, so the controller must keep
  the rider's centre of mass well inside it and move slowly. That is
  acceptable at 0.4 m/s.
* **Deeper knee bend.** The knee works at a more folded angle, which is
  where the fail-safe brake earns its keep (holding a folded, loaded knee
  is exactly the case that overheats a motor).
* **Less obstacle clearance.** A tucked leg lifts the foot less far
  outboard on swing. Fine for a slow walk on prepared ground.

---

## 4. How the subsystems fit together

```
                        rider (~100 kg) on saddle + footrests + harness
                                        │
                    ┌───────────────────┴───────────────────┐
                    │   CHASSIS  (welded Al/steel tube frame  │
                    │   + deck, saddle post, battery + e-bay) │
                    └───────────────────┬───────────────────┘
                                        │  6 identical legs, 60° apart
        ┌──────────────────────────────┼──────────────────────────────┐
        │ per leg:                                                       │
        │   hip-yaw   RMD-X8-120  (direct drive, 48 V)                   │
        │      └─ coxa ─ hip-pitch  RMD-X15-450 + 6:1 secondary (72 V)   │
        │                  └─ femur ─ knee  RMD-X15-450 + 6:1 secondary  │
        │                              + FAIL-SAFE BRAKE on input shaft  │
        │                              └─ tibia ─ foot                    │
        └────────────────────────────────────────────────────────────────┘
```

* **Actuation ([`DRIVETRAIN.md`](DRIVETRAIN.md)).** The two high-torque
  joints per leg (hip-pitch, knee) each use one **MyActuator RMD-X15-450**
  QDD actuator (145 N·m rated / 450 N·m peak, internal 20.25:1) behind a
  **~6:1 secondary reduction** (HTD-14M belt, #40 chain, or a planetary
  stage). That delivers **~2,430 N·m peak** and **~783 N·m continuous** at
  the joint, at **~98 °/s** — comfortably above the 1.2 kN·m design hip
  torque and the 82 °/s gait requirement. The hip-yaw joint uses a
  smaller **RMD-X8-120** (43 N·m / 120 N·m peak) **direct-drive**.
* **Load holding ([`DRIVETRAIN.md`](DRIVETRAIN.md) §the brake).** Because
  QDD motors overheat if asked to hold high continuous torque, the machine
  does **not** stand on motor current. Each knee (and optionally each hip)
  carries a **spring-applied, electrically-released electromagnetic brake**
  mounted on the **actuator output shaft = the input of the 6:1 secondary**.
  The brake only has to hold **~160 N·m** there; the secondary multiplies
  it up to the full joint torque. Power-off = brake engaged = legs locked
  with the rider supported. This is the central structural feature of the
  redesign.
* **Structure ([`STRUCTURE.md`](STRUCTURE.md)).** Legs are **welded 4130
  chromoly steel space-frames** (triangulated), ~3 kg each, ~20 kg for all
  six. The chassis is a welded aluminium or steel tube frame with a deck
  and saddle.
* **Power ([`POWER_SYSTEM.md`](POWER_SYSTEM.md)).** A **72 V, 20S Li-ion**
  pack (~3 kWh, ~20 kg) feeds the twelve RMD-X15 hip/knee actuators
  directly; a **72→48 V DC-DC** rail feeds the six RMD-X8 yaw actuators,
  which draw little power. Peak walking draw is ~2.5 kW.

---

## 5. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Rider payload | 1 rider, ~100 kg |
| Vehicle dry mass | **~280 kg** |
| Total mass on the ground | **~380 kg** (~3.73 kN) |
| Cruise speed | ~0.4 m/s (statically stable) |
| Leg stance | Tucked, foot ~0.5–0.6 m under the hip |
| Per-leg static load (tripod) | ~1.3 kN nominal / ~2.0 kN design (1.5×) |
| Hip-pitch torque | ~0.8 kN·m nominal / ~1.2 kN·m design |
| Knee torque | ~0.85 kN·m holding (500–850 N·m band) |
| Hip-yaw torque | ~50–80 N·m |
| Hip-pitch / knee actuator | RMD-X15-450 + 6:1 secondary → 2,430 N·m peak, 783 N·m cont., 98 °/s |
| Hip-yaw actuator | RMD-X8-120 direct drive → 120 N·m peak, 43 N·m cont. |
| Load-hold | Spring-applied fail-safe e-brake per knee (opt. per hip), ~160 N·m on the input shaft |
| Battery | 72 V (20S) Li-ion, ~3 kWh, ~42 Ah, ~20 kg |
| Peak walking power | ~2.5 kW (~35 A at 72 V) |
| Run time | ~30–60 min |
| Drivetrain cost | **~$28.7 k** (18 actuators + 12 brakes + 12 reductions) |
| Full-build cost (est.) | **~$38–45 k** (add structure, battery, electronics) |

---

## 6. Mass budget (~280 kg dry)

Order-of-magnitude, mid-2026. The actuator + drivetrain stack and the
chassis dominate; the QDD actuators are far lighter than the harmonic
drives they replace, so the structure and battery carry proportionally
more of the budget.

| System | Mass (kg) | Basis |
|---|---:|---|
| Hip-pitch + knee actuators (12 × RMD-X15-450 @ 3.5 kg) | 42 | [BOM](BOM.md) |
| Hip-yaw actuators (6 × RMD-X8-120 @ 1.4 kg) | 8 | [BOM](BOM.md) |
| Secondary reductions (12 sets: pulleys/sprockets, shafts, bearings, housing) | 42 | ~3.5 kg/set |
| Fail-safe brakes (12 @ ~0.5 kg) | 6 | knees + hips |
| Leg space-frames (6 × welded 4130) | 20 | ~3.3 kg/leg |
| Feet (6 × urethane pad + steel disc) | 9 | ~1.5 kg each |
| Chassis frame + deck (welded tube + plate) | 48 | one-off |
| Saddle, footrests, handlebars, harness | 12 | |
| Battery (72 V, ~3 kWh Li-ion + BMS) | 20 | [POWER_SYSTEM](POWER_SYSTEM.md) |
| Power electronics (72→48 DC-DC, contactor, fusing, wiring) | 15 | |
| Compute, IMU, foot force sensors, cabling | 4 | |
| Joint bearings, shafts, brackets, fasteners | 26 | |
| Guards, covers, paint, consumables | 8 | |
| **Subtotal** | **260** | |
| Contingency (~7%) | 20 | unmodelled brackets, cable, hardware |
| **Total dry** | **~280** | |
| Rider | + ~100 | |
| **Total walking** | **~380** | matches [§2](#2-design-basis-decided) |

---

## 7. Cost roll-up

Full detail (with quantities and links) is in [`BOM.md`](BOM.md). Headline:

| Bucket | Cost (USD, mid-2026) |
|---|---:|
| 12 × RMD-X15-450 (hip-pitch + knee) @ ~$1,515 | ~$18,180 |
| 6 × RMD-X8-120 (hip-yaw) @ ~$645 | ~$3,870 |
| 12 × secondary-reduction sets @ ~$350 | ~$4,200 |
| 12 × fail-safe electromagnetic brakes @ ~$200 | ~$2,400 |
| **Drivetrain subtotal** | **~$28,650** |
| Leg space-frames (4130 tube + weld + machining) | ~$1,500 |
| Chassis (tube + deck + saddle + fab) | ~$2,500 |
| Battery (72 V 3 kWh + BMS + charger) | ~$1,500 |
| Power electronics (DC-DC, contactor, fuses, e-stop, wire) | ~$1,200 |
| Compute + IMU + foot sensors | ~$1,500 |
| Bearings, shafts, fasteners, misc hardware | ~$2,000 |
| Consumables, paint, tooling | ~$500 |
| **Full-build estimate** | **~$39,350 (≈ $38–45 k)** |

The drivetrain (actuators + brakes + reductions) is ~$27–30 k and is by
far the largest bucket — this is where the tucked-stance / QDD decision
pays off versus a ~$110 k+ harmonic-drive build.

---

## 8. Design assumptions to confirm

These are the load-bearing assumptions behind the numbers. Confirm them
before committing money to actuators or metal.

1. **Tucked-stance moment arms.** Hip torque scales directly with the
   0.6 m foot-under-hip offset; if the real leg geometry / IK ends up
   with a larger stance radius, hip torque climbs and the 2× peak margin
   shrinks. The leg-length ratios that realise the 0.5–0.6 m tuck are
   still open (see [`STRUCTURE.md`](STRUCTURE.md)).
2. **Secondary-reduction efficiency ~0.9** and **6:1 ratio**. The brake
   sizing (160 N·m) and the joint peak (2,430 N·m) both assume these.
3. **Continuous joint torque (~783 N·m) is *not* the standing-hold path.**
   It is only ~equal to the ~800 N·m nominal static hip torque, so the
   actuators cannot hold the rider indefinitely on current — **the
   fail-safe brakes are mandatory, not optional**, for any standing.
4. **RMD-X15-450 / RMD-X8-120 published specs** (torque, ratio, speed,
   mass, price) are taken at face value; verify against a current
   MyActuator datasheet at order time.
5. **Rider ≤ 100 kg** and **private-property, prepared-ground operation**
   at ≤ 0.4 m/s. Faster speeds, slopes, or a heavier rider invalidate the
   static-stability and torque margins.

---

## 9. Safety & reality check

This vehicle can crush, pinch, fall, and shock. Non-negotiables:

* **Fail-safe brakes are the primary safety system.** The knee (and
  optional hip) brakes are **spring-applied / electrically-released**:
  with no power they are **engaged**. A power loss, a blown fuse, or an
  e-stop therefore **locks every leg in place with the rider supported**
  rather than collapsing. This is designed in, not bolted on — see
  [`DRIVETRAIN.md`](DRIVETRAIN.md).
* **The hardware e-stop removes power, which *engages* the brakes.** A
  mushroom e-stop within the rider's reach must drop the main contactor
  (and thereby the brake-release rail), not merely command the motors to
  zero torque. Removing power = brakes on = machine frozen.
* **72 V DC is a shock and arc hazard.** The 20S pack sits above the
  ~60 V "low-voltage" line under load transients. Fuse it, contactor it,
  bleed the bus, and treat it as live until metered dead.
* **Static stability, but only on ≥ 3 well-spaced feet.** The controller
  must sequence any e-stop as *freeze gait → ensure a valid tripod is
  planted → engage brakes*, never mid-swing with a foot in the air.
* **Private property only.** A ~280 kg unregistered walking vehicle is
  not street-legal in most jurisdictions. Build and operate it on private
  land.
* **Professional review required.** These docs are a starting point.
  A real build needs an independent **FEA pass** on the legs and chassis,
  a **fatigue analysis** on the welds, a **thermal model** of the actuator
  stack, and a **controls-safety review** of the e-stop / brake interlock
  before a human rides it.

---

## 10. License

Personal exploration — no license declared yet, same as the parent
project.

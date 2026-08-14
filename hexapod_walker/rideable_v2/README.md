# Rideable Hexapod Walker — `rideable_v2`

> A one-rider, six-legged walking vehicle sized to carry a **~77 kg
> (170 lb) rider** at a slow, statically-stable walk (~0.3 m/s). This is
> the **second rideable design iteration**: where
> [`../rideable_v1/`](../rideable_v1/README.md) was a ~360 kg, 72 V
> machine on twelve $1,515 RMD-X15s behind 6:1 duplex-chain secondaries,
> `v2` is a **~165 kg, 48 V machine on eighteen identical CubeMars
> AK80-64s** ($890, 850 g, 120 N·m peak each) behind **per-joint wide
> synchronous-belt stages** — 4:1 hip-pitch, 3:1 knee, 2:1 hip-yaw —
> with the **knee actuator relocated to the hip end of the femur** and
> every joint riding its **own large structural bearing**, not the
> motor's output bearing.
>
> **This is still a serious machine.** ~242 kg walking with a rider,
> a 12S lithium bus, and joints that can deliver 418 N·m. Read
> [§9 Safety](#9-safety--ballast-protocol) before building anything.
> **No rider until the machine has survived substantial ballast
> testing.**

`design_spec.yaml` is the machine-readable source of truth;
`tools/design_consistency_check.py` (run by `make check`) re-derives
every load, torque, belt tension, lock sizing, and power figure from it
and fails if these docs drift.

---

## 1. Documents in this directory

| File | What it covers |
|---|---|
| [`README.md`](README.md) *(this file)* | Overview, design basis, what changed vs v1 and vs the original brief, mass budget, cost roll-up, safety/ballast protocol. |
| [`DRIVETRAIN.md`](DRIVETRAIN.md) | The AK80-64 + belt-stage lineup, per-joint sizing and margins, the knee-motor-at-the-hip layout, the structural-joint-bearing load path, and the joint-side parking-pin load holding. |
| [`PARTS.md`](PARTS.md) | SKU-level part specs: belt/sprocket part numbers with center distances, bearing SKUs, lock hardware, electrical SKUs — the orderable detail behind `BOM.md`. |
| [`STRUCTURE.md`](STRUCTURE.md) | Aluminum legs and the wide chassis: loads, sections, why 6061 works here where v1 needed 4130 steel. |
| [`POWER_SYSTEM.md`](POWER_SYSTEM.md) | The 12S (48 V-class) bus: contactor + pre-charge, **six fused leg branches**, the 24 V solenoid/logic rail, budgets, interlocks. |
| [`BOM.md`](BOM.md) | Orderable bill of materials with live-verified Aug-2026 prices. |
| [`design_spec.yaml`](design_spec.yaml) | Machine-readable design basis (checked by `make check`). |

> **Status: mechanical-design draft. Not built or tested.** Hand-calcs
> with single-digit-percent tolerance, plus first-pass FEA on the three
> flagged joint nodes (`tools/fea_joint_shaft.py`, `tools/fea_leg_nodes.py`
> — all SF ≥ 3.0 working); a real build still needs FEA of the machined
> geometry, a thermal model, and a controls-safety review. The BuildViz
> scene lives in
> `full_robot_viz/` (regenerate with `make viz`; `make check` runs the
> full geometry + articulation suite; view with `make view-buildviz`).
> Sustained cruise is the **ripple/wave gait** — tripod-walking runs the
> hips at ~104% of continuous torque and is for maneuvering only
> ([`DRIVETRAIN.md` §5](DRIVETRAIN.md#5-torque--speed-margins-against-the-design-basis)).

---

## 2. Design basis (decided)

| Parameter | Value | Notes |
|---|---|---|
| Purpose | Carry **1 rider, ~77 kg (170 lb)** | Private-property use only. |
| Cruise speed | **~0.3 m/s** (0.4 m/s stretch) | Statically stable at all times. |
| Total operating mass | **~242 kg** | 77 kg rider + **~165 kg vehicle (dry)**. |
| Ground weight | **~2.37 kN** | 242 kg × 9.81 m/s². |
| Gait | **Alternating tripod** (design gait) | Ripple/wave at creep speeds for a bigger polygon. |
| DOF | 6 legs × 3 DOF = **18 identical actuators** | hip-yaw, hip-pitch, knee — one SKU. |
| Per-leg foot load | **0.79 kN nominal / 1.19 kN design (1.5×)** | Tripod, 3 support legs. |
| Body | **~880 mm across the yaw axes** | "Wide body = stability without huge hip torque." |
| Leg | coxa 120 / **femur 350 / tibia 450 mm** | Per the brief. |
| Stance | Tucked — foot **0.30 m out / 0.55 m below** the hip-pitch joint | Foot radius 860 mm → **~1.72 m footprint**. |
| Hip-pitch torque | 0.24 kN·m parked / **0.36 kN·m design** (worst-in-stride arm 0.45 m) | The sizing driver. |
| Knee torque | 0.03 kN·m parked / **0.20 kN·m design** | Near-vertical tibia → tiny static arm (0.035 m). |
| Hip-yaw torque | ~0.03–0.08 kN·m | Friction + lateral imbalance only. |
| Joint speeds needed | ~31 °/s loaded (stance) / ~94 °/s unloaded (swing) | At 0.3 m/s tripod. |
| Rollover angle (CoM ~0.65 m) | **~49° parked on 6 feet / ~33° in tripod** | See §3. |

Everything downstream — actuator ratios, belt tensions, lock sizing,
structure, battery — is dimensioned to the **1.19 kN design foot load**
and the **0.36 kN·m design hip torque**.

---

## 3. What changed vs the brief (and vs `rideable_v1`)

The original v2 brief (AK80-64s + belt stages, knee motor inboard, wide
body, structural joint bearings, 48 V + contactor + six fused branches,
ballast before rider) **survives essentially intact** — it is the right
architecture, and the arithmetic checks out. Four things moved when the
numbers were run honestly:

1. **Continuous torque, not peak, is the binding constraint.** With a
   realistic 165 kg dry mass, the parked hip carries ~237 N·m static —
   but the 4:1 hip stage only delivers **~167 N·m continuous** (48 N·m
   rated × 4 × 0.87). The machine **cannot stand on motor current**, and
   a rideable spends most of its life standing. It therefore inherits
   v1's central *rule* — the machine never stands on motor current —
   implemented as **12 spring-applied parking pin locks** (hip-pitch +
   knee) that pin the driven pulley to the parent link at the joint.
   Power-off = pins in = rider supported. (The parts-spec pass showed
   COTS friction brakes at this torque are 4–8 kg each — ~50 kg for
   twelve — which is what forced, and improved, this design.)
   ([`DRIVETRAIN.md` §6](DRIVETRAIN.md#6-load-holding-joint-side-parking-pin-locks-v1s-rule-better-hardware))
2. **The hip peak margin is 1.16×, not "comfortable".** 418 N·m
   effective peak (120 × 4 × 0.87) vs 0.36 kN·m worst-in-stride. The
   architecture's own answer: **belt ratio is a pulley swap**, so 5:1
   (~1.47× margin, −20% speed) is a ~$150-per-joint tuning change after
   ballast testing, not a redesign. That tunability is a headline
   advantage of the belt stage over v1's chain-and-sprocket sets.
3. **The 2026 catalog check** (the brief asked for this): AK80-64 is
   current at **$889.90** (store.cubemars.com, Aug 2026). CubeMars also
   shipped the **AKH70-48** in 03/2026 — 222 N·m peak / 74 N·m rated,
   1.4 kg, dual-CAN, **$698.90, i.e. cheaper than the AK80-64 with ~2×
   the torque**. It does *not* make the external reduction unnecessary
   (the shock-isolation and structural-bearing arguments stand), but it
   is the designated **hip fallback at 3:1** if ballast testing shows the
   margin short.
4. **The ~45° rollover only holds parked.** On all six feet the support
   hexagon (inradius ~0.745 m) gives **~49°**; in tripod gait the
   3-foot triangle shrinks it to **~33°**. Still comfortable at 0.3 m/s
   — and the wide-body-over-long-legs logic is exactly why — but ride
   gait selection must respect the smaller number.

Vs **`rideable_v1`**, the wins compound: half the total mass (242 vs
460 kg) → a third of the hip torque (0.36 vs 1.2 kN·m) → belts become
buildable where v1's review killed them (3.3 kN of strand tension vs
8.2 kN) → aluminum legs replace welded 4130 truss → **~$35 k build vs
~$41 k** with a machine that is vastly less dangerous to develop. And
the joint-side parking pins delete v1's chain-snap single point of
failure outright.

---

## 4. How the subsystems fit together

```
                     rider (~77 kg) on saddle + footrests + harness
                                     │
                 ┌───────────────────┴───────────────────┐
                 │  CHASSIS: welded 6061 frame, ~880 mm   │
                 │  wide + deck, battery + e-bay low      │
                 └───────────────────┬───────────────────┘
                                     │ 6 identical legs, 60° apart
   ┌─────────────────────────────────┼─────────────────────────────────┐
   │ per leg — ALL THREE AK80-64s mounted at/near the body:            │
   │   yaw   AK80-64 → 2:1 belt → yaw joint bearing → coxa             │
   │   hip   AK80-64 → 4:1 belt → hip shaft/bearings [+ pin lock]      │
   │   knee  AK80-64 at the HIP end of the femur                       │
   │            └── 3:1 belt runs ~212 mm DOWN the femur ──► knee      │
   │                 shaft/bearings [+ pin lock] → tibia → sensing foot│
   └───────────────────────────────────────────────────────────────────┘

   mechanically:  motor → belt → LARGE bearing-supported joint shaft → leg
   electrically:  12S pack → fuse → pre-charge + contactor → 6 fused
                  leg branches → 18 × AK80-64 (CAN)   [POWER_SYSTEM.md]
```

* **Drivetrain ([`DRIVETRAIN.md`](DRIVETRAIN.md)).** One actuator SKU,
  three belt ratios tuned per joint. Effective joint figures (η = 0.87):
  hip **418 N·m peak / 167 cont / 72 °/s**, knee **313 / 125 / 96 °/s**,
  yaw **209 / 84 / 144 °/s**. The knee actuator lives at the hip end of
  the femur — moving that ~1.4 kg inboard cuts leg swing inertia
  ~20–25% and is why the machine will be markedly easier to control.
* **Load holding.** 12 spring-applied **parking pin locks** (hips +
  knees) engage hole rings machined into the driven pulleys; standing
  costs zero current. Because the pin holds **at the joint**, the belt
  is *not* in the holding load path — v1's chain-snap single point of
  failure is designed out.
* **Structure ([`STRUCTURE.md`](STRUCTURE.md)).** 6061-T6 aluminum
  box-section legs (~3.5 kg each), wide welded-tube chassis. Design
  foot load 1.19 kN is roughly *half* v1's, which is what lets aluminum
  replace steel truss.
* **Power ([`POWER_SYSTEM.md`](POWER_SYSTEM.md)).** One 12S (~44.4 V
  nom, 50.4 V full — the AK80-64's 12S max is why "48 V" means 12S)
  ~1.8 kWh pack → main fuse → pre-charge + contactor → **six 30 A fused
  leg branches**; a 48→24 V ≥350 W rail holds the 12 lock solenoids +
  logic.

---

## 5. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating tripod |
| Rider payload | 1 rider, ~77 kg (170 lb) |
| Vehicle dry mass | **~165 kg** |
| Total walking mass | **~242 kg** (~2.37 kN) |
| Cruise speed | ~0.3 m/s (0.4 stretch) |
| Footprint | Ø ~1.72 m; body ~880 mm wide; hip axis ~0.55 m up |
| Actuators | **18 × CubeMars AK80-64** (120 N·m pk / 48 N·m rated / 850 g) |
| Reductions | Wide synchronous belt: 4:1 hip / 3:1 knee / 2:1 yaw (η ≈ 0.87) |
| Joint capability | hip 418 N·m pk, 167 cont, 72 °/s · knee 313/125/96 · yaw 209/84/144 |
| Design joint loads | hip 0.36 kN·m · knee 0.20 kN·m · yaw 0.08 kN·m |
| Load-hold | 12 × joint-side parking pin, 540 N·m rated (power-off = pinned) |
| Battery | 12S Li-ion, ~40 Ah / ~1.8 kWh, ~10 kg |
| Power | ~0.9 kW cruise / ~1.8 kW peak walk / 70–90 A stand-up transient |
| Run time | ~2 h theoretical cruise → plan 45–75 min real |
| Drivetrain cost | **~$23.6 k** (18 actuators + 18 belt stages + 18 bearing sets + 12 locks) |
| Full build (est.) | **~$33 k (≈ $31–36 k)** |

---

## 6. Mass budget (~165 kg dry)

Order-of-magnitude, Aug 2026. The honest heavyweights are the belt
stages (a lightened 100T Ø255 mm pulley + belt + tensioner + guard is
~2.3 kg at the hip) and the chassis that must carry a rider on an
880 mm-wide frame. Rows are pinned by `tools/design_consistency_check.py`.

| System | Mass (kg) | Basis |
|---|---:|---|
| Actuators (18 × AK80-64 @ 0.85) | 15.3 | [BOM](BOM.md) |
| Belt stages (6 × [hip 2.3 + knee 2.2 + yaw 1.0]) | 33.0 | pulleys, belts, tensioners, guards |
| Structural joint bearings (18 sets @ ~1.0) | 18.0 | shafts, tapered/thin-section brgs, housings |
| Parking pin locks (12 @ ~0.35) | 4.2 | hips + knees, joint-side |
| Legs (6 × 6061 femur+tibia+hubs @ ~3.5) | 21.0 | [STRUCTURE](STRUCTURE.md) |
| Feet (6 × pad + load cell @ ~0.7) | 4.0 | |
| Chassis frame + deck (~880 mm wide) | 20.0 | |
| Saddle, pegs, bars, harness | 8.0 | |
| Battery (12S ~1.8 kWh) | 10.0 | [POWER_SYSTEM](POWER_SYSTEM.md) |
| Power electronics (contactor, fuses, DC-DC, wiring) | 5.0 | |
| Compute, IMU, sensors | 3.0 | |
| Fasteners, mounts, misc hardware | 8.0 | |
| Guards, covers, finishing | 2.0 | |
| **Subtotal** | **151.5** | |
| Contingency (~9%) | 13.5 | |
| **Total dry** | **~165** | |
| Rider | + ~77 | |
| **Total walking** | **~242** | matches [§2](#2-design-basis-decided) |

> **Mass discipline is a torque budget.** Every +10 kg dry adds
> ~15 N·m to the worst-in-stride hip torque (10 kg × g / 3 legs ×
> 0.45 m) and eats directly into the 1.16× hip margin. Weigh
> everything; treat the contingency row as a hard ceiling.

---

## 7. Cost roll-up

Full detail in [`BOM.md`](BOM.md). Headline:

| Bucket | Cost (USD, Aug 2026) |
|---|---:|
| 18 × AK80-64 @ $890 | $16,020 |
| 18 × belt stages (pulleys, Poly Chain belts, tensioners) | $4,080 |
| 18 × structural joint-bearing assemblies | $2,520 |
| 12 × parking pin locks | $960 |
| **Drivetrain subtotal** | **~$23,580** |
| Legs (6061 + machined hubs) | $2,000 |
| Feet + pads | $400 |
| Chassis + saddle + rider interface | $2,300 |
| Battery + power electronics | $1,200 |
| Compute, IMU, CAN, foot sensors | $1,700 |
| Fasteners, machining, misc | $1,500 |
| Consumables & finishing | $400 |
| **Full-build estimate** | **~$33,080 (≈ $31–36 k)** |

Swapping all 18 units to the AKH70-48 ($699) would save ~$3.4 k and
raise torque at the cost of speed and +10 kg of actuator mass — see
[`DRIVETRAIN.md` §7](DRIVETRAIN.md#7-the-akh70-48-alternative) before
ordering.

---

## 8. Design assumptions to confirm

1. **Vehicle dry mass ≤ 165 kg.** The hip margin dies quickly above it
   (§6 note). Weigh subassemblies against the budget as they are built.
2. **Belt allowable (~7.7 kN class for 8MGT × 36 mm)** — now
   catalog-anchored (the Gates manual's own 12 mm worked example runs
   ~2.6 kN; Table 11 width constants scale 36 mm to ~3×), giving the
   hip's 3.28 kN peak >2× margin. Have Gates application engineering
   sign off the final drives at order time. Fallbacks: 5:1 hip ratio,
   or v1's duplex #40 chain at the hip only.
3. **Belt-stage efficiency ~0.87** (the brief's 85–90%). All joint
   torque figures include it.
4. **Moment arms**: hip 0.30 m static / 0.45 m worst-in-stride
   (±0.15 m stride excursion); knee 0.035/0.185 m from the solved
   tucked pose. If the gait sweeps further, torque climbs
   proportionally.
5. **AK80-64 figures** (120/48 N·m, 48 rpm rated @ 48 V, 850 g, 12S
   max, $890) live-verified Aug 2026 — re-verify at order time,
   including whether the store price includes the driver board.
6. **Uneven load share**: a tripod handover can briefly put ~half the
   vehicle weight on one leg (~1.19 kN — exactly the design foot
   load). The controller must not park in a two-leg-dominant stance.
7. **The rider is on a suspension saddle**, so gait bob is comfort, not
   structure — but the CoM-height ~0.65 m rollover numbers assume the
   rider sits, not stands.

---

## 9. Safety & ballast protocol

Non-negotiables, inherited from v1 and extended:

* **The parking pin locks are the primary safety system.** Power-off =
  pins in = legs locked with the rider supported (each joint settles
  ≤15° to the next lock hole). The hardware e-stop must **remove
  power** (contactor + solenoid rail), not merely command zero torque.
* **The pins hold at the joint, so the belt is NOT in the holding load
  path** — a snapped belt loses drive, not support. Belt inspection is
  ordinary maintenance, not a safety item (v1's chain caveat, deleted
  by design).
* **Controlled stops sequence:** freeze gait → plant a valid tripod →
  align joints to lock holes → drop pins. Never pin mid-swing except in
  an already-detected emergency.
* **12S lithium (50.4 V full) below the 60 V LV line but still an arc
  and fire hazard**: main fuse at the pack, pre-charge before contactor
  close, six fused branches, bleed indicator, charge on a balance
  charger in a fire-safe area.
* **Mechanical travel stops on every joint** (per the brief) sized to
  take a full-torque runaway without letting links collide.
* **No rider until ballast testing passes, in stages:**
  1. Single-joint bench rig (see [`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18)).
  2. Full machine, **no ballast**: stand-up, stand-down, tripod walk.
  3. **40 kg ballast** (half rider): 10 h cumulative walking, pin
     locks hold at zero current, deliberate e-stops mid-walk.
  4. **80 kg ballast** (≥ rider): repeat, plus worst-case stances,
     slope tests to the rollover margin, and a deliberate
     single-motor-disable walk-home test.
  5. Only then a rider, with harness, helmet, spotters, and the e-stop
     tested that same session.
* **Private property only.** A ~165 kg walking vehicle is not
  street-legal in most jurisdictions.
* **Professional review required** before any rider: FEA on legs and
  chassis, thermal model of the actuator+belt stack, controls-safety
  review of the e-stop/lock interlock.

---

## 10. License

Personal exploration — no license declared yet, same as the parent
project.

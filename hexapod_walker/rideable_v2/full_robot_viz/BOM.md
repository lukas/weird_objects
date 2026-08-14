# `rideable_v2` — Bill of Materials & Shopping List

> Orderable BOM for **one rideable hexapod**, consistent with
> [`README.md`](README.md), [`DRIVETRAIN.md`](DRIVETRAIN.md),
> [`STRUCTURE.md`](STRUCTURE.md), [`POWER_SYSTEM.md`](POWER_SYSTEM.md).
> Actuator prices and Gates belt part numbers **live-verified Aug
> 2026**; everything else is order-of-magnitude. SKU-level engineering
> detail (part numbers, center distances, bushings, fallback SKUs) is
> in [`PARTS.md`](PARTS.md).
>
> **Status: design draft, unbuilt.** De-risk by buying **one AK80-64 +
> one belt stage + one pin lock + one bearing set** first and building
> the single-joint bench rig in [§L](#l-bench-test-order-de-risk-before-buying-18)
> before committing to 18.

---

## A. Actuators (the long pole — one SKU)

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 18 | **CubeMars AK80-64 KV80** | 120 N·m peak / 48 N·m rated, 64:1, 48 rpm rated @ 48 V, 19 A peak / 7 A rated, 850 g, 12S max, CAN, XT30. **6 yaw + 6 hip-pitch + 6 knee** — one spare pool. Confirm the driver-board option is included at checkout. | $890 | **$16,020** | [store.cubemars.com AK80-64](https://store.cubemars.com/products/ak80-64) |
| — | *(alternative, decide at order time)* | **CubeMars AKH70-48 V1.0** — 222 N·m peak / 74 N·m rated, 1.4 kg, dual-CAN, $698.90. Hip fallback at 3:1 if the margin looks short on the bench rig — see [`DRIVETRAIN.md` §7](DRIVETRAIN.md#7-the-akh70-48-alternative). | ($699) | (−$3,438 if ×18) | [cubemars.com AKH70-48](https://www.cubemars.com/product/akh70-48-v-1-0-kv41-hollow-shaft-planetary-actuator.html) |
| | | | **Actuators subtotal** | **$16,020** | |

---

## B. Belt stages (18 sets)

Gates **Poly Chain GT Carbon 8MGT × 36 mm**. Small sprockets are Gates
stock (there is no stock 24T — the hip is 25T→100T); large driven
pulleys are lightened aluminum, machined with the joint-hub bolt circle
and the parking-pin hole ring. Belt lengths are stock catalog sizes;
full drive geometry in [`PARTS.md` §2](PARTS.md#2-belt-drives-18-stages).

| Qty | Item | Spec to buy | Unit $ | Line $ |
|---:|---|---|---:|---:|
| 6 | **Hip stage: 25T + 100T pulleys + belt + tensioner/guard** | 100T Ø254.6 mm PD lightened Al driven pulley (machined, with lock ring), 25T stock drive sprocket, **8MGT-960-36** belt, C≈208 mm, eccentric idler + guard. | $250 | $1,500 |
| 6 | **Knee stage: 28T + 84T + femur belt + tensioner/guard** | 84T Ø213.9 mm at the knee (with lock ring), 28T stock at the hip-end motor (~140 mm from the hip axis), **8MGT-896-36** belt, C≈212 mm down the femur, idler + full-length guard. | $250 | $1,500 |
| 6 | **Yaw stage: 36T + 72T + belt + tensioner** | 72T Ø183.3 mm large-bore driven pulley on the yaw bearing, 36T stock drive, **8MGT-720-36** belt, C≈136 mm. | $180 | $1,080 |
| | | | **Belt subtotal** | **$4,080** |

> **Chain fallback:** if the Gates rating disappoints at the hip, v1's
> duplex #40 chain (12T→48T for 4:1) drops into the same envelope —
> heavier and needing lube, but with known 27.8 kN ultimate strength.

---

## C. Structural joint-bearing assemblies (18)

The load-path decision ([`DRIVETRAIN.md` §4](DRIVETRAIN.md#4-the-structural-joint-bearing-the-load-path-decision)):
foot loads go **shaft → paired bearings → link**, never into the
actuator's output bearing.

| Qty | Item | Spec to buy | Unit $ | Line $ |
|---:|---|---|---:|---:|
| 12 | **Hip/knee joint set** | Hollow 25–30 mm shaft, 2 × tapered-roller bearings (moment-load pair), machined housing/clevis bores, pulley bolt circle. | $150 | $1,800 |
| 6 | **Yaw joint set** | Thin-section four-point-contact bearing (large bore for cable pass-through), seat machining. | $120 | $720 |
| | | | **Bearings subtotal** | **$2,520** |

---

## D. Parking pin locks (12)

Spring-applied / solenoid-retracted / power-off-engaged, **at the
joint** (pin into the driven pulley's hole ring) — 6 hip-pitch + 6
knee, **all 12 required**
([`DRIVETRAIN.md` §6](DRIVETRAIN.md#6-load-holding-joint-side-parking-pin-locks-v1s-rule-better-hardware)).
Mostly shop-built from catalog pieces; per-piece SKUs in
[`PARTS.md` §4](PARTS.md#4-parking-pin-locks-12), including why COTS
friction brakes (INTORQ BFK458 class, 4–8 kg each) lost.

| Qty | Item | Spec to buy | Unit $ | Line $ |
|---:|---|---|---:|---:|
| 12 | **Parking pin lock assembly** | Ø12 hardened dowel + double-shear clevis block + return spring + **24 V tubular pull solenoid (~25 W pull / 12 W hold, 12 mm stroke)** + hardened ring bushings in the pulley web. ~0.35 kg. | $80 | **$960** |

**Drivetrain grand subtotal (A+B+C+D): ~$23,580** — the figure headlined
in [`README.md` §7](README.md#7-cost-roll-up).

---

## E. Legs (6 × 6061-T6)

| Qty | Item | Spec | Line $ |
|---:|---|---|---:|
| ~15 m | 60×40×3 box + Ø50×3 tube, 6061-T6 | Femurs + tibias ([`STRUCTURE.md`](STRUCTURE.md)) | $400 |
| 1 lot | Machined hub blocks, clevises, hard-stop tabs | CNC, 6 legs | $1,200 |
| 1 lot | Doubler plates, tensioner pads, misc | | $400 |
| | | **Legs subtotal** | **$2,000** |

## F. Feet

| Qty | Item | Spec | Line $ |
|---:|---|---|---:|
| 6 | Urethane pad + Al disc + load-cell boss | 60A cast urethane; sensor itself in §H | $400 |

## G. Chassis, saddle & rider interface

| Qty | Item | Spec | Line $ |
|---:|---|---|---:|
| 1 | Welded 6061 tube frame + deck, ~880 mm wide | carries yaw mounts, battery, e-bay | $1,600 |
| 1 | Saddle + suspension seatpost | | $250 |
| 1 | 4-point harness | | $150 |
| 2 | Footpegs + mounts | | $120 |
| 1 | Handlebar + grips + e-stop mount | | $180 |
| | | **Chassis subtotal** | **$2,300** |

## H. Battery & power (12S bus)

| Qty | Item | Spec | Line $ |
|---:|---|---|---:|
| 1 | **12S Li-ion pack ~40 Ah / 1.8 kWh**, BMS ≥ 100 A | 50.4 V full — **12S, not 13S** ([`POWER_SYSTEM.md`](POWER_SYSTEM.md)) | $550 |
| 1 | 12S balance charger | | $130 |
| 1 | Main contactor ≥120 A + pre-charge | | $120 |
| 1 | Class-T fuse ~100 A + holder | | $50 |
| 1 | Fused branch bus bar + 6 × 30 A | one per leg | $60 |
| 1 | 48→24 V DC-DC **≥ 350 W** | 12 lock solenoids + logic | $80 |
| 1 lot | 6 AWG main / 12 AWG branch wire, XT30s, lugs | | $210 |
| | | **Battery + power subtotal** | **$1,200** |

## I. Control electronics & sensors

| Qty | Item | Spec | Line $ |
|---:|---|---|---:|
| 1 | Compute (Jetson Orin Nano class) | gait + IK + estimation over CAN | $500 |
| 2–3 | CAN interfaces | 18 nodes on 2–3 buses | $180 |
| 1 | 9-DOF IMU | body attitude | $300 |
| 6 | Foot load cell + amp | contact/force per foot | $270 |
| 12 | Lock-solenoid drivers (fail-safe, default-open, PWM economizer) | | $96 |
| 1 lot | Signal wiring, connectors, glands, carriers | | $354 |
| | | **Electronics subtotal** | **$1,700** |

## J. Fasteners, machining & misc hardware

| Qty | Item | Line $ |
|---:|---|---:|
| 1 lot | M8/M10 10.9 fasteners, nylocs, washers | $500 |
| 1 lot | Motor/lock mount + bracket machining | $700 |
| 1 lot | Misc bearings, keys, bushings, shims | $300 |
| | **Hardware subtotal** | **$1,500** |

## K. Consumables & finishing

| 1 lot | Threadlocker, heat-shrink, paint, pinch-point decals, guards | **$400** |
|---|---|---:|

---

## Grand total

| Bucket | Cost (USD, Aug 2026) |
|---|---:|
| A. Actuators (18 × AK80-64) | $16,020 |
| B. Belt stages (18) | $4,080 |
| C. Joint-bearing assemblies (18) | $2,520 |
| D. Parking pin locks (12) | $960 |
| **→ Drivetrain subtotal** | **$23,580** |
| E. Legs | $2,000 |
| F. Feet | $400 |
| G. Chassis + rider interface | $2,300 |
| H. Battery + power | $1,200 |
| I. Electronics + sensors | $1,700 |
| J. Hardware + machining | $1,500 |
| K. Consumables | $400 |
| **Full-build estimate** | **~$33,080 (≈ $31–36 k)** |
| Recommended spares (1 actuator + 1 lock + 1 belt set) | +$1,220 |

Roughly **$8 k under v1** with less than half the mass — the AK80-64's
torque density, the belt stages, and $960 of pin locks doing the work
the RMD-X15s' price tags and $3k of brakes did in v1.

---

## L. Bench-test order (de-risk before buying 18)

1. Buy **1 × AK80-64 + 1 hip belt stage + 1 pin-lock kit + 1 hip
   bearing set** (~$1,370).
2. Build a single hip-joint rig: actuator → 4:1 belt →
   bearing-supported shaft → lever arm loaded to **0.36 kN·m**.
3. Verify the **pin lock holds** the loaded joint at zero motor current
   (no creep, no ring deformation), **engages on power-off**, and that
   a deliberate mid-motion power cut settles ≤15° onto the pin without
   drama (add the phase-short relay if the settle is harsh).
4. Verify **418 N·m peak** against a stop without belt tooth-jump or
   thermal fault, and measure real stage efficiency vs the assumed 0.87.
5. **Get Gates application engineering to sign off** the three drives
   (hip 8MGT-960-36 at 3.28 kN transient / 1.31 kN continuous is the
   worst) — catalog-anchored margin is >2×, but certify it.
6. Measure hip thermal at a simulated walk duty cycle; decide 4:1 vs
   5:1 vs AKH70-48 hips **now**, while it's a pulley/PO change.
7. Only then order the remaining 17 actuators, 17 stages, 11 locks,
   17 bearing sets.

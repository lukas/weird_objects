# `rideable_v1` — Bill of Materials & Shopping List

> Orderable bill of materials for **one rideable hexapod walker**.
> Quantities are for a complete vehicle with a small overage on
> consumables and fasteners. Prices are mid-2026 USD, order-of-magnitude.
> Numbers are consistent with [`README.md`](README.md),
> [`DRIVETRAIN.md`](DRIVETRAIN.md), [`STRUCTURE.md`](STRUCTURE.md), and
> [`POWER_SYSTEM.md`](POWER_SYSTEM.md).
>
> **Status: design draft, unbuilt.** De-risk by buying **one
> RMD-X15-450 + one secondary set + one brake** first and building a
> single-joint bench rig before committing to the full 18-actuator set.

Vendor links are stable search / product links rather than one-off SKUs,
because listings churn. Confirm every spec against a current datasheet at
order time.

---

## A. Actuators (the long pole)

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 12 | **MyActuator RMD-X15-450** | QDD actuator, 145 N·m rated / **450 N·m peak**, 72 V, internal 20.25:1, ~98 rpm rated, ~3.5 kg, integrated driver + encoder (CAN). **6 for hip-pitch + 6 for knee.** Buy all from one batch. | ~$1,515 | **$18,180** | [MyActuator RMD-X15](https://www.myactuator.com/product-page/rmd-x15) |
| 6 | **MyActuator RMD-X8-120** | QDD actuator, 43 N·m rated / **120 N·m peak**, 48 V, internal 19.6:1, ~127 rpm, ~1.4 kg, integrated driver + encoder (CAN). **Hip-yaw, direct drive.** | ~$645 | **$3,870** | [MyActuator RMD-X8](https://www.myactuator.com/product-page/rmd-x8) |
| — | *(recommended spare)* | 1 × RMD-X15-450 + 1 × RMD-X8-120 held as spares | — | *(+~$2,160)* | — |
| | | | **Actuators subtotal** | **$22,050** | |

---

## B. Secondary reductions (12 sets, 6:1)

One set per hip-pitch and per knee joint (the 12 RMD-X15 joints). The
hip-yaw joints are **direct-drive** and need no secondary. Pick **one**
of the three build styles per [`DRIVETRAIN.md` §3](DRIVETRAIN.md#3-the-6-1-secondary-reduction);
the belt option is the recommended default and is costed here.

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 12 | **HTD-14M driven pulley (large)** | ~72T HTD-14M, bore/keyway to the joint shaft. The 6:1 slow-side pulley. | ~$110 | $1,320 | [HTD 14M timing pulley](https://www.sdp-si.com/products/timing-belt-pulleys/) |
| 12 | **HTD-14M drive pulley (small)** | ~12T HTD-14M, bore to the RMD-X15 output. The fast-side pulley. | ~$55 | $660 | [HTD 14M timing pulley](https://www.sdp-si.com/products/timing-belt-pulleys/) |
| 12 | **HTD-14M belt** | 40 mm wide, length to centre distance. | ~$35 | $420 | [HTD 14M timing belt](https://www.sdp-si.com/products/timing-belts/) |
| 12 | **Joint output shaft + 2 bearings + pillow-block/housing** | Machined shaft, 2 × tapered-roller or deep-groove bearings, housing/tensioner plate per joint. | ~$150 | $1,800 | machine shop / [McMaster bearings](https://www.mcmaster.com/) |
| | | | **Secondary-reduction subtotal (~$350/set × 12)** | **$4,200** | |

> **Chain alternative:** #40 roller-chain sprockets (~12T / ~72T) + chain
> + guard land at roughly the same ~$350/set, more shock-tolerant, noisier,
> needs lubrication. **Planetary alternative:** a 6:1 planetary gearhead
> rated for ~2.4 kN·m output runs meaningfully more per joint but is the
> most compact and stiff. Cost the option you pick; totals below use the
> belt.

---

## C. Fail-safe load-holding brakes (12)

Spring-applied, electrically-released electromagnetic brakes on the
**actuator output shaft (input of the secondary)** — see
[`DRIVETRAIN.md` §4](DRIVETRAIN.md#4-the-knee-brake-load-holding-architecture).
Each holds ~160 N·m there; buy at a ≥ ~240 N·m static rating for margin.

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 12 | **Spring-applied / power-off electromagnetic brake** | Static holding ≥ ~240 N·m, 24 V or 48 V coil (see power rail below), through-bore or shaft-mount to fit the RMD-X15 output shaft, ~0.5 kg. **6 knee (required) + 6 hip (optional but funded).** | ~$200 | **$2,400** | [power-off / fail-safe brake](https://www.google.com/search?q=spring+applied+electromagnetic+fail+safe+brake+240+Nm) |
| | | | **Brakes subtotal** | **$2,400** | |

> Drop to **6** (knees only) and this line halves to ~$1,200 if a static
> hip-torque analysis clears the hip of needing a brake.

**Drivetrain grand subtotal (A + B + C, no spares):** **~$28,650** — the
~$27–30 k drivetrain figure headlined in [`README.md`](README.md).

---

## D. Legs — welded 4130 chromoly space-frames

Six identical triangulated space-frame legs, ~3 kg each. Full rationale
and stress handling in [`STRUCTURE.md`](STRUCTURE.md).

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| ~40 m | **4130 chromoly tube, 38 mm OD × 2.4 mm wall** | Seamless / DOM 4130, main chords + diagonals for 6 legs. | ~$14/m | ~$560 | [4130 chromoly tube](https://www.aircraftspruce.com/catalog/mepages/4130tubing.php) |
| ~10 m | **4130 chromoly tube, 25 mm OD × 1.6 mm wall** | Lighter diagonals / gussets. | ~$9/m | ~$90 | [4130 chromoly tube](https://www.aircraftspruce.com/catalog/mepages/4130tubing.php) |
| 1 set | **4130 sheet gussets + joint plates** (3–5 mm) | Laser/water-jet cut node plates, bearing bosses, motor mounts. | ~$450 | ~$450 | local laser shop |
| — | **TIG weld + normalising** (job or in-house) | Weld all six legs; stress-relieve/normalise the weldments. | ~$400 | ~$400 | local fab shop |
| | | | **Legs subtotal** | **~$1,500** | |

---

## E. Feet

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 6 | **Urethane foot pad + 6 mm steel disc** | 60-A cast urethane around a steel disc, M-bolted to the tibia. Grippy, compliant, replaceable. | ~$60 | ~$360 | cast in-house / [Smooth-On urethane](https://www.smooth-on.com/) |
| | | | **Feet subtotal** | **~$360** | |

---

## F. Chassis, saddle & rider interface

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 1 | **Chassis tube frame** | Welded 6061-T6 or A36 square tube hex frame + deck plate; carries 6 hip-yaw actuators, battery, e-bay, saddle. | ~$1,600 | ~$1,600 | local fab shop |
| 1 | **Saddle + suspension seatpost** | Motorcycle/bicycle saddle on a suspension post. | ~$250 | ~$250 | [suspension seatpost](https://www.google.com/search?q=suspension+seatpost) |
| 1 | **4-point harness** | Automotive 4-point. | ~$150 | ~$150 | [4-point harness](https://www.google.com/search?q=4+point+racing+harness) |
| 2 | **Footrests / pegs + mounts** | Welded to the deck. | ~$60 | ~$120 | [motorcycle foot pegs](https://www.google.com/search?q=motorcycle+foot+pegs) |
| 1 | **Handlebar + control grip + e-stop mount** | Rider inceptor + mushroom e-stop within reach. | ~$180 | ~$180 | — |
| | | | **Chassis + rider subtotal** | **~$2,300** | |

---

## G. Battery & power (72 V bus)

Full sizing and current budget in [`POWER_SYSTEM.md`](POWER_SYSTEM.md).

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 1 | **72 V (20S) Li-ion pack, ~42 Ah / ~3 kWh** | 20S Li-ion (18650/21700), integrated BMS ≥ 80 A continuous, ~20 kg. 72 V nom / 84 V full. | ~$1,100 | ~$1,100 | [20S ebike battery](https://www.google.com/search?q=20S+72V+lithium+ebike+battery+pack) |
| 1 | **72→48 V DC-DC converter** | ≥ ~300 W for the six RMD-X8 yaw actuators (48 V rail). | ~$120 | ~$120 | [72V to 48V DC-DC converter](https://www.google.com/search?q=72V+to+48V+DC-DC+converter+300W) |
| 1 | **72→24 V DC-DC converter** | Logic + brake-release coils + contactor + compute. | ~$60 | ~$60 | [72V to 24V DC-DC converter](https://www.google.com/search?q=72V+to+24V+DC-DC+converter) |
| 1 | **Main contactor + pre-charge** | ≥ 100 A DC contactor, pre-charge resistor + relay, on the 72 V bus. | ~$120 | ~$120 | [EV DC contactor 100A](https://www.google.com/search?q=EV+DC+contactor+100A+precharge) |
| 1 | **Main fuse — Class-T / MIDI, ~80 A** | At the pack +, sized to the main-lead ampacity. | ~$40 | ~$40 | [Class T fuse 80A](https://www.google.com/search?q=Class+T+fuse+80A+holder) |
| 1 | **Handlebar e-stop (latching mushroom)** | Drops the contactor **and** the brake-release rail → brakes engage. | ~$25 | ~$25 | [latching e-stop mushroom](https://www.google.com/search?q=latching+emergency+stop+mushroom+button) |
| 1 | **72 V balance charger** | 20S Li-ion charger, ~5–10 A. | ~$150 | ~$150 | [20S 84V lithium charger](https://www.google.com/search?q=20S+84V+lithium+battery+charger) |
| 1 | **Power wiring, lugs, bus bars, DC clamp meter** | 8 AWG bus, 12 AWG branch, ring lugs, heat-shrink. | ~$120 | ~$120 | [8 AWG silicone wire](https://www.google.com/search?q=8+AWG+silicone+wire) |
| | | | **Battery + power subtotal** | **~$1,735** | |

---

## H. Control electronics & sensors

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 1 | **Compute (Jetson Orin Nano / industrial PC)** | Runs gait + IK + state estimation over CAN. | ~$500 | ~$500 | [Jetson Orin Nano](https://www.google.com/search?q=Jetson+Orin+Nano+developer+kit) |
| 2 | **CAN-FD interface / hub** | Talks to the 18 RMD actuators (CAN). | ~$120 | ~$240 | [CAN FD USB interface](https://www.google.com/search?q=CAN+FD+USB+interface) |
| 1 | **9-DOF IMU** | Body attitude; e.g. VectorNav-class or BMI088 breakout. | ~$300 | ~$300 | [9 DOF IMU](https://www.google.com/search?q=9+DOF+IMU+module) |
| 6 | **Foot force sensor + amp** | ≥ 200 kgf load cell per foot + amplifier; foot-contact detection. | ~$45 | ~$270 | [200 kg load cell HX711](https://www.google.com/search?q=200kg+load+cell+HX711) |
| 12 | **Brake-release driver (relay/MOSFET module)** | One per fitted brake coil; fail-safe (defaults OFF). | ~$8 | ~$96 | [solid state relay module](https://www.google.com/search?q=solid+state+relay+module+DC) |
| 1 | **Wiring, connectors, CAN cable, glands, cable carriers** | Trunk + per-joint. | ~$300 | ~$300 | — |
| | | | **Electronics subtotal** | **~$1,706** | |

---

## I. Bearings, shafts, fasteners & misc hardware

| Qty | Item | Spec to buy | Unit $ | Line $ | Link |
|---:|---|---|---:|---:|---|
| 1 lot | **Joint bearings (yaw/hip/knee) beyond the reduction sets** | Cross-roller / tapered-roller at the 6 yaw axes + support bearings. | ~$900 | ~$900 | [McMaster bearings](https://www.mcmaster.com/) |
| 1 lot | **Structural fasteners** | M8/M10 grade 10.9 SHCS, nyloc, washers for motor mounts, brackets, foot pads. | ~$500 | ~$500 | [McMaster fasteners](https://www.mcmaster.com/) |
| 1 lot | **Motor mount / bracket machining** | CNC motor flanges + brake mounts. | ~$600 | ~$600 | Xometry / local shop |
| | | | **Hardware subtotal** | **~$2,000** | |

---

## J. Consumables & finishing

| Qty | Item | Unit $ | Line $ |
|---:|---|---:|---:|
| 1 lot | Threadlocker, anti-seize, cable ties, heat-shrink | — | ~$150 |
| 1 lot | Etch primer + powder-coat / paint | — | ~$250 |
| 1 lot | Pinch-point decals, guards, misc | — | ~$100 |
| | | **Consumables subtotal** | **~$500** |

---

## K. Grand total

| Bucket | Cost (USD, mid-2026) |
|---|---:|
| A. Actuators — 18 installed, no spares (12 × $1,515 + 6 × $645) | $22,050 |
| B. Secondary reductions (12 sets) | $4,200 |
| C. Fail-safe brakes (12) | $2,400 |
| **→ Drivetrain subtotal (A₁₈ + B + C)** | **~$28,650** |
| D. Legs (4130 space-frames) | $1,500 |
| E. Feet | $360 |
| F. Chassis + saddle + rider interface | $2,300 |
| G. Battery + power | $1,735 |
| H. Control electronics + sensors | $1,706 |
| I. Bearings, shafts, fasteners, misc | $2,000 |
| J. Consumables & finishing | $500 |
| **Full-build subtotal** | **~$38,750** |
| Recommended spares (1× X15 + 1× X8) | +$2,160 |
| **Full-build with spares** | **~$40,900 (≈ $38–45 k)** |

The 18 installed actuators (12 × $1,515 + 6 × $645 = **$22,050**) are the
figure rolled into the drivetrain subtotal; the recommended spare pair
(1 × X15 + 1 × X8 = $2,160) is tracked separately on the last line so it
doesn't inflate the per-vehicle build cost.

The **drivetrain (A+B+C) is ~$28.7 k**, ~74% of the installed cost — the
payoff of the tucked-stance / QDD decision is that this stays in the high
‑$20 k range instead of the $100 k+ of a harmonic-drive build.

---

## L. Bench-test order (de-risk before buying 18)

1. Buy **1 × RMD-X15-450 + 1 × 6:1 secondary set + 1 × fail-safe brake**.
2. Build a single knee-joint rig: actuator → brake on the fast shaft →
   6:1 secondary → a lever arm loaded to ~0.85 kN·m at the joint.
3. Verify the **brake holds** the loaded joint at **zero motor current**
   (measure winding current = 0, confirm no creep, confirm no heat).
4. Verify **power-off engages** the brake and holds the load (pull the
   coil supply and confirm the joint locks).
5. Verify the actuator delivers **~98 °/s** unloaded and hits its peak
   torque against a stop without thermal fault.
6. Only then order the remaining 17 actuators, 11 reductions, 11 brakes.

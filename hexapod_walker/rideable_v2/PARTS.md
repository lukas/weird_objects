# `rideable_v2` — Part Specifications (SKU level)

> The orderable detail behind [`BOM.md`](BOM.md): exact part numbers,
> drive geometry, bushings, bearings, and the surveyed-and-rejected
> alternatives. Prices/SKUs checked **Aug 2026**; re-verify at PO time.
> Quantities are for one machine (spares in [`BOM.md`](BOM.md)).

---

## 1. Actuators (18×)

| Part | Spec | Notes |
|---|---|---|
| **CubeMars AK80-64 KV80** ×18 | 120 N·m peak / 48 N·m rated, 64:1 internal planetary, 48 rpm rated @ 48 V (75 no-load), 7 A rated / 19 A peak, 850 g, CAN, XT30 power | [store.cubemars.com](https://store.cubemars.com/products/ak80-64), $889.90. Confirm integrated-driver option at checkout. Envelope ~Ø98 × 60 mm; bolt pattern per current datasheet — download the STEP at order time, the mount plates key off it. |
| *(hip fallback)* **CubeMars AKH70-48 V1.0** | 222 N·m peak / 74 N·m rated, 28 rpm rated, 1.396 kg, dual 21-bit encoders, dual-CAN daisy-chain | $698.90. Only if ballast testing shows the 4:1 hip margin short — see [`DRIVETRAIN.md` §7](DRIVETRAIN.md#7-the-akh70-48-alternative). |

## 2. Belt drives (18 stages)

Family: **Gates Poly Chain GT Carbon, 8 mm pitch (8MGT), 36 mm width**.
Stock Gates 8M sprocket grooves are 22/25/26/28/30/32/34/36/38/40/… —
**there is no stock 24T**, which is why the hip is 25T→100T. Driven
pulleys are custom (they double as the joint hub disc and carry the
parking-pin hole ring), quoted from B&B Manufacturing or any shop with
an 8MGT hob; 7075-T6 or hard-anodized 6061, lightening pockets, hubs
per [`DRIVETRAIN.md` §4](DRIVETRAIN.md#4-the-structural-joint-bearing-the-load-path-decision).

| Drive (×6 each) | Driver (stock) | Driven (custom) | Belt (stock) | Center distance | Peak / cont tension |
|---|---|---|---|---:|---|
| **Hip 4:1** | Gates **8MX-25S-36** 25T sprocket (or MPB-bushed equal), bore to AK80-64 output | 100T, PD Ø254.6 mm, lock-hole ring at r = 90 mm | **8MGT-960-36** (Gates 9274-2120, 120 teeth) | ~208 mm | 3.28 / 1.31 kN |
| **Knee 3:1** | Gates **8MX-28S-36** 28T | 84T, PD Ø213.9 mm, lock-hole ring at r = 90 mm (tooth root is at r≈102 — a ring at 100 would break into the teeth) | **8MGT-896-36** (Gates 9274-2112, 112 teeth) | ~212 mm (down the femur; motor ~140 mm from the hip axis, opposite femur face from the hip pulley) | 2.93 / 1.17 kN |
| **Yaw 2:1** | Gates **8MX-36S-36** 36T | 72T, PD Ø183.3 mm, large bore (cable pass) | **8MGT-720-36** (Gates 9274-2090, 90 teeth) | ~136 mm | 2.28 / 0.91 kN |

* **Capacity anchor (the verification the draft flagged):** the Gates
  Poly Chain GT Carbon Drive Design Manual (17595_2012) worked example
  runs a **12 mm** 8MGT at ~2.6 kN effective tension as a normal rated
  point; Table 11 width constants (Y = 65 @ 12 mm → 194 @ 36 mm) put
  the **36 mm** belt in the **~7.7 kN class** → >2× margin over the
  hip's 3.28 kN transient (enforced by `make check`). Geometry
  guidelines also pass: belt width (36) < small-sprocket PD (63.7 mm);
  center distances ≪ 8× small-sprocket PD. **Have Gates application
  engineering certify all three drives at PO time.**
* **Installation tension:** Gates Formula 14 with Table 11 constants
  (8M×36: M = 0.97, Y = 194, minimum static tension **84 lb/span**).
  At the hip's continuous load expect ~1.3–2.0 kN/span — set with a
  Gates 350C sonic tension meter, and note the joint bearings and femur
  are sized for the resulting **~5 kN shaft-pull bound**
  ([`STRUCTURE.md` §2](STRUCTURE.md#2-femur-350-mm-the-bending-critical-link)).
* **Driver-sprocket pilot bearing (18×):** the belt shaft pull
  (~2.9 kN continuous / ~4.9 kN peak at the hip, working + pretension)
  is far beyond the AK80-64's own output bearing (2.0 kN dyn /
  2.5 kN static), so each driver sprocket hub extends through a
  **6905-2RS** (25 × 42 × 9 mm, ~$8) seated in the mount plate,
  outboard of the belt — the motor bearing carries only its share.
  The mount itself is a **face-plate at the motor flange with a
  clearance bore for the sprocket hub** (the belt plane must stay
  clear of solid plate; the BuildViz mount block is symbolic).
* **Tensioner:** smooth back-side idler Ø60–70 mm on an eccentric
  bushing mount, one per stage (fixed centers everywhere else); 6061
  guard over each run, full-length on the femur.

## 3. Structural joint bearings (18 sets)

| Joint (qty) | Bearing | Shaft | Notes |
|---|---|---|---|
| Hip-pitch + knee (12) | **2 × 30205 tapered roller** (25 × 52 × 16.25 mm) **back-to-back (O-arrangement)**, light preload | Ø25 4140 HT, shouldered | Basic dynamic ~28 kN each — foot load (1.19 kN design) + belt shaft pull (~5 kN bound) are loafing. Contact seals + lithium grease; set preload at assembly, check at ballast milestones. NOTE the joint is **single-shear** (clevis plate on one side only; belt + lock own the pulley face): the ~0.53 kN·m joint moment becomes a force couple across the pair, so the arrangement matters — back-to-back puts the pressure centers ~48 mm apart (≈ +14.1 / −7.9 kN per row, well inside C0 ≈ 27 kN); face-to-face would collapse the effective spread toward zero and overload the rows. First-pass FEA (`tools/fea_joint_shaft.py`, `tools/fea_leg_nodes.py`): shaft ~220 MPa working → SF ~3.0 static on 4140 HT; clevis node ~85 MPa → SF ~3.3 on 6061-T6. Escalation if fatigue margins demand it: Ø30 / 30206 in the same envelope. |
| Hip-yaw (6) | **2 × 32006 tapered roller** (30 × 55 × 17 mm), vertical axis | Ø30 hollow (cable pass-through) | Reacts the leg cantilever moment; the 72T pulley bolts to the rotating hub. |

(v1's thin-section four-point bearing survives as an alternative for
the yaw if packaging demands it; the 32006 pair is cheaper and stiffer.)

## 4. Parking pin locks (12×)

Shop-built assembly, ~0.35 kg / ~$80 per joint
([`DRIVETRAIN.md` §6](DRIVETRAIN.md#6-load-holding-joint-side-parking-pin-locks-v1s-rule-better-hardware)):

| Piece | Spec |
|---|---|
| Pin | Ø12 × 60 mm hardened dowel (ISO 8734 / DIN 6325), double-shear engagement through the clevis into the pulley web |
| Hole ring | Ø12.2 mm hardened steel ring bushings pressed into the driven pulley web at r = 90 mm, one per 15° of joint travel across the working range (r = 90 clears the knee 84T tooth root at r≈102; ligament between Ø18 bushings is ~5.6 mm — or use one hardened insert ring) |
| Clevis block | 6061, bolts to the parent link (coxa at the hip, femur at the knee), guides the pin both sides of the pulley web |
| Spring | Die spring, ~60–80 N at engaged length — pin extends on power loss |
| Solenoid | **24 V DC tubular pull, ≥12 mm stroke, ≥80 N pull at full stroke** (pin must clear ~10 mm of web + bushing engagement plus 2 mm; the solenoid is weakest at full extension, the spring strongest) — e.g. Johnson Electric/Ledex STA 195207-series or Guardian Electric T12x19 class |
| Driver | Low-side MOSFET + **PWM economizer** (full 25 W pull ~200 ms, then ~12 W hold), default-open ([`POWER_SYSTEM.md` §5](POWER_SYSTEM.md#5-the-fail-safe-lock-power-interlock)) |
| Feedback | Reed/hall switch on the pin for engaged/retracted state to the controller |

**Why not a COTS friction brake** (the draft's original plan — surveyed
Aug 2026 and rejected on mass):

| Candidate | Static torque | Mass | Street price | Verdict |
|---|---:|---:|---:|---|
| INTORQ **BFK458**-14 | 60–80 N·m | ~2.6 kg | ~$250 | Too small for the hip's 103 N·m fast-shaft hold, and already 7× the pin's mass |
| INTORQ **BFK458**-16 | 100–145 N·m | ~4.3 kg | ~$350–450 | Fits hip hold; ×12 ≈ **+50 kg** → eats the entire hip torque margin |
| Mayr ROBA-stop-M / KEB Combistop (same class) | 120–260 N·m | 4–8 kg | $400–600 | Same story |

The pin lock holds **540 N·m rated at the joint** (1.5× hip design;
actual pin double-shear capacity ~10× that), for 0.35 kg. The trade —
discrete 15° lock positions and align-before-park — is handled by the
controller and verified on the single-joint rig
([`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18)).

## 5. Power system

| Part | Spec / SKU | Notes |
|---|---|---|
| Pack | **12S8P** 21700 (Molicel P42A/P45B class), ~40 Ah / ~1.8 kWh, from a reputable pack builder | 50.4 V full — **12S, not 13S** (AK80-64 limit). Spot-welded nickel + potted, XT90 out. |
| BMS | JBD/JK smart BMS, **12S, ≥100 A continuous**, UART/BT telemetry | Discharge FETs rated ≥ stand-up transient. |
| Charger | 12S (50.4 V) 10 A balance-capable | Fire-safe area charging per [`POWER_SYSTEM.md` §6](POWER_SYSTEM.md#6-hazards--commissioning). |
| Main contactor | **TE Kilovac EV200AAANA** (500 A, 12–900 VDC) or Gigavac GV200MA; budget alt: Albright SW180B-4 | Coil on the 24 V rail through the e-stop chain. |
| Pre-charge | 47 Ω / 50 W wirewound + small relay, ~250 ms into the 18 actuators' bus capacitance | |
| Main fuse | **Class-T 100 A** + Blue Sea 5502 holder | At the pack +. |
| Branch fuses | 6 × **MIDI/AMI 30 A** on a fused 6-way bus bar | One per leg. |
| DC-DC | **Mean Well SD-500L-24** (19–72 V in, 24 V / 500 W out) | Covers 12 solenoids + logic + contactor coil with margin (≥350 W required). |
| Wiring | 6 AWG silicone main, 12 AWG branches, XT30 at each actuator, XT90 pack side | |

## 6. Compute, sensing & rider controls

| Part | Spec / SKU | Notes |
|---|---|---|
| Compute | NVIDIA **Jetson Orin Nano** dev kit (or x86 NUC class) | Gait/IK/estimation at 100–200 Hz over CAN. |
| CAN | 3 × CANable 2.0 / PEAK PCAN-USB | 18 nodes on 2–3 buses, 1 Mbps, terminated both ends. |
| IMU | **BMI088 + BNO086** breakout pair (or one VN-100 if budget allows) | Body attitude + redundancy. |
| Foot sensing | 6 × **TAS606 200 kg** compression load cell + ADS1232 24-bit amp | ~80 Hz/foot contact detection; the urethane pad bolts over the cell boss. |
| E-stop | 22 mm NC latching mushroom (IDEC XW/Schneider XB4 class) on the bars + keyed battery isolator | Drops contactor coil AND solenoid rail — hardware chain, no software in the loop. |
| Lock feedback | 12 × reed/hall pin-position switches (in §4) | Parked state must be verified, not assumed. |

## 7. Ordering sequence

Follow [`BOM.md` §L](BOM.md#l-bench-test-order-de-risk-before-buying-18):
one actuator + one hip belt stage + one pin lock + one bearing set
first (~$1,370), certify the Gates drives, prove the lock on the rig,
then commit to the fleet.

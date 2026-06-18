# Hexapod Prototype BOM

This is the controlled bill of materials for **one complete tabletop
hexapod prototype**. The design is built around **FEETECH STS3215
(ST-3215-C018, 12 V / 30 kg-cm) serial-bus servos** in a **bearing-sandwich
joint** (one STS3215 on the driven side, a 688-2RS ball bearing on the
passive side, an Ø8 mm carbon-fibre tube as the leg segment). Do not
substitute other servo models unless you are ready to measure them and
regenerate the printed brackets/yokes.

> **Jun 2026 refit.** This supersedes the earlier DS3225 + Arduino Mega +
> 2×PCA9685 + drop-in-cradle design. The STS3215 are smart serial-bus
> servos daisy-chained on a single half-duplex TTL bus driven straight
> from an **Arduino Uno Q** (on-board Linux SoC + MCU) — there is **no
> Arduino Mega, no PCA9685, no Raspberry Pi, and no separate USB-to-TTL
> bus adapter** any more.  The Uno Q drives the half-duplex TTL bus
> directly, replacing both the Pi and the bus adapter.  It rides a
> stacked printed deck (`uno_q_tray` lower + `buck_tray` upper) on four
> M3 standoff columns above `chassis_top`; a **XINGYHENG 12 V→5 V buck
> converter** on the upper tray powers the Uno Q.  The LiPo is now
> **velcro-strapped to the top of `chassis_bottom`** (no clip-in
> `battery_holder`).

Links are stable Amazon search links rather than one-off ASINs, because
Amazon listings churn. Pick a well-reviewed Prime listing that matches the
spec exactly.

## Required Purchases

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 20 | FEETECH STS3215 serial-bus servo | **STS3215 (ST-3215-C018), 12 V, 30 kg-cm metal-gear smart serial-bus servo**, standard ~40 x 20 x 40 mm case with a 25T output spline carrying the flush 20 mm disc horn (4×M3 on a Ø14 cross).  The body is bolted to the cradle by 4× M2.5 into the servo's real END-face 10×10 mm hole square (plus the printed strap/clamp), not by output-face screws. Daisy-chained on a 1 Mbps half-duplex TTL bus. Buy all 20 from the same batch. 18 used + 2 spares. | [Amazon: FEETECH STS3215 30kg serial bus servo](https://www.amazon.com/s?k=FEETECH+STS3215+30kg+serial+bus+servo) |
| 1 | Arduino Uno Q | **Arduino Uno Q** (on-board Linux SoC + MCU). Runs Python gait/RL/teleop AND drives the half-duplex STS3215 TTL bus directly — replaces both the Raspberry Pi and the separate USB-to-TTL bus adapter. Mounts on the lower printed deck (`uno_q_tray`). | [Arduino Store: Uno Q](https://store.arduino.cc/products/uno-q) |
| 1 | microSD card | 32 GB or 64 GB, A1/A2 rated (Uno Q OS/storage if not using on-board eMMC). | [Amazon: 32GB A1 microSD card](https://www.amazon.com/s?k=32GB+A1+microSD+card) |
| 1 | LiPo battery | 3S 2200 mAh, 25C or higher, XT60 connector. The 11.1 V nominal (12.6 V full) drives the STS3215 bus V+ rail directly. Velcro-strapped to the top of `chassis_bottom` in the inter-plate gap. | [Amazon: 3S 2200mAh 25C lipo XT60](https://www.amazon.com/s?k=3S+2200mAh+25C+lipo+XT60) |
| 1 | XINGYHENG buck converter | **XINGYHENG 12 V→5 V step-down buck converter** (3 A+), drops the 3S rail to 5 V for the Uno Q (the STS3215 run on the raw 3S rail; the buck is for logic only). Mounts on the upper printed deck (`buck_tray`). Same part as prototype_v1. | [Amazon: XINGYHENG 12V to 5V buck converter](https://www.amazon.com/s?k=XINGYHENG+12V+to+5V+buck+converter) |
| 1 | Velcro straps | Hook-and-loop cinch straps (~15–20 mm wide) looped through the chassis_bottom strap slots to retain the LiPo on the plate top face. | [Amazon: velcro cinch straps 15mm](https://www.amazon.com/s?k=velcro+cinch+straps+15mm) |
| 1 | Bus servo cables | FEETECH 3-pin serial-bus servo cables (servos ship with one each; buy a small spare pack for the chassis-to-first-servo runs). | [Amazon: FEETECH serial bus servo cable](https://www.amazon.com/s?k=FEETECH+serial+bus+servo+cable) |
| 1 | LiPo charger | 3S balance charger, e.g. SkyRC B6 style or better. | [Amazon: 3S lipo balance charger](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | LiPo safety bag | Fire-resistant charging/storage bag. | [Amazon: lipo safety bag fireproof](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 1 | Anti-spark switch | XT60 RC LiPo anti-spark/on-off switch for servo rail power. | [Amazon: rc lipo anti-spark switch xt60](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |
| 2 | XT60 pigtails | Male/female XT60 silicone-wire pigtails, 12-14 AWG. | [Amazon: XT60 pigtail 12awg](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 12 | 688-2RS ball bearing | **Ø8 mm bore × Ø16 mm OD × 5 mm wide deep-groove sealed bearing.** Passive side of every hip + knee sandwich joint (2 per leg × 6 legs). Buy a 12+ pack for spares. | [Amazon: 688-2RS bearing 8x16x5](https://www.amazon.com/s?k=688-2RS+bearing+8x16x5) |
| 12 | 6706-2RS ball bearing (YAW) | **Ø30 mm bore × Ø37 mm OD × 4 mm wide thin-section sealed bearing.** The yaw joint of every leg uses a **SPACED PAIR** (2 per leg × 6 legs) stacked ~7 mm apart on the `coxa_yaw_hub` boss: the lower bearing rides around the disc horn, the upper a few mm above it. The pair reacts the cantilever MOMENT (tilt stiffness ∝ spacing²) into the `chassis_bottom` tower instead of the servo spline. Outer races press into the tower's Ø37 shoulders (heat-assisted press + a printed retaining lip — do NOT rely on PLA/PETG creep); inner races clamp on the hub boss. Buy a 12+ pack for spares. | [Amazon: 6706-2RS bearing 30x37x4](https://www.amazon.com/s?k=6706-2RS+bearing+30x37x4) |
| 1 | Carbon-fibre tube, Ø8 mm | **Ø8 mm OD × Ø6 mm ID roll-wrapped CF tube.** Femur + tibia leg segments (2 per leg). One ~1 m length yields all 12 segments (femur ≈ 70 mm + tibia ≈ 110 mm cut lengths). Epoxy-bonded into the printed sockets. | [Amazon: 8mm carbon fiber tube 6mm ID](https://www.amazon.com/s?k=8mm+carbon+fiber+tube+6mm+id) |
| 12 | Ø2.5 mm roll pin (spring pin) | Transverse retention pin through each CF-tube socket (epoxy + pin locks pull-out and spin). 2 per leg; buy an assortment. | [Amazon: 2.5mm spring roll pin assortment](https://www.amazon.com/s?k=2.5mm+spring+roll+pin+assortment) |
| 1 | Two-part epoxy | Slow-cure (30 min) structural epoxy for bonding the CF tubes into the printed yoke/bracket/foot sockets. | [Amazon: 30 minute structural epoxy](https://www.amazon.com/s?k=30+minute+two+part+epoxy) |
| 1 | Nylon zip-ties, 100-pack | 24 used for cable strain relief: 3 per leg at the cradle `cable_zip_post`s (yaw / hip / knee) + 1 per leg looped through the chassis_bottom leg-harness drop slot (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  2-3 mm wide, ~ 100 mm long. | [Amazon: nylon zip ties 4 inch 100 pack](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | Dupont jumper kit | Mixed M-F / F-F / M-M jumper wires for I2C and logic wiring. | [Amazon: dupont jumper wires 120 pcs](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | USB-C cable | USB-C cable for flashing / powering / console access to the Arduino Uno Q. | [Amazon: USB C cable data](https://www.amazon.com/s?k=USB+C+cable+data) |
| 1 | M3 screw assortment | M3 socket-head screws, nuts, washers, lengths 6/8/10/12/16/20 mm. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | M3 x 16 pan-head bolts | Foot/tibia hinge pin -- one per leg.  Passes through the foot_pad's FORK (3.5 + 6.4 + 3.5 = 13.4 mm) and engages an M3 nylock on the far cheek; the tibia's TANG sits in the slot (May 2026 inversion: fork on the foot, tang on the tibia -- pre-2026 it was the other way round, same pin + nut + 16 mm length).  Pan-head (low profile), threaded full length.  Stainless.  Reuse from the M3 assortment if it includes 16 mm. | [Amazon: M3 x 16 pan head stainless](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | M3 nyloc nuts | 100-pack nylon-insert lock nuts. **14 used (Jun 2026 deck redesign):** 6 on the foot-pad hinge pins (one per leg) + 4 UNDER chassis_bottom retaining the brass M-F standoffs' male threads on the rotated-45-deg 35-mm-radius pattern (CHASSIS_STANDOFF_HOLES_XY) + 4 UNDER chassis_top retaining the lowest deck standoff columns at `DECK_COLUMN_XY` (±41, ±33).  The servo is bolted by 4× M2.5 into its END-face case holes (plus the strap/clamp cap), so no servo-retention nut is used. | [Amazon: M3 nyloc lock nut 100 pack](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | M3 heat-set inserts (`94459A130`) | McMaster knurled brass M3 heat-set insert, Phi 4.0 mm pilot, Phi 5.7 mm OD, 5.0 mm length. **14 used (Jun 2026 deck redesign):** 8 in the printed deck trays for the board-mount bosses (4 `uno_q_tray` + 4 `buck_tray`) + 4 in the imu_pad (MPU-6050 mount) + 2 in chassis_top's printed bosses (switch_holster).  (The 24 sandwich-joint clamp-cap bolts self-tap and use NO inserts; the 72 M2.5 servo body-retention screws thread directly into the servo's own metal end-face case holes, also no inserts.)  Installed with a soldering iron at ~220 deg C, light downward pressure, ~10-15 s per insert, then cool ~30 s; an M3 x 8 SHCS (deck board / IMU pad) or M3 x 10 SHCS (switch_holster) threads into the brass instead of self-tapping into plastic. | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | M3 x 10 SHCS (`91290A114`) | M3 x 10 mm socket-head cap screw, black-oxide steel.  **14 used (Jun 2026 deck redesign):** 2 switch_holster mount bolts (DOWN through the holster ear into chassis_top boss inserts) + 4 chassis_top → brass-standoff bolts (DOWN from above chassis_top into the M-F standoff female top threads on the rotated-45-deg 35-mm-radius pattern) + **8 deck standoff-column bolts (DOWN through each tray's column bosses into the brass standoff female threads — 4 at `uno_q_tray`, 4 at `buck_tray`)**. | [McMaster 91290A114](https://www.mcmaster.com/91290A114/) |
| 1 | M3 x 8 SHCS (`91290A113`) | M3 x 8 mm socket-head cap screw, black-oxide steel.  **36 used (Jun 2026 deck redesign):** 8 deck board-mount bolts (4 Uno Q + 4 buck, DOWN through each PCB into the tray's M3 heat-set inserts) + 4 imu_pad (MPU-6050 mount) + **24 sandwich-joint clamp-cap bolts (2 per hip + knee clamp cap, self-tapping into the cradle ±X wall-end pilots; 12 clamp caps robot-wide)**.  Buy a 100-pack so you have spares. | [McMaster 91290A113](https://www.mcmaster.com/91290A113/) |
| 1 | M2.5 x 8 SHCS (`91290A104`) | M2.5 x 8 mm socket-head cap screw.  **72 used** for POSITIVE servo body retention (4 per cradle × 3 cradles per leg × 6 legs).  Each cradle bolts the servo's real **END-face M2.5 10 × 10 mm hole square** (measured Waveshare ST3215) through the cradle's −X wall; the head is recessed in a counterbore and the screw threads into the servo's own metal case.  This bolts the servo positively instead of only gripping it.  (Same M2.5 × 8 stock also serves as the spline center screws; buy a 100-pack.) | [McMaster 91290A104](https://www.mcmaster.com/91290A104/) |
| 1 | M3 standoffs (chassis) | M3 x 32 mm male-female brass standoffs, pack of 20.  **4 used** on the rotated-45-deg 35-mm-radius pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±35, 0) and (0, ±35)) clamping chassis_top to chassis_bottom across the CHASSIS_GAP = 32 mm inter-plate gap.  Male thread drops DOWN through chassis_bottom's Phi 3.4 mm clearance hole and is captured by an M3 nyloc nut underneath; female top accepts the M3 × 10 SHCS dropped DOWN from above chassis_top.  Re-verify lengths whenever CHASSIS_GAP changes. | [Amazon: M3 32mm standoffs male female brass](https://www.amazon.com/s?k=M3+32mm+standoffs+male+female+brass) |
| 8 | M3 standoff columns (deck) | M3 male-female brass standoffs for the stacked electronics deck on the `DECK_COLUMN_XY` (±41, ±33) pattern above chassis_top.  **4 at `DECK_LEVEL_1_STANDOFF_H` (16 mm, chassis_top → `uno_q_tray`)** + **4 at `DECK_LEVEL_2_STANDOFF_H` (22 mm, `uno_q_tray` → `buck_tray`)**.  Lowest column male threads pass through chassis_top and are retained by M3 nyloc nuts underneath; each upper level bolts down with an M3 × 10 SHCS.  Buy a mixed M-F standoff assortment that covers both lengths. | [Amazon: M3 standoff assortment male female brass](https://www.amazon.com/s?k=M3+standoff+assortment+male+female+brass) |
| 1 | M2.5 screw pack | M2.5 x 8 mm screws, useful for horn/adapter work if the servo-included screws are bad. | [Amazon: M2.5 8mm screw 50 pack](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M3x6 SHCS | 91290A111 | 72 | link-to-disc-horn bolts |
| M2.5 SHCS into servo case | 91290A104 | 72 | cradle servo body-retention bolts (M2.5 into servo end face) |
| M2.5x8 spline screw | 91290A104 | 18 | servo spline center screws |
| M3x8 SHCS into heat-set insert | 91290A113 | 12 | deck board-mount bolts (Uno Q + buck, M3 x 8 SHCS into inserts), imu_pad heat-set inserts (MPU-6050 mount) |
| M3x8 SHCS self-tap | 91290A113 | 24 | sandwich-joint clamp-cap bolts (M3 SHCS self-tap) |
| M3x10 SHCS | 91290A114 | 14 | chassis_top brass standoff bolts (M3 x 10 SHCS into standoff female thread), deck standoff column bolts (M3 x 10 SHCS into standoff female thread), switch_holster heat-set inserts |
| M3 heat-set insert | 94459A130 | 14 | deck board-mount heat-set inserts (Uno Q + buck), imu_pad heat-set inserts (MPU-6050 mount), switch_holster heat-set inserts |
| M3x16 pan-head | 92010A130 | 6 | foot hinge pins |
| M3 nyloc nut | 90576A102 | 14 | chassis_bottom brass standoff retention nuts, deck standoff column retention nuts (M3 nyloc under chassis_top), foot hinge pins |
|  |  | **246** | **total fasteners** |

Notes:
- The servo OUTPUT face is reserved for the flush 20 mm disc
  horn (no front-face mounting).  POSITIVE body retention bolts
  into the servo's REAL END-face M2.5 holes: each STS3215 +/-X end
  face carries a 10 x 10 mm square of 4 M2.5 case holes (measured
  Waveshare ST3215), and every cradle drives 4 x M2.5 x 8 SHCS
  through its -X wall into that square (`91290A104`, 4 per cradle
  x 3 cradles x 6 legs = 72; head recessed in a wall counterbore,
  threads into the servo's own metal case).  This positively bolts
  the servo instead of only gripping it; the hip/knee clamp cap
  and the yaw retainer strap are retained as secondary capture.
- Link-to-disc-horn bolts (72 x M3x6 SHCS / `91290A111`) thread
  into the 20 mm aluminum 25T disc horn's M3 TAPPED holes on a
  14 mm bolt circle (cross pattern at 0/90/180/270 deg); the
  aluminum is the thread-engagement medium -- no self-tap, no
  heat-set (June 2026 disc-horn switch, retiring the now-retired
  plastic 4-arm X-horn's M2x8 self-tap scheme).  See
  `fasteners/README.md` for the full rationale.
- Captive nyloc nuts are used at the foot-pad hinge pins (6 total,
  one per leg); the through-hole bolt is captured by the nyloc on
  the opposite side of the foot.  The previous 24 x M3 x 14
  coxa-bracket-to-chassis nyloc'd through-bolts were retired in
  the May 2026 chassis_bottom-integrated yaw cradle redesign --
  the printed bracket flange they clamped is gone, replaced by
  per-leg cradle bosses inside chassis_bottom.
- The M2.5 spline center screw ships with each STS3215 servo --
  it's listed here so the screwdriver-access verifier check knows the
  fastener exists, but you do NOT order it separately.
- See `fasteners/README.md` for the McMaster STEP swap-in flow if you
  want to replace the parametric fallback geometry with real CAD.

<!-- END FASTENERS -->
| 1 kg | PLA or PETG filament | 1.75 mm. Structural printed parts. PLA is easiest; PETG is tougher. | [Amazon: PLA filament 1.75mm 1kg](https://www.amazon.com/s?k=PLA+filament+1.75mm+1kg) |
| 250 g | TPU 95A filament | Foot pads. If you skip TPU, print feet in PLA and glue rubber tread into the cups. | [Amazon: TPU 95A filament 1.75mm](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | Heat-shrink kit | Assorted small heat-shrink tubing for power wiring cleanup. | [Amazon: heat shrink tubing assorted](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |

## Strongly Recommended Tools

| Qty | Item | Why | Link |
|---:|---|---|---|
| 1 | Digital calipers | Check STS3215 body/tab dimensions before printing all six legs. | [Amazon: digital calipers](https://www.amazon.com/s?k=digital+calipers) |
| 1 | Soldering iron kit | Required to install the M3 brass heat-set inserts in the deck trays / imu_pad / switch_holster bosses. | [Amazon: soldering iron kit beginner](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | Metric hex key set | M3 socket-head screws usually need a 2.5 mm hex key. | [Amazon: metric ball end hex key set](https://www.amazon.com/s?k=metric+ball+end+hex+key+set) |
| 1 | Super glue gel | Foot tread / small print fixes. | [Amazon: super glue gel](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | Needle file set | Useful if a servo well or femur slot is slightly tight from FDM over-extrusion. | [Amazon: needle file set](https://www.amazon.com/s?k=needle+file+set) |

## Printed Parts

Files are in `hexapod_walker/prototype/stl_prototype/`.

| Qty | STL |
|---:|---|
| 1 | `chassis_top.stl` |
| 1 | `chassis_bottom.stl` |
| 1 | `uno_q_tray.stl` (lower deck — Arduino Uno Q) |
| 1 | `buck_tray.stl` (upper deck — XINGYHENG buck converter) |
| 1 | `switch_holster.stl` |
| 12 | `servo_clamp_cap.stl` (MJF PA12 — 1 per hip + knee sandwich joint) |
| 6 | `coxa_link.stl` (yaw pad + arm + hip fixed side) |
| 6 | `femur_hip_yoke.stl` (hip moving yoke + CF-tube socket) |
| 6 | `femur_knee_bracket.stl` (knee fixed side + CF-tube socket) |
| 6 | `tibia_knee_yoke.stl` (knee moving yoke + CF-tube socket) |
| 6 | `tibia_foot_fitting.stl` (CF-tube socket + foot hinge tang) |
| 6 | `foot_pad.stl` (TPU) |

Femur = `femur_hip_yoke` + Ø8 CF tube + `femur_knee_bracket`.
Tibia = `tibia_knee_yoke` + Ø8 CF tube + `tibia_foot_fitting` + `foot_pad`.

Design B (May 2026) retired the printed `servo_horn_adapter` -- each
link now bolts DIRECTLY onto a purchased servo horn, so it no longer
appears in the printed-parts set.  June 2026 disc-horn switch: the
purchased horn is now a 20 mm aluminum 25T disc (Amazon B07D56FVK5),
bolted with 4 x M3 x 6 SHCS on a 14 mm bolt circle into the disc's
M3 tapped holes (replacing the old plastic 4-arm X-horn + M2 scheme).

For a one-leg test, print only:

| Qty | STL |
|---:|---|
| 1 | `coxa_link.stl` |
| 1 | `femur_hip_yoke.stl` |
| 1 | `femur_knee_bracket.stl` |
| 1 | `tibia_knee_yoke.stl` |
| 1 | `tibia_foot_fitting.stl` |
| 1 | `foot_pad.stl` |

## Bench Test Order

1. Buy **one STS3215** + **one Arduino Uno Q** + **two 688-2RS bearings** first to de-risk the joint.
2. Print one full leg set (above) and the `tools/sts3215_testfit.py` test-fit part to validate the bracket + bearing + disc-horn fit before printing six sets.
3. Confirm the STS3215 body is bolted by 4× M2.5 into its END-face 10×10 hole square (driven through the cradle's −X wall) plus the printed retainer (yaw strap / hip-knee clamp cap), the flush disc horn drives the yoke top arm via its 4× M3 leg bolts on the Ø14 cross, and the stub rides the 688 bearing with no bind.
4. Wire one STS3215 to the Uno Q's TTL bus pins and run:

```bash
python hexapod_walker/prototype_sts3215/pi_control/feetech_bus.py --port /dev/ttyUSB0 wiggle --joint 0
```

5. Once fit and motion are good, buy/print the rest.

## Rough Cost

| Bucket | Estimate |
|---|---:|
| STS3215 serial-bus servos, 20 total | $360-$500 |
| Arduino Uno Q + microSD + USB-C cable | $60-$100 |
| XINGYHENG buck converter | $8-$15 |
| Battery + charger + safety bag + switch + velcro | $70-$120 |
| 688 bearings + CF tube + roll pins + epoxy | $30-$55 |
| Fasteners / standoffs / deck columns / wiring consumables | $30-$55 |
| Filament + MJF clamp caps | $30-$55 |
| **Total** | **~$590-$955** |

If you already own a charger, tools, or filament, the actual
cash outlay is much lower.

# Hexapod prototype — shopping list & print queue

**Status: Jun 2026 bearing-sandwich refit (STS3215).** All printed parts
build watertight and the standing-pose assembly is kinematically frozen.
The `_verify_prototype.py` suite is being re-based from the old DS3225
drop-in-cradle paradigm to the STS3215 face-mount sandwich (mesh
watertightness, bolt engagement, harness drop, horn-stack/-sweep, insertion
path, cradle pockets, mating-face all pass; the cradle-openness /
servo-clearance / self-collision checks are being recalibrated for the
nested sandwich joint).

This is the everything-you-need-to-buy-and-print sheet. Numbers are
sized for **one complete walking robot** with a small spare margin
(~ 10% on fasteners, +2 servos).

---

## A. STL files to print

All files live under `hexapod_walker/prototype/stl_prototype/`. Filenames are
exactly as the generator writes them.

| # | Filename | Qty | Material | Layer | Infill | Walls | Print time (Ender 3) | Notes |
|---|---|---:|---|---|---:|---:|---:|---|
| 1 | `chassis_top.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | Identical to bottom |
| 2 | `chassis_bottom.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | Identical to top |
| 3 | `uno_q_tray.stl` | **1** | PLA / PETG | 0.2 mm | 20% gyroid | 3 | ~ 1.5 h | Lower deck — Arduino Uno Q (brain + direct STS3215 TTL bus driver). Stacks on 4 M3 standoff columns above chassis_top at `DECK_LEVEL_1_STANDOFF_H` (16 mm). |
| 4 | `buck_tray.stl` | **1** | PLA / PETG | 0.2 mm | 20% gyroid | 2 | ~ 1.5 h | Upper deck — XINGYHENG 12V→5V buck converter. Stacks on the uno_q_tray columns at `DECK_LEVEL_2_STANDOFF_H` (22 mm). |
| 4b | `spider_carapace.stl` | **1** | PLA / PETG | 0.2 mm | 10–15% gyroid | 3 | ~ 5–7 h | Domed cephalothorax shell (~141 × 124 × 34 mm) with the 8-eye spider face; bolts on as the 3rd deck level on 4 M3 standoffs at `DECK_COLUMN_XY` above `buck_tray`. Print **rim-down** (open skirt on the bed, apex up — self-supporting); colour it black/dark for the spider look. Open skirt + rear window keep the stack vented. |
| 4b | `switch_holster.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.3 h | Snap-in holster for anti-spark on/off switch.  Bolts to chassis_top's +X edge via 2 x M3 x 10 SHCS into 2 chassis_top heat-set inserts. |
| 4c | `imu_pad.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.2 h | 25 x 20 mm IMU mounting pad with 4 M3 heat-set insert bosses on the GY-521 15 x 11 mm hole pattern.  No fasteners between pad and chassis_top -- the pad is foam-taped to the chassis_top centre for vibration isolation. |
| 4d | `servo_clamp_cap.stl` | **12** | MJF PA12 (PLA OK) | 0.2 mm | solid | 3 | ~ 0.2 h ea | Sandwich-joint clamp cap — 1 per hip + knee joint (2 per leg x 6 legs). Bolts to the cradle ±X wall ends with 2 x M3 x 8 SHCS self-tapping into the cradle pilots. |
| 5 | `coxa_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Yaw pad + arm + hip fixed side (symmetric servo cradle — the passive disc horn rides the servo's own rear idler boss, so there is NO 688 bearing or back housing). Yaw-pad face down on the bed; the bracket opens UP. |
| 5b | `yaw_bearing_cap.stl` | **6** | PLA / PETG | 0.2 mm | 40% gyroid | 4 | ~ 0.3 h ea | TOP half of the SPLIT yaw-bearing tower (Jun 2026 insertion fix). Bolts onto each `chassis_bottom` tower with 3 x M3 x 8 SHCS self-tapping into the tower pilots (3 outboard ear lugs); a clean Ø37 through-bore holds the upper 6706 race. Print **flat, ring face down** on the bed (no supports). One per leg. |
| 6 | `femur_hip_yoke.stl` | **6** | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~ 4 h total | Hip moving yoke — SYMMETRIC clevis: both arms bolt to a disc horn (driven horn on the front, passive horn on the rear boss). + Ø8 CF-tube socket. Print spine-down. |
| 7 | `femur_knee_bracket.stl` | **6** | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~ 4 h total | Knee fixed side (symmetric servo cradle — passive disc horn rides the servo's rear idler boss, no 688 bearing) + Ø8 CF-tube socket. |
| 8 | `tibia_knee_yoke.stl` | **6** | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~ 4 h total | Knee moving yoke — SYMMETRIC clevis: both arms bolt to a disc horn (driven + passive). + Ø8 CF-tube socket. |
| 8c | `passive_horn_adapter.stl` | **12** | PLA / PETG | 0.2 mm | solid | 3 | ~ 0.2 h ea | Printed centering adapter (1 per hip + knee joint = 2 per leg × 6). Press-fits the servo's rear idler boss and centres the reused aluminium 25T disc horn on the passive side; the central M2.5 screw retains it. Print collar-down. |
| 8b | `tibia_foot_fitting.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 1.5 h total | Ø8 CF-tube socket + single foot-hinge tang (M3 pin). |
| 9 | `foot_pad.stl` | **6** | TPU 95A (PLA OK) | 0.25 mm | 100% (TPU) | 3 | ~ 1 h | TPU = grip; PLA = slips.  Foot carries the 2-cheek FORK (3.5 mm cheeks + 6.4 mm slot) straddling the tibia tang.  Print disk-on-bed, fork pointing UP. |

> **Carbon-fibre leg segments.** The femur and tibia are NOT printed as
> single spars any more — each is two printed end-fittings bonded onto an
> Ø8 mm CF tube (epoxy + a transverse Ø2.5 mm roll pin). Cut the tubes to
> the femur/tibia lengths, epoxy into the sockets, and drill/drive the
> retention pin through the cross-hole. See the test-fit part in
> `tools/sts3215_testfit.py` before committing six legs.

> **`servo_horn_adapter.stl` has been retired** (Design B, May 2026 -- commit
> `be06741`).  Each link now bolts DIRECTLY onto a purchased servo
> horn, so there is no printed adapter in the stack any more.  Skip
> this row if you have an older copy of this checklist.
>
> **June 2026 disc-horn switch:** the robot now drives a **20 mm
> aluminum 25T disc horn** (Amazon B07D56FVK5, "10Pcs Servo Horn Metal
> Aluminum 25T Silvery Servo Disc ... MG945 MG995 MG996") at every
> servo joint instead of the plastic 4-arm X-horn.  Each link bolts to
> the disc's flat top face with 4 x M3 x 6 SHCS on a 14 mm bolt circle,
> threading into the disc's M3 tapped holes.
>
> **June 2026 symmetric-yoke refit:** the hip and knee joints now also
> carry a SECOND disc horn on the servo's REAR idler boss (passive
> support), so the yoke bolts to a disc on BOTH faces instead of
> running a 688 ball bearing on the passive side.  You now need **30
> discs** — yaw 1 + hip 2 + knee 2 = 5 per leg × 6 legs (was 18).  The
> part ships in 10-packs, so order **3 packs** (30 + the M3 x 6 screws
> that come with them).  The rear disc is centred by the printed
> `passive_horn_adapter.stl` and retained by one M2.5 screw.

**Total print time (single Ender 3 / Bambu A1):** ~ 22 hours of
machine time, spread across 6 – 7 print sessions.

> **Don't have a printer?** The same files (re-oriented for MJF) live
> in `hexapod_walker/prototype/xometry_upload/` with a `manifest.csv` and a
> README that takes you through the Xometry / Shapeways / JLCPCB
> upload flow. Total there is ~ $580 in MJF PA12, vs ~ $20 in
> filament if you self-print.

If you only have a 220 × 220 mm printer (Ender 3), every part fits
the bed individually. The chassis plates (200 × 230 mm in their
default orientation) need to be rotated 30° to fit, or printed in
two halves and bolted — but most likely they fit on a 235 × 235 mm
hot bed (the actual usable area on most Ender 3 / Bambu A1 mini
printers) without rotation.

---

## B. Amazon shopping list (US, mid-2026)

> **Tip on the search links:** Amazon's URL format keeps the query
> intact, so the links below survive the inevitable ASIN churn.
> Pick the listing with the most reviews and `Prime` shipping. If
> you want the absolute cheapest, AliExpress / Banggood is roughly
> 50% cheaper but ships in 2 – 4 weeks.

### B.1 Servos (the controlled actuator for this design)

| Qty | Part | Why this one | Search link |
|---:|---|---|---|
| **20** | **FEETECH STS3215 (ST-3215-C018), 12 V, 30 kg·cm metal-gear serial-bus servo** | The controlled actuator: ~40 × 20 × 40 mm case, 25T output spline carrying the flush 20 mm disc horn, real END-face 10×10 mm M2.5 hole square for body retention. Smart **serial-bus** servo — built-in 12-bit encoder, daisy-chained on a 1 Mbps half-duplex TTL bus, with closed-loop position/load/voltage/current/temp feedback (no PCA9685, no external encoders). Buy all 20 from the same batch. | [Amazon search: "FEETECH STS3215 30kg serial bus servo"](https://www.amazon.com/s?k=FEETECH+STS3215+30kg+serial+bus+servo) |

**Buy the STS3215 (ST-3215-C018, the 12 V / 30 kg·cm variant), not a
random alternate serial-bus servo.** The model can be regenerated for
another servo, but the current STLs assume the STS3215 case + disc-horn
geometry above. Buy **20** total (18 needed + 2 spares — the weakest
link is gear stripping during tuning). They run directly off the raw 3S
rail; **do not** order PCA9685 PWM driver boards or servo BECs — the old
DS3225 + 2×PCA9685 PWM stack is retired (see PROTOTYPE_BOM.md).

### B.2 Battery / power

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **3S 2200 mAh LiPo, 25C+, XT60 connector** | 11.1 V nominal, ~ 30 min run time. "Zeee", "OVONIC", "CNHL", or "Tattu" are all fine. Velcro-strapped to the top of chassis_bottom in the inter-plate gap. | [Amazon search: "3S 2200mAh 25C lipo XT60"](https://www.amazon.com/s?k=3S+2200mAh+25C+lipo+XT60) |
| 1 | **Velcro hook-and-loop cinch straps (~ 15–20 mm wide)** | Loop through the chassis_bottom velcro-strap slots that straddle the battery footprint to retain the LiPo on the plate top face (replaces the old clip-in `battery_holder`). | [Amazon search: "velcro cinch straps 15mm"](https://www.amazon.com/s?k=velcro+cinch+straps+15mm) |
| 1 | **iSDT D2 / SkyRC B6 / HOTA D6 — any 3S balance charger** | Don't cheap out on charging — this is the fire-risk part of the build. | [Amazon search: "3S lipo balance charger"](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | **LiPo safety bag (medium)** | Charge AND store inside this. $8. | [Amazon search: "lipo safety bag fireproof"](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 2 | **XT60 pigtail (M and F, 12–14 AWG silicone wire)** | One on the battery cable; one to feed the **main fuse → power distribution bus bar** (which then fans out the per-leg branches + the buck) through the anti-spark switch. 12–14 AWG carries the full-robot ~9–13 A walking draw. | [Amazon search: "XT60 pigtail 12awg"](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | **Anti-spark on/off switch with XT60 ends** | Hard cut-off / e-stop so you don't have to unplug the LiPo every time. The "anti-spark" variant has a precharge resistor so you don't pop the switch the first time you connect. | [Amazon search: "rc lipo anti-spark switch xt60"](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |

#### Power distribution (NEW — required for the serial-bus refit)

The 18 servos draw ~9 A walking (up to ~49 A at multi-stall), but the
FEETECH 3-pin Molex 5264 connector pins are rated only **~3 A each**.
You therefore **cannot daisy-chain power through the servo bus** — it
melts the upstream connector. Power is **distributed**: one **main
fuse** → a **bus bar** → **per-leg (3-servo) branches** in heavy
silicone, so no 5264 pin carries more than one leg (~1.5 A walking).
The data chain stays as-is (milliamps). See `firmware/WIRING.md` §6.

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Power distribution bus bar / fused distribution block (12 V DC)** | Single V+ and GND tie point fed from the main fuse; the 6 per-leg branches + the buck tap off it. A small brass/copper bus bar, a fused distribution block, or paralleled ring-terminal posts all work. | [Amazon search: "12v power distribution bus bar"](https://www.amazon.com/s?k=12v+power+distribution+bus+bar) |
| 1 | **Main fuse 15–20 A + inline holder (blade or ANL)** | Sits between the anti-spark switch and the bus bar; protects the whole servo rail against a dead short. Sized above the ~9–13 A walking draw, below a short. | [Amazon search: "inline blade fuse holder 12awg"](https://www.amazon.com/s?k=inline+blade+fuse+holder+12awg) |
| 6 | **(Optional) per-branch fuse 5–7 A + holder** | One per leg branch; catches a sustained single-leg multi-servo stall before it cooks that leg's 16–18 AWG harness. Optional but cheap insurance. | [Amazon search: "5a 7a mini blade fuse holder"](https://www.amazon.com/s?k=5a+7a+mini+blade+fuse+holder) |
| 1 | **16–18 AWG silicone wire (red + black, ~5 m each)** | The six per-leg power branches from the bus bar to each leg's 5264 injection pigtail. Silicone = flexible, high strand count, survives leg motion. | [Amazon search: "16 awg silicone wire red black"](https://www.amazon.com/s?k=16+awg+silicone+wire+red+black) |
| 1 | **Molex 5264 / Mini-SPOX 2.5 mm connector + crimp kit (3-pin, ~20 sets)** | Crimp the per-leg V+/GND injection pigtails that mate to each leg's first servo, and make up the leg-to-leg **signal+GND-only** data jumpers (V+ pin omitted). Match the FEETECH bus connector. A crimper or pre-crimped pigtails save a lot of pain. | [Amazon search: "molex 5264 2.5mm connector kit crimp"](https://www.amazon.com/s?k=molex+5264+2.5mm+connector+kit+crimp) |

### B.3 Control electronics

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Arduino Uno Q** | The whole brain: on-board Linux SoC (Python gait/RL/teleop) + MCU that drives the half-duplex STS3215 TTL bus DIRECTLY — replaces the Raspberry Pi, the Arduino Mega, the 2×PCA9685, AND the separate USB-to-TTL bus adapter. Mounts onto the lower `uno_q_tray` deck via 4 × M3 brass heat-set inserts + 4 × M3 × 8 mm SHCS (rows in §B.4) on the `UNO_Q_HOLES` board pattern. | [Arduino Store: "Uno Q"](https://store.arduino.cc/products/uno-q) |
| 1 | **XINGYHENG 12V→5V buck converter** | Step-down buck converter (3 A+) that drops the 3S rail to 5 V for the Uno Q (the STS3215 run on the raw 3S rail; the buck is for logic only). Same part as prototype_v1. Mounts onto the upper `buck_tray` deck via 4 × M3 brass heat-set inserts + 4 × M3 × 8 mm SHCS on the `BUCK_HOLES` board pattern. | [Amazon search: "XINGYHENG 12V to 5V buck converter"](https://www.amazon.com/s?k=XINGYHENG+12V+to+5V+buck+converter) |
| 1 | **MPU-6050 IMU breakout (GY-521)** | Closed-loop body-attitude control: 3-axis gyro + 3-axis accelerometer over I²C.  PCB is ~ 21.2 × 16.4 × 1.6 mm with 4 × Φ 3.0 mm holes on a 15 × 11 mm pattern; bolts to `imu_pad.stl` via 4 × M3 × 8 SHCS into M3 brass heat-set inserts.  The pad mounts foam-taped to chassis_top centre for vibration isolation (see PROTOTYPE.md §"IMU mount"). | [Amazon search: "MPU-6050 GY-521 module"](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 1 | **FEETECH 3-pin serial-bus cables, assorted lengths (pack)** | The **DATA daisy-chain**: each STS3215 ships with one 3-pin bus cable, but you need a few extra/longer ones for the chassis-to-first-servo run and any leg with a longer reach. These carry V+/GND/Signal within a leg (one leg ≤ ~1.5 A walking — under the 3 A pin rating). **Leg-to-leg, make the data jumper signal+GND only** (omit V+) from the 5264 crimp kit in B.2, so power is never chained between legs (see `firmware/WIRING.md` §6). The old per-servo PWM extension-cable star (and the `wire_harness_plan.py` PCA-reach BOM) is **retired** — the serial bus needs short jumpers, not 18 individual runs back to a driver board. | [Amazon search: "FEETECH serial bus servo cable"](https://www.amazon.com/s?k=FEETECH+serial+bus+servo+cable) |
| 1 | **Nylon zip-ties, 2-3 mm wide x ~ 100 mm long, 100-pack** | 4 per leg = 24 used for harness strain relief: one at each cradle's printed `cable_zip_post` (yaw / hip / knee = 3 per leg) plus one looped through each leg's `chassis_bottom` cable drop slot per leg (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  Plus a few extras for tidying up the inter-plate cable runs.  A standard 100-pack is far more than enough. | [Amazon search: "nylon zip ties 4 inch 100 pack"](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | **Dupont jumper wire kit (M-F, F-F, M-M, 20 cm)** | I²C, power, IMU wiring. | [Amazon search: "dupont jumper wires 120 pcs"](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | **Heat-shrink assortment** | Power-side wiring tidy-up. | [Amazon search: "heat shrink tubing assorted"](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |
| 1 | **USB-C cable, 6 ft** | Flashing / powering / console access to the Arduino Uno Q. | [Amazon search: "USB C cable data"](https://www.amazon.com/s?k=USB+C+cable+data) |

### B.4 Fasteners — get a kit, not individual sizes

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **M3 socket-head cap screw + nut + washer assortment kit (~ 500 pieces, 6 / 8 / 10 / 12 / 16 / 20 mm lengths, A2 stainless)** | Simpler than buying lengths separately. Use 8 mm for servo tabs, 12 mm for chassis-spacer bolts, 16 mm for coxa-bracket → chassis and **for the 6 foot/tibia hinge pins** (May 2026 inversion: pin now passes through the foot_pad's FORK + tibia's TANG instead of the old tibia-clevis + foot-tongue; same M3 × 16 mm pan-head + nylock, just installed in the opposite direction), 20 mm for the rare longer reach. | [Amazon search: "M3 stainless screw kit assortment"](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | **M3 × 16 mm pan-head bolts (foot hinge pins)** | One per leg: passes through the foot_pad's +Y fork cheek (3.5 mm), the tibia tang (6 mm), and the foot_pad's -Y fork cheek (3.5 mm) for 13 mm of plastic + ~ 2.6 mm into an M3 nylock nut on the far side.  May 2026 hinge inversion: previously the FORK was on the tibia and the tongue was on the foot; the geometry is now reversed so the tibia can print as a uniform LINK_THICKNESS-wide slab (no supports), but the hinge axis, bolt, and nut are unchanged.  Pan-head sits flatter against the cheek than a socket head.  The M3 assortment above usually covers this if it has 16 mm + pan-head; otherwise buy this row separately. | [Amazon search: "M3 x 16 pan head stainless"](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | **M3 nylon-insert (nyloc) lock nut, ~ 100 pieces** | **14 used (Jun 2026 deck redesign):** 6 on the foot-pad hinge pins (one per leg) + 4 UNDER chassis_bottom retaining the brass M-F standoffs' male threads at the rotated-45-deg 35-mm-radius pattern (CHASSIS_STANDOFF_HOLES_XY) + 4 UNDER chassis_top retaining the lowest deck standoff columns at `DECK_COLUMN_XY` (±41, ±33).  The servo is bolted by 4× M2.5 into its END-face case holes (plus the strap/clamp cap), so no servo-retention nut is used. | [Amazon search: "M3 nyloc lock nut 100 pack"](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | **M3 brass heat-set inserts — McMaster `94459A130`, 100-pack** | **18 used (Jun 2026 deck + carapace):** 8 in the printed deck trays for the board-mount bosses (4 `uno_q_tray` for the Arduino Uno Q + 4 `buck_tray` for the buck converter) + 4 imu_pad (MPU-6050 mount) + 2 chassis_top (switch_holster mount bosses) + 4 `spider_carapace` feet (dome → standoff mount).  (The 24 sandwich-joint clamp-cap bolts self-tap and use NO inserts; the 72 M2.5 servo body-retention screws thread directly into the servo's own metal end-face case holes, also no inserts.)  A 100-pack gives plenty of spares.  Knurled brass M3, Φ 4.0 mm pilot, Φ 5.7 mm OD, 5.0 mm length, ≈ $0.10 ea.  Installed with a soldering iron at ~ 220 °C, light downward pressure, ~ 10–15 s per insert. | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | **M3 × 32 mm hex round standoffs, M-F brass, set of 20** | Sandwich the chassis plates 32 mm apart with **4** of these on the rotated-45-deg 35-mm-radius pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±35, 0) and (0, ±35)).  Male thread drops DOWN through chassis_bottom's Φ 3.4 mm clearance hole and is captured by an M3 nyloc underneath; female top accepts an M3 × 10 SHCS dropped DOWN from above chassis_top.  Whenever `CHASSIS_GAP` in `hexapod_prototype.py` changes, this standoff length MUST change with it. | [Amazon search: "M3 32mm standoffs male female brass"](https://www.amazon.com/s?k=M3+32mm+standoffs+male+female+brass) |
| 12 | **M3 deck standoff columns, M-F brass (mixed-length assortment)** | The stacked electronics deck rises on the `DECK_COLUMN_XY` (±41, ±33) pattern above chassis_top: **4 at `DECK_LEVEL_1_STANDOFF_H` (16 mm, chassis_top → `uno_q_tray`)** + **4 at `DECK_LEVEL_2_STANDOFF_H` (22 mm, `uno_q_tray` → `buck_tray`)** + **4 at the carapace level (≈ 24 mm, `buck_tray` → `spider_carapace`)**.  Lowest column male threads pass through chassis_top and are retained by M3 nyloc nuts underneath; each upper level bolts down with an M3 × 10 SHCS; the top carapace level threads UP into the dome feet's M3 heat-set inserts.  An M-F standoff assortment kit covers all three lengths. | [Amazon search: "M3 standoff assortment male female brass"](https://www.amazon.com/s?k=M3+standoff+assortment+male+female+brass) |
| 1 | **M3 × 10 mm socket-head cap screws, A2 stainless, ~ 20 pieces** | **14 used (Jun 2026 deck redesign):** 2 switch_holster (DOWN through the holster ear's clearance holes into chassis_top's 2 boss inserts) + 4 chassis_top → brass-standoff bolts (DOWN from above chassis_top into the M-F standoff female top threads on the rotated-45-deg 35-mm-radius pattern) + **8 deck standoff-column bolts (DOWN through each tray's column bosses into the brass standoff female threads — 4 at `uno_q_tray`, 4 at `buck_tray`)**.  Stock from the same M3 kit row above is fine, but listed separately because the M3x10 length isn't always in the 6/8/12/16/20 mm mix. | [Amazon search: "M3 10mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+10mm+SHCS+A2+stainless) |
| 1 | **M3 × 8 mm socket-head cap screws (deck board bolts + IMU pad + clamp caps + yaw cap), ~ 100 pieces** | **8 deck board bolts** are driven DOWN through the Arduino Uno Q (4) + buck converter (4) into the M3 brass heat-set inserts in the deck tray bosses; **4 more** clamp the MPU-6050 PCB onto `imu_pad.stl`'s heat-set inserts; **24 more (Jun 2026 deck redesign) self-tap the 12 `servo_clamp_cap`s** onto the cradle ±X wall-end pilots (2 per hip + knee joint); and **18 more (Jun 2026 split-tower fix) self-tap the 6 `yaw_bearing_cap`s** DOWN onto the chassis_bottom bearing-tower pilots (3 per leg) to capture the spaced 6706 pair.  54 load-bearing + spares -- a 100-pack covers it.  (Servo body retention uses M2.5 into the servo's END-face case holes -- see the M2.5 row below -- not these M3s.) | [Amazon search: "M3 8mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+8mm+SHCS+A2+stainless+100+pack) |
| 1 | **M2.5 × 8 mm SHCS (servo body-retention + horn screws), 100-pack** | **48 used** for POSITIVE servo body retention (4 per **HIP and KNEE** cradle × 2 cradles × 6 legs): each cradle bolts the servo's real **END-face 10 × 10 mm M2.5 hole square** (measured Waveshare ST3215) through the cradle's −X wall, head recessed in a counterbore, threading into the servo's own metal case — this bolts the servo instead of only gripping it.  **The YAW cradle takes NONE** (Jun 2026 flush-horn refit — the yaw servo hangs below the chassis floor and is held by the `yaw_servo_retainer` strap instead), matching `PROTOTYPE_BOM.md` and the auto-generated fastener table below.  The same M2.5 × 8 stock also covers the spline center screws.  A 100-pack gives spares. | [Amazon search: "M2.5 8mm SHCS 100 pack"](https://www.amazon.com/s?k=M2.5+8mm+SHCS+100+pack) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M3x6 SHCS self-tap | 91290A111 | 24 | yaw_servo_retainer L0 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard -Y chassis anchor M3 self-tap |
| M2.5 SHCS into servo case | 91290A104 | 48 | cradle servo body-retention bolts (M2.5 into servo end face) |
| M2.5x8 spline screw | 91290A104 | 30 | hip passive-horn retention screw L0, hip passive-horn retention screw L1, hip passive-horn retention screw L2, hip passive-horn retention screw L3, hip passive-horn retention screw L4, hip passive-horn retention screw L5, knee passive-horn retention screw L0, knee passive-horn retention screw L1, knee passive-horn retention screw L2, knee passive-horn retention screw L3, knee passive-horn retention screw L4, knee passive-horn retention screw L5, servo spline center screws |
| M3x8 disc-horn SHCS | 91290A113 | 24 | link-to-disc-horn bolts |
| M3x8 SHCS into heat-set insert | 91290A113 | 12 | deck board-mount bolts (Uno Q + buck, M3 x 8 SHCS into inserts), imu_pad heat-set inserts (MPU-6050 mount) |
| M3x8 SHCS self-tap | 91290A113 | 42 | sandwich-joint clamp-cap bolts (M3 SHCS self-tap), yaw_bearing_cap join screws (cap -> chassis_bottom tower, M3 x 8 SHCS self-tap) |
| M3x10 SHCS | 91290A114 | 14 | chassis_top brass standoff bolts (M3 x 10 SHCS into standoff female thread), deck standoff column bolts (M3 x 10 SHCS into standoff female thread), switch_holster heat-set inserts |
| M3x10 disc-horn SHCS | 91290A114 | 96 | link-to-disc-horn bolts |
| M3 heat-set insert | 94459A130 | 14 | deck board-mount heat-set inserts (Uno Q + buck), imu_pad heat-set inserts (MPU-6050 mount), switch_holster heat-set inserts |
| M3x16 pan-head | 92010A130 | 6 | foot hinge pins |
| M3 nyloc nut | 90576A102 | 14 | chassis_bottom brass standoff retention nuts, deck standoff column retention nuts (M3 nyloc under chassis_top), foot hinge pins |
| M2.5 self-tap into servo rear case | 96877A150 | 24 | yaw_servo_retainer L0 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2-Y rear case-mount M2.5 self-tap |
|  |  | **348** | **total fasteners** |

Notes:
- The servo OUTPUT face is reserved for the flush 20 mm disc
  horn (no front-face mounting).  POSITIVE body retention bolts
  into the servo's REAL END-face M2.5 holes: each STS3215 +/-X end
  face carries a 10 x 10 mm square of 4 M2.5 case holes (measured
  Waveshare ST3215).  The HIP and KNEE cradles each drive 4 x M2.5
  x 8 SHCS through their -X wall into that square (`91290A104`, 4
  per cradle x 2 cradles x 6 legs = 48; head recessed in a wall
  counterbore, threads into the servo's own metal case).  The YAW
  cradle takes NONE (Jun 2026 flush-horn refit): both yaw bearings
  moving above the flush horn lowered the yaw output 5.5 mm, so the
  servo now hangs ~20 mm below the -6 chassis floor and even its
  upper end-face row clears no -X wall -- the yaw servo is held by
  the `yaw_servo_retainer` strap + anchor bolts + output-face seat
  instead.  These screws positively bolt the servo instead of only
  gripping it; the hip/knee clamp cap is retained as secondary
  capture.
- Link-to-disc-horn bolts (120 total) thread into the 20 mm
  aluminum 25T disc horn's M3 TAPPED holes on a 14 mm bolt
  circle (cross pattern at 0/90/180/270 deg); the aluminum is
  the thread-engagement medium -- no self-tap, no heat-set.
  Two lengths (Jun 2026 flush-output refit): the DRIVEN hip +
  knee front-horn bolts are **48 x M3x10 SHCS (`91290A114`)**
  -- 2 mm longer because the flush output seats the driven
  disc horn 2 mm lower, so the bolt traverses an extra pad
  before the disc -- while the yaw front horn + the two
  passive rear-boss horns stay **72 x M3x8 SHCS (`91290A113`)**.
  (June 2026 disc-horn switch, retiring the now-retired plastic
  4-arm X-horn's M2x8 self-tap scheme.)  See
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

### B.5 Filament (skip if you have any in the workshop)

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 kg | **PLA, 1.75 mm, any colour** | Whole structural BOM. | [Amazon search: "PLA filament 1.75mm 1kg"](https://www.amazon.com/s?k=PLA+filament+1.75mm+1kg) |
| 0.25 kg | **TPU 95A, 1.75 mm, black** | Foot pads only. Sample roll is enough. | [Amazon search: "TPU 95A filament 1.75mm"](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |

### B.6 Nice-to-haves

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Soldering iron + flux + 60/40 solder** | Required to install the M3 brass heat-set inserts in the deck trays / imu_pad / switch_holster bosses (and any header soldering). | [Amazon search: "soldering iron kit beginner"](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | **Hex (Allen) key set, 1.5 / 2 / 2.5 / 3 / 4 mm** | M3 cap screws need 2.5 mm, M2.5 needs 2 mm. | [Amazon search: "hex key set metric ball end"](https://www.amazon.com/s?k=hex+key+set+metric+ball+end) |
| 1 | **Cyanoacrylate (super-glue) 20 g** | Glue the rubber sleeve into the foot pad cup. | [Amazon search: "super glue gel"](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | **Inner-tube section** (or skateboard-tape strip) | Foot tread. Cut a circle from a bicycle inner tube and CA-glue inside `foot_pad.stl`'s cup. | n/a — old bicycle tube |
| 1 | **3M VHB / generic 3 mm double-sided mounting foam tape, ~ 30 cm strip** | Bonds `imu_pad.stl` to the chassis_top centre AND acts as the vibration damper that decouples the MPU-6050 from the servo-driven chassis frame.  Cut a ~ 25 × 20 mm rectangle from the strip and apply to the pad's flat underside; one 30 cm strip yields ~ 100 IMU pads.  3M VHB 5952 / 4910 is the recommended grade, but any 3 mm closed-cell foam tape works for v1. | [Amazon search: "3M VHB double-sided foam tape 3mm"](https://www.amazon.com/s?k=3M+VHB+double-sided+foam+tape+3mm) |

---

## C. Cost summary

| Bucket | Cost (USD, mid-2026) |
|---|---:|
| 20 × STS3215 serial-bus servos | $360–$500 |
| Battery + charger + bag + XT60/velcro | $70 |
| Power distribution (bus bar + 15–20 A main fuse + 6 branch fuses + 16–18 AWG silicone + 5264 crimp kit) | $25–$45 |
| Arduino Uno Q + XINGYHENG buck converter + IMU + serial-bus cables + jumpers | $90 |
| Fasteners (M3 kit + nylocs + chassis standoffs + 8 deck columns) | $30 |
| Filament (PLA 1 kg + TPU 250 g) + 12 × MJF servo_clamp_cap | $40 |
| Soldering iron / hex keys / glue (if you don't have them) | $30 |
| **Total** | **~ $645–$805** |

The cost assumes STS3215 serial-bus servos. Cheaper servos are
intentionally not listed here because they change the risk profile and
may not fit the printed wells without regenerating the STLs.

---

## D. Print queue (suggested order)

Print **legs first**, body **last** — that way you can dry-fit each
leg on the servo before committing the chassis plates. Filenames here
mirror §A / `PROTOTYPE_BOM.md` (the Jun 2026 sandwich-joint set; the
old `coxa_bracket` / `femur_link` / `tibia_link` single-spar names are
retired).

1. **1 full leg set first** — `coxa_link` + `femur_hip_yoke` +
   `femur_knee_bracket` + `tibia_knee_yoke` + `tibia_foot_fitting` +
   `foot_pad` (+ a `passive_horn_adapter`) — to validate the symmetric
   bracket + dual-disc-horn fit before committing six sets.
2. **6 × `coxa_link.stl`** + **6 × `yaw_bearing_cap.stl`** — ~4 h.
3. **6 × `femur_hip_yoke.stl`** + **6 × `femur_knee_bracket.stl`** — ~8 h.
4. **6 × `tibia_knee_yoke.stl`** + **6 × `tibia_foot_fitting.stl`** — ~6 h.
5. **12 × `passive_horn_adapter.stl`** + **12 × `servo_clamp_cap.stl`**
   (MJF PA12 or PLA) — ~5 h.
6. **`chassis_top.stl` + `chassis_bottom.stl` + `uno_q_tray.stl` +
   `buck_tray.stl` + `switch_holster.stl` + `imu_pad.stl` +
   `spider_carapace.stl`** — ~12 h (small parts share a bed; each
   chassis plate + the carapace get their own bed).
7. **6 × `foot_pad.stl`** in TPU — ~1 h.

After step 2 you can start mounting the yaw servos into the
`chassis_bottom` cradles + split bearing towers and verifying the bolt
patterns line up — that gives you a parallel track between "printing
the rest" and "starting final assembly".

---

## E. Troubleshooting tips during assembly

| Symptom | Likely cause | Fix |
|---|---|---|
| Servo body won't drop into well | Slight FDM over-extrusion narrowing the cavity | Sand inside walls or scale `WELL_BODY_CL` from 0.4 → 0.6 mm and reprint |
| M3 SHCS bottoms out in the heat-set insert | Insert pressed in crooked or the pilot is FDM-narrowed | Heat the insert again with a soldering iron, push it straight down into the Φ 4 mm pocket while applying light axial pressure; if the printed pilot is too tight, drill it to 4.1 mm with a hand drill before re-installing |
| Heat-set insert sinks below the boss top | Soldering iron too hot or pressed too long | Set the iron to ≈ 220 °C and dwell ~ 10–15 s only; the insert should stop when its top face is flush with the printed boss minus ~ 0.5 mm so the bolt head clamps the ear onto the plastic, not the brass |
| Coxa bracket flange wobbles on chassis | Chassis plate top face has a 0.1 mm fdm bow | Add an M3 washer under each chassis bolt nut (already have them in the M3 kit) |
| Femur slot too tight to slide servo through | Slot designed with 1 mm clearance per side; FDM commonly eats 0.4 mm of that | File the slot edges flat with a needle file (5 minutes per leg) |
| Foot pad slips on hardwood | FDM in PLA, not TPU | Either reprint in TPU 95A, OR cut a circle from a bicycle inner tube and CA-glue it into the foot cup |

---

## F. Verification before you order or print

The script `hexapod_walker/prototype/_verify_prototype.py` re-checks the
geometry and tells you in 6 seconds whether anything regressed since
the last edit. Run it whenever you tweak `hexapod_prototype.py`:

```bash
./run.sh hexapod_walker/prototype/_verify_prototype.py
```

Last clean run (this checkout) — **all four checks PASS**:

```
[1] Mesh watertightness / manifoldness:    10/10 PASS
[2] Cradle insertion-path openness:         3/3  PASS  (0/625 samples blocked)
[3] Bolt-hole material engagement:          5/5  PASS
[4] Self-collision in standing pose:        6/6  PASS
```

You're clear to order parts and start the printer.

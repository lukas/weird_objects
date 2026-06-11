# Hexapod prototype — shopping list & print queue

**Status: design verification PASSED** (manifoldness, cradle openness,
bolt-hole engagement, self-collision — all four checks clean against
`_verify_prototype.py`).

This is the everything-you-need-to-buy-and-print sheet. Numbers are
sized for **one complete walking robot** with a small spare margin
(~ 10% on fasteners, +2 servos, +1 BEC).

---

## A. STL files to print

All files live under `hexapod_walker/prototype/stl_prototype/`. Filenames are
exactly as the generator writes them.

| # | Filename | Qty | Material | Layer | Infill | Walls | Print time (Ender 3) | Notes |
|---|---|---:|---|---|---:|---:|---:|---|
| 1 | `chassis_top.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | Identical to bottom |
| 2 | `chassis_bottom.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | Identical to top |
| 3 | `battery_holder.stl` | **1** | PLA / PETG | 0.2 mm | 20% gyroid | 3 | ~ 1.5 h | LiPo tray |
| 4 | `electronics_tray.stl` | **1** | PLA / PETG | 0.2 mm | 20% gyroid | 2 | ~ 2 h | 160 x 130 mm deck for Arduino Mega 2560 + Raspberry Pi 4/5 + 2 x PCA9685 (May 2026 expansion + "essentials" 2nd-PCA bump) |
| 4a | `bec_cradle.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.3 h | Snap-fit clip for 2 x 5V 5A switching BECs.  Sits on the electronics_tray. No fasteners (friction fit). |
| 4b | `switch_holster.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.3 h | Snap-in holster for anti-spark on/off switch.  Bolts to chassis_top's +X edge via 2 x M3 x 10 SHCS into 2 chassis_top heat-set inserts. |
| 4c | `imu_pad.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.2 h | 25 x 20 mm IMU mounting pad with 4 M3 heat-set insert bosses on the GY-521 15 x 11 mm hole pattern.  No fasteners between pad and chassis_top -- the pad is foam-taped to the chassis_top centre for vibration isolation. |
| 5 | `coxa_bracket.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Flange-down on bed; the well opens UP. The flange bolts to the chassis. |
| 6 | `coxa_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Hub face down on bed; the well opens UP. |
| 7 | `femur_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 5 h total | Spar's broad face flat on the bed (hip-pad flat).  Knee cradle opens through the print -- the cradle floor was REMOVED in the May 2026 supports-free refactor (cf. PROTOTYPE.md §3.2), so the part is now a 4-wall pen open at both Y faces and there is no ~30 × 40 mm bridged ceiling.  **NO SUPPORTS NEEDED** -- every overhang is < 45° in this orientation. |
| 8 | `tibia_link.stl` | **6** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 4 h total | Spar's broad face flat on the bed.  May 2026 hinge inversion: the tibia is now `LINK_THICKNESS` = 6 mm wide in Y EVERYWHERE -- the foot end terminates in a single 6 mm tang (in-plane with the spar) instead of the old 12 mm-wide clevis (which used to stick 6 mm above the spar's broad face and required supports under the cheeks).  **NO SUPPORTS NEEDED** -- the part prints as a flat 6 mm-tall slab. |
| 9 | `foot_pad.stl` | **6** | TPU 95A (PLA OK) | 0.25 mm | 100% (TPU) | 3 | ~ 1 h | TPU = grip; PLA = slips.  May 2026 hinge inversion: foot now carries the 2-cheek FORK (3.5 mm cheeks + 6.4 mm slot) instead of a single tongue.  Print disk-on-bed, fork pointing UP -- the cheeks are 3.5 mm vertical TPU walls with no overhangs; the 6.4 mm slot between them is open air (no bridge).  **NO SUPPORTS NEEDED**. |

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
> threading into the disc's M3 tapped holes.  You need 18 discs (3
> joints x 6 legs); the part ships in 10-packs, so order **2 packs**
> (2 spares + the M3 x 6 screws that come with them).

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
| **20** | **DS3225 25 kg·cm digital servo, metal gear, standard-size case** | This is the actuator the printed wells were designed around: ~40 × 20 × 38 mm body, ~54 mm tab span, ~49.5 mm tab-hole spacing, output shaft offset ~10 mm from body centre. Buy one brand/listing and stick with it for all 20. Pack of 4 is usually $52 – $60; five 4-packs = 20 servos with 2 spares. | [Amazon search: "DS3225 servo 25kg metal gear"](https://www.amazon.com/s?k=DS3225+servo+25kg+metal+gear) |

**Buy the DS3225, not a random alternate 25 kg servo.** The model can be
regenerated for another servo, but the current STLs assume the DS3225
geometry above. Buy **20** total (18 needed + 2 spares — the weakest
link is gear stripping during tuning).

### B.2 Battery / power

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **3S 2200 mAh LiPo, 25C+, XT60 connector** | 11.1 V nominal, ~ 30 min run time. "Zeee", "OVONIC", "CNHL", or "Tattu" are all fine. | [Amazon search: "3S 2200mAh 25C lipo XT60"](https://www.amazon.com/s?k=3S+2200mAh+25C+lipo+XT60) |
| 2 | **5 V 5 A switching BEC, 2S–4S input** | Two BECs split the 18-servo current draw; "Hobbywing 5A UBEC", "Castle Creations 10A SBEC", "Skyrc UBEC" all work. Order 2 (one per PCA9685) — single-BEC will brown out during tripod swing. | [Amazon search: "5V 5A UBEC switching"](https://www.amazon.com/s?k=5V+5A+UBEC+switching) |
| 1 | **iSDT D2 / SkyRC B6 / HOTA D6 — any 3S balance charger** | Don't cheap out on charging — this is the fire-risk part of the build. | [Amazon search: "3S lipo balance charger"](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | **LiPo safety bag (medium)** | Charge AND store inside this. $8. | [Amazon search: "lipo safety bag fireproof"](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 2 | **XT60 pigtail (M and F, with silicone wire)** | One on the battery cable, one to feed the BEC pair through a switch. | [Amazon search: "XT60 pigtail 12awg"](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | **Anti-spark on/off switch with XT60 ends** | Hard cut-off so you don't have to unplug the LiPo every time. The "anti-spark" variant has a precharge resistor so you don't pop the switch the first time you connect. | [Amazon search: "rc lipo anti-spark switch xt60"](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |

### B.3 Control electronics

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Arduino Mega 2560 (ELEGOO R3 clone is fine)** | Servo-bridge firmware host.  Need 18 PWM channels + I²C bus. Mega has both. The Uno does *not* have enough hardware PWM, but you don't need hardware PWM because the PCA9685 is generating the PWM. Mounts onto the `electronics_tray` via 4 × M3 brass heat-set inserts + 4 × M3 × 8 mm SHCS (next two rows in §B.4); bolt pattern is the published Arduino Mega 2560 R3 footprint (2.54/15.24, 50.8/15.24, 7.62/66.04, 50.8/90.17 mm). | [Amazon search: "Arduino Mega 2560 R3"](https://www.amazon.com/s?k=Arduino+Mega+2560+R3) |
| 1 | **Raspberry Pi 4 Model B (or Pi 5)** | High-level brain: ROS 2 / Python gait planner + vision + Wi-Fi.  85 x 56 mm board mounts onto the `electronics_tray` via 4 × M2.5 brass heat-set inserts + 4 × M2.5 × 8 mm SHCS (rows in §B.4); the Pi's standard 49 x 58 mm 4-hole pattern fits the tray's `PI_HOLES` bosses 1:1.  Pi 5 has the same mounting footprint; choose either. | [Amazon search: "Raspberry Pi 4 Model B 4GB"](https://www.amazon.com/s?k=Raspberry+Pi+4+Model+B+4GB) |
| 2 | **PCA9685 16-channel 12-bit PWM driver (I²C)** | Two boards, daisy-chained, give you 32 PWM lines. Adafruit-clone listings are typically $4–6 each. **Both** PCA9685s bolt to the `electronics_tray` via 4 × M3 heat-set inserts + 4 × M3 × 8 mm SHCS each (8 of each total — see §B.4); the secondary daisy-chains over I²C at address 0x41 (jumper). | [Amazon search: "PCA9685 16 channel servo driver"](https://www.amazon.com/s?k=PCA9685+16+channel+servo+driver) |
| 1 | **MPU-6050 IMU breakout (GY-521)** | Closed-loop body-attitude control: 3-axis gyro + 3-axis accelerometer over I²C.  PCB is ~ 21.2 × 16.4 × 1.6 mm with 4 × Φ 3.0 mm holes on a 15 × 11 mm pattern; bolts to `imu_pad.stl` via 4 × M3 × 8 SHCS into M3 brass heat-set inserts.  The pad mounts foam-taped to chassis_top centre (NOT to the electronics_tray — see PROTOTYPE.md §"IMU mount"). | [Amazon search: "MPU-6050 GY-521 module"](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 1 | **Servo extension cables, 30 cm, 3-pin male-female, pack of 20** | The DS3225 stock pigtails are ~ 30 cm and most of the 18 joints reach the PCA on stock pigtail alone; the remainder need ONE 30 cm extension to reach the +X-half PCAs.  The BOM auto-generated by `python -m hexapod_walker.prototype.pi_control.wire_harness_plan` is the source of truth.  As of the May 2026 chassis_bottom-integrated yaw cradle redesign the per-leg yaw run shortened by ~ 62 mm of Manhattan distance (the wire-exit corridor moved from the cradle's +X face to its -X face, putting the harness mouth right next to the leg drop slot), so a few of the previously-required yaw extensions are no longer needed -- re-derive the count from the harness plan output before ordering.  The 20-pack is the cheapest stocking unit and leaves headroom either way. | [Amazon search: "servo extension cable 30cm 20 pack"](https://www.amazon.com/s?k=servo+extension+cable+30cm+20+pack) |
| 1 | **Nylon zip-ties, 2-3 mm wide x ~ 100 mm long, 100-pack** | 4 per leg = 24 used for harness strain relief: one at each cradle's printed `cable_zip_post` (yaw / hip / knee = 3 per leg) plus one looped through each leg's `chassis_bottom` cable drop slot per leg (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  Plus a few extras for tidying up the inter-plate cable runs.  A standard 100-pack is far more than enough. | [Amazon search: "nylon zip ties 4 inch 100 pack"](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | **Dupont jumper wire kit (M-F, F-F, M-M, 20 cm)** | I²C, power, IMU wiring. | [Amazon search: "dupont jumper wires 120 pcs"](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | **Heat-shrink assortment** | Power-side wiring tidy-up. | [Amazon search: "heat shrink tubing assorted"](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |
| 1 | **USB-A → USB-B cable, 6 ft** | Programming the Mega. | [Amazon search: "USB A to B cable Arduino"](https://www.amazon.com/s?k=USB+A+to+B+cable+Arduino) |

### B.4 Fasteners — get a kit, not individual sizes

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **M3 socket-head cap screw + nut + washer assortment kit (~ 500 pieces, 6 / 8 / 10 / 12 / 16 / 20 mm lengths, A2 stainless)** | Simpler than buying lengths separately. Use 8 mm for servo tabs, 12 mm for chassis-spacer bolts, 16 mm for coxa-bracket → chassis and **for the 6 foot/tibia hinge pins** (May 2026 inversion: pin now passes through the foot_pad's FORK + tibia's TANG instead of the old tibia-clevis + foot-tongue; same M3 × 16 mm pan-head + nylock, just installed in the opposite direction), 20 mm for the rare longer reach. | [Amazon search: "M3 stainless screw kit assortment"](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | **M3 × 16 mm pan-head bolts (foot hinge pins)** | One per leg: passes through the foot_pad's +Y fork cheek (3.5 mm), the tibia tang (6 mm), and the foot_pad's -Y fork cheek (3.5 mm) for 13 mm of plastic + ~ 2.6 mm into an M3 nylock nut on the far side.  May 2026 hinge inversion: previously the FORK was on the tibia and the tongue was on the foot; the geometry is now reversed so the tibia can print as a uniform LINK_THICKNESS-wide slab (no supports), but the hinge axis, bolt, and nut are unchanged.  Pan-head sits flatter against the cheek than a socket head.  The M3 assortment above usually covers this if it has 16 mm + pan-head; otherwise buy this row separately. | [Amazon search: "M3 x 16 pan head stainless"](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | **M3 nylon-insert (nyloc) lock nut, ~ 100 pieces** | **10 used** as of the May 2026 tray-mount fix: 6 on the foot-pad hinge pins (one per leg) + 4 UNDER chassis_bottom retaining the brass M-F standoffs' male threads at the rotated-45-deg 35-mm-radius pattern (CHASSIS_STANDOFF_HOLES_XY).  The cradle servo mounts thread into brass heat-set inserts (next row) and do **not** use a nut.  Earlier docs claimed 24 + 6 on a since-retired coxa-bracket flange; that count is obsolete. | [Amazon search: "M3 nyloc lock nut 100 pack"](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | **M3 brass heat-set inserts — McMaster `94459A130`, 100-pack** | **Design E (May 2026, mixed-mode cradle bolts) + tray-mount fix:** the 4 cradle bolts per cradle are split by X sign -- only the 2 -X bolts use a heat-set insert (2 -X × 3 cradles × 6 legs = **36** cradle inserts), and the 2 +X bolts self-tap into a bare Φ 2.5 mm pilot.  Plus 4 battery_holder feet + 8 electronics_tray (4 Mega + 4 primary PCA9685) + 4 electronics_tray (4 secondary PCA9685) + 2 chassis_top (switch_holster mount bosses) + 4 imu_pad (MPU-6050 mount) + **4 chassis_bottom tray-mount bosses (May 2026 tray-mount fix)** = **62** needed.  A 100-pack still gives healthy spares.  Knurled brass M3, Φ 4.0 mm pilot, Φ 5.7 mm OD, 5.0 mm length, ≈ $0.10 ea.  Installed with a soldering iron at ~ 220 °C, light downward pressure, ~ 10–15 s per insert. | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | **M2.5 brass heat-set inserts — McMaster `94459A106`, 50-pack** | 4 needed for the Raspberry Pi 4 / Pi 5 mount on the electronics_tray (the Pi's holes are M2.5 clearance, smaller than the Mega's M3 clearance, so a smaller insert is required).  Knurled brass M2.5, Φ 3.0 mm pilot, Φ 3.6 mm OD, 4.0 mm length.  Installed with the same soldering-iron technique as the M3 inserts; the printed Phi 6 mm boss around each pilot leaves a 1.5 mm plastic wall, which is enough for thermal install without slumping. | [McMaster 94459A106](https://www.mcmaster.com/94459A106/) |
| 1 | **M3 × 32 mm hex round standoffs, M-F brass, set of 20** | Sandwich the chassis plates 32 mm apart with **4** of these on the rotated-45-deg 35-mm-radius pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±35, 0) and (0, ±35)). **May 2026 tray-mount fix:** moved off the (±24.75, ±24.75) tray-mount pattern onto the rotated pattern so the standoff body no longer conflicts with the new heat-set inserts in chassis_bottom's tray-mount bosses.  Earlier May 2026 audit also bumped the length from 25 mm to 32 mm so the 28 mm-tall battery_holder fits between the plates with 4 mm headroom -- whenever `CHASSIS_GAP` in `hexapod_prototype.py` changes, this standoff length MUST change with it. | [Amazon search: "M3 32mm standoffs male female brass"](https://www.amazon.com/s?k=M3+32mm+standoffs+male+female+brass) |
| 1 | **M3 × 10 mm socket-head cap screws, A2 stainless, ~ 20 pieces** | **10 used:** 4 battery-holder feet (UP through chassis_bottom into the holder's 4 brass inserts) + 2 switch_holster (DOWN through the holster ear's clearance holes into chassis_top's 2 boss inserts) + 4 chassis_top → brass-standoff bolts (DOWN from above chassis_top into the M-F standoff female top threads on the rotated-45-deg 35-mm-radius pattern, May 2026 tray-mount fix).  Stock from the same M3 kit row above is fine, but listed separately because the M3x10 length isn't always in the 6/8/12/16/20 mm mix. | [Amazon search: "M3 10mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+10mm+SHCS+A2+stainless) |
| 1 | **M3 × 8 mm socket-head cap screws (cradle bolts + electronics_tray board bolts + IMU pad + tray-mount), ~ 100 pieces** | **72 cradle bolts** (4 per cradle × 3 cradles per leg × 6 legs) use this stock under the Design E mixed-mode pattern: 36 of them (the -X bolts) thread into M3 brass heat-set inserts, and 36 of them (the +X bolts) self-tap into a Φ 2.5 mm pilot in plastic.  An additional 12 are driven DOWN through the Mega 2560 (4) + primary PCA9685 (4) + secondary PCA9685 (4) into the M3 brass heat-set inserts in the electronics_tray bosses; 4 more clamp the MPU-6050 PCB onto `imu_pad.stl`'s heat-set inserts; and **4 more (May 2026 tray-mount fix) thread DOWN through the electronics_tray's chassis-mount cbore floor into M3 heat-set inserts in chassis_bottom's tray-mount bosses**.  92 load-bearing + spares -- a 100-pack is the right size. | [Amazon search: "M3 8mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+8mm+SHCS+A2+stainless+100+pack) |
| 1 | **M2.5 × 8 mm screws (servo horn screws + Raspberry Pi mount), 50-pack** | Comes free with the servos as self-tappers, but a 50-pack of M2.5 × 8 + M2.5 nuts is $5 and saves a trip if you strip one.  May 2026 update: 4 of these are now also load-bearing on the electronics_tray as the Raspberry Pi 4 / Pi 5 board-mount bolts (threading into the M2.5 brass heat-set inserts above). | [Amazon search: "M2.5 8mm screw 50 pack"](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M3x6 SHCS | 91290A111 | 72 | link-to-disc-horn bolts |
| M2.5x8 spline screw | 91290A104 | 18 | servo spline center screws |
| M2.5x8 SHCS into heat-set insert | 91290A102 | 4 | electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685) |
| M2.5 heat-set insert | 94459A106 | 4 | electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685) |
| M3x8 SHCS into heat-set insert | 91290A113 | 56 | cradle servo mounts (M3 SHCS into heat-set insert), electronics_tray chassis-mount bolts (M3 x 8 SHCS into chassis_bottom heat-set insert), electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685), imu_pad heat-set inserts (MPU-6050 mount) |
| M3x8 SHCS self-tap | 91290A113 | 36 | cradle servo mounts (M3 SHCS self-tap) |
| M3x10 SHCS | 91290A114 | 10 | battery_holder heat-set inserts, chassis_top brass standoff bolts (M3 x 10 SHCS into standoff female thread), switch_holster heat-set inserts |
| M3 heat-set insert | 94459A130 | 62 | battery_holder heat-set inserts, chassis_bottom tray-mount heat-set inserts, cradle heat-set inserts, electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685), imu_pad heat-set inserts (MPU-6050 mount), switch_holster heat-set inserts |
| M3x16 pan-head | 92010A130 | 6 | foot hinge pins |
| M3 nyloc nut | 90576A102 | 10 | chassis_bottom brass standoff retention nuts, foot hinge pins |
|  |  | **278** | **total fasteners** |

Notes:
- Cradle servo mounts (72 x `M3x8 SHCS into heat-set insert` /
  `91290A113`) are driven VERTICALLY from above each servo ear and
  thread into an M3 brass heat-set insert (`94459A130`) installed
  flush with the boss top.  May 2026 fix: the previous self-tap
  pilots grazed the cradle wall material at 7 of 12 sites (audit:
  0.00-1.50 mm of plastic radially); the heat-set switch forces
  Phi 8 mm bosses around every pilot and gives real metal threads.
- Heat-set inserts (72 x `94459A130`) are installed BEFORE the
  servo cradle is mated to its neighbour: heat the insert with a
  soldering iron at ~220 deg C, drop it into the printed Phi 4 mm
  x 6 mm pocket, apply light downward pressure for ~10-15 s until
  the knurl displaces plastic into the boss wall, then cool ~30 s
  before threading the M3 x 8 SHCS in.
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
- The M2.5 spline center screw ships with each DS3225-class servo --
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
| 1 | **Soldering iron + flux + 60/40 solder** | If you don't have one. PCA9685 boards usually need their headers soldered in. | [Amazon search: "soldering iron kit beginner"](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | **Hex (Allen) key set, 1.5 / 2 / 2.5 / 3 / 4 mm** | M3 cap screws need 2.5 mm, M2.5 needs 2 mm. | [Amazon search: "hex key set metric ball end"](https://www.amazon.com/s?k=hex+key+set+metric+ball+end) |
| 1 | **Cyanoacrylate (super-glue) 20 g** | Glue the rubber sleeve into the foot pad cup. | [Amazon search: "super glue gel"](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | **Inner-tube section** (or skateboard-tape strip) | Foot tread. Cut a circle from a bicycle inner tube and CA-glue inside `foot_pad.stl`'s cup. | n/a — old bicycle tube |
| 1 | **3M VHB / generic 3 mm double-sided mounting foam tape, ~ 30 cm strip** | Bonds `imu_pad.stl` to the chassis_top centre AND acts as the vibration damper that decouples the MPU-6050 from the servo-driven chassis frame.  Cut a ~ 25 × 20 mm rectangle from the strip and apply to the pad's flat underside; one 30 cm strip yields ~ 100 IMU pads.  3M VHB 5952 / 4910 is the recommended grade, but any 3 mm closed-cell foam tape works for v1. | [Amazon search: "3M VHB double-sided foam tape 3mm"](https://www.amazon.com/s?k=3M+VHB+double-sided+foam+tape+3mm) |

---

## C. Cost summary

| Bucket | Cost (USD, mid-2026) |
|---|---:|
| 20 × DS3225 servos | $260 |
| Battery + 2 × BEC + charger + bag + cables | $80 |
| Arduino Mega + Raspberry Pi 4 + 2 × PCA9685 + IMU + servo cables + jumpers | $100 |
| Fasteners (M3 kit + nylocs + standoffs) | $25 |
| Filament (PLA 1 kg + TPU 250 g) | $25 |
| Soldering iron / hex keys / glue (if you don't have them) | $30 |
| **Total** | **~ $470** |

The cost assumes DS3225 servos. Cheaper servos are intentionally not
listed here because they change the risk profile and may not fit the
printed wells without regenerating the STLs.

---

## D. Print queue (suggested order)

Print **legs first**, body **last** — that way you can dry-fit each
leg on the servo before committing the chassis plates.

1. **6 × `coxa_bracket.stl`** — 4 hours total.
2. **6 × `coxa_link.stl`** — 4 hours.
3. **6 × `femur_link.stl`** — 5 hours. ⚠ Watch the first one come
   off the bed and **dry-fit the knee servo body** (no horn yet) —
   it should slide straight in through the slot from the +Y side
   with finger pressure. If it binds, sand the slot lightly.
4. **6 × `tibia_link.stl`** — 4 hours.
5. **`chassis_top.stl` + `chassis_bottom.stl` + `battery_holder.stl` + `electronics_tray.stl`** — 6 hours (single bed for the small parts, separate bed for each chassis plate).
6. **6 × `foot_pad.stl`** in TPU — 1 hour.

After step 3 you can start mounting the yaw servos in the coxa
brackets and verifying the bolt patterns line up — that gives you a
72-hour parallel-track between "printing the rest" and "starting
final assembly".

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

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
| 1 | `chassis_top.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | Top hex; 4× `CHASSIS_STANDOFF_HOLES_XY` (±31.1) take sandwich standoffs below + magnet posts above |
| 2 | `chassis_bottom.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 2 h | LiPo velcro under chassis; data Wagos near yaw retainers |
| 3 | `switch_holster.stl` | **1** | PLA / PETG | 0.2 mm | 25% gyroid | 3 | ~ 0.3 h | Snap-in holster for anti-spark on/off switch.  Bolts to chassis_top's +X edge via 2 x M3 x 10 SHCS into 2 chassis_top heat-set inserts. |
| 3b | `extra_stl/hex_mount_plate_110_with_leg_holes.stl` (+ `.svg`) | **1** | PLA / PETG or laser-cut | 0.2 mm | 20% gyroid | 3 | ~ 0.5 h | Ø110 magnet-held board for Uno Q + breakout. Generate with `tools/make_xtool_hex_raised_platform.py`. |
| 3c | `extra_stl/hex_raised_platform_110_h72_screen.stl` | **1** | PLA / PETG | 0.2 mm | 15% gyroid | 3 | ~ 2 h | Raised platform on hex plate: screen on top face, MPU under top plate (72 mm legs). |
| 4 | `servo_clamp_cap.stl` | **12** | MJF PA12 (PLA OK) | 0.2 mm | solid | 3 | ~ 0.2 h ea | Sandwich-joint clamp cap — 1 per hip + knee joint (2 per leg x 6 legs). Bolts to the cradle ±X wall ends with 2 x M3 x 8 SHCS self-tapping into the cradle pilots. |
| 5 | `coxa_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | ONE piece (Aug 2026 merge): yaw turntable hub (rides the spaced 6706 pair) + hip fixed side (symmetric servo cradle — the passive disc horn rides the servo's own rear idler boss, NO 688 bearing). 5 head-access shafts reach the yaw horn screws through the empty hip well. Print on its SIDE (yaw axis horizontal, cradle end wall on the bed). |
| 5b | `yaw_bearing_cap.stl` | **6** | PLA / PETG | 0.2 mm | 40% gyroid | 4 | ~ 0.3 h ea | TOP half of the SPLIT yaw-bearing tower (Jun 2026 insertion fix). Bolts onto each `chassis_bottom` tower with 3 x M3 x 8 SHCS self-tapping into the tower pilots (3 outboard ear lugs); a clean Ø37 through-bore holds the upper 6706 race. Print **flat, ring face down** on the bed (no supports). One per leg. |
| 6 | `femur_link.stl` | **6** | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~ 8 h total | The WHOLE femur, ONE printed part (Jul 2026 merge): hip moving yoke (SYMMETRIC clevis — driven horn on the front, passive horn on the rear boss) + SOLID Ø14 spar + knee fixed side (symmetric servo cradle, no 688 bearing). No CF tube, no socket, no roll pin. Print yoke spine-down / spar horizontal; support the knee servo well through its open back. |
| 8 | `tibia_knee_yoke.stl` | **6** | PLA / PETG | 0.2 mm | 35% gyroid | 4 | ~ 4 h total | Knee moving yoke — SYMMETRIC clevis: both arms bolt to a disc horn (driven + passive). + Ø8 CF-tube socket. |
| 8b | `tibia_foot_fitting.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 1.5 h total | Ø8 CF-tube socket + single foot-hinge tang (M3 pin). |
| 9 | `foot_pad.stl` | **6** | TPU 95A (PLA OK) | 0.25 mm | 100% (TPU) | 3 | ~ 1 h | TPU = grip; PLA = slips.  Foot carries the 2-cheek FORK (3.5 mm cheeks + 6.4 mm slot) straddling the tibia tang.  Print disk-on-bed, fork pointing UP. |
| 9b | `yaw_servo_retainer.stl` | **6** | PLA / PETG | 0.2 mm | 20% gyroid | 3 | ~ 0.4 h ea | Anti-rotation saddle under each yaw servo (Aug 2026 flat-belly rework: the 38 mm ground stand is REMOVED — belly is flat except the hanging servos + saddles; use an external bench support). Central wire drop window. |

> **Leg segments.** The TIBIA is two printed end-fittings bonded onto an
> Ø8 mm CF tube (epoxy + a transverse Ø2.5 mm roll pin): cut the tube to
> length, epoxy into the sockets, and drive the retention pin through the
> cross-hole.  The FEMUR needs NO assembly at all (Jul 2026 one-piece
> merge): `femur_link` prints as a single finished body — no tube, no
> pins, no epoxy. See the
> test-fit part in `tools/sts3215_testfit.py` before committing six legs.

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
> **June 2026 symmetric-yoke refit + July 2026 stock-horn refit:** the
> hip and knee joints also carry a SECOND horn on the servo's REAR
> idler boss (passive support), so the yoke bolts to a horn on BOTH
> faces instead of running a 688 ball bearing on the passive side.
> The passive horn is the STS3215's own **stock metal rear horn**
> (ships with the servo; same 4-hole pattern): its centre bore slides
> over the rear idler boss so it seats flush on the back face,
> retained by one M2.5 screw.  No printed adapter and no extra
> purchased discs — you only buy the **18 driven** discs (yaw 1 +
> hip 1 + knee 1 = 3 per leg × 6 legs; 10-packs, order **2 packs**).

**Total print time (single Ender 3 / Bambu A1):** ~ 16 hours of
machine time, spread across 6 – 7 print sessions (Aug 2026: trays /
carapace / imu_pad retired).

> **Don't have a printer?** Run
> `scripts/prepare_xometry_upload.py` to generate an order bundle in
> `xometry_upload/` (re-oriented for MJF, with a `manifest.csv` and a
> README that takes you through the Xometry / Shapeways / JLCPCB
> upload flow; not checked in — generated on demand). Total there is
> ~ $580 in MJF PA12, vs ~ $20 in filament if you self-print.

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
| 1 | **3S LiPo, 25C+, XT60 connector, up to 138 × 46 × 24 mm** | 11.1 V nominal. "Zeee", "OVONIC", "CNHL", or "Tattu" are all fine. The CAD reserves a 138 × 46 × 24 mm envelope on the chassis_bottom top face (Jul 2026 battery-fit rework) — anything up to that size fits. Velcro-strapped to the top of chassis_bottom in the inter-plate gap. | [Amazon search: "3S 5200mAh 25C lipo XT60"](https://www.amazon.com/s?k=3S+5200mAh+25C+lipo+XT60) |
| 1 | **Velcro hook-and-loop cinch straps (~ 15–20 mm wide)** | Loop through the chassis_bottom velcro-strap slots that straddle the battery footprint to retain the LiPo on the plate top face (replaces the old clip-in `battery_holder`). | [Amazon search: "velcro cinch straps 15mm"](https://www.amazon.com/s?k=velcro+cinch+straps+15mm) |
| 1 | **iSDT D2 / SkyRC B6 / HOTA D6 — any 3S balance charger** | Don't cheap out on charging — this is the fire-risk part of the build. | [Amazon search: "3S lipo balance charger"](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | **LiPo safety bag (medium)** | Charge AND store inside this. $8. | [Amazon search: "lipo safety bag fireproof"](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 2–3 | **XT60 pigtail (M and F, 12–14 AWG silicone wire)** | Battery cable; main fuse → PDB feed through the anti-spark switch; optional separate Uno Q battery tap. 12–14 AWG carries the full-robot ~9–13 A walking draw. | [Amazon search: "XT60 pigtail 12awg"](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | **Anti-spark on/off switch with XT60 ends** | Hard cut-off / e-stop so you don't have to unplug the LiPo every time. The "anti-spark" variant has a precharge resistor so you don't pop the switch the first time you connect. | [Amazon search: "rc lipo anti-spark switch xt60"](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |

#### Power distribution (required — serial-bus / Aug 2026 as-built)

The 18 servos draw ~9 A walking (up to ~49 A at multi-stall), but the
FEETECH 3-pin Molex 5264 connector pins are rated only **~3 A each**.
You therefore **cannot daisy-chain power through the servo bus** — it
melts the upstream connector. Power is **distributed**: one **main
fuse** → **PDB** → **chassis-top power Wagos** → **per-leg branches**
in heavy silicone. **No external buck** — battery also taps the Uno Q
directly (share ground only). See `firmware/WIRING.md` §6.

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Matek PDB-XT60 power distribution board (the "bus bar")** | Single V+ and GND tie point fed from the main fuse; the 6 per-leg branches leave via peripheral power Wagos.  36 × 50 mm, **11 g**, XT60 input, 6 output pad pairs at 15 A continuous each (~4× the ~3.7 A worst-case leg branch), 9–18 V so 3S is fine.  Ignore its small on-board BECs — Uno Q takes a direct battery tap. | [Matek PDB-XT60 (JSumo)](https://www.jsumo.com/matek-pdb-xt60-wbec-5v-and-12v) / [Amazon search: "matek pdb-xt60"](https://www.amazon.com/s?k=matek+pdb+xt60) |
| 1 | **Wago 221 lever-nuts (pack ~20)** | **Power** (12 V+G) on chassis-top periphery for motor branches; **data** under chassis near yaw retainers. | [Amazon search: "Wago 221 lever nut"](https://www.amazon.com/s?k=Wago+221+lever+nut) |
| 1 | **Main fuse 20 A + inline holder (blade or ANL)** | Sits between the anti-spark switch and the PDB; protects the whole servo rail against a dead short. Sized above the ~9–13 A walking draw and the simulated ~10 A standing hold / 16.7 A ~50 ms stand-up spike (`scripts/standup_current_sim.py`, `firmware/WIRING.md` §6.5), below a dead short. 15 A also survives the spike but 20 A never nuisance-blows. | [Amazon search: "inline blade fuse holder 12awg"](https://www.amazon.com/s?k=inline+blade+fuse+holder+12awg) |
| 6 | **(Optional) per-branch fuse 5–7 A + holder** | One per leg branch; catches a sustained single-leg multi-servo stall before it cooks that leg's 16–18 AWG harness. Optional but cheap insurance. | [Amazon search: "5a 7a mini blade fuse holder"](https://www.amazon.com/s?k=5a+7a+mini+blade+fuse+holder) |
| 1 | **16–18 AWG silicone wire (red + black, ~5 m each)** | The six per-leg power branches from the PDB / power Wagos to each leg's 5264 injection pigtail. Silicone = flexible, high strand count, survives leg motion. | [Amazon search: "16 awg silicone wire red black"](https://www.amazon.com/s?k=16+awg+silicone+wire+red+black) |
| 1 | **Molex 5264 / Mini-SPOX 2.5 mm connector + crimp kit (3-pin, ~20 sets)** | Crimp the per-leg V+/GND injection pigtails that mate to each leg's first servo, and make up the leg-to-leg **signal+GND-only** data jumpers (V+ pin omitted). Match the FEETECH bus connector. A crimper or pre-crimped pigtails save a lot of pain. | [Amazon search: "molex 5264 2.5mm connector kit crimp"](https://www.amazon.com/s?k=molex+5264+2.5mm+connector+kit+crimp) |

### B.3 Control electronics

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **Arduino Uno Q** | The whole brain: on-board Linux SoC (Python gait/RL/teleop) + MCU — replaces the Raspberry Pi, Mega, and 2×PCA9685. Mounts on the Ø110 **`hex_mount_plate_110`** (with breakout), held by four magnet posts at `CHASSIS_STANDOFF_HOLES_XY`. Powered **directly from the 3S battery** (no external buck). Preferred bus path is the MCU `feetech_bridge` sketch; USB adapter is the fallback (see `firmware/WIRING.md`). | [Arduino Store: "Uno Q"](https://store.arduino.cc/products/uno-q) |
| 1 | **USB bus-servo adapter — FEETECH FE-URT-2** (or Waveshare **Bus Servo Adapter (A)**, SKU 25514) | USB↔half-duplex-TTL bridge (bring-up / fallback). Plug the servo chain into the **TTL-BUS** port; set FE-URT-2 **3.3 V/5 V → 5 V** (Waveshare: **A/B → B = USB**). Signal only: `D`→bus signal, `G`→common GND; servo **12 V is fed per-leg from the PDB / power Wagos**, not through the adapter (see `firmware/WIRING.md` §2/§6). | [FEETECH FE-URT-2](https://www.feetechrc.com) / [Waveshare 25514](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) |
| 1 | **USB-C OTG adapter / small USB-C hub** | Needed when using the USB adapter fallback (Uno Q Type-C in host/OTG mode). Prefer a hub with power passthrough. | [Amazon search: "USB-C OTG hub power delivery"](https://www.amazon.com/s?k=USB+C+OTG+hub+power+delivery) |
| 1 | **MPU-6050 IMU breakout (GY-521)** | Closed-loop body-attitude control: 3-axis gyro + 3-axis accelerometer over I²C @ 3V3. Mounts **under the top plate** of `hex_raised_platform_110` (status screen on the top face). Chassis-top `imu_pad` is retired. | [Amazon search: "MPU-6050 GY-521 module"](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 4 | **M3 × 20 mm brass standoffs** | Magnet-post bases at `CHASSIS_STANDOFF_HOLES_XY` (±31.1) above chassis_top. | [Amazon search: "M3 20mm standoffs brass"](https://www.amazon.com/s?k=M3+20mm+standoffs+brass) |
| 4 | **M3 knurled thumb nuts (~2.5 mm thick)** | Sit on the 20 mm posts under the magnets. | [Amazon search: "M3 knurled thumb nut"](https://www.amazon.com/s?k=M3+knurled+thumb+nut) |
| 4 | **Ø8 × 8 mm neodymium disc magnets** | Top of each post; hold the Ø110 hex mount plate. | [Amazon search: "8x8mm neodymium disc magnet"](https://www.amazon.com/s?k=8x8mm+neodymium+disc+magnet) |
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
| 1 | **M3 nylon-insert (nyloc) lock nut, ~ 100 pieces** | **6 used (Jul 2026 F/F standoff switch):** one per foot-pad hinge pin.  Standoff-retention nylocs are gone — chassis F-F brass standoffs bolt from both ends (M3 × 14 up through chassis_bottom; M3 × 10 down through chassis_top).  Servo retention uses no nuts either. | [Amazon search: "M3 nyloc lock nut 100 pack"](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | **M3 brass heat-set inserts — McMaster `94459A130`, 100-pack** | **2 used (Aug 2026 as-built):** chassis_top switch_holster bosses only. Deck-tray / imu_pad / carapace inserts are retired. Clamp-cap bolts self-tap; M2.5 servo body screws thread into the servo case. | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | **M3 × 32 mm hex round standoffs, FEMALE-FEMALE brass, set of 10-20** | Sandwich the chassis plates 32 mm apart with **4** of these on the 44-mm-radius diagonal pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±31.1, ±31.1); Jul 2026 battery-fit rework).  M3 × 14 SHCS up from below chassis_bottom; M3 × 10 SHCS down from above chassis_top. Whenever `CHASSIS_GAP` changes, this length MUST change with it. | [Amazon search: "M3 32mm standoffs female female brass"](https://www.amazon.com/s?k=M3+32mm+standoffs+female+female+brass) |
| 1 | **M3 × 10 mm socket-head cap screws, A2 stainless, ~ 20 pieces** | **6 used (Aug 2026):** 2 switch_holster + 4 chassis_top → brass-standoff bolts. Deck-column tray bolts retired. | [Amazon search: "M3 10mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+10mm+SHCS+A2+stainless) |
| 1 | **M3 × 14 mm socket-head cap screws, ~ 10 pieces** | **4 used (Jul 2026 F/F standoff switch):** one per chassis standoff, driven UP from below chassis_bottom's −6 mm floor face, through the 8 mm plate + floor stack, into the F-F standoff's bottom female thread.  Often missing from 6/8/10/12/16/20 mm kits — check yours before ordering (12 mm does NOT leave enough thread: only ~4 mm engages). | [McMaster 91290A115](https://www.mcmaster.com/91290A115/) |
| 1 | **M3 × 8 mm socket-head cap screws (clamp caps + yaw caps), ~ 100 pieces** | **~42 used (Aug 2026):** 24 `servo_clamp_cap` self-tap + 18 `yaw_bearing_cap` self-tap. Deck board / imu_pad / deck-column bolts retired. | [Amazon search: "M3 8mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+8mm+SHCS+A2+stainless+100+pack) |
| 1 | **M2.5 × 8 mm SHCS (servo body-retention + horn screws), 100-pack** | **24 used** for POSITIVE servo body retention (4 per **HIP** cradle × 6 legs): the hip cradle bolts the servo's real **END-face 10 × 10 mm M2.5 hole square** (measured Waveshare ST3215) through the cradle's −X wall, head recessed in a counterbore, threading into the servo's own metal case — this bolts the servo instead of only gripping it.  **The YAW cradle takes NONE** (Jun 2026 flush-horn refit — the yaw servo hangs below the chassis floor and is held by the `yaw_servo_retainer` strap instead) **and the KNEE cradle takes NONE either** (Jul 2026 one-piece femur — the fused spar covers that wall; the clamp cap + lip hold the body), matching `PROTOTYPE_BOM.md` and the auto-generated fastener table below.  The same M2.5 × 8 stock also covers the spline center screws.  A 100-pack gives spares. | [Amazon search: "M2.5 8mm SHCS 100 pack"](https://www.amazon.com/s?k=M2.5+8mm+SHCS+100+pack) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M3x6 SHCS self-tap | 91290A111 | 24 | yaw_servo_retainer L0 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L0 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L1 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L2 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L3 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L4 saddle outboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle inboard -Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard +Y chassis anchor M3 self-tap, yaw_servo_retainer L5 saddle outboard -Y chassis anchor M3 self-tap |
| M2.5 SHCS into servo case | 91290A104 | 24 | cradle servo body-retention bolts (M2.5 into servo end face) |
| M2.5x8 spline screw | 91290A104 | 30 | hip passive-horn retention screw L0, hip passive-horn retention screw L1, hip passive-horn retention screw L2, hip passive-horn retention screw L3, hip passive-horn retention screw L4, hip passive-horn retention screw L5, knee passive-horn retention screw L0, knee passive-horn retention screw L1, knee passive-horn retention screw L2, knee passive-horn retention screw L3, knee passive-horn retention screw L4, knee passive-horn retention screw L5, servo spline center screws |
| M3x8 SHCS self-tap | 91290A113 | 42 | sandwich-joint clamp-cap bolts (M3 SHCS self-tap), yaw_bearing_cap join screws (cap -> chassis_bottom tower, M3 x 8 SHCS self-tap) |
| M3x10 SHCS | 91290A114 | 6 | chassis_top brass standoff bolts (M3 x 10 SHCS into standoff female thread), switch_holster heat-set inserts |
| M3x10 disc-horn SHCS | 91290A114 | 96 | link-to-disc-horn bolts |
| M3x14 SHCS | 91290A115 | 4 | chassis_bottom brass standoff bolts (M3 x 14 SHCS up into standoff female thread) |
| M3x20 disc-horn SHCS | 91290A120 | 24 | link-to-disc-horn bolts |
| M3 heat-set insert | 94459A130 | 2 | switch_holster heat-set inserts |
| M3x16 pan-head | 92010A130 | 6 | foot hinge pins |
| M3 nyloc nut | 90576A102 | 6 | foot hinge pins |
| M2.5 self-tap into servo rear case | 96877A150 | 24 | yaw_servo_retainer L0 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L0 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L1 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L2 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L3 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L4 saddle x2-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x1-Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2+Y rear case-mount M2.5 self-tap, yaw_servo_retainer L5 saddle x2-Y rear case-mount M2.5 self-tap |
|  |  | **288** | **total fasteners** |

Notes:
- The servo OUTPUT face is reserved for the flush 20 mm disc
  horn (no front-face mounting).  POSITIVE body retention bolts
  into the servo's REAL END-face M2.5 holes: each STS3215 +/-X end
  face carries a 10 x 10 mm square of 4 M2.5 case holes (measured
  Waveshare ST3215).  Only the HIP cradle drives 4 x M2.5
  x 8 SHCS through its -X wall into that square (`91290A104`, 4
  per hip cradle x 6 legs = 24; head recessed in a wall
  counterbore, threads into the servo's own metal case).  The YAW
  cradle takes NONE (Jun 2026 flush-horn refit): both yaw bearings
  moving above the flush horn lowered the yaw output 5.5 mm, so the
  servo now hangs ~20 mm below the -6 chassis floor and even its
  upper end-face row clears no -X wall -- the yaw servo is held by
  the `yaw_servo_retainer` strap + anchor bolts + output-face seat
  instead.  The KNEE cradle takes NONE either (Jul 2026 one-piece
  femur): the fused Phi 14 spar covers the knee cradle's -X wall
  from outside, so the screws could never be driven there -- the
  knee servo is held by the clamp cap + retaining lip.  At the hip
  these screws positively bolt the servo instead of only gripping
  it; the clamp cap is retained as secondary capture.
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
| 1 | **Soldering iron + flux + 60/40 solder** | Required to install the M3 brass heat-set inserts in the switch_holster bosses (and any header soldering). | [Amazon search: "soldering iron kit beginner"](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | **Hex (Allen) key set, 1.5 / 2 / 2.5 / 3 / 4 mm** | M3 cap screws need 2.5 mm, M2.5 needs 2 mm. | [Amazon search: "hex key set metric ball end"](https://www.amazon.com/s?k=hex+key+set+metric+ball+end) |
| 1 | **Cyanoacrylate (super-glue) 20 g** | Glue the rubber sleeve into the foot pad cup. | [Amazon search: "super glue gel"](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | **Inner-tube section** (or skateboard-tape strip) | Foot tread. Cut a circle from a bicycle inner tube and CA-glue inside `foot_pad.stl`'s cup. | n/a — old bicycle tube |

---

## C. Cost summary

| Bucket | Cost (USD, mid-2026) |
|---|---:|
| 20 × STS3215 serial-bus servos | $360–$500 |
| Battery + charger + bag + XT60/velcro | $70 |
| Power distribution (PDB + Wagos + 15–20 A main fuse + branch fuses + 16–18 AWG silicone + 5264 crimp kit) | $40–$60 |
| Arduino Uno Q + USB bus-servo adapter + USB-C OTG/hub + IMU + serial-bus cables + jumpers | $100 |
| Fasteners (M3 kit + nylocs + chassis standoffs + 20 mm magnet posts + thumb nuts + magnets) | $35 |
| Filament (PLA 1 kg + TPU 250 g) + 12 × MJF servo_clamp_cap + hex plate / raised platform | $40 |
| Soldering iron / hex keys / glue (if you don't have them) | $30 |
| **Total** | **~ $665–$825** |

The cost assumes STS3215 serial-bus servos. Cheaper servos are
intentionally not listed here because they change the risk profile and
may not fit the printed wells without regenerating the STLs.

---

## D. Print queue (suggested order)

Print **legs first**, body **last** — that way you can dry-fit each
leg on the servo before committing the chassis plates. Filenames here
mirror §A / `PROTOTYPE_BOM.md` (the Jun 2026 sandwich-joint set; note
`femur_link.stl` is BACK as a real printable since the Jul 2026
one-piece femur merge — the old single-spar `coxa_bracket` /
`tibia_link` names stay retired).

1. **1 full leg set first** — `coxa_link` + `femur_link` +
   `tibia_knee_yoke` + `tibia_foot_fitting` +
   `foot_pad` — to validate the symmetric bracket + dual-disc-horn fit
   before committing six sets (the passive side uses the servo's stock
   metal rear horn; nothing extra to print).
2. **6 × `coxa_link.stl`** + **6 × `yaw_bearing_cap.stl`** — ~4 h.
3. **6 × `femur_link.stl`** (one-piece femurs) — ~8 h.
4. **6 × `tibia_knee_yoke.stl`** + **6 × `tibia_foot_fitting.stl`** — ~6 h.
5. **12 × `servo_clamp_cap.stl`** (MJF PA12 or PLA) — ~3 h.
6. **`chassis_top.stl` + `chassis_bottom.stl` + `switch_holster.stl`**
   plus `extra_stl` **hex_mount_plate_110** / **hex_raised_platform_110**
   — ~6 h.
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

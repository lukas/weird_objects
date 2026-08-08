# Hexapod Prototype BOM

This is the controlled bill of materials for **one complete tabletop
hexapod prototype**. The design is built around **FEETECH STS3215
(ST-3215-C018, 12 V / 30 kg-cm) serial-bus servos** in a **symmetric
disc-horn sandwich joint** (one STS3215 driving a disc horn on the front
spline, a SECOND reused disc horn on the servo's own rear idler boss for
passive support, the yoke bolting identically to both — no external ball
bearing — with an Ø8 mm carbon-fibre tube as the TIBIA segment and the
WHOLE FEMUR printed as one part, `femur_link` — hip yoke + solid Ø14
spar + knee bracket, Jul 2026). Do not
substitute other servo models unless you are ready to measure them and
regenerate the printed brackets/yokes.

> **Jun 2026 refit / Aug 2026 as-built stack.** This supersedes the earlier
> DS3225 + Arduino Mega + 2×PCA9685 + drop-in-cradle design. The STS3215
> are smart serial-bus servos daisy-chained on a single half-duplex TTL
> bus driven from an **Arduino Uno Q** (on-board Linux SoC + MCU) — there
> is **no Arduino Mega, no PCA9685, no Raspberry Pi**.  **No external
> buck:** the battery feeds the PDB (→ servos via peripheral power Wagos)
> and the Uno Q on a separate tap (share ground only).  Electronics deck:
> four posts at `CHASSIS_STANDOFF_HOLES_XY` (±31.1) — 20 mm standoff +
> thumb nut + Ø8×8 mm magnet — hold a Ø110 **`hex_mount_plate_110`**
> (Uno Q + breakout); **`hex_raised_platform_110`** sits above (screen on
> top, MPU under the top plate).  Printed `uno_q_tray` / `buck_tray` /
> `spider_carapace` / chassis-top `imu_pad` are retired.  LiPo is
> **velcro-strapped under the chassis** on `chassis_bottom`; data Wagos
> sit near the yaw retainers.

Links are stable Amazon search links rather than one-off ASINs, because
Amazon listings churn. Pick a well-reviewed Prime listing that matches the
spec exactly.

## Required Purchases

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 20 | FEETECH STS3215 serial-bus servo | **STS3215 (ST-3215-C018), 12 V, 30 kg-cm metal-gear smart serial-bus servo**, standard ~40 x 20 x 40 mm case with a 25T output spline carrying the flush 20 mm disc horn (4×M3 on a Ø14 cross).  The body is bolted to the cradle by 4× M2.5 into the servo's real END-face 10×10 mm hole square (plus the printed strap/clamp), not by output-face screws. Daisy-chained on a 1 Mbps half-duplex TTL bus. Buy all 20 from the same batch. 18 used + 2 spares. | [Amazon: FEETECH STS3215 30kg serial bus servo](https://www.amazon.com/s?k=FEETECH+STS3215+30kg+serial+bus+servo) |
| 1 | Arduino Uno Q | **Arduino Uno Q** (on-board Linux SoC + MCU). Runs Python gait/RL/teleop AND drives the half-duplex STS3215 TTL bus — replaces both the Raspberry Pi and the Mega/PCA stack. Mounts on the Ø110 `hex_mount_plate_110` (with breakout), held by the four magnet posts. Powered directly from the 3S battery (no external buck). | [Arduino Store: Uno Q](https://store.arduino.cc/products/uno-q) |
| 1 | microSD card | 32 GB or 64 GB, A1/A2 rated (Uno Q OS/storage if not using on-board eMMC). | [Amazon: 32GB A1 microSD card](https://www.amazon.com/s?k=32GB+A1+microSD+card) |
| 1 | LiPo battery | 3S, 25C or higher, XT60 connector, up to **138 × 46 × 24 mm** (the envelope the CAD reserves — Jul 2026 battery-fit rework). The 11.1 V nominal (12.6 V full) feeds the PDB→servo domain and a separate Uno Q tap. Velcro-strapped under the chassis on `chassis_bottom`. | [Amazon: 3S 5200mAh 25C lipo XT60](https://www.amazon.com/s?k=3S+5200mAh+25C+lipo+XT60) |
| 1 | Velcro straps | Hook-and-loop cinch straps (~15–20 mm wide) looped through the chassis_bottom strap slots to retain the LiPo. | [Amazon: velcro cinch straps 15mm](https://www.amazon.com/s?k=velcro+cinch+straps+15mm) |
| 1 | Bus servo cables | FEETECH 3-pin serial-bus servo cables (servos ship with one each; buy a small spare pack for the chassis-to-first-servo runs). | [Amazon: FEETECH serial bus servo cable](https://www.amazon.com/s?k=FEETECH+serial+bus+servo+cable) |
| 1 | LiPo charger | 3S balance charger, e.g. SkyRC B6 style or better. | [Amazon: 3S lipo balance charger](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | LiPo safety bag | Fire-resistant charging/storage bag. | [Amazon: lipo safety bag fireproof](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 1 | Anti-spark switch | XT60 RC LiPo anti-spark/on-off switch for servo rail power (also the e-stop). | [Amazon: rc lipo anti-spark switch xt60](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |
| 2 | XT60 pigtails | Male/female XT60 silicone-wire pigtails, 12-14 AWG. | [Amazon: XT60 pigtail 12awg](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | Power distribution board (the "bus bar") | **Matek PDB-XT60** drone power-distribution board (36 × 50 mm, 11 g w/ XT60): 6 output pad pairs at 15 A continuous each, XT60 input socket, 9–18 V (3S OK).  Feed the 6 per-leg 16–18 AWG branches via chassis-top peripheral **power Wagos** (12 V+G).  Rated 4× above the ~3.7 A worst-case branch draw.  Everywhere these docs say "bus bar" / PDB, this is the part.  **Required** because the 18 servos cannot be power-daisy-chained through the ~3 A Molex 5264 pins (see `firmware/WIRING.md` §6).  Ignore its tiny on-board BECs — the Uno Q takes a direct battery tap (no external buck). | [Matek PDB-XT60](https://www.jsumo.com/matek-pdb-xt60-wbec-5v-and-12v) / [Amazon: matek pdb-xt60](https://www.amazon.com/s?k=matek+pdb+xt60) |
| 1 | Wago 221 lever-nuts | Compact splices for **power** (12 V+G motor branches on chassis-top periphery) and **data** (under chassis near yaw retainers). Pack of ~20 covers both. | [Amazon: Wago 221 lever nuts assortment](https://www.amazon.com/s?k=Wago+221+lever+nut) |
| 4 | M3 × 20 mm brass standoffs | Magnet-post bases at `CHASSIS_STANDOFF_HOLES_XY` (±31.1) above chassis_top. | [Amazon: M3 20mm standoffs brass](https://www.amazon.com/s?k=M3+20mm+standoffs+brass) |
| 4 | M3 knurled thumb nuts (~2.5 mm) | Sit on the 20 mm posts under the magnets. | [Amazon: M3 knurled thumb nut](https://www.amazon.com/s?k=M3+knurled+thumb+nut) |
| 4 | Ø8 × 8 mm disc magnets | Top of each post; hold the Ø110 hex mount plate. | [Amazon: 8x8mm neodymium disc magnets](https://www.amazon.com/s?k=8x8mm+neodymium+disc+magnet) |
| 1 | MPU-6050 IMU (GY-521) | 6-DOF gyro + accel, I²C @ 3V3. Glued on `chassis_bottom` inboard of physical leg 1. | [Amazon: MPU-6050 GY-521 module](https://www.amazon.com/s?k=MPU-6050+GY-521+module) |
| 1 | Main fuse + holder | **20 A** blade fuse + inline holder, between the anti-spark switch and the PDB; protects the whole servo rail. (Sized by the Jul 2026 MuJoCo stand-up sim — `standup_current_sim.py` / `firmware/WIRING.md` §6.5: ~10 A standing hold, 16.7 A ~50 ms stand-up spike — 20 A never nuisance-blows; 15 A also survives but with less margin.) | [Amazon: inline blade fuse holder 12awg](https://www.amazon.com/s?k=inline+blade+fuse+holder+12awg) |
| 6 | Per-branch fuse + holder (optional) | 5-7 A mini-blade fuse + holder, one per leg branch; catches a sustained single-leg multi-servo stall before it cooks the leg's 16-18 AWG harness. | [Amazon: 5a 7a mini blade fuse holder](https://www.amazon.com/s?k=5a+7a+mini+blade+fuse+holder) |
| 1 | 16-18 AWG silicone wire | Red + black, ~5 m each. The six per-leg V+/GND power branches from the bus bar to each leg's 5264 injection pigtail. | [Amazon: 16 awg silicone wire red black](https://www.amazon.com/s?k=16+awg+silicone+wire+red+black) |
| 1 | Molex 5264 connector + crimp kit | 3-pin Molex 5264 / Mini-SPOX 2.5 mm connector kit (~20 sets) to crimp the per-leg V+/GND injection pigtails and the leg-to-leg **signal+GND-only** data jumpers (V+ pin omitted so power never bridges legs). | [Amazon: molex 5264 2.5mm connector kit crimp](https://www.amazon.com/s?k=molex+5264+2.5mm+connector+kit+crimp) |
| 18 | 20 mm aluminium 25T disc horn | **Ø20 mm aluminium 25T servo disc horn (4× M3 tapped on a Ø14 cross).** Drives every joint (3 per leg × 6 legs = 18). Ships in 10-packs — buy **2 packs**. The PASSIVE rear support on the hip + knee uses the STS3215's own **stock metal passive horn** (ships with the servo; same 4-hole pattern): its centre bore slides over the rear idler boss so it seats flush on the back face, held by one M2.5 screw — no extra purchased horns and no printed adapter (Jul 2026 stock-horn refit). | [Amazon: 10Pcs 25T aluminum servo disc horn (B07D56FVK5)](https://www.amazon.com/s?k=25T+aluminum+servo+disc+horn) |
| 12 | 6706-2RS ball bearing (YAW) | **Ø30 mm bore × Ø37 mm OD × 4 mm wide thin-section sealed bearing.** The yaw joint of every leg uses a **SPACED PAIR** (2 per leg × 6 legs) stacked ~7 mm apart on the `coxa_link`'s hub boss (Aug 2026: the coxa is ONE printed part again — hub + hip bracket merged): the lower bearing rides around the disc horn, the upper a few mm above it. The pair reacts the cantilever MOMENT (tilt stiffness ∝ spacing²) into the `chassis_bottom` tower instead of the servo spline. **The tower is SPLIT (Jun 2026):** the LOWER race drops onto the open-top Ø37 pocket in `chassis_bottom`, the UPPER race drops into the bolt-on `yaw_bearing_cap`, then 3 M3 screws pull the cap down to capture both races at the correct spacing — so each race seats on an OPEN face (the old one-piece tower trapped both races between Ø34 constrictions and was impossible to assemble). Inner races clamp on the hub boss. Buy a 12+ pack for spares. | [Amazon: 6706-2RS bearing 30x37x4](https://www.amazon.com/s?k=6706-2RS+bearing+30x37x4) |
| 1 | Carbon-fibre tube, Ø8 mm | **Ø8 mm OD × Ø6 mm ID roll-wrapped CF tube.** TIBIA leg segments only (1 per leg — the femur is one printed part with no tube, Jul 2026). One ~1 m length yields all 6 × ~110 mm tibia segments with spare. Epoxy-bonded into the printed sockets. | [Amazon: 8mm carbon fiber tube 6mm ID](https://www.amazon.com/s?k=8mm+carbon+fiber+tube+6mm+id) |
| 12 | Ø2.5 mm roll pin (spring pin) | Transverse retention pins: 2 per tibia (one per CF-tube socket, cross-holes printed in, no drilling). The femur uses NONE since the Jul 2026 one-piece merge (`femur_link` has no joints to pin). Buy an assortment. | [Amazon: 2.5mm spring roll pin assortment](https://www.amazon.com/s?k=2.5mm+spring+roll+pin+assortment) |
| 1 | Two-part epoxy | Slow-cure (30 min) structural epoxy for bonding the tibia CF tubes into the printed yoke/fitting sockets (the femur needs no epoxy — it is one printed part). | [Amazon: 30 minute structural epoxy](https://www.amazon.com/s?k=30+minute+two+part+epoxy) |
| 1 | Nylon zip-ties, 100-pack | 24 used for cable strain relief: 3 per leg at the cradle `cable_zip_post`s (yaw / hip / knee) + 1 per leg looped through the chassis_bottom leg-harness drop slot (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  2-3 mm wide, ~ 100 mm long. | [Amazon: nylon zip ties 4 inch 100 pack](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | Dupont jumper kit | Mixed M-F / F-F / M-M jumper wires for I2C and logic wiring. | [Amazon: dupont jumper wires 120 pcs](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | USB-C cable | USB-C cable for flashing / powering / console access to the Arduino Uno Q. | [Amazon: USB C cable data](https://www.amazon.com/s?k=USB+C+cable+data) |
| 1 | M3 screw assortment | M3 socket-head screws, nuts, washers, lengths 6/8/10/12/16/20 mm. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | M3 x 16 pan-head bolts | Foot/tibia hinge pin -- one per leg.  Passes through the foot_pad's FORK (3.5 + 6.4 + 3.5 = 13.4 mm) and engages an M3 nylock on the far cheek; the tibia's TANG sits in the slot (May 2026 inversion: fork on the foot, tang on the tibia -- pre-2026 it was the other way round, same pin + nut + 16 mm length).  Pan-head (low profile), threaded full length.  Stainless.  Reuse from the M3 assortment if it includes 16 mm. | [Amazon: M3 x 16 pan head stainless](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | M3 nyloc nuts | 100-pack nylon-insert lock nuts. **6 used:** one per foot-pad hinge pin.  Chassis F-F brass standoffs bolt from both ends (no nylocs).  Servo retention uses no nuts either. | [Amazon: M3 nyloc lock nut 100 pack](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | M3 heat-set inserts (`94459A130`) | McMaster knurled brass M3 heat-set insert, Phi 4.0 mm pilot, Phi 5.7 mm OD, 5.0 mm length. **2 used (Aug 2026 as-built):** chassis_top's printed bosses for `switch_holster` only — deck-tray / imu_pad / carapace inserts are retired.  (Clamp-cap and M2.5 servo body-retention screws use no inserts.)  Installed with a soldering iron at ~220 deg C; an M3 x 10 SHCS (switch_holster) threads into the brass. | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | M3 x 10 SHCS (`91290A114`) | M3 x 10 mm socket-head cap screw, black-oxide steel.  **6 used (Aug 2026):** 2 switch_holster mount bolts + 4 chassis_top → brass-standoff bolts (DOWN into the F-F standoff female top threads on the 44-mm-radius diagonal pattern).  Deck-column tray bolts are retired.  Buy a 50-pack so you have spares. | [McMaster 91290A114](https://www.mcmaster.com/91290A114/) |
| 1 | M3 x 14 SHCS (`91290A115`) | M3 x 14 mm socket-head cap screw, black-oxide steel.  **4 used (Jul 2026 F/F standoff switch):** one per chassis standoff, driven UP from below chassis_bottom's −6 mm floor face, through the 8 mm plate + floor stack, into the F-F brass standoff's bottom female thread.  Replaces the old M-F male stud + nyloc, which could not span the merged 8 mm plate. | [McMaster 91290A115](https://www.mcmaster.com/91290A115/) |
| 1 | M3 x 8 SHCS (`91290A113`) | M3 x 8 mm socket-head cap screw, black-oxide steel.  **~42 used (Aug 2026):** **24 sandwich-joint clamp-cap bolts** + **18 yaw_bearing_cap join screws** (deck board-mount / imu_pad / deck-column bolts retired with the magnet-hex stack).  Buy a 100-pack so you have spares. | [McMaster 91290A113](https://www.mcmaster.com/91290A113/) |
| 1 | M2.5 x 8 SHCS (`91290A104`) | M2.5 x 8 mm socket-head cap screw.  **24 used** for POSITIVE servo body retention: 4 per HIP cradle (4 × 6 legs = 24).  The **YAW cradle takes NONE** (Jun 2026 flush-horn refit): moving both yaw bearings above the flush horn lowered the yaw output 5.5 mm, so the servo now hangs ~20 mm below the −6 chassis floor and even its upper end-face row clears no −X wall — the yaw servo is held instead by the `yaw_servo_retainer` strap + anchor bolts + the output-face seat on the mount plate.  The **KNEE cradle takes NONE either** (Jul 2026 one-piece femur): the fused Ø14 spar covers the knee cradle's −X wall from outside, so the screws could never be driven there — the empty holes were removed; the knee servo is held by the clamp cap + retaining lip.  Each HIP cradle bolts the servo's real **END-face M2.5 10 × 10 mm hole square** (measured Waveshare ST3215) through the cradle's −X wall; the head is recessed in a counterbore and the screw threads into the servo's own metal case.  (Same M2.5 × 8 stock also serves as the spline center screws; buy a 100-pack.) | [McMaster 91290A104](https://www.mcmaster.com/91290A104/) |
| 1 | M3 standoffs (chassis) | M3 x 32 mm FEMALE-FEMALE brass standoffs, pack of 10-20.  **4 used** on the 44-mm-radius diagonal pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±31.1, ±31.1); Jul 2026 battery-fit rework moved them off (±35, 0)/(0, ±35) so the 138 × 46 mm LiPo has a clear lane) clamping chassis_top to chassis_bottom across the CHASSIS_GAP = 32 mm inter-plate gap.  Jul 2026 F/F switch: an M3 × 14 SHCS enters UP from below chassis_bottom's −6 mm floor face into the standoff's bottom female thread; an M3 × 10 SHCS drops DOWN from above chassis_top into the female top.  Re-verify lengths whenever CHASSIS_GAP changes. | [Amazon: M3 32mm standoffs female female brass](https://www.amazon.com/s?k=M3+32mm+standoffs+female+female+brass) |
| 1 | M2.5 screw pack | M2.5 x 8 mm screws, useful for horn/adapter work if the servo-included screws are bad. | [Amazon: M2.5 8mm screw 50 pack](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

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
| 1 kg | PLA or PETG filament | 1.75 mm. Structural printed parts. PLA is easiest; PETG is tougher. | [Amazon: PLA filament 1.75mm 1kg](https://www.amazon.com/s?k=PLA+filament+1.75mm+1kg) |
| 250 g | TPU 95A filament | Foot pads. If you skip TPU, print feet in PLA and glue rubber tread into the cups. | [Amazon: TPU 95A filament 1.75mm](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | Heat-shrink kit | Assorted small heat-shrink tubing for power wiring cleanup. | [Amazon: heat shrink tubing assorted](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |

## Strongly Recommended Tools

| Qty | Item | Why | Link |
|---:|---|---|---|
| 1 | Digital calipers | Check STS3215 body/tab dimensions before printing all six legs. | [Amazon: digital calipers](https://www.amazon.com/s?k=digital+calipers) |
| 1 | Soldering iron kit | Required to install the M3 brass heat-set inserts in the switch_holster bosses (and any header soldering). | [Amazon: soldering iron kit beginner](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | Metric hex key set | M3 socket-head screws usually need a 2.5 mm hex key. | [Amazon: metric ball end hex key set](https://www.amazon.com/s?k=metric+ball+end+hex+key+set) |
| 1 | Super glue gel | Foot tread / small print fixes. | [Amazon: super glue gel](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | Needle file set | Useful if a servo well or femur slot is slightly tight from FDM over-extrusion. | [Amazon: needle file set](https://www.amazon.com/s?k=needle+file+set) |

## Printed Parts

Files are in `hexapod_walker/prototype/stl_prototype/`.

| Qty | STL |
|---:|---|
| 1 | `chassis_top.stl` |
| 1 | `chassis_bottom.stl` (single merged Jun 2026 part: genuinely FLAT-bottom 200 mm hex plate with a folded-in ~8 mm-thick flat floor carrying the 6 yaw-servo cradles + upward yaw-bearing tower; prints face-DOWN, no supports) |
| 1 | `switch_holster.stl` |
| 1 | `extra_stl/hex_mount_plate_110_with_leg_holes.stl` (Ø110 magnet-held board for Uno Q + breakout; also SVG cut file) |
| 1 | `extra_stl/hex_raised_platform_110_h72_screen.stl` (raised platform — screen on top, MPU under top plate) |
| 12 | `servo_clamp_cap.stl` (MJF PA12 — 1 per hip + knee sandwich joint) |
| 6 | `coxa_link.stl` (ONE piece, Aug 2026 merge: yaw turntable hub + hip fixed side; 5 head-access shafts reach the yaw horn screws through the empty hip servo well) |
| 6 | `yaw_bearing_cap.stl` (TOP half of the split yaw-bearing tower; bolts onto `chassis_bottom` with 3 M3 to capture the spaced 6706 pair — print flat, spigot-side down) |
| 6 | `femur_link.stl` (the WHOLE femur, one printed part — hip moving yoke + solid Ø14 spar + knee fixed side, Jul 2026) |
| 6 | `tibia_knee_yoke.stl` (knee moving yoke + CF-tube socket) |
| 6 | `tibia_foot_fitting.stl` (CF-tube socket + foot hinge tang) |
| 6 | `foot_pad.stl` (TPU) |
| 6 | `yaw_servo_retainer.stl` (anti-rotation saddle; Aug 2026 flat-belly rework removed the 38 mm ground stand — no printed belly stand) |

Short CF legs 0/4 (~124 mm): print 2× `extra_stl/tibia_foot_fitting_plus4.stl`
(+4 mm longer tang) instead of the nominal fitting — same `foot_pad` on all six.

Femur = `femur_link`, one printed body — no pins, no sockets, no CF tube
(Jul 2026 merge of the old hip yoke + knee bracket; the connecting spar
is solid Ø14, the old socket-boss outer diameter).
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
| 1 | `femur_link.stl` |
| 1 | `tibia_knee_yoke.stl` |
| 1 | `tibia_foot_fitting.stl` |
| 1 | `foot_pad.stl` |

## Bench Test Order

1. Buy **one STS3215** + **one Arduino Uno Q** + **one 25T disc horn** (driven side; the passive side uses the servo's stock metal rear horn) first to de-risk the joint.
2. Print one full leg set (above) to validate the symmetric bracket + dual-disc-horn fit before printing six sets.
3. Confirm the STS3215 body seats snug in each cradle: at the HIP it is bolted by 4× M2.5 into its END-face 10×10 hole square (driven through the cradle's −X wall) plus the printed clamp cap (which seats FLUSH against the body's +Y face so the bolts trap it with no slop); at the KNEE there are NO end-face bolts (Jul 2026 one-piece femur — the fused spar covers that wall), so verify the clamp cap + retaining lip alone hold the body with no slop.  The front disc horn drives the yoke top arm via its 4× M3 leg bolts on the Ø14 cross, and the rear STOCK passive horn — its centre bore over the servo's idler boss, flush on the back face, retained by one M2.5 screw — takes the bottom arm's 4× M3 the same way.
4. Wire one STS3215 to the Uno Q's TTL bus pins and run:

```bash
python hexapod_walker/prototype_sts3215/motor_setup/feetech_bus.py --port /dev/ttyUSB0 wiggle --joint 0
```

5. Once fit and motion are good, buy/print the rest.

## Rough Cost

| Bucket | Estimate |
|---|---:|
| STS3215 serial-bus servos, 20 total | $360-$500 |
| Arduino Uno Q + microSD + USB-C cable | $60-$100 |
| Battery + charger + safety bag + switch + velcro | $70-$120 |
| Power distribution (PDB + fuses + Wagos + 16-18 AWG silicone + 5264 crimp kit) | $40-$60 |
| Disc horns (30) + CF tube + roll pins + epoxy | $30-$55 |
| Fasteners / standoffs / magnet posts / wiring consumables | $30-$55 |
| Filament + MJF clamp caps + hex plate / raised platform | $30-$55 |
| **Total** | **~$615-$1000** |

If you already own a charger, tools, or filament, the actual
cash outlay is much lower.

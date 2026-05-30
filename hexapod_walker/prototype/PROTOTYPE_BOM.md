# Hexapod Prototype BOM

This is the controlled bill of materials for **one complete tabletop
hexapod prototype**. The design is built around **DS3225 servos**; do not
substitute other servo models unless you are ready to measure them and
regenerate the printed wells.

Links are stable Amazon search links rather than one-off ASINs, because
Amazon listings churn. Pick a well-reviewed Prime listing that matches the
spec exactly.

## Required Purchases

| Qty | Item | Spec to buy | Link |
|---:|---|---|---|
| 20 | DS3225 servo | **DS3225 25 kg-cm metal gear digital servo**, standard-size case, ~40 x 20 x 38 mm body, ~54 mm tab span. Buy all 20 from the same listing/batch. 18 used + 2 spares. | [Amazon: DS3225 servo 25kg metal gear](https://www.amazon.com/s?k=DS3225+servo+25kg+metal+gear) |
| 1 | Raspberry Pi | Raspberry Pi 4 or Pi 5, 4 GB+ RAM. Runs Python gait/RL/teleop and talks USB serial to Arduino. | [Amazon: Raspberry Pi 5 4GB kit](https://www.amazon.com/s?k=Raspberry+Pi+5+4GB+kit) |
| 1 | Pi power supply | Official or equivalent USB-C 5 V supply, 3 A minimum for Pi 4, 5 A preferred for Pi 5. | [Amazon: Raspberry Pi USB C power supply](https://www.amazon.com/s?k=Raspberry+Pi+USB+C+power+supply) |
| 1 | microSD card | 32 GB or 64 GB, A1/A2 rated. | [Amazon: 32GB A1 microSD card](https://www.amazon.com/s?k=32GB+A1+microSD+card) |
| 1 | Arduino Mega 2560 | Mega 2560 R3 compatible board. Acts as the real-time USB-to-PCA9685 servo bridge. | [Amazon: Arduino Mega 2560 R3](https://www.amazon.com/s?k=Arduino+Mega+2560+R3) |
| 2 | PCA9685 servo driver | 16-channel I2C PWM servo driver boards. One at `0x40`, one at `0x41`. | [Amazon: PCA9685 16 channel servo driver](https://www.amazon.com/s?k=PCA9685+16+channel+servo+driver) |
| 1 | LiPo battery | 3S 2200 mAh, 25C or higher, XT60 connector. | [Amazon: 3S 2200mAh 25C lipo XT60](https://www.amazon.com/s?k=3S+2200mAh+25C+lipo+XT60) |
| 2 | 5-6 V BEC / UBEC | Switching BEC, 5 A minimum each. Use two rails: ~9 servos per BEC. | [Amazon: 5V 5A UBEC switching](https://www.amazon.com/s?k=5V+5A+UBEC+switching) |
| 1 | LiPo charger | 3S balance charger, e.g. SkyRC B6 style or better. | [Amazon: 3S lipo balance charger](https://www.amazon.com/s?k=3S+lipo+balance+charger) |
| 1 | LiPo safety bag | Fire-resistant charging/storage bag. | [Amazon: lipo safety bag fireproof](https://www.amazon.com/s?k=lipo+safety+bag+fireproof) |
| 1 | Anti-spark switch | XT60 RC LiPo anti-spark/on-off switch for servo rail power. | [Amazon: rc lipo anti-spark switch xt60](https://www.amazon.com/s?k=rc+lipo+anti-spark+switch+xt60) |
| 2 | XT60 pigtails | Male/female XT60 silicone-wire pigtails, 12-14 AWG. | [Amazon: XT60 pigtail 12awg](https://www.amazon.com/s?k=XT60+pigtail+12awg) |
| 1 | Servo extensions | 20-pack, 3-pin male-female, 30 cm.  The 18-servo harness uses 12 stock DS3225 pigtails on their own + 6 stock pigtails-plus-one-30cm-extension; see `pi_control/wire_harness_plan.py` for the per-joint table (auto-generated, validated by `_verify_prototype.check_harness_reach`). | [Amazon: servo extension cable 30cm 20 pack](https://www.amazon.com/s?k=servo+extension+cable+30cm+20+pack) |
| 1 | Nylon zip-ties, 100-pack | 24 used for cable strain relief: 3 per leg at the cradle `cable_zip_post`s (yaw / hip / knee) + 1 per leg looped through the chassis_bottom leg-harness drop slot (each leg's cable drop slot doubles as the zip-tie anchor -- pass a zip-tie through the slot to bundle the per-leg harness).  2-3 mm wide, ~ 100 mm long. | [Amazon: nylon zip ties 4 inch 100 pack](https://www.amazon.com/s?k=nylon+zip+ties+4+inch+100+pack) |
| 1 | Dupont jumper kit | Mixed M-F / F-F / M-M jumper wires for I2C and logic wiring. | [Amazon: dupont jumper wires 120 pcs](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | USB cable, Pi to Arduino | USB-A to USB-B if using Arduino Mega with full-size USB-B. If your clone uses USB-C or micro-USB, buy that cable instead. | [Amazon: USB A to B cable Arduino](https://www.amazon.com/s?k=USB+A+to+B+cable+Arduino) |
| 1 | M3 screw assortment | M3 socket-head screws, nuts, washers, lengths 6/8/10/12/16/20 mm. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | M3 x 16 pan-head bolts | Foot/tibia hinge pin -- one per leg.  Passes through the foot_pad's FORK (3.5 + 6.4 + 3.5 = 13.4 mm) and engages an M3 nylock on the far cheek; the tibia's TANG sits in the slot (May 2026 inversion: fork on the foot, tang on the tibia -- pre-2026 it was the other way round, same pin + nut + 16 mm length).  Pan-head (low profile), threaded full length.  Stainless.  Reuse from the M3 assortment if it includes 16 mm. | [Amazon: M3 x 16 pan head stainless](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | M3 nyloc nuts | 100-pack nylon-insert lock nuts. **10 used as of the May 2026 tray-mount fix:** 6 on the foot-pad hinge pins (one per leg) + 4 UNDER chassis_bottom retaining the brass M-F standoffs' male threads on the rotated-45-deg 35-mm-radius pattern (CHASSIS_STANDOFF_HOLES_XY).  The 72 cradle servo mounts still thread into brass heat-set inserts (see below) and do NOT use a nut.  Earlier docs (pre-2026 coxa-bracket flange) counted 24 + 6; that flange has been retired. | [Amazon: M3 nyloc lock nut 100 pack](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | M3 heat-set inserts (`94459A130`) | McMaster knurled brass M3 heat-set insert, Phi 4.0 mm pilot, Phi 5.7 mm OD, 5.0 mm length. **62 used** under the May 2026 Design E mixed-mode cradle scheme + tray-mount fix: 36 in the cradles (the 2 -X bolts per cradle only -- the 2 +X bolts self-tap into Phi 2.5 mm pilots) + 4 in the battery_holder feet + 8 in the electronics_tray (4 Mega + 4 primary PCA9685) + 4 more in the electronics_tray (secondary PCA9685) + 4 in the imu_pad (MPU-6050 mount) + 2 in chassis_top's printed bosses (switch_holster) + **4 in chassis_bottom's new tray-mount bosses on the (±24.75, ±24.75) pattern (May 2026 tray-mount fix)**. Installed with a soldering iron at ~220 deg C, light downward pressure, ~10-15 s per insert, then cool ~30 s; an M3 x 8 SHCS (-X cradle bolts / board mount / IMU pad) or M3 x 10 SHCS (battery_holder foot / switch_holster) threads into the brass instead of self-tapping into plastic.  Same SKU re-used everywhere M3 inserts appear in the printed-parts BOM.  (Down from the 94 of the brief all-heat-set Design D iteration; commit history under "Design E" for the +X channel-vs-boss conflict that forced the partial revert.) | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | M2.5 heat-set inserts (`94459A106`) | McMaster knurled brass M2.5 heat-set insert, Phi 3.0 mm pilot, Phi 3.6 mm OD, 4.0 mm length.  **4 used** -- electronics_tray Raspberry Pi 4 / Pi 5 board-mount bosses (May 2026 hardware-arrival pass).  Same soldering-iron install workflow as the M3 inserts; the printed Phi 6 mm boss around each pilot leaves a 1.5 mm plastic wall, which is enough for thermal install without slumping. | [McMaster 94459A106](https://www.mcmaster.com/94459A106/) |
| 1 | M3 x 10 SHCS (`91290A114`) | M3 x 10 mm socket-head cap screw, black-oxide steel.  **10 used:** 4 battery-holder foot bolts (UP from under chassis_bottom into the holder feet's M3 inserts) + 2 switch_holster mount bolts (DOWN through the holster ear into chassis_top boss inserts) + **4 chassis_top → brass-standoff bolts (DOWN from above chassis_top into the M-F standoff female top threads on the rotated-45-deg 35-mm-radius pattern; May 2026 tray-mount fix)**. | [McMaster 91290A114](https://www.mcmaster.com/91290A114/) |
| 1 | M3 x 8 SHCS (`91290A113`) | M3 x 8 mm socket-head cap screw, black-oxide steel.  **92 used:** 72 cradle servo mounts + 4 Mega 2560 + 4 primary PCA9685 + 4 secondary PCA9685 + 4 imu_pad (MPU-6050 mount) + **4 electronics_tray → chassis_bottom tray-mount bolts (DOWN through the tray's flush-recessed cbores into M3 heat-set inserts in chassis_bottom's tray-mount bosses on the (±24.75, ±24.75) pattern; May 2026 tray-mount fix)**.  Buy a 100-pack so you have spares. | [McMaster 91290A113](https://www.mcmaster.com/91290A113/) |
| 1 | M2.5 x 8 SHCS (`91290A102`) | M2.5 x 8 mm socket-head cap screw, black-oxide steel.  4 used at the electronics_tray Raspberry Pi 4 / Pi 5 board-mount sites (May 2026 hardware-arrival pass); same stock as the DS3225 servo spline screw `91290A104` so a single 50-pack covers both roles. | [McMaster 91290A102](https://www.mcmaster.com/91290A102/) |
| 1 | M3 standoffs | M3 x 32 mm male-female brass standoffs, pack of 20.  **4 used** on the rotated-45-deg 35-mm-radius pattern (`CHASSIS_STANDOFF_HOLES_XY` = (±35, 0) and (0, ±35); May 2026 tray-mount fix moved them OFF the (±24.75, ±24.75) tray-mount pattern so they no longer conflict with the heat-set inserts in chassis_bottom's tray-mount bosses).  Earlier May 2026 audit also bumped 25 mm -> 32 mm so the 28 mm-tall battery_holder fits between the chassis plates with 4 mm headroom (CHASSIS_GAP = 32 mm in hexapod_prototype.py).  Male thread drops DOWN through chassis_bottom's Phi 3.4 mm clearance hole at the standoff pattern and is captured by an M3 nyloc nut underneath; female top accepts the M3 × 10 SHCS dropped DOWN from above chassis_top.  Re-verify lengths whenever CHASSIS_GAP changes. | [Amazon: M3 32mm standoffs male female brass](https://www.amazon.com/s?k=M3+32mm+standoffs+male+female+brass) |
| 1 | M2.5 screw pack | M2.5 x 8 mm screws, useful for horn/adapter work if the servo-included screws are bad; also the same stock that the Raspberry Pi mount uses (above). | [Amazon: M2.5 8mm screw 50 pack](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

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
  DIRECTLY into the 20 mm aluminium 25T disc horn's 4 x M3 TAPPED
  holes on a 14 mm bolt circle (June 2026 disc-horn switch, replacing
  the old plastic 4-arm X-horn + M2 self-tap scheme).  The aluminium
  IS the thread-engagement medium -- no heat-set, no self-tap into
  plastic.  M3 x 6 screws usually ship with the disc-horn 10-packs
  (Amazon B07D56FVK5); `91290A111` is the McMaster equivalent for
  spares.  See `fasteners/README.md` for the full rationale.
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
| 1 kg | PLA or PETG filament | 1.75 mm. Structural printed parts. PLA is easiest; PETG is tougher. | [Amazon: PLA filament 1.75mm 1kg](https://www.amazon.com/s?k=PLA+filament+1.75mm+1kg) |
| 250 g | TPU 95A filament | Foot pads. If you skip TPU, print feet in PLA and glue rubber tread into the cups. | [Amazon: TPU 95A filament 1.75mm](https://www.amazon.com/s?k=TPU+95A+filament+1.75mm) |
| 1 | Heat-shrink kit | Assorted small heat-shrink tubing for power wiring cleanup. | [Amazon: heat shrink tubing assorted](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |

## Strongly Recommended Tools

| Qty | Item | Why | Link |
|---:|---|---|---|
| 1 | Digital calipers | Check DS3225 body/tab dimensions before printing all six legs. | [Amazon: digital calipers](https://www.amazon.com/s?k=digital+calipers) |
| 1 | Soldering iron kit | PCA9685 boards often need header pins soldered. | [Amazon: soldering iron kit beginner](https://www.amazon.com/s?k=soldering+iron+kit+beginner) |
| 1 | Metric hex key set | M3 socket-head screws usually need a 2.5 mm hex key. | [Amazon: metric ball end hex key set](https://www.amazon.com/s?k=metric+ball+end+hex+key+set) |
| 1 | Super glue gel | Foot tread / small print fixes. | [Amazon: super glue gel](https://www.amazon.com/s?k=super+glue+gel) |
| 1 | Needle file set | Useful if a servo well or femur slot is slightly tight from FDM over-extrusion. | [Amazon: needle file set](https://www.amazon.com/s?k=needle+file+set) |

## Printed Parts

Files are in `hexapod_walker/prototype/stl_prototype/`.

| Qty | STL |
|---:|---|
| 1 | `chassis_top.stl` |
| 1 | `chassis_bottom.stl` |
| 1 | `battery_holder.stl` |
| 1 | `electronics_tray.stl` |
| 1 | `bec_cradle.stl` |
| 1 | `switch_holster.stl` |
| 6 | `coxa_bracket.stl` |
| 6 | `coxa_link.stl` |
| 6 | `femur_link.stl` |
| 6 | `tibia_link.stl` |
| 6 | `foot_pad.stl` |

Design B (May 2026) retired the printed `servo_horn_adapter` -- each
link now bolts DIRECTLY onto a purchased servo horn, so it no longer
appears in the printed-parts set.  June 2026 disc-horn switch: the
purchased horn is now a 20 mm aluminum 25T disc (Amazon B07D56FVK5),
bolted with 4 x M3 x 6 SHCS on a 14 mm bolt circle into the disc's
M3 tapped holes (replacing the old plastic 4-arm X-horn + M2 scheme).

For a one-leg test, print only:

| Qty | STL |
|---:|---|
| 1 | `coxa_bracket.stl` |
| 1 | `coxa_link.stl` |
| 1 | `femur_link.stl` |
| 1 | `tibia_link.stl` |
| 1 | `foot_pad.stl` |

## Bench Test Order

1. Buy **one DS3225** first if you want to de-risk fit.
2. Print `coxa_bracket.stl`, `coxa_link.stl`, `femur_link.stl` (the servo horn is the purchased 20 mm aluminum disc, not a printed part).
3. Confirm the DS3225 slides into all three wells and the tab holes line up.
4. Wire one DS3225 to PCA9685 channel 0 and run:

```bash
python hexapod_walker/prototype/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 0
```

5. Once fit and motion are good, buy/print the rest.

## Rough Cost

| Bucket | Estimate |
|---|---:|
| DS3225 servos, 20 total | $260-$360 |
| Pi + power + microSD | $90-$130 |
| Arduino + PCA9685 + cables | $35-$60 |
| Battery + BECs + charger + safety bag + switch | $90-$140 |
| Fasteners / standoffs / wiring consumables | $40-$70 |
| Filament | $25-$40 |
| **Total** | **~$540-$800** |

If you already own a Raspberry Pi, charger, tools, or filament, the actual
cash outlay is much lower.

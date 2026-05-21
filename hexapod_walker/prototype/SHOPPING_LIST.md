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
| 5 | `coxa_bracket.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Flange-down on bed; the well opens UP. The flange bolts to the chassis. |
| 6 | `coxa_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Hub face down on bed; the well opens UP. |
| 7 | `femur_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 5 h total | Spar's broad face on the bed (hip-pad flat). Knee cradle sticks UP with its open mouth facing DOWN — the closed cradle floor becomes a ~40×20 mm bridged ceiling, the spar prints with almost no overhangs. |
| 8 | `tibia_link.stl` | **6** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 4 h total | Flat on bed |
| 9 | `foot_pad.stl` | **6** | TPU 95A (PLA OK) | 0.25 mm | 100% (TPU) | 3 | ~ 1 h | TPU = grip; PLA = slips |

> **`servo_horn_adapter.stl` has been retired** (Design B, May 2026 -- commit
> `be06741`).  Each link now bolts DIRECTLY onto the plastic 4-arm
> X-horn that ships with the DS3225, so there is no printed adapter
> in the stack any more.  Skip this row if you have an older copy of
> this checklist.

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
| 1 (optional) | **MPU-6050 IMU breakout** | Closed-loop body-attitude control. Skippable for v1 — open-loop tripod gait works fine. | [Amazon search: "MPU-6050 module"](https://www.amazon.com/s?k=MPU-6050+module) |
| 1 | **Servo extension cables, 30 cm, 3-pin male-female, pack of 20** | The DS3225 cables are barely long enough. 18 used + 2 spare. | [Amazon search: "servo extension cable 30cm 20 pack"](https://www.amazon.com/s?k=servo+extension+cable+30cm+20+pack) |
| 1 | **Dupont jumper wire kit (M-F, F-F, M-M, 20 cm)** | I²C, power, IMU wiring. | [Amazon search: "dupont jumper wires 120 pcs"](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | **Heat-shrink assortment** | Power-side wiring tidy-up. | [Amazon search: "heat shrink tubing assorted"](https://www.amazon.com/s?k=heat+shrink+tubing+assorted) |
| 1 | **USB-A → USB-B cable, 6 ft** | Programming the Mega. | [Amazon search: "USB A to B cable Arduino"](https://www.amazon.com/s?k=USB+A+to+B+cable+Arduino) |

### B.4 Fasteners — get a kit, not individual sizes

| Qty | Part | Why | Search link |
|---:|---|---|---|
| 1 | **M3 socket-head cap screw + nut + washer assortment kit (~ 500 pieces, 6 / 8 / 10 / 12 / 16 / 20 mm lengths, A2 stainless)** | Simpler than buying lengths separately. Use 8 mm for servo tabs, 12 mm for chassis-spacer bolts, 16 mm for coxa-bracket → chassis and **for the 6 foot/tibia clevis hinge pins**, 20 mm for the rare longer reach. | [Amazon search: "M3 stainless screw kit assortment"](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 6 | **M3 × 16 mm pan-head bolts (foot hinge pins)** | One per leg: passes through the tibia clevis (3.5 mm cheek) + foot tongue (4 mm) + 5 mm gap and engages an M3 nylock nut on the far side (~ 4 mm of thread in the nut). Pan-head sits flatter against the cheek than a socket head. The M3 assortment above usually covers this if it has 16 mm + pan-head; otherwise buy this row separately. | [Amazon search: "M3 x 16 pan head stainless"](https://www.amazon.com/s?k=M3+x+16+pan+head+stainless) |
| 1 | **M3 nylon-insert (nyloc) lock nut, ~ 100 pieces** | 30 used: 24 on the coxa-bracket → chassis through-bolts and 6 on the foot-pad hinge pins. The cradle servo mounts thread into brass heat-set inserts (next row) and do **not** use a nut. | [Amazon search: "M3 nyloc lock nut 100 pack"](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | **M3 brass heat-set inserts — McMaster `94459A130`, 100-pack** | 4 per servo cradle × 3 cradles per leg × 6 legs (72) + 4 battery_holder feet + 8 electronics_tray (4 Mega + 4 primary PCA9685) + 4 electronics_tray (4 secondary PCA9685) + 2 chassis_top (switch_holster mount bosses) = **90** needed; a 100-pack gives spares. Knurled brass M3, Φ 4.0 mm pilot, Φ 5.7 mm OD, 5.0 mm length, ≈ $0.10 ea. Installed with a soldering iron at ~ 220 °C, light downward pressure, ~ 10–15 s per insert. **May 2026 "essentials" pass:** added the 2 chassis_top inserts (for the new switch_holster) and 4 more electronics_tray inserts (for the previously cable-tied secondary PCA9685). | [McMaster 94459A130](https://www.mcmaster.com/94459A130/) |
| 1 | **M2.5 brass heat-set inserts — McMaster `94459A106`, 50-pack** | 4 needed for the Raspberry Pi 4 / Pi 5 mount on the electronics_tray (the Pi's holes are M2.5 clearance, smaller than the Mega's M3 clearance, so a smaller insert is required).  Knurled brass M2.5, Φ 3.0 mm pilot, Φ 3.6 mm OD, 4.0 mm length.  Installed with the same soldering-iron technique as the M3 inserts; the printed Phi 6 mm boss around each pilot leaves a 1.5 mm plastic wall, which is enough for thermal install without slumping. | [McMaster 94459A106](https://www.mcmaster.com/94459A106/) |
| 1 | **M3 × 32 mm hex round standoffs, M-F brass, set of 20** | Sandwich the chassis plates 32 mm apart with 4 of these on the inner bolt circle. **May 2026 fix:** bumped from 25 mm to **32 mm** so the 28 mm-tall battery_holder fits between the plates with 4 mm headroom — earlier 20 / 25 mm gaps had the holder ramming through the 4 mm chassis_top deck.  Whenever `CHASSIS_GAP` in `hexapod_prototype.py` changes, this standoff length MUST change with it. | [Amazon search: "M3 32mm standoffs male female brass"](https://www.amazon.com/s?k=M3+32mm+standoffs+male+female+brass) |
| 1 | **M3 × 10 mm socket-head cap screws, A2 stainless, ~ 20 pieces** | 4 for the battery-holder feet (UP through chassis_bottom into the holder's 4 brass inserts) + 2 for the switch_holster (DOWN through the holster ear's clearance holes into chassis_top's 2 boss inserts) = **6** load-bearing.  Stock from the same M3 kit row above is fine, but listed separately because the M3x10 length isn't always in the 6/8/12/16/20 mm mix. | [Amazon search: "M3 10mm SHCS A2 stainless 100 pack"](https://www.amazon.com/s?k=M3+10mm+SHCS+A2+stainless) |
| 1 | **M3 × 8 mm socket-head cap screws (electronics_tray board bolts), ~ 20 pieces** | 12 are driven DOWN through the Mega 2560 (4) + primary PCA9685 (4) + secondary PCA9685 (4) into the M3 brass heat-set inserts in the electronics_tray bosses (May 2026 "essentials" pass); the same SKU also clamps the chassis-side standoffs through the tray's flush-recessed counterbores.  Spares are useful. | [Amazon search: "M3 8mm SHCS A2 stainless"](https://www.amazon.com/s?k=M3+8mm+SHCS+A2+stainless) |
| 1 | **M2.5 × 8 mm screws (servo horn screws + Raspberry Pi mount), 50-pack** | Comes free with the servos as self-tappers, but a 50-pack of M2.5 × 8 + M2.5 nuts is $5 and saves a trip if you strip one.  May 2026 update: 4 of these are now also load-bearing on the electronics_tray as the Raspberry Pi 4 / Pi 5 board-mount bolts (threading into the M2.5 brass heat-set inserts above). | [Amazon search: "M2.5 8mm screw 50 pack"](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

<!-- BEGIN FASTENERS (auto-generated by scripts/render_fastener_bom.py) -->

## Fasteners

Auto-derived from `fastener_registry.build_all_fastener_instances()`.
Edit the registry (not this table) and re-run `make bom-fasteners`.

| Spec | McMaster P/N | Qty | Used in |
|------|--------------|-----|---------|
| M2x8 SHCS | 91290A005 | 72 | link-to-X-horn bolts |
| M2.5x8 spline screw | 91290A104 | 18 | servo spline center screws |
| M2.5x8 SHCS into heat-set insert | 91290A102 | 4 | electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685) |
| M2.5 heat-set insert | 94459A106 | 4 | electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685) |
| M3x8 SHCS into heat-set insert | 91290A113 | 84 | cradle servo mounts (M3 SHCS into heat-set insert), electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685) |
| M3x10 SHCS | 91290A114 | 6 | battery_holder heat-set inserts, switch_holster heat-set inserts |
| M3 heat-set insert | 94459A130 | 90 | battery_holder heat-set inserts, cradle heat-set inserts, electronics_tray heat-set inserts (Mega + Pi + 2 x PCA9685), switch_holster heat-set inserts |
| M3x32 SHCS | 91290A123 | 24 | coxa-bracket-to-chassis bolts |
| M3x16 pan-head | 92010A130 | 6 | foot hinge pins |
| M3 nyloc nut | 90576A102 | 30 | coxa-bracket-to-chassis bolts, foot hinge pins |
|  |  | **338** | **total fasteners** |

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
- Link-to-X-horn bolts (72 x M2x8 SHCS / `91290A005`) self-tap into
  the plastic 4-arm X-horn's existing Phi ~ 2.0 mm M2-sized untapped
  arm holes (May 2026 fastener-spec fix: the X-horn arms are NOT
  M3-sized -- an M3 SHCS won't fit through them).  Optional thread-
  forming upgrade: McMaster `99461A340` (M2x8 thread-form for
  plastic).  See `fasteners/README.md` for the full rationale.
- Captive nyloc nuts are still used at the foot-pad hinge pins (6)
  and at the coxa-bracket-to-chassis bolts (24); both joints have
  through-hole bolts with the nut on the opposite side.
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

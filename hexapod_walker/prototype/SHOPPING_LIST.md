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
| 4 | `electronics_tray.stl` | **1** | PLA / PETG | 0.2 mm | 20% gyroid | 2 | ~ 1 h | Arduino + PCA9685 mount |
| 5 | `coxa_bracket.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Flange-down on bed; the well opens UP. The flange bolts to the chassis. |
| 6 | `coxa_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 4 h total | Hub face down on bed; the well opens UP. |
| 7 | `femur_link.stl` | **6** | PLA / PETG | 0.2 mm | 30% gyroid | 4 | ~ 5 h total | Spar's broad face on the bed (hip-pad flat). Knee cradle sticks UP with its open mouth facing DOWN — the closed cradle floor becomes a ~40×20 mm bridged ceiling, the spar prints with almost no overhangs. |
| 8 | `tibia_link.stl` | **6** | PLA / PETG | 0.2 mm | 25% gyroid | 4 | ~ 4 h total | Flat on bed |
| 9 | `foot_pad.stl` | **6** | TPU 95A (PLA OK) | 0.25 mm | 100% (TPU) | 3 | ~ 1 h | TPU = grip; PLA = slips |
| 10 | `servo_horn_adapter.stl` | **18** | PLA / PETG | 0.16 mm | 100% | 4 | ~ 1 h | Tiny, print 6 per bed × 3 batches |

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
| 1 | **Arduino Mega 2560 (clone is fine)** | Need 18 PWM channels + I²C bus. Mega has both. The Uno does *not* have enough hardware PWM, but you don't need hardware PWM because the PCA9685 is generating the PWM — so an **Arduino Nano / ESP32 / Pi Pico will also work**. Mega is the safest bet for a beginner. | [Amazon search: "Arduino Mega 2560 R3"](https://www.amazon.com/s?k=Arduino+Mega+2560+R3) |
| 2 | **PCA9685 16-channel 12-bit PWM driver (I²C)** | Two boards, daisy-chained, give you 32 PWM lines. Adafruit-clone listings are typically $4–6 each. | [Amazon search: "PCA9685 16 channel servo driver"](https://www.amazon.com/s?k=PCA9685+16+channel+servo+driver) |
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
| 1 | **M3 nylon-insert (nyloc) lock nut, ~ 100 pieces** | Use these on every joint that sees vibration, especially the coxa-bracket → chassis bolts, the 6 foot-hinge pins, and any joint inside the leg. | [Amazon search: "M3 nyloc lock nut 100 pack"](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | **M3 × 25 mm hex round standoffs, M-F brass, set of 20** | Sandwich the chassis plates 25 mm apart with 4 of these on the inner bolt circle. | [Amazon search: "M3 25mm standoffs male female brass"](https://www.amazon.com/s?k=M3+25mm+standoffs+male+female+brass) |
| 1 | **M2.5 × 8 mm screws (servo horn screws — also pre-fit)** | Comes free with the servos as self-tappers, but a 50-pack of M2.5 × 8 + M2.5 nuts is $5 and saves a trip if you strip one. | [Amazon search: "M2.5 8mm screw 50 pack"](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |

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
| Arduino Mega + 2 × PCA9685 + IMU + servo cables + jumpers | $50 |
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
5. **18 × `servo_horn_adapter.stl`** — 1 hour (single bed).
6. **`chassis_top.stl` + `chassis_bottom.stl` + `battery_holder.stl` + `electronics_tray.stl`** — 6 hours (single bed for the small parts, separate bed for each chassis plate).
7. **6 × `foot_pad.stl`** in TPU — 1 hour.

After step 3 you can start mounting the yaw servos in the coxa
brackets and verifying the bolt patterns line up — that gives you a
72-hour parallel-track between "printing the rest" and "starting
final assembly".

---

## E. Troubleshooting tips during assembly

| Symptom | Likely cause | Fix |
|---|---|---|
| Servo body won't drop into well | Slight FDM over-extrusion narrowing the cavity | Sand inside walls or scale `WELL_BODY_CL` from 0.4 → 0.6 mm and reprint |
| M3 self-tapper bottoms out before tightening | Pilot hole too tight after layer-line shrinkage | Drill out the pilot to 2.6 mm with a hand drill |
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

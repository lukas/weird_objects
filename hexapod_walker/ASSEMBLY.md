# Human-Carrying Hexapod Walker — Build Guide

> This guide describes a one-rider, six-legged walking vehicle assembled
> from the STL parts produced by [`hexapod_walker.py`](hexapod_walker.py),
> 18 industrial harmonic-drive servomotors, an aluminium-extrusion leg
> structure, and a 96 V LiFePO₄ battery system. Top speed at completion
> is a brisk crawl (~ 0.4 m/s); payload is one rider up to 110 kg.
>
> **This is a serious, large-scale project.** A finished walker masses
> ~ 280 kg (vehicle) + rider, draws kilowatts, and can pinch fingers in
> ways that do not heal. Read the [Safety](#11-safety) section first.

---

## 1. The big idea

The vehicle is six identical legs spaced 60 ° around a regular hexagonal
chassis. Each leg has the same three-DOF kinematic chain found in a real
spider or insect:

```
    chassis edge --( yaw )-- coxa --( pitch )-- femur --( pitch )-- tibia --|--- foot
                hip-yaw                  hip-pitch              knee-pitch
```

So the full vehicle has **6 × 3 = 18 actuators**.

Two design choices collapse most of the build complexity:

1. **All 18 joints use the *same* actuator part number.** Pick one
   harmonic-drive servomotor with enough torque for the worst joint
   (knee at full extension) and use it everywhere. You replace 18
   parts from one bin instead of three. Spares are a single SKU.
2. **All six legs are *mechanically identical*.** The same five STLs
   (`coxa_bracket`, `coxa_link`, `femur_link`, `tibia_link`, `foot_pad`)
   plus three motors per leg — multiplied by six — make up everything
   below the chassis. The chassis and saddle/electronics stack are
   one-offs.

The walker is a tripod-gait machine: at any instant three legs
(alternating triangles) are planted on the ground while the other
three lift, swing forward, and replant. This is statically stable
in both half-cycles, so the vehicle stays upright even if all power
fails.

---

## 2. Final specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Overall envelope (foot-to-foot, fully extended) | ~ 4.0 × 4.5 × 1.2 m |
| Standing height (rider seat) | ~ 1.3 m |
| Stride length | ~ 0.6 m |
| Cruise speed (tripod gait) | ~ 0.4 m/s (1.4 km/h, brisk crawl) |
| Climb capability | ~ 25 ° slope, 200 mm step |
| Vehicle dry mass | ~ 280 kg |
| Payload | 1 rider, ≤ 110 kg |
| Total mass on the ground | ~ 390 kg |
| Per-leg static load (tripod stance) | ~ 1.27 kN (130 kg) |
| Peak knee torque | ~ 770 N·m |
| Peak hip-pitch torque | ~ 1.4 kN·m |
| Battery | 2 × 48 V 50 Ah LiFePO₄ in series → 96 V 50 Ah |
| Continuous draw (cruise) | ~ 1.6 kW |
| Range, level ground | ~ 2 hours / ~ 3 km |
| Mounting | Step-up footrest on the +X side; rider faces +X |

Total build cost (typical, 2026 USD): **$45,000 – $110,000.** See §10.

---

## 3. STL files in `stl/`

Run `./run.sh hexapod_walker/hexapod_walker.py` to (re)generate every
file. All dimensions are in millimetres. All STLs are watertight
manifolds suitable for CAM, casting investment, or 3D-printing slicers.

### 3.1 Body parts (one of each)

| File | Envelope | Function |
|---|---|---|
| `chassis_hex.stl` | 1461 × 1344 × 121 mm | Welded square-tube hexagonal frame. The CAD models the 80 × 80 mm tubes as solid (~ 55 L); the *real* fabricated chassis uses 80 × 80 × 6 mm wall tube and weighs ~ 35 kg. Six bolt pads on the outer edges carry the hip-yaw motor brackets. |
| `chassis_top_deck.stl` | 1200 × 1386 × 12 mm | 12 mm 6061-T6 aluminum hex plate. CNC-cut from a single sheet. Lightening holes pre-drilled. Bolts to the top of `chassis_hex.stl`. |
| `saddle_mount.stl` | 320 × 220 × 238 mm | Vertical post (40 mm OD) + horizontal saddle plate. Accepts a standard motorcycle / bicycle suspension seatpost or a Crashbox-style harness. |
| `battery_box.stl` | 480 × 320 × 220 mm | Welded-aluminum or 3D-printed (Onyx/PA12-CF) box that holds two 48 V × 50 Ah LiFePO₄ packs in series. Open top — bolt-on lid is fabricated separately from a flat plate. |
| `electronics_bay.stl` | 360 × 240 × 140 mm | Sealed enclosure for the main controller, IMU, power-distribution PCB, and 18 motor-driver harnesses (cable glands modelled). |

### 3.2 Per-leg parts (print/cast 6 of each)

Each leg uses one of every part below, plus three motors. See §4 for a
manifest.

| File | Envelope | Function |
|---|---|---|
| `coxa_bracket.stl` | 210 × 224 × 220 mm | Bolts to the chassis edge bolt pad. Cradles the **hip-yaw motor**: motor body sits in the bottom plate, output flange points up. |
| `coxa_link.stl` | 320 × 130 × 210 mm | Rotates with the hip-yaw output. Carries the **hip-pitch motor** on a flat plate hanging off the arm's tangential side, axis perpendicular to the arm. |
| `femur_link.stl` | 810 × 109 × 210 mm | Thigh segment between hip-pitch output and knee-pitch motor. Box-section profile with 5 lightening holes. Hip plate clamps to the hip-pitch output flange (joint axis tangential); knee plate cradles the knee-pitch motor body. |
| `tibia_link.stl` | 954 × 107 × 210 mm | Shin segment. Knee plate clamps to the knee output flange (joint axis tangential, parallel to the hip-pitch axis — both legs swing in a single vertical plane); foot-side socket bolts to `foot_pad.stl` with 4 × M10. |
| `foot_pad.stl` | 220 × 220 × 94 mm | Compliant ground pad. The CAD STL is the **mould pattern** — the production part is cast 60-A urethane around a 6 mm steel disc with a moulded toe spike. |

> **Joint-axis convention.**  All hip-pitch and knee-pitch axes are
> parallel and run **tangential to the chassis** (perpendicular to the
> leg's outboard direction).  The motors hang on the leg's tangential
> side; their output flanges sit in the joint plane, and the femur /
> tibia spars run perpendicular to the joint axis.  This is the same
> kinematic topology as Spot, ANYmal, and most modern legged robots:
> two parallel pitch axes give a planar leg that can lift, swing, and
> plant in a single vertical plane.

### 3.3 Generic motor adapter (print 18 + spares)

| File | Envelope | Function |
|---|---|---|
| `motor_flange.stl` | 200 × 200 × 20 mm | Universal D = 200 mm flange-mount adapter ring with the 8 × M8 motor bolt circle and a 4-tab outer pattern. **Print/CNC 18** to mount each motor body to its host bracket. |

### 3.4 Visualization

| File | Envelope | Function |
|---|---|---|
| `assembly_preview.stl` | 3.95 × 4.52 × 1.14 m | All 1 + 1 + 1 + 1 + 1 + 6×5 + 1 = 35 parts placed in standing pose for visual sanity-checking. Not watertight — purely for visualization in MeshLab / Blender / Cursor's STL preview. |

---

## 4. Bill of materials

Quantities below are for **one walker, with 10 % overage on every
fastener and consumable** so a dropped bolt doesn't stop the build.
All prices are mid-2026 USD, order of magnitude only. Industrial
servomotors and batteries dominate the budget.

> Conventions used throughout this section:
> *  **SHCS** = socket-head cap screw (Allen-key drive).
> *  **A2-70** = 18-8 stainless, 700 MPa proof — fine for non-loaded
>    cosmetic / cover hardware.
> *  **10.9 ZP** = grade 10.9 alloy steel, zinc-plated — required for
>    every load-path bolt (motor mounts, coxa→chassis, foot mounts,
>    extrusion clamps).
> *  Torques are for **dry, clean threads with one drop of medium
>    threadlocker (Loctite 243)**. Reduce 25 % if you use anti-seize
>    instead.

### 4.1 Actuators (the expensive bit)

You need **18 servomotors of one part number.** The worst-loaded joint
is the hip-pitch at full extension, which sees ~ 1.4 kN·m peak.
Candidates (any one will work, listed by torque):

| Vendor | Model | Cont. torque | Peak torque | Price/each | Notes |
|---|---|---|---|---|---|
| Harmonic Drive | FHA-40C-100 | 740 N·m | 1.5 kN·m | $6 – 8 k | Pricey but bullet-proof. Hits the spec on hip joints with margin. |
| MyActuator | RMD-X12-150 | 350 N·m | 700 N·m | $1.8 – 2.5 k | Cheaper, lighter; OK at the knee, marginal at the hip — use 2 × per hip-pitch in a parallel-output gearbox if you must. |
| CubeMars | AK80-64 | 25 N·m | 120 N·m | $0.7 – 1 k | **Too small** by a factor of 10 for human carry; listed only so you don't accidentally pick one. |
| Custom | 6 kW BLDC + 1 : 80 cycloidal reducer | 600 N·m | 1.2 kN·m | $1.5 – 3 k DIY | The cost-effective DIY route. See §6.4. |

**Recommended:** 18 × Harmonic Drive **FHA-40C-100** with built-in
encoder + brake, controlled over EtherCAT.

> Whatever you pick, make sure the *flange OD ≤ 170 mm*, *bolt-circle
> diameter = 150 mm*, *output OD ≈ 100 mm*, and *axial length ≤ 130 mm*
> so it bolts straight into the geometry the STLs expect. If your motor
> deviates from those dimensions, edit the `MOTOR_*` constants near the
> top of `hexapod_walker.py` and re-run the generator.

### 4.2 Master fastener BOM

Every bolt the walker needs, counted from the STL bolt-hole patterns
plus the structural fab steps. **Buy each line at the listed quantity
(includes ~10 % overage).** Lengths are conservative — verify against
your actual extrusion thickness and motor flange depth before final
ordering.

| ID | Fastener | Grade | Use | Per-walker qty |
|---|---|---|---|---|
| **F1** | M8 × 30 mm SHCS | 10.9 ZP | Motor body & motor output flange to bracket / link end-cap (8 holes per joint face × 2 faces × 18 motors) | **320** |
| **F2** | M8 split-lock washer | Steel | One under every F1 head | **320** |
| **F3** | M10 × 90 mm SHCS, full thread | 10.9 ZP | Coxa bracket → chassis edge pad (4 per leg × 6) | **30** |
| **F4** | M10 nyloc nut, flanged | 10.9 ZP | Mate for F3 | **30** |
| **F5** | M10 flat washer (2 mm thick, OD 22) | Steel | Two per F3 (one each side) | **60** |
| **F6** | M10 × 50 mm SHCS | 10.9 ZP | Foot pad → tibia (4 per leg × 6) | **30** |
| **F7** | M10 × 120 mm SHCS, full thread | 10.9 ZP | Femur / tibia extrusion → end caps (4 per cap × 4 caps per leg × 6) | **110** |
| **F8** | M10 nyloc nut | 10.9 ZP | Mate for F7 | **110** |
| **F9** | M10 flat washer | Steel | Two per F7 | **220** |
| **F10** | M8 × 25 mm SHCS | A2-70 | Top deck → chassis (12), battery box → top deck (4), electronics bay → top deck (4) | **25** |
| **F10b** | M8 rivnut, knurled, ribbed shank, alu/steel | Steel | Installed in the **top wall of every chassis tube + every top-deck deck mount point** so F10 has something to thread into without internal access | **25** |
| **F11** | M8 nyloc nut | A2-70 | Reserved for accessories where there *is* underside access (none in the standard build) | **5** |
| **F12** | M10 × 25 mm SHCS | A2-70 | Saddle mount base → chassis hub (4) | **5** |
| **F13** | M6 × 16 mm SHCS | A2-70 | Cable carrier mounts (2 ends × 2 bolts per end × 18 carriers = 72) | **80** |
| **F14** | M6 × 20 mm SHCS | A2-70 | Pinch-point decals, harness anchors, misc cable-tie mount points | **30** |
| **F15** | M5 × 16 mm SHCS | A2-70 | Battery box lid (12), electronics bay lid (8), internal BMS / contactor mount (16), foot load cell sandwich (4 × 6 = 24) | **70** |
| **F16** | M5 nyloc nut | A2-70 | Mate for F15 (used inside electronics bay) | **20** |
| **F17** | M3 × 8 mm self-tapper | Steel | IMU, foot load cells, status LEDs | **30** |
| **F18** | M12 × 1.75 footrest stud (welded to top deck) | 8.8 ZP | Rider footpegs | **2** |

> **Total tally** (rounded to box quantities you can buy on one
> Misumi / McMaster cart): 320 × M8 × 30, 25 × M8 × 25, 25 × M8
> rivnuts, 30 × M10 × 90, 30 × M10 × 50, 110 × M10 × 120, 5 × M10 ×
> 25, 80 × M6 × 16, 30 × M6 × 20, 70 × M5 × 16, 30 × M3 × 8.

### 4.3 Fastener detail by joint

A second view of the same hardware, this time grouped by *where it
goes* so you can lay out per-leg kits before assembly.

| Joint / interface | Fasteners | Length basis |
|---|---|---|
| **Coxa bracket → chassis edge pad** (×6) | 4 × F3 (M10 × 90) + 4 × F4 nyloc + 8 × F5 washer | through 18 mm bracket flange + 40 mm pad + nut clearance |
| **Yaw motor body → coxa bracket bottom plate** (×6) | 8 × F1 (M8 × 30) + 8 × F2 lock washer | thread into motor body's tapped flange |
| **Coxa link bottom hub → yaw motor output flange** (×6) | 8 × F1 + 8 × F2 | thread into motor output |
| **Hip-pitch motor body → coxa link pitch plate** (×6) | 8 × F1 + 8 × F2 | thread into motor body |
| **Femur hip plate → hip-pitch motor output flange** (×6) | 8 × F1 + 8 × F2 | thread into motor output |
| **Knee motor body → femur knee plate** (×6) | 8 × F1 + 8 × F2 | thread into motor body |
| **Tibia knee plate → knee motor output flange** (×6) | 8 × F1 + 8 × F2 | thread into motor output |
| **Foot pad → tibia foot hub** (×6) | 4 × F6 (M10 × 50) | thread into foot pad's embedded steel disc |
| **Femur extrusion → hip end cap** (×6) | 4 × F7 (M10 × 120) + 4 × F8 + 8 × F9 | through-bolt cap + 90 mm extrusion + cap |
| **Femur extrusion → knee end cap** (×6) | 4 × F7 + 4 × F8 + 8 × F9 | same |
| **Tibia extrusion → knee end cap** (×6) | 4 × F7 + 4 × F8 + 8 × F9 | through-bolt cap + 70 mm extrusion + cap |
| **Tibia extrusion → foot end cap** (×6) | 4 × F7 + 4 × F8 + 8 × F9 | same |
| **Top deck → chassis** | 12 × F10 (down through deck into 12 × F10b rivnuts in chassis top wall) | rivnut required because the chassis is a closed welded tube |
| **Saddle mount → chassis hub** | 4 × F12 (M10 × 25) | thread into 4 × tapped M10 holes in the centre hub disc (90 mm PCD) |
| **Battery box → top deck** | 4 × F10 (through box base into 4 × F10b rivnuts in top deck) | rivnut keeps the install one-sided |
| **Battery box lid** | 12 × F15 | thread into 12 × M5 weld-nuts on the box rim |
| **Electronics bay → top deck** | 4 × F10 (through bay base into 4 × F10b rivnuts in top deck) | same |
| **Electronics bay lid** | 8 × F15 | thread into bay rim |
| **Cable carrier mounts** | 72 × F13 (2 ends per carrier × 2 bolts per end × 18 carriers) | through carrier mount-plate into link |
| **Foot load-cell sandwich** (×6) | 4 × F15 (M5 × 16) per foot | 2 anchoring the load-cell foot end to a 60 × 30 × 2 mm 6061 sandwich plate, 2 anchoring the load-cell tibia end to the tibia foot hub bottom face |
| **Misc / decals / harness** | 30 × F14, 30 × F17, 2 × F18 | as required |

**Per-leg fastener sub-total (for kitting):** 48 × F1 (motor mounting),
4 × F3, 4 × F4, 8 × F5 (chassis), 4 × F6 (foot), 16 × F7 + 16 × F8 +
32 × F9 (extrusion clamps), 6 × F13 (cable carriers).

### 4.4 Torque schedule

| Fastener | Substrate | Torque (10.9 ZP, dry + Loctite 243) |
|---|---|---|
| M5 SHCS | aluminum or steel | **5 N·m**  (3.7 ft·lb) |
| M6 SHCS | aluminum or steel | **9 N·m**  (6.6 ft·lb) |
| M8 SHCS into FHA-40C flange | hardened steel motor body | **28 N·m**  (20.7 ft·lb) |
| M8 SHCS, A2-70, decorative | aluminum | **18 N·m**  (13.3 ft·lb) |
| M10 SHCS through-bolt + nyloc | steel pad / aluminum extrusion | **56 N·m**  (41 ft·lb) |
| M10 SHCS into tapped foot disc | hardened steel | **56 N·m** |
| M12 stud (footrest) | weld | weld pre-load only |

> Use a calibrated 0–60 N·m click-type torque wrench. **Tighten every
> 8-bolt motor flange in a star pattern** (1, 5, 3, 7, 2, 6, 4, 8) in
> three passes: 30 %, 70 %, 100 % of final torque.

### 4.5 Structural — leg extrusions

The femur and tibia STLs model a solid 90 × 120 (femur) and 70 × 90
(tibia) block. The **real** parts are hollow rectangular aluminium
extrusions (6061-T6) bolted into the printed/cast end caps. Buy:

| Part | Section | Length each | Wall | Qty | Notes |
|---|---|---|---|---|---|
| Femur extrusion | 90 × 120 × 6 mm wall | 480 mm | 6 mm | 6 | 6061-T6, mill-finish |
| Tibia extrusion | 70 × 90 × 5 mm wall | 690 mm | 5 mm | 6 | 6061-T6, mill-finish |

(`FEMUR_LENGTH` and `TIBIA_LENGTH` in `hexapod_walker.py` include the
length of the hubs at each end; the bare extrusion is shorter.)

> **Drilling the extrusions for F7:** match each end-cap's 4-hole
> pattern to the extrusion using a marking gauge, drill Φ 11 mm
> through both walls, deburr inside with a 90 ° countersink so the
> through-bolt seats square on the steel washer.

### 4.6 Structural — chassis & body plates

| Part | Spec | Qty |
|---|---|---|
| 80 × 80 × 6 mm wall sq. tube, 6063-T5 aluminum (or A36 mild steel) | 6 × 693 mm perimeter (30 ° mitres) + 6 × 380 mm spokes | 1 set |
| Centre hub disc, 12 mm 6061-T6 plate | 360 mm Ø, 4 × M10 tapped at 90 mm PCD (4 holes at 45 ° offsets, 45 mm radius) for saddle mount | 1 |
| Chassis edge pads, **40 × 200 × 112 mm**, 6061-T6 plate | Drilled 4 × Φ 11 mm at the 160 × 60 mm pattern from `chassis_hex.stl`, drilled radially through the 40 mm thickness | 6 |
| Top deck, 12 mm 6061-T6 plate | CNC-cut to `chassis_top_deck.stl` outline + 12 × Φ 9 mm mount holes | 1 |
| Battery box body, 4 mm 5052-H32 aluminum sheet | TIG-welded box per `battery_box.stl` | 1 |
| Battery box lid, 8 mm 6061-T6 plate | Drilled 12 × Φ 5.3 mm | 1 |
| Electronics bay body | 3 mm 5052 sheet, TIG-welded, IP-65 gaskets | 1 |

### 4.7 End-cap cast/printed parts (the STL files)

Six per per-leg STL plus one per body STL plus 18 universal flanges.
Two sourcing options:

* **CNC-machined 6061-T6 aluminum** (recommended for the load-bearing
  links: `coxa_bracket`, `coxa_link`, `femur_link`, `tibia_link`).
  Send the STL to a CNC shop (Xometry, PCBWay Mechanical, Protolabs,
  any local 3-axis aluminum shop). Expect $80 – $300 per part for
  CNC, $1,800 – $5,400 for the full link set × 6.

* **MJF or SLS PA12-CF nylon** (acceptable for `coxa_bracket`,
  `chassis_top_deck`, `saddle_mount`, `battery_box`,
  `electronics_bay`, `foot_pad` mould patterns, and the
  `motor_flange` adapters). $20 – $80 per part. The femur and
  tibia links should *not* be 3D-printed — the bending loads are too
  high. CNC them.

### 4.8 Battery system

| Part | Spec | Qty |
|---|---|---|
| LiFePO₄ pack | 48 V (16S) × 50 Ah / 2.4 kWh, with integrated 100 A BMS, M8 stud terminals | 2 |
| Inter-pack series link | 4 AWG (21 mm² Cu) tinned, double-crimp ring lugs (M8) | 1 (300 mm) |
| Series-link inline fuse | 100 A class-T fuse + holder | 1 |
| Main contactor | Tyco / TE EV200AAANA, 96 V DC, 500 A make / 2 kA break | 1 |
| Pre-charge resistor | 50 Ω, 50 W wire-wound | 1 |
| Pre-charge relay | 96 V DC coil, 5 A | 1 |
| E-stop button (handlebar) | DPST mushroom, latching, 10 A | 1 |
| Bus bleed LED + 100 kΩ resistor | green panel-mount, "BUS LIVE" | 1 |
| Power cable, controller side | 8 AWG silicone, 2 m | 2 m |
| Power cable, motor side | 12 AWG silicone, 18 × 2.5 m | 45 m |
| XT90 / Anderson SB175 main connector | as bus disconnect | 1 |
| Battery hold-down strap | 50 mm wide nylon, ratchet | 2 |

### 4.9 Motor controllers, compute, sensors

| Part | Spec | Qty |
|---|---|---|
| Motor drivers | 18 × matched to your motor (Elmo Gold Twitter for FHA, ODrive Pro v3 for the DIY route) | 18 |
| 24 V house battery | 24 V × 7 Ah AGM, for logic + brakes | 1 |
| 24 V → 12 V DC-DC | 200 W, for IPC fans / displays | 1 |
| Main controller | Industrial PC or NVIDIA Jetson Orin Nano, ROS 2 + EtherCAT master | 1 |
| 9-DOF IMU | Bosch BMI088 + magnetometer, e.g. VectorNav VN-100 | 1 |
| Foot force sensors | 200 kgf load cell per foot, e.g. SparkFun TAL220 | 6 |
| Foot load-cell sandwich plate | 60 × 30 × 2 mm 6061 (CNC scrap or hand-cut) | 6 |
| Foot load-cell amplifier | HX711 breakout | 6 |
| Joystick + small UI screen | Any USB joystick + 7" touchscreen | 1 |
| Status LEDs (RGB strip on top deck) | WS2812B, 60 LED/m, 1 m | 1 |

### 4.10 Wiring harness manifest

Cable lengths assume the controller + drive stack lives in the
`electronics_bay.stl` on the top deck (centre, +X side). Each cable
allows ~150 mm of slack at every joint for the cable carrier loop.

| Cable | Spec | Length each | Qty | Total |
|---|---|---|---|---|
| **Yaw motor power** | 12 AWG 4-conductor (3-phase + PE) shielded | 1.2 m | 6 | 7.2 m |
| **Hip-pitch motor power** | 12 AWG 4-conductor shielded | 1.8 m | 6 | 10.8 m |
| **Knee motor power** | 12 AWG 4-conductor shielded | 2.5 m | 6 | 15.0 m |
| **EtherCAT (M12 X-coded ↔ M12 X-coded)** | Cat-6A SF/UTP, 4-pair | 0.8 – 1.2 m | 18 | ~ 18 m |
| **24 V brake-release line, per joint** | 22 AWG twisted pair | 1.5 m | 18 | 27 m |
| **IMU cable** | RS-232 / USB, shielded | 1.5 m | 1 | 1.5 m |
| **Foot load cell** | 4-conductor + shield | 2.6 m (yes, all the way down to the foot) | 6 | 15.6 m |
| **Cable carrier** (igus E-chain or equivalent) | 30 mm × 50 mm cross-section, 90 ° rotation | 250 mm | 18 | 4.5 m |
| **PG-13.5 cable gland** | electronics bay outer wall | — | 7 | — |
| **PG-9 cable gland** | foot load-cell entry into tibia hub | — | 6 | — |
| **Heat-shrink, ¾"** | Black, 3 : 1 adhesive-lined | — | 5 m | — |
| **Insulated ring lugs, 8 AWG / M8** | for battery + contactor | — | 12 | — |
| **Insulated ring lugs, 12 AWG / M5** | for motor power | — | 144 (4/cable × 36) | — |
| **Heat-shrink ferrules, 12 AWG** | for driver-side terminations | — | 200 | — |
| **Spiral cable wrap, 25 mm** | Black, for trunk cable bundle | — | 4 m | — |

### 4.11 Saddle / rider attachment

| Part | Spec | Qty |
|---|---|---|
| Suspension seatpost | 30.9 / 31.6 mm, 100 mm travel | 1 |
| Bicycle / motorcycle saddle | Any | 1 |
| 4-point harness | Automotive 4-point | 1 |
| Footrests (rider's feet) | Off-the-shelf motorbike pegs welded to the top deck | 2 |
| High-vis decals & pinch-point stickers | Yellow vinyl, 50 mm | 24 |

### 4.12 Tools required

This is the *complete* kit for one builder. You can substitute
brand-equivalents but don't skip the torque wrench or the load-rated
hoist — both are critical for safety.

| Tool | Purpose | Notes |
|---|---|---|
| **Calibrated torque wrench, 5 – 60 N·m, 3/8" drive** | Every M5–M10 fastener | Click-type minimum; digital preferred |
| **Calibrated torque wrench, 20 – 200 N·m, 1/2" drive** | Reserve for chassis welds + lifting eyes (none in std build) | optional |
| **Allen / hex bit set, metric, ball-end, 3 – 10 mm** | All SHCS heads | 10.9 SHCS bit must be hardened |
| **Hex socket set, 8 – 17 mm, 3/8" drive** | Nyloc nuts | |
| **TIG welder + Ar gas + 4043 filler** | Chassis fab + battery box | Or job out per §6.1 |
| **Bandsaw or chop-saw with carbide blade** | Aluminum extrusion + tube | |
| **Drill press with cross-vise** | Chassis pad holes, extrusion through-holes | |
| **Step drill bits, Φ 9 + Φ 11** | M8 / M10 clearance | |
| **Tap set, M3 → M10, with handle** | Re-tap any blocked motor flange holes | |
| **Rivnut tool (handheld, M5 / M6 / M8 jaws)** | Setting F10b in chassis tube + top deck | Astro 1442 or equivalent |
| **Engine hoist or shop crane, 500 kg** | Lifting the chassis with all 6 legs (~ 220 kg) | Hire one for $50/day |
| **Hydraulic table lift, 1.2 m max, 300 kg cap.** | Sub-assembly bench | One welding-deck table works too |
| **Calibrated multimeter (CAT III 1000 V)** | Battery / contactor commissioning | Never skip |
| **DC clamp meter, 0 – 200 A** | Cruise-current measurements | |
| **Insulated screwdriver set (1000 V rated)** | Battery work | |
| **Crimp tool for ring lugs (8 AWG)** | Hammer-crimp or hydraulic | |
| **Crimp tool for ferrules (12 AWG)** | Ratcheting four-indent | |
| **Heat gun** | Heat-shrink termination | |
| **Cable cutter (8 AWG capable)** | Flush cuts for power cables | |
| **Marking gauge + scribe** | Hole transfer from STL → extrusion | |
| **Calipers, 200 mm, 0.01 mm** | Verify CNC parts on receipt | |
| **Spirit level, 600 mm, magnetic** | Chassis welding flatness | |
| **Pinch-point safety blocks (wood)** | Prop legs for service | 6 × 200 × 200 × 200 mm |
| **Saw-horses, 600 mm tall, 250 kg cap.** | Service stand for the walker | 4 |

### 4.13 Consumables

| Item | Spec | Qty |
|---|---|---|
| Loctite 243 (medium thread-locker) | Blue | 50 mL |
| Loctite 271 (high-strength thread-locker) | Red, only on F7 extrusion through-bolts | 50 mL |
| Anti-seize compound | Copper-based | 100 g tube |
| Dielectric grease | Silicone | 100 g tube |
| Cable ties, 200 mm × 4.8 mm UV-stable | Black | 200 |
| Adhesive cable-tie mounts | 25 mm sq | 50 |
| Self-fusing silicone tape, 25 mm | for connector strain relief | 1 roll |
| Powder-coat / paint, satin black | Chassis topcoat | 1 L |
| Etch primer | Pre-paint on bare aluminum | 0.5 L |
| Isopropyl alcohol, 99 % | Surface prep | 1 L |
| Lint-free wipes | Blue shop towel | 1 box |
| Latex gloves | Box of 100 | 2 boxes |
| Safety glasses | Polycarbonate | 2 |
| Welding shield + gloves | Auto-darkening | 1 set |
| First-aid kit | Standard 50-piece | 1 |
| Class C fire extinguisher | 5 lb minimum, lithium-rated if possible | 1 |

---

## 5. Mechanical design rationale

### 5.1 Leg-link length ratios

```
    coxa  : femur : tibia  =  150 : 600 : 800  mm
                           ≈  1   : 4   : 5
```

Two reasons for the long tibia:

1. **Knee torque is the limiting joint.** Knee torque scales linearly
   with the *horizontal* moment arm of the foot from the knee, which
   is dominated by tibia length × cos(tibia pitch). A long tibia
   working at a steep angle (knee folded, tibia near vertical) gives
   the same foot reach as a short tibia at a shallow angle, but with
   far less knee torque.
2. **Step height.** A long, near-vertical tibia at the knee can lift
   the foot up to ~ 250 mm clear of the ground in the swing phase
   without the femur having to pitch up dramatically — letting the
   walker step over typical ground obstacles.

### 5.2 Joint torque budget

Worst-case static (one-leg / fully-extended) loads:

```
    rider + vehicle  ≈  390 kg     →  3.83 kN total weight
    tripod gait shares this between 3 legs  →  1.27 kN per leg

    standing pose   femur pitch = -25°    tibia pitch (rel. femur) = +60°
                    => tibia is +35° below horizontal
                    => foot is 0.66 m horizontal + 0.46 m vertical from knee
                    => foot is 1.20 m horizontal from hip-pitch axis

    τ_knee  ≈  1.27 kN × 0.66 m   ≈    840 N·m   (+10 % shock = 920)
    τ_hip   ≈  1.27 kN × 1.20 m   ≈   1520 N·m   (+10 % shock = 1670)
    τ_yaw   ≈  20 N·m  (only friction + lateral imbalance — small)
```

Pick a motor with **continuous torque ≥ τ_max / 2** so it can sustain the
load indefinitely without thermal limits, and **peak torque ≥ 1.5 ×
τ_max** for the duty-cycle stride peaks. Either the FHA-40C-100 or a 6
kW BLDC + 80 : 1 cycloidal reducer comfortably hits this.

### 5.3 Weight breakdown

| System | Mass |
|---|---|
| Chassis (welded tube + top deck + pads) | ~ 35 kg |
| 6 × leg structural (femur + tibia + foot + brackets) | ~ 60 kg |
| 18 × motor + driver | ~ 130 kg (FHA-40C-100 is ~ 7 kg each) |
| Battery (2 × 48 V × 50 Ah LiFePO₄) | ~ 32 kg |
| Electronics, wiring, fasteners, paint | ~ 10 kg |
| Saddle + harness | ~ 8 kg |
| Hydraulics / soft parts (foot urethane) | ~ 5 kg |
| **Total dry** | **~ 280 kg** |
| Rider | + 110 kg max |
| **Total walking** | **≈ 390 kg** |

---

## 6. Manufacturing the parts

### 6.1 Chassis (welded square tube)

This is a manufacturing summary; for the bolt-by-bolt build sequence
see §8.2. Use 80 × 80 × 6 mm wall 6063-T5 aluminum or A36 mild steel:

1. Lay out a regular hexagon, flat-to-flat = 1200 mm, on a flat-deck
   welding table.
2. Cut **6 perimeter pieces to 693 mm** (the side length of a regular
   hexagon with 1200 mm flat-to-flat) with 30 ° mitres. Tack-weld
   into a hexagon.
3. Cut **6 spokes to 380 mm** (apothem 600 − hub radius 180 − 2 ×
   tube half-thickness 40). Tack weld each spoke from the centre hub
   disc out to the midpoint of each perimeter tube.
4. Insert and weld the **6 mounting pads** (cast or CNC-cut **40 ×
   200 × 112 mm** 6061-T6 plates) flush with the perimeter outboard
   face.
5. Drill the four Φ 11 mm M10 clearance holes through each pad in a
   **160 × 60 mm** pattern (`hexapod_walker.py` already cuts these
   holes in the STL — use it as a drill template).
6. Powder-coat or anodize.

Compare your finished welded chassis against `chassis_hex.stl` —
overall envelope and bolt-pad locations should match within ± 2 mm.

### 6.2 Femur and tibia links

These are the highest-stress parts on the vehicle. Don't print them.

1. Cut the aluminium extrusion to length (480 mm femur, 690 mm tibia
   — the rest of the STL length is the end caps).
2. CNC-machine the **end caps** from 6061-T6 plate, using
   `femur_link.stl` and `tibia_link.stl` as the input geometry. Each
   STL already includes the bolt-circle bores and clearance holes for
   the harmonic drive output flange.
3. Bolt the end caps to the extrusion with 4 × M10 through-bolts per
   end (or weld; bolting is preferred so the parts can be replaced
   after a fall).

### 6.3 Coxa bracket and coxa link

CNC from 6061-T6, or — for a more sculptural finish — investment-cast in
A356 aluminum (PCBWay Casting / Xometry / a local foundry will quote
from the STL directly). Min wall in the cast is 6 mm; the brackets are
modelled with 12 – 18 mm walls so they cast easily.

### 6.4 Cycloidal reducer (DIY motor route)

If you went with the cheap-motor route — 6 kW BLDC + cycloidal
reducer — you'll also need to fabricate the reducer. This is a
substantial mini-project on its own. See James Bruton's open-source
[CycloidDrive](https://github.com/XRobots/CycloidalDrive) repository or
the Skyentific MagicaCS series on YouTube.

### 6.5 Foot pad

The STL is the **mould pattern**, not the part. Production process:

1. 3D-print `foot_pad.stl` in PLA. Smooth with filler primer.
2. Wrap the printed pattern in 0.5 mm wax sheet.
3. Pour a 2-part silicone (e.g. Smooth-On Mold Star 30) around it in a
   wooden boxing.
4. Once cured, remove the pattern and pour the foot:
   * insert a 6 mm × 200 mm steel disc (CNC-cut, with **four
     M10 × 1.5 *tapped* holes** on the 36 × 36 mm pattern from
     `foot_pad.stl` — *not* clearance holes, the foot bolts F6
     thread directly into the disc from above),
   * pour 60-A urethane (Smooth-On VytaFlex 60) around it.
5. Demould after 24 h. Bolt to the tibia (see §8.3.3).

---

## 7. Electronics & control architecture

```
                   +-----------------------+
                   |  Controller (Jetson) |
                   |  ROS 2 / EtherCAT M  |
                   +-----------+----------+
                               |
                  EtherCAT or CAN bus (ring topology)
                               |
       +-------+-------+-------+-------+-------+-------+
       |       |       |       |       |       |       |
     leg 0  leg 1  leg 2  leg 3  leg 4  leg 5  ... + IMU + foot sensors
        |
        |  one slave per joint:
        |  yaw drive -> pitch drive -> knee drive
        |
        v
     joint motor (encoder + brake + temp)
```

* **Main loop** runs at 200 Hz on the Jetson. Each iteration:
  1. Read IMU + 6 foot loads + 18 joint angles.
  2. Run state estimation (chassis pose).
  3. Run the gait scheduler (which legs are stance vs swing).
  4. Run inverse kinematics per leg → 18 target angles.
  5. Send target angles + feed-forward torques to the EtherCAT
     drives.

* **Joint drives** close their own current/torque/position loops at
  10 kHz. Drives like Elmo Gold Twitter or the harmonic-drive built-in
  servo packs handle this natively.

* **E-stop** drops the main contactor *and* writes a "freewheel" flag
  to the EtherCAT master so the drives release their brakes only after
  the chassis is detected to be stationary on the ground (otherwise
  it'd collapse on the rider's legs).

### 7.1 Software stack

* ROS 2 Iron / Jazzy
* `ros2_control` for the joint drivers
* `champ` or your own gait scheduler (champ is hexapod-friendly even
  though it's named after a quadruped)
* `robot_state_publisher` consuming a URDF generated from this same
  geometry — see `urdf/hexapod_walker.urdf.xacro` (TODO; not yet in
  this repo)

---

## 8. Assembly sequence

Allow **~ 60 hours of skilled labour for a first build**, half that
for a second. The sequence below is intentionally bolt-by-bolt: every
step gives the fastener ID from §4.2, the count, and the torque. If
you've kitted the per-leg bags in §4.3 you'll always know what comes
next.

> **Before you start:**
> *  Receive every part. Tick off the BOM in §4 line-by-line.
> *  Verify CNC parts with calipers — every motor bore should accept
>    the actual motor with ≤ 0.1 mm gap, every M10 hole should clear
>    a Φ 10.5 mm pin gauge.
> *  Bag fasteners *per joint* using zip-lock pouches labelled
>    "Leg N — yaw / hip-pitch / knee / foot / extrusion". A single
>    leg's hardware fits in 7 bags.
> *  Set up a 1.2 m × 2.0 m flat welding table as the assembly bench.
> *  Read §11 (Safety) end-to-end. Specifically: have a class-C
>    extinguisher within arm's reach during all battery work, and
>    *never* lift a leg sub-assembly without a second person.

### 8.1 Phase 1 — pre-assembly checklist  (~ 4 h)

| Check | Tool | Pass criterion |
|---|---|---|
| Each motor spins freely by hand | bare hands | No grinding, < 0.05 N·m breakaway |
| Each motor's 8 × M8 flange holes accept F1 SHCS | M8 SHCS | Threads start clean, no chip residue |
| Each `coxa_bracket.stl` chassis-pad pattern transfers to the chassis pad with ≤ 0.5 mm error | scribe + calipers | 4 holes line up |
| Each end-cap bore matches its motor flange OD with ≤ 0.1 mm gap | calipers | Slip-fit, no rocking |
| Femur / tibia extrusions cut to 480 / 690 mm ± 1 mm | tape measure | length tolerance |
| Each foot pad has 4 × M10 tapped holes in its embedded steel disc | M10 SHCS | threads start clean |
| EtherCAT cable inventory matches §4.10 (18 motor cables + carriers) | visual | every cable labelled |
| Battery packs charge to 54.6 V each at rest | DMM | matches 16S LiFePO₄ HV |

Sign and date this table on the build log before proceeding.

### 8.2 Phase 2A — chassis fabrication  (~ 8 h, weld + paint)

This subsection assumes you're welding the chassis in-house. If you've
job'd it out (perfectly fine — most local fab shops will quote ~$1,200
from `chassis_hex.stl`), skip to step 12.

1.  Print `chassis_hex.stl` 1 : 1 on a tiled plotter as a layout aid.
2.  Cut **6 × 693 mm** of 80 × 80 × 6 mm sq. tube with **30 ° mitres**
    on each end (perimeter members). 693 mm is the side length of a
    regular hexagon with 1200 mm flat-to-flat — verify with the
    plotter print.
3.  Cut **6 × 380 mm** of the same stock with **square cuts**
    (spokes). 380 mm = apothem (600) − hub radius (180) − 2 × tube
    half-thickness (40), so the spokes butt-weld between the hub OD
    and the perimeter inboard wall.
4.  Cut **1 × Φ 360 mm × 12 mm** disc as the centre hub from
    6061-T6 plate (waterjet or CNC). Drill 4 × M10 × 1.5 tapped
    holes on a **90 mm PCD** (4 holes at 45 ° offsets, 45 mm from
    centre) — these become the saddle-mount bolts F12 later.
5.  Tack-weld the 6 perimeter pieces into a regular hexagon on a flat
    welding deck, flat-to-flat = 1200 mm. Verify with diagonals (any
    two opposite-vertex distances should match within ± 1 mm).
6.  Tack-weld the centre hub disc co-planar with the bottom face of
    the perimeter ring, then tack the 6 spokes from hub OD to each
    perimeter midpoint inboard wall.
7.  CNC-cut (or saw + drill) the **6 chassis edge pads**: **40 × 200
    × 112 mm** 6061-T6 plate, each drilled with 4 × Φ 11 mm clearance
    holes **radially through the 40 mm thickness** in the 160 × 60 mm
    rectangle pattern (160 in Y, 60 in Z; use `chassis_hex.stl` pads
    as the drill template). Each pad sits with its broad face flush
    to the perimeter outboard surface, centred on the perimeter
    midpoint.
8.  Tack-weld each pad to the perimeter tube. **The 4 holes must end
    up exactly aligned with the bracket's chassis-side bolt pattern**
    — clamp the bracket itself onto each pad as a fixture before
    welding.
9.  Inspect for square. Final-weld every joint with full-penetration
    fillet (4 mm leg). Allow to cool naturally — do *not* quench,
    you'll warp the 6063 alloy.
10. Drill **12 × Φ 11.5 mm holes for M8 rivnuts (F10b)** in the
    *top wall of the chassis tube only* — 6 at the perimeter
    midpoints and 6 at the spoke crossings (use `chassis_top_deck.stl`
    as a transfer template; the 12 hole positions are where the deck
    will sit on the tubes). Set one F10b rivnut in each hole with a
    handheld rivnut tool. The rivnut gives F10 (M8 × 25 SHCS) a
    threaded grip without needing access to the inside of the closed
    chassis tube.
11. Etch-prime + powder-coat the chassis (satin black is forgiving of
    weld discolouration). **Mask the rivnut threads and the chassis
    edge pad faces** before painting.
12. **Acceptance:** with the chassis on a flat deck, each chassis pad
    bolt-hole centroid should sit at the same height ± 1.5 mm. If a
    pad is off, shim with a precision washer at install.

### 8.3 Phase 2B — leg sub-assembly  (~ 3 h per leg, ×6 = 18 h)

Repeat this entire subsection **6 times**, once per leg. Each leg
finishes as a free-standing sub-assembly: chassis bracket + yaw motor
+ coxa link + hip-pitch motor + femur + knee motor + tibia + foot,
ready to bolt onto the chassis. Use a sturdy bench; a finished leg
masses ~ 36 kg.

> **How `femur_link.stl` and `tibia_link.stl` map onto the real
> parts.**  Each STL is one continuous solid for visualization, but
> the *physical* leg is **3 separate parts**: a hollow rectangular
> aluminium extrusion (the spar) plus **two CNC-machined end caps**
> (one at each end of the spar). Each end cap consists of:
>
> 1.  **The joint plate** — the 200 mm OD disc you can see at each
>     end of the STL, perpendicular to the joint (Y) axis, with the
>     8 × M8 motor bolt circle and the central output-bore.
> 2.  **A 60 mm boot sleeve** — *not shown in the STL*. Cut from the
>     same 6061-T6 plate stock as the joint plate, this is a
>     rectangular sleeve sized to slide 60 mm onto the end of the
>     extrusion (90 × 120 mm OD for femur, 70 × 90 mm OD for tibia,
>     with a 6 mm wall sleeve so the inside dimension matches the
>     extrusion's outside). The sleeve and joint plate are CNC'd as
>     one piece, or welded into one part before machining.
>
> The 4 × F7 (M10 × 120) through-bolts pass *radially* through the
> sleeve + extrusion + opposite sleeve wall. So **drill 4 × Φ 11 mm
> holes through both walls of the extrusion** in the Y direction
> (femur: through the 90 mm dimension; tibia: through the 70 mm
> dimension), positioned to match the corresponding holes you'll
> drill in the sleeve.

#### 8.3.1 Build the femur extrusion sub-assembly

1.  Lay the femur extrusion (480 mm of 90 × 120 × 6 mm tube) on the
    bench. Slide on the **hip end cap** (a CNC `femur_link.stl`
    half-cap, hip side) over one end.
2.  Mark the 4 × Φ 11 mm through-hole positions from the cap onto
    the extrusion using a scribe; remove the cap; drill through both
    walls of the extrusion on a drill press with a Φ 11 mm bit.
    Deburr inside.
3.  Re-fit the cap. Insert **4 × F7 (M10 × 120)** through the cap +
    extrusion + opposite cap wall, with **2 × F9 washers** under the
    head and the nyloc, and **4 × F8 (M10 nyloc)** on the far side.
    Apply Loctite 271 (red) to threads.
4.  **Torque to 56 N·m** in a star pattern.
5.  Repeat steps 1–4 for the **knee end cap** at the other end of the
    femur extrusion.
6.  Result: assembled femur link, knee plate facing outboard, hip
    plate facing inboard. **Total fasteners used so far on this leg:
    8 × F7 + 8 × F8 + 16 × F9.**

#### 8.3.2 Build the tibia extrusion sub-assembly

7.  Repeat 8.3.1 with the **tibia extrusion** (690 mm of 70 × 90 × 5
    mm tube), the **knee end cap** (`tibia_link.stl` knee side),
    and the **foot end cap** (`tibia_link.stl` foot side).
8.  Same hardware: **8 × F7 + 8 × F8 + 16 × F9**, Loctite 271, 56 N·m.
9.  Result: assembled tibia link with knee and foot plates.

#### 8.3.3 Mount the foot load cell + foot pad to the tibia

The foot pad sandwich, bottom-up, is: **urethane pad → embedded steel
disc (M10-tapped) → load cell sandwich plate → load cell → tibia foot
hub.** The load cell measures the compressive force between the
tibia and the foot pad — this is what tells the controller when each
foot is on the ground.

10. **Foot load cell prep.** Bolt the foot-side of each TAL220 (or
    equivalent) load cell to a small **2 mm × 60 × 30 mm 6061
    sandwich plate** with the 2 × M5 holes the load cell ships with;
    bolt the tibia-side end of the load cell to the **bottom face of
    the tibia foot hub** with another 2 × M5. Centre the load cell
    on the 4 × F6 bolt pattern so the M10 bolts pass on either side
    of it, not through it.
11. With the tibia foot end facing up, place the **foot pad** (cast
    urethane production part) over the load cell sandwich plate so
    the embedded steel disc's 4 tapped M10 holes line up with the
    tibia's Φ 11 mm clearance holes. The load cell + sandwich plate
    sit in the small gap between the foot pad's top face and the
    tibia hub.
12. Insert **4 × F6 (M10 × 50)** down through the tibia hub, past
    the load cell on either side, into the steel disc's tapped
    holes. Loctite 243.
13. **Torque to 56 N·m** in a cross pattern. The load cell is now
    pre-loaded by the M10 bolts — its zero offset will be captured
    in §8.9 commissioning.
14. Route the load cell's 4-conductor cable up through the tibia's
    PG-9 cable gland (entering the tibia hub side wall) and leave
    the bay-side end coiled at the knee for now — wired up properly
    in §8.7 step 4.

#### 8.3.4 Mount the knee motor (motor #2 of this leg)

> **Bolt-direction convention used in §8.3.4–§8.3.6:** every F1 SHCS
> always passes **through the link plate** (which has Φ 9 mm
> *clearance* holes) and **threads into the motor flange** (which
> has tapped M8 holes). So the head of every F1 sits against the
> link plate, the threads bite the motor flange. With the link
> plate-down and the motor sitting on top of it, you drive each F1
> from *underneath* the bench (lay the link on edge or use a hole in
> the bench top). With the motor-down and the link on top of it,
> drive each F1 from above. Either is correct as long as **the head
> is always on the link side**.

15. Place the femur link knee-plate-up on the bench, knee plate
    horizontal. Set the **knee motor** body-down onto the plate so
    the **motor body's mounting flange** mates flat to the knee
    plate, and the motor's 8 tapped M8 holes align with the plate's
    8 Φ 9 mm clearance bolt circle.
16. Insert **8 × F1 (M8 × 30 SHCS)** through the plate **upward into
    the motor body flange.** A drop of Loctite 243 on each thread.
    **One F2 (split-lock) under every head.**
17. **Torque to 28 N·m in a star pattern (1, 5, 3, 7, 2, 6, 4, 8) in
    three passes (30 / 70 / 100 %).**
18. Cradle the femur in a soft-jaw vice. Pick up the tibia
    sub-assembly and align its knee plate against the knee motor's
    output flange (now sticking up from the femur). The output
    flange's 8 tapped holes line up with the tibia knee plate's 8
    Φ 9 mm clearance bolt circle.
19. Insert **8 × F1 + 8 × F2** down through the tibia plate **into
    the motor output flange.** Loctite 243.
20. **Torque to 28 N·m in star pattern.**
21. Verify the knee rotates freely through its full ± 70 ° range
    when you back-drive it by hand (motor power off, brake released
    by tapping the brake-release pin to GND with the 24 V house
    battery if your motor needs it). **No binding.**

#### 8.3.5 Mount the hip-pitch motor (motor #1 of this leg)

22. Build the coxa link sub-assembly first. Set `coxa_link.stl`
    (CNC 6061-T6 part) on the bench, pitch plate horizontal,
    pitch-plate face up.
23. Set the **hip-pitch motor** body-down on the pitch plate. **8 ×
    F1 + 8 × F2**, Loctite 243, **28 N·m star pattern.** The
    motor's output flange now sticks up from the coxa link.
24. Pick up the femur sub-assembly (which now carries the knee motor
    + tibia + foot — about 22 kg). With a helper, lower it so the
    femur's hip plate sits flat on top of the hip-pitch motor's
    output flange. The femur's spar extends sideways from the coxa
    link, parallel to the bench.
25. Insert **8 × F1 + 8 × F2** down through the femur hip plate
    **into the motor output flange.** Loctite 243.
26. **Torque to 28 N·m in star pattern.**
27. Back-drive the hip-pitch joint by hand through ± 60 °. No
    binding.

#### 8.3.6 Mount the yaw motor and coxa bracket

> **Motor orientation assumption:** the standard FHA-40C-100 has its
> mounting flange on the **rear** (motor body) face, and its rotating
> output protrudes from the **front** face. Step 28 below puts the
> rear flange on the *bottom* plate of the bracket cradle, with the
> output protruding *upward* through the cradle's top bore. If your
> motor instead has the flange on the front (output side) — common
> for ODrive / MyActuator — flip the orientation: motor flange on
> the *top* plate of the cradle, body hanging *down* into the
> cradle. The 8-bolt circle is the same either way.

28. Set the **coxa bracket** (`coxa_bracket.stl`) bottom-plate-up on
    the bench. Drop the **yaw motor** rear-flange-down into the
    cradle so its 8-hole flange aligns with the bottom plate's 8
    Φ 9 mm clearance bolt circle, and its output rotor points *up*
    through the bracket's top bore.
29. Insert **8 × F1 + 8 × F2** up through the bracket bottom plate
    **into the motor rear flange.** Loctite 243.
30. **Torque to 28 N·m in star pattern.**
31. Pick up the coxa link + femur + everything below sub-assembly
    (~ 30 kg — get a helper or a small hoist). Present the coxa
    link's bottom hub against the yaw motor's output flange on top
    of the bracket. The hub's 8 Φ 9 mm clearance bolt circle aligns
    with the output flange's 8 tapped holes.
32. Insert **8 × F1 + 8 × F2** down through the coxa hub **into the
    output flange.** Loctite 243.
33. **Torque to 28 N·m in star pattern.**
34. Back-drive the yaw joint by hand through ± 45 °. No binding.

#### 8.3.7 Per-leg fastener tally

You should have used, per leg:

* **48 × F1 (M8 × 30)** + **48 × F2 (lock washer)** — motor mounts
* **16 × F7 (M10 × 120)** + **16 × F8 (nyloc)** + **32 × F9 (washer)**
  — extrusion through-bolts
* **4 × F6 (M10 × 50)** — foot pad

If your bag is empty and you have no leftovers, you're done. If you
have leftovers — stop and figure out which joint is missing fasteners
*before* the leg goes on the chassis.

#### 8.3.8 Pre-flight bench check (before installing on chassis)

35. With the leg lying horizontal on the bench, manually flex each of
    the 3 joints through its full range. Listen for any tick or grind.
36. Mark each joint's "zero" position with a paint pen so you can
    verify encoder zero during commissioning.
37. Tag the leg with its leg number (0–5) and which side it goes on.

### 8.4 Phase 2C — battery pack sub-assembly  (~ 4 h, electrician
required)

> Battery work below 60 V DC is generally OK for a non-electrician;
> the inter-pack series link in this section creates a 96 V DC bus
> that **can kill you on contact**. Have an electrician present.

1.  Assemble the **battery box** body per `battery_box.stl`
    (TIG-welded 4 mm 5052 sheet, or order as a sheet-metal job).
2.  Mount **2 × 48 V × 50 Ah LiFePO₄ packs** inside, separated by 5
    mm closed-cell EPDM foam.
3.  Strap each pack down with one of the **2 × ratchet straps**.
4.  Wire the **inter-pack series link** (one 4 AWG M8-ringed cable
    with the **100 A class-T fuse** in line). Torque the M8 stud
    nuts to **15 N·m**.
5.  Verify with a meter: **bus voltage at rest = 96 ± 2 V DC.**
6.  Bring a **2 AWG positive lead** to the **main contactor's load
    side**, and a matching negative lead to the bus negative.
    **Do not connect the contactor to the bus yet.**
7.  Wire the **pre-charge resistor + relay** in parallel with the
    main contactor.
8.  Wire the **bus-bleed LED + 100 kΩ resistor** between bus + and
    bus – inside the box. This LED tells you whether the
    high-voltage bus is live.
9.  Bolt the **8 mm 6061 lid** down with **12 × F15 (M5 × 16)**,
    **5 N·m each.** Apply silicone sealant on the gasket.

### 8.5 Phase 2D — electronics bay sub-assembly  (~ 4 h)

> **Where do the 18 motor drivers actually live?**  The
> `electronics_bay.stl` (360 × 240 × 140 mm internal) is sized for
> the **controller + power-distribution** stack, *not* for 18
> standalone drivers. There are two architectures depending on which
> motor you bought (§4.1):
>
> *  **FHA-40C-100 with built-in drives (recommended).**  Each motor
>    has its own EtherCAT-slave drive integrated in the housing.
>    The bay holds *only* the controller + IMU + DC-DC + EtherCAT
>    master + power-distribution PCB. **No driver row inside.**
>
> *  **ODrive / external-driver route.**  Each ODrive Pro is ~120 ×
>    65 × 30 mm and won't fit 18-up in the bay. Mount the 18 drivers
>    in **a separate driver enclosure** (e.g. 600 × 400 × 200 mm
>    sealed steel cabinet from any electrical wholesaler, ~$150) and
>    bolt that to the top deck **in the rear position alongside the
>    battery box**, with its own 4 × F10 + 4 × F10b mount pattern.
>    The bay then only carries the controller + IMU + ECat master.
>
> The remaining steps assume the FHA-with-built-in-drive route. For
> the ODrive route, replace step 2 with "wire each ODrive's
> EtherCAT-or-CAN port + power leads through the driver enclosure's
> own glands" and skip step 5.

1.  Assemble the **electronics bay** body per `electronics_bay.stl`.
    Fit **7 × PG-13.5 cable glands** to the punched holes.
2.  DIN-rail mount the **EtherCAT master**, the **24 V → 12 V
    DC-DC**, the **24 V house battery terminal block**, and the
    **96 V → 24 V isolated DC-DC** inside the bay.
3.  Mount the **main controller (Jetson / IPC)** on M3 standoffs at
    the rear of the bay (4 × F17 self-tappers).
4.  Mount the **9-DOF IMU** on a foam-isolated standoff at the bay
    centre, IMU axes aligned with the chassis frame (verify in
    §8.8).
5.  Run the EtherCAT trunk cable from the master out through one of
    the cable glands. The 18 motor jumpers will daisy-chain to it
    in §8.7.
6.  Mount the **6 × HX711 load-cell amplifier boards** in a row
    along one wall, with their 4-pin terminals facing the bottom
    glands.
7.  Bolt the **3 mm sheet-metal lid** down with **8 × F15 (M5 × 16),
    5 N·m each.** Apply silicone sealant on the gasket.

### 8.6 Phase 3 — final mechanical mating  (~ 6 h, 2-person)

1.  Place the **bare chassis** on 4 saw-horses, **chassis spokes
    radiating outward from the centre hub, edge pads pointing
    outboard, top face (the side with the 12 F10b rivnuts) facing
    up**. The saw-horses should support the chassis with all 6
    edge pads at least 700 mm above the floor — this gives the legs
    room to hang once they're mounted.
2.  Pick up **leg 0** (your first sub-assembly from §8.3). With a
    helper or a small chain hoist, present the coxa bracket's
    chassis-side flange against the chassis pad in the leg-0
    position. The bracket's 4 Φ 11 mm holes line up with the pad's
    4 Φ 11 mm holes.
3.  Insert **4 × F3 (M10 × 90)** through bracket + pad. **Each F3
    gets: 1 washer F5 under the head, 1 washer F5 under the F4
    nyloc nut.** Loctite 243 on threads.
4.  **Torque to 56 N·m in cross pattern.**
5.  Repeat for legs 1, 2, 3, 4, 5. Each chassis pad is identical, so
    the legs are interchangeable. **Total: 24 × F3, 24 × F4, 48 ×
    F5.** Walking time: ~ 4 h with one helper.
6.  Stand back. The walker now hangs from the saw-horses with all
    6 legs dangling.
7.  Drill the top deck. Lower the **top deck**
    (`chassis_top_deck.stl`, 12 mm 6061 plate) onto the chassis.
    Mark each of the **12 chassis-tube rivnut centres** through the
    deck with a transfer punch, then lift the deck off and drill
    **12 × Φ 9 mm clearance holes** at the punch marks.
8.  Re-lower the deck. Insert **12 × F10 (M8 × 25 A2-70)** down
    through the deck **into the F10b rivnuts** in the chassis tube
    top wall. Loctite 243. **Torque to 18 N·m.**
9.  Install the **saddle mount** (`saddle_mount.stl`) by lowering
    its base over the centre hub disc's 4 tapped M10 holes (90 mm
    PCD). Insert **4 × F12 (M10 × 25 A2-70)** through the base into
    the hub. Loctite 243. **Torque to 56 N·m.**
10. Install the **suspension seatpost + saddle** in the saddle
    mount's collar.
11. Install the **2 × footrest studs** (F18) by welding them to the
    +X edge of the top deck (rider's foot stations), per the rider
    geometry. Bolt the **rider footpegs** onto the studs.
12. Install the **harness mounting points**: 2 × F14 (M6 × 20) at
    the saddle's rear corners, 2 × F14 over the rider's shoulders
    on a tubular crossbar welded to the seatpost top.
13. Place the **battery box sub-assembly** from §8.4 onto the top
    deck in the rear position. Mark its 4 mounting holes through
    the deck with a transfer punch. Lift the box, drill **4 × Φ 11.5
    mm holes**, and set **4 × F10b rivnuts** at those positions.
    Re-set the box, then bolt **4 × F10 (M8 × 25) into the F10b
    rivnuts.** Loctite 243. **Torque to 18 N·m.**
14. Place the **electronics bay sub-assembly** from §8.5 onto the
    top deck in the front position. Same procedure: punch, drill,
    set 4 × F10b rivnuts, bolt with **4 × F10**, **18 N·m.**

### 8.7 Phase 4 — power and signal wiring  (~ 6 h, electrician required)

> All steps in this section assume the **main contactor is open and
> the E-stop is pressed** — i.e. the 96 V bus is dead at the
> drivers. **Verify with a meter before touching any motor lead.**

> **Cable routing convention.** All trunk cables run **between the
> top deck and the top face of the chassis tubes** — i.e. they hug
> the underside of the deck, fanning out from the electronics bay
> straight to each leg's chassis edge pad. The 12 mm gap created by
> the rivnut shoulder washers + sealing gaskets is enough for the
> bundle. **Do not** try to route cables through the welded chassis
> tubes — they're sealed.
>
> At each chassis edge pad, every cable destined for that leg drops
> down through a **single PG-13.5 cable gland in the side wall of
> the coxa bracket** and into the leg-side cable carrier system
> (3 carriers per leg: yaw, hip-pitch, knee).

#### 8.7.1 Mount cable carriers on each leg  (do this first)

1.  At each of the 18 joints, mount one **igus E-chain carrier
    (250 mm long, 30 × 50 mm cross-section)** with **2 × F13 (M6 ×
    16)** at each carrier end mount-plate (one mount-plate to the
    static side, one to the rotating side). 18 carriers × 2 ends ×
    2 F13 = **72 F13** for the carrier mounts. Per-leg: 6 F13 across
    3 carriers.
2.  For the **yaw carrier**: static mount on the coxa bracket's
    outboard wall; moving mount on the coxa link's bottom hub
    rim. Pre-arc so the carrier curls *downward* through ±45 ° of
    yaw motion.
3.  For the **hip-pitch carrier**: static mount on the coxa link's
    pitch plate; moving mount on the femur's hip-end cap rim.
4.  For the **knee carrier**: static mount on the femur's knee-end
    cap rim; moving mount on the tibia's knee-end cap rim.

#### 8.7.2 Route motor power and load cell cables

5.  Route the **6 × yaw motor power cables (1.2 m each, F4
    cable from §4.10)** from the electronics bay along the
    deck-underside trunk, out to each chassis edge pad's gland,
    and into the **bottom-plate side-wall connector of the coxa
    bracket** (yaw motor sits inside the bracket — the cable
    enters via a PG-9 gland in the bracket's side wall and lands
    on the FHA-40C's connector). Termination (driver side):
    4 × ferrule per cable into the driver power terminals.
6.  Route the **6 × hip-pitch motor power cables (1.8 m each)** the
    same way, but pass each through the **yaw cable carrier**
    before plugging into the hip-pitch motor. The carrier protects
    the cable as the yaw joint rotates.
7.  Route the **6 × knee motor power cables (2.5 m each)** through
    **two carriers per cable** (yaw → hip-pitch) before terminating
    at the knee motor (which is mounted on the femur — does not
    move with the knee joint, so the knee carrier itself is *not*
    needed for the knee motor's power cable).
8.  Wire the **6 × foot load cell cables** through the **PG-9
    glands at the bottom of each tibia foot hub** (already coiled
    there from §8.3.3, step 14). Each load cell cable runs up
    through *all three* carriers (knee → hip-pitch → yaw) and into
    one of the bay's HX711 board terminals. Use spiral cable wrap on
    the long sections.

#### 8.7.3 Route signal cables

9.  Route the **18 × M12 X-coded EtherCAT jumpers** alongside the
    power cables (separated by ≥ 25 mm or with a grounded shield).
    **Daisy-chain order (chain, not ring):** ECat master → leg 0
    yaw → leg 0 hip → leg 0 knee → leg 1 yaw → ... → leg 5 knee →
    *terminator plug*. **Cable count: 18** for the chain. *(Optional
    upgrade: add a 19th cable from leg 5 knee back to the master to
    close into a redundant ring — buy one extra M12 jumper if you
    want this.)*
10. Route the **18 × 24 V brake-release lines** (one per joint),
    sharing the EtherCAT bundle. Land at each drive's 24 V
    brake-out terminal.
11. Wire the **handlebar E-stop (1 ×)** from the saddle area to the
    contactor coil. The E-stop is in series with the **key switch**
    on the saddle mount; both must be closed for the contactor to
    pull in.

#### 8.7.4 Final cable check

12. **Visual sanity check:** with the legs in mid-flex, gently sweep
    each joint through its motion range. **No cable should pull
    tight or pinch in any joint position.** If one does, add slack
    at the bay end. Re-verify after every joint extreme.

### 8.8 Phase 5 — first power-up (logic only)  (~ 2 h)

1.  **Confirm:** main contactor open, E-stop pressed, key switch
    off, bus-bleed LED off (no high voltage), main battery
    disconnect open, walker on saw-horses with feet clear of the
    floor.
2.  Connect the **24 V house battery** to the electronics bay.
3.  Power up the IPC / Jetson. Watch boot logs.
4.  In a terminal: `ros2 launch hexapod_walker bringup.launch.py`.
5.  **Acceptance:** all 18 EtherCAT slaves enumerate, each reports
    a sane encoder position, no error flags.
6.  Fire the IMU. Check `ros2 topic echo /imu/data` — gravity
    vector should point through the chassis +Z axis when the
    walker sits flat on the bench. If not, your IMU mounting is
    rotated; fix in the URDF transforms.
7.  Power down. Confirm log file written for first-power-up event.

### 8.9 Phase 6 — joint commissioning (one joint at a time)
(~ 4 h)

> **Critical safety:** for each joint, a helper holds the link
> against possible runaway rotation while you energise. Both wear
> safety glasses. The legs are still hanging in the air; if a motor
> commands a hard slew, the leg whips around and can break a wrist.

1.  Close the main battery disconnect and the contactor key switch.
    Press E-stop release.
2.  Issue **brake-release** on **only the leg 0 yaw drive**. Manually
    confirm that the leg now swings freely about the yaw axis.
3.  Issue a **slow position command (5 °/s, ± 5 °)** to leg 0 yaw.
    Watch direction of rotation — must match the URDF's positive
    yaw convention. If reversed, flip the motor's `inverted` flag
    in the driver config.
4.  Repeat for leg 0 hip-pitch, then leg 0 knee. Then move to leg 1,
    and so on, all 18 joints.
5.  For each joint, write down the **encoder zero offset** (the
    encoder count when the joint sits at the paint-pen mark from
    §8.3.8). Save these to the URDF.
6.  Re-press E-stop. Verify all 18 brakes engage within 100 ms.

### 8.10 Phase 7 — slow-stand bring-up  (~ 1 h)

1.  Walker still on saw-horses, all 6 feet ~ 200 mm above the
    floor. Confirm safety perimeter (no tools, hands, cables under
    the feet).
2.  Run `ros2 service call /walker/slow_stand
    std_srvs/srv/Trigger`. The controller commands all 6 feet
    downward at 0.02 m/s in unison.
3.  When all 6 feet contact the floor (foot load cells > 50 N), the
    controller transitions to **"floor-contact stand"** and slowly
    transfers chassis weight from the saw-horses to the legs.
4.  After ~ 30 s, all foot load cells should read **~ 460 N each**
    (the 280 kg chassis evenly distributed across 6 feet).
5.  **Lift the saw-horses out from under the chassis.** The walker
    is now free-standing on its own legs.
6.  Verify the chassis stays level (IMU pitch + roll within ± 1 °)
    for at least 5 minutes with the standing controller engaged.

### 8.11 Phase 8 — first walk (no rider)  (~ 2 h)

1.  Clear a 5 m × 5 m flat, clean, level area around the walker.
2.  Issue `/walker/walk_slow` (0.05 m/s, wave gait — only 1 leg in
    swing at a time, 5 planted). The walker should advance ~ 0.05
    m/s for 1 minute, then halt.
3.  Listen for any binding, ticking, or motor whine. Watch for
    cable pinching at any joint extreme.
4.  Inspect every joint's bolts for any sign of loosening (paint-pen
    a witness mark across each F1 head + plate before the walk;
    if the mark is broken after, that bolt has rotated).
5.  Repeat at 0.1 m/s, then 0.2 m/s, then full cruise (0.4 m/s).
6.  Run for 30 minutes total. **Re-torque every accessible F1 to
    28 N·m, every F3 to 56 N·m** after the first run.
7.  Charge battery. Re-test next day.

### 8.12 Phase 9 — first ride  (~ 1 h, 2 spotters required)

> A 280 kg machine moving 0.4 m/s with a 110 kg human aboard has
> serious kinetic energy. The first ride is on **flat indoor
> concrete** with a 4-point harness and a helmet, surrounded by 4
> spotters with foam barriers, at quarter-speed (0.1 m/s).

1.  Rider mounts the saddle and clips into the harness. Spotters in
    position. E-stop within rider's reach.
2.  Issue `/walker/walk_slow` at 0.1 m/s. Walker carries rider 5 m
    forward, halts.
3.  Repeat at 0.2 m/s, then 0.4 m/s. Each test, listen for any
    change in motor sound or chassis behaviour.
4.  After 30 min of cumulative ride time, **re-torque every F1
    (28 N·m) and F3 (56 N·m).**
5.  You're done. The walker is commissioned.

### 8.13 Maintenance schedule

| Interval | Task |
|---|---|
| Every 1 h ride time | Visual: any cable pinch, any leaking grease, any unusual noise |
| Every 8 h ride time | Re-torque every F3 (56 N·m), every F6 (56 N·m), every F1 visible without disassembly (28 N·m) |
| Every 40 h ride time | Full F1 torque pass on all 18 motors (requires partial disassembly), check encoder zeros, re-grease cable carriers |
| Every 100 h | Inspect femur / tibia extrusions for any visible deformation or paint cracking near the end caps; if cracking, replace extrusion |
| Every battery cycle | Log capacity. Replace pack at 80 % capacity loss (≈ 800 cycles for LiFePO₄) |
| After any fall | Full 18-motor torque pass. Inspect every link with dye penetrant or visual for cracks. Replace any visibly bent extrusion. |

---

## 9. Gait

The default gait is a classic **alternating tripod**:

```
                 leg 0     <- right-front
        leg 5  ---X---  leg 1
              /       \
   leg 4 --- chassis --- leg 2
              \       /
        leg 3  ---X---
                 (no leg here -- legs are at edges, not vertices)
```

Legs `{0, 2, 4}` form one tripod, `{1, 3, 5}` form the other. Each
half-cycle:

1. Tripod A's three feet are planted ("stance"). They push the chassis
   forward by sweeping their hip-yaw joints rearward. Femur and knee
   joints maintain a constant chassis height.
2. Simultaneously, tripod B's three feet are in the air ("swing").
   They lift, swing forward to the next foothold, and re-plant.
3. The roles swap.

At cruise the cycle takes ~ 1.5 s per stride; with a 0.6 m stride that
gives 0.4 m/s ground speed.

For climbing or stepping over obstacles the controller falls back to a
**wave gait**: one leg in swing at a time, the other five planted —
slower (~ 0.1 m/s) but maximally stable.

---

## 10. Cost summary

| Bucket | Low | High |
|---|---|---|
| 18 × actuators (FHA-40C-100 retail vs. used / DIY cycloidal) | $25,000 | $130,000 |
| Structural (extrusion + tube + plate) | $1,500 | $3,500 |
| CNC / cast end caps & brackets | $4,000 | $9,000 |
| 3D-printed parts (HP MJF or local SLS) | $1,500 | $4,000 |
| Battery system (packs + contactor + pre-charge + fuses + E-stop, §4.8) | $1,400 | $2,800 |
| Motor drivers (18 ×) | $5,000 | $30,000 |
| Compute + IMU + foot sensors (§4.9) | $1,800 | $5,200 |
| Wiring harness (cables + glands + lugs + carriers, §4.10) | $1,000 | $2,500 |
| Saddle, harness, footpegs (§4.11) | $400 | $1,200 |
| Fasteners (every line in §4.2 + §4.3) | $300 | $700 |
| Tools you don't already own (§4.12, mostly the torque wrench + crane hire) | $300 | $1,500 |
| Consumables (Loctite, paint, anti-seize, cable ties, §4.13) | $200 | $500 |
| **Total** | **~ $42,400** | **~ $190,900** |

Realistic mid-range: **$60 – $80 k** if you go DIY on the actuators
(BLDC + custom cycloidal) and use Xometry / PCBWay for the structural
parts.

---

## 11. Safety

This vehicle can crush, pinch, electrocute, and fall. Read this list
and *do not skip any of it*.

* **Eighteen motors with 1.5 kN·m peak torque can amputate fingers.**
  Never reach into a moving leg. Park the walker on saw-horses with
  the legs unloaded before any maintenance. Fit the motor brakes *on*
  before unbolting any link.
* **96 V DC is enough to kill.** Treat the battery loop as live unless
  the contactor is open *and* the bus capacitors are bled (LED on the
  electronics bay). Never service the battery without an electrician
  present.
* **The walker is statically stable in tripod stance, but only if at
  least three properly-spaced feet are on the ground.** Power loss
  during a swing-phase = the chassis drops 200 mm onto whatever leg
  was about to plant. Software must sequence E-stop → gait freeze
  → land all 6 → de-energise. The supplied controller pseudocode
  (§7) does this.
* **Falls on uneven ground are when riders get hurt.** Wear a 4-point
  harness and a helmet. Don't ride above 0.2 m/s on slopes > 15 °.
* **Pinch points** at every hub (where the link rotates past the
  motor body). Bright-yellow pinch-point decals on the
  outboard face of every coxa, femur, and tibia hub.
* **Thermal:** an FHA-40C running near continuous torque draws 800 W
  and dissipates ~ 100 W as heat. After a 30-minute walk the motor
  housings are at ~ 70 ° C — hot enough to burn skin. Don't grab a
  motor housing post-walk.
* **Legal:** in most jurisdictions a 280 kg unregistered electric
  vehicle is **not** street-legal. Build it for private property.
  Check local UAV/robot regulations; some states class anything
  > 25 kg with autonomous control as a robotic vehicle subject to
  registration.

---

## 12. Hot-rodding ideas

Once the basic walker is running, here's where to take it:

* **Hydraulic upgrade:** swap the rotary servos for double-acting
  hydraulic cylinders on parallelogram links. Pump driven by a
  Kubota 3-cyl diesel on the top deck. Now you have a Stompy.
* **Pose-shifting platform:** add a 6 × 6 Stewart-platform top deck
  so the rider's saddle stays level when the chassis pitches over
  uneven ground.
* **Quadrupedal mode:** lift the front two legs off the ground and
  use them as manipulators. The arms have plenty of reach (1.5 m).
* **Cargo variant:** strip the saddle, add a 600 mm flatbed top
  deck, carry up to 250 kg of cargo over rough ground.
* **Outdoor autonomy:** add a Velodyne or Livox LiDAR on a
  3 m mast for dynamic foothold planning.

---

## 13. References

* **Open-source big walkers:**
  - [Project Hexapod / Stompy](https://www.projecthexapod.com/) — the only successful >100 kg-payload hexapod with public docs.
  - [Mantis hexapod](https://www.youtube.com/watch?v=OrsMdsLG7DU) by Matt Denton — engineering reference.
* **Servomotors:**
  - Harmonic Drive [FHA-C series datasheet](https://www.harmonicdrive.net/products/actuators/fha-c-mini-series).
  - MyActuator [RMD-X12 series](https://www.myactuator.com/rmd-x12-series).
* **Software:**
  - [`ros2_control`](https://control.ros.org/), [`champ`](https://github.com/chvmp/champ) hexapod gait library.
  - James Bruton's [openDog / hexapod tutorials](https://github.com/XRobots) — invaluable for the DIY actuator route.

---

## 14. Disclaimer

This design is provided as a starting point for hobbyist builders who
have professional engineering review available. **The author has not
built and tested this exact walker.** All numbers in §5 and §10 are
rough hand-calcs with single-digit-percent tolerance; a real build
needs a proper FEA pass on the femur and tibia, fatigue analysis on
all six hip-yaw weld joints, and a thermal-management plan for the
motor stack. Build at your own risk.

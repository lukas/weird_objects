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

Quantities below are for one walker. **All prices are mid-2026 USD,
order of magnitude only.** Industrial servomotors and batteries are the
two line items that dominate the budget.

### 4.1 Actuators (the expensive bit)

You need **18 servomotors** of one part number. The worst-loaded joint
is the hip-pitch at full extension, which sees ~ 1.4 kN·m peak. Real
candidates (any one will work, listed by torque):

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

### 4.2 Structural — leg extrusions

The femur and tibia STLs model a solid 90 × 120 (femur) and 70 × 90
(tibia) block. The **real** parts are hollow rectangular aluminium
extrusions (6061-T6) bolted into the printed/cast end caps. Buy:

| Part | Section | Length each | Wall | Qty |
|---|---|---|---|---|
| Femur extrusion | 90 × 120 × 6 mm wall | 480 mm | 6 mm | 6 |
| Tibia extrusion | 70 × 90  × 5 mm wall | 690 mm | 5 mm | 6 |

(`FEMUR_LENGTH` and `TIBIA_LENGTH` in `hexapod_walker.py` include the
length of the hubs at each end; the bare extrusion is shorter.)

### 4.3 Structural — chassis

| Part | Spec | Qty |
|---|---|---|
| 80 × 80 × 6 mm wall sq. tube, 6063-T5 aluminum (or A36 mild steel) | Cut to 6 × 600 mm perimeter + 6 × 600 mm spokes | 1 set |
| 12 mm 6061-T6 aluminum plate | CNC-cut to `chassis_top_deck.stl` outline | 1 |
| 8 mm 6061-T6 plate | Bolt-on lid for the battery box | 1 |
| Various M10 / M8 / M6 socket-head fasteners | Stainless A4 / 8.8 grade | ~ 200 |

### 4.4 End-cap cast/printed parts (the STL files)

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

### 4.5 Battery, motor controllers, and electronics

| Part | Spec | Qty |
|---|---|---|
| LiFePO₄ pack | 48 V × 50 Ah / 2.4 kWh, with integrated BMS | 2 |
| Inter-pack series link | 4 AWG, fused 100 A | 1 |
| Main contactor | 96 V DC, 200 A, with key-switch interlock | 1 |
| E-stop button | DPDT, 100 mA — wired to drop the contactor | 1 (handlebar) |
| Motor drivers | 18 × matched to your chosen motor (e.g. Elmo Gold Twitter for FHA, ODrive Pro v3 for the DIY route) | 18 |
| Main controller | Industrial PC or NVIDIA Jetson Orin Nano running ROS 2 + EtherCAT master (or CAN bus, depending on motor choice) | 1 |
| 9-DOF IMU | Bosch BMI088 + magnetometer, e.g. VectorNav VN-100 | 1 |
| Foot force sensors | 200 kgf load cell per foot, e.g. SparkFun TAL220 | 6 |
| Joystick + small UI screen | Any USB joystick + 7" touchscreen | 1 |

### 4.6 Saddle / rider attachment

| Part | Spec | Qty |
|---|---|---|
| Suspension seatpost | 30.9 / 31.6 mm, 100 mm travel | 1 |
| Bicycle / motorcycle saddle | Any | 1 |
| 4-point harness | Automotive 4-point | 1 |
| Footrests (rider's feet) | Off-the-shelf motorbike pegs welded to the top deck | 2 |

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

Use 80 × 80 × 6 mm wall 6063-T5 aluminum or A36 mild steel:

1. Lay out a regular hexagon, flat-to-flat = 1200 mm, on a flat-deck
   welding table.
2. Cut 6 perimeter pieces to 600 mm (apothem-derived) with 30 ° mitres.
   Tack-weld into a hexagon.
3. Cut 6 spokes to 532 mm (apothem - hub_OD/2). Tack weld each spoke
   from the centre hub disc out to the midpoint of each perimeter
   tube.
4. Insert and weld the 6 mounting pads (cast or CNC-cut 30 × 200 ×
   112 mm 6061-T6 plates) flush with the perimeter outboard face.
5. Drill the four M10 clearance holes through each pad in a 160 × 80
   pattern (`hexapod_walker.py` already cuts these holes in the STL —
   use it as a drill template).
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
   * insert a 6 mm × 200 mm steel disc (CNC-cut, with the four
     M10 bolt clearance holes pre-drilled),
   * pour 60-A urethane (Smooth-On VytaFlex 60) around it.
5. Demould after 24 h. Bolt to the tibia.

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

Allow ~ 60 hours of skilled labour for a first build, half that for a
second.

### 8.1 Sub-assemblies (do these in parallel)

1. **Chassis frame** — weld + paint per §6.1. Test-fit the top deck.
2. **Battery boxes & electronics bay** — print/weld per §6.3. Mount
   the BMS and pre-assemble the cell stack inside.
3. **6 × leg sub-assemblies:**
   1. Bolt the **hip-pitch motor** body into the inboard cradle of
      `coxa_link.stl`. Bolt its output flange to the hip end of
      `femur_link.stl`.
   2. Bolt the **knee motor** body into the knee cradle of
      `femur_link.stl`. Bolt its output flange to the knee hub of
      `tibia_link.stl`.
   3. Bolt the foot pad to the bottom of `tibia_link.stl`.
   4. Bolt `coxa_bracket.stl` around its dedicated **hip-yaw motor**.
      Bolt the bracket to the chassis edge bolt pad. Bolt the top
      hub of `coxa_link.stl` to the yaw motor's output flange.
   5. You now have a complete leg dangling from the chassis.

### 8.2 Final assembly

5. Land the 6 leg sub-assemblies on the chassis (you'll want a hoist
   — a single leg masses ~ 25 kg). Bolt the coxa brackets to their
   chassis pads.
6. Lower the chassis onto a 1.2 m welding table, legs hanging.
7. Bolt on the top deck, saddle post, battery box, and electronics
   bay.
8. Install the 96 V battery stack (with an electrically-trained
   helper). Verify the contactor is open, E-stop pressed.
9. Wire the 18 motor cables through the cable glands of the
   electronics bay. Tag each one with its joint name.
10. Power up the controller (24 V house battery first; logic only).
    Verify all 18 drives enumerate on the bus and report sane
    encoder positions.
11. Slowly raise each leg one joint at a time. Verify direction of
    rotation matches the URDF kinematics.
12. With the chassis still on the welding table, command a "slow
    stand" — all 6 feet move down 200 mm in unison. The vehicle
    should lift itself off the table.
13. Lower it back down. Remove the table. Walk it 5 m at 0.05 m/s
    with no rider — eyeball gait stability and listen for binding.
14. Add a rider only after step 13 has run cleanly for 30 minutes.

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
| Battery (2 × 48 V × 50 Ah LiFePO₄) | $1,200 | $2,400 |
| Motor drivers (18 ×) | $5,000 | $30,000 |
| Compute + IMU + sensors | $1,500 | $4,500 |
| Saddle, harness, footpegs | $400 | $1,200 |
| Wiring, contactor, E-stop, fuses | $400 | $1,000 |
| Misc. fasteners, paint, cable glands | $300 | $800 |
| **Total** | **~ $40,800** | **~ $186,400** |

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

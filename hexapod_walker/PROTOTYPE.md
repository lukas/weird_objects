# Hexapod Walker — Tabletop Prototype Build Guide

> A scaled-down sibling of the [full-size walker](ASSEMBLY.md) intended
> for proving out the geometry, kinematics, and gait controller before
> you commit to industrial servomotors. Same architecture: regular hex
> chassis, six identical 3-DOF legs, alternating-tripod gait — but
> everything is shrunk roughly 6× and every joint is driven by a
> generic 25 kg·cm hobby servo (DS3225 / MG996R class) instead of a
> $5000 harmonic-drive servomotor.
>
> Total parts cost: **~$150 – $300** in 2026 USD. A motivated builder
> can have it walking on a tabletop in a weekend.

![Cycles render of the prototype hexapod](renders/prototype.png)

---

## 1. Why a separate prototype?

The full-size walker is a serious build: a 280 kg vehicle, $40 k+ BOM,
custom CNC and casting work. Even if your end goal is a rideable
walker, you almost certainly want to:

1. **Validate the kinematics and gait controller end-to-end** on cheap
   hardware before machining anything in aluminum.
2. **Iterate on the leg link ratios** without a 6-week lead time on
   castings — you can re-print every link in 12 hours.
3. **Train the gait scheduler in simulation, then prove it on real
   hardware** at 1/200 the mass and 1/250 the cost.
4. **Show the design works** to collaborators, funders, or your spouse
   before committing to the bigger build.

The prototype is **mechanically and kinematically identical** to the
full-size walker. The same six-legged tripod gait, the same per-joint
PID + feed-forward controller architecture, even the same coxa : femur
: tibia ~ 1 : 4 : 5 ratio. You can re-use the gait code unchanged
when you graduate to the big version — just re-tune the gains.

---

## 2. Specification at a glance

| Property | Value |
|---|---|
| Configuration | 6 legs × 3 DOF, alternating-tripod gait |
| Overall envelope (foot-to-foot, fully extended) | ~ 580 × 670 × 110 mm |
| Standing height (chassis underside) | ~ 70 mm |
| Stride length | ~ 120 mm |
| Cruise speed (tripod gait) | ~ 80 mm/s |
| Vehicle dry mass | ~ 1.3 kg |
| Per-leg static load (tripod stance) | ~ 4.3 N (~ 0.43 kg) |
| Peak knee torque | ~ 0.6 N·m (~ 6 kg·cm) |
| Battery | 1 × 3S 2200 mAh LiPo (11.1 V → 5 V BEC) |
| Continuous draw (cruise) | ~ 1.5 A @ 5 V |
| Run time, level ground | ~ 30 min |

**Knee torque margin:** the DS3225 servo provides ~ 25 kg·cm at 6.8 V,
giving a ~ 4× safety factor over the worst-case knee torque. The
weaker MG996R (10 kg·cm) gives ~ 1.6×, which is fine for tabletop
walking but leaves no margin for shock loads.

---

## 3. STL files in `stl_prototype/`

Run `./run.sh hexapod_walker/hexapod_prototype.py` to (re)generate
every file. All dimensions are in millimetres. All STLs are sized to
fit a 220 × 220 mm 3D-printer bed (Ender 3 / Bambu A1 mini class).

### 3.1 Body parts (one of each)

| File | Function | Suggested print settings |
|---|---|---|
| `chassis_top.stl` | Top hex deck (4 mm PLA, 200 mm flat-to-flat) | 0.2 mm layer, 25% gyroid infill, 4 walls |
| `chassis_bottom.stl` | Identical bottom plate | same as top |
| `battery_holder.stl` | Open-top tray for one 3S 2200 mAh LiPo | 0.2 mm layer, 20% infill |
| `electronics_tray.stl` | Mount plate for Arduino + PCA9685 | 0.2 mm, 20% infill, 2 walls |

### 3.2 Per-leg parts (print 6 sets)

| File | Function | Print orientation |
|---|---|---|
| `coxa_bracket.stl` | Bolts to chassis edge, holds yaw servo | Flat on bed, mounting pad down |
| `coxa_link.stl` | U-bracket driven by yaw servo, holds hip-pitch servo | Flat on bed, hub face down |
| `femur_link.stl` | Thigh link with cradle for the knee servo | Flat on bed, spar face down |
| `tibia_link.stl` | Shin link, ends in foot socket | Flat on bed |
| `foot_pad.stl` | Compliant foot — print in TPU for grip | Hub up, no supports |

### 3.3 Generic hardware (print 18 + spares)

| File | Function | Print orientation |
|---|---|---|
| `servo_horn_adapter.stl` | Bolts to a stock plastic horn, presents a 4 × M3 bolt pattern that the link plate clamps to | Flat |

### 3.4 Visualization (do not print)

| File | Function |
|---|---|
| `assembly_preview.stl` | All parts placed in standing pose. Open in MeshLab or Cursor's STL viewer to sanity-check before printing. |

---

## 4. Bill of materials

### 4.1 Actuators (the long pole)

| Item | Spec | Qty | Approx. cost |
|---|---|---|---|
| Hobby servo | DS3225 (25 kg·cm, metal gear, 6.8 V) — recommended | 18 + 2 spare | $13 each on AliExpress, $18 on Amazon ($240 – $360 total) |
| (Alternative) | MG996R (10 kg·cm, metal gear, 6 V) — works but no margin | 18 + 2 spare | $4 each ($80 total) |
| (Alternative — premium) | Dynamixel XL-330-M288-T (smart serial, daisy-chain) | 18 + 2 spare | $30 each ($600 total) |

### 4.2 Power

| Item | Spec | Qty | Cost |
|---|---|---|---|
| LiPo battery | 3S 2200 mAh 25C, XT60 connector | 1 | $20 |
| BEC (5–6 V regulator) | 5 V 5 A switching, 3S input | 1 | $8 |
| LiPo charger | iSDT D2, B6AC, or any decent 3S balance charger | 1 | $30 |
| LiPo bag | Fire-safe charging | 1 | $10 |

### 4.3 Control electronics

| Item | Spec | Qty | Cost |
|---|---|---|---|
| Arduino Mega 2560 (clone) | ATmega 2560, 5 V | 1 | $15 |
| PCA9685 16-channel PWM driver | I²C, 12-bit | 2 | $4 each |
| MPU-6050 IMU | 6-DOF gyro + accel, I²C (optional but useful) | 1 | $4 |
| Jumper wires | F-F, 20 cm × 50 + servo extensions × 18 | — | $15 |
| Logic-level wiring + heat-shrink | — | — | $5 |

### 4.4 Fasteners

| Item | Qty | Notes |
|---|---|---|
| M3 × 8 mm socket-head cap screws | ~ 100 | Servo tab mounting, link bolts, chassis tie-rods |
| M3 × 12 mm | 24 | Standoffs between top + bottom chassis plates |
| M3 × 16 mm | 24 | Coxa bracket → chassis (4 × 6 = 24) |
| M3 nuts | ~ 100 | Self-locking nyloc preferred at high-vibration joints |
| M3 × 25 mm round standoffs (M-F) | 8 | Top-to-bottom chassis spacers |
| Self-tapping screws (servo horn → M3 nut) | 18 | Ships with the servos |

### 4.5 3D-printed material

| Item | Spec | Qty | Cost |
|---|---|---|---|
| PLA filament | 1.75 mm, any colour | ~ 250 g | $5 |
| TPU filament | 1.75 mm, 95A | ~ 50 g (foot pads only) | $5 |

### 4.6 Total

| Bucket | Cheap (MG996R) | Recommended (DS3225) | Premium (XL-330) |
|---|---|---|---|
| Actuators | $80 | $300 | $600 |
| Power | $70 | $70 | $70 |
| Electronics | $50 | $50 | $50 |
| Fasteners + filament | $20 | $20 | $20 |
| **Total** | **~ $220** | **~ $440** | **~ $740** |

---

## 5. Print plan

A single Ender 3-class printer runs the whole BOM in roughly **22 hours**:

| Pass | Parts | Bed | Time |
|---|---|---|---|
| 1 | 6 × coxa_bracket | 6 brackets pack onto a 220 mm bed | ~ 4 h |
| 2 | 6 × coxa_link | Same | ~ 4 h |
| 3 | 6 × femur_link | Same | ~ 5 h |
| 4 | 6 × tibia_link | Same | ~ 4 h |
| 5 | chassis_top + chassis_bottom + battery_holder + electronics_tray | ~ 4 h |
| 6 | 18 × servo_horn_adapter (PLA) | Single bed | ~ 1 h |
| 7 | 6 × foot_pad (TPU) | Single bed | ~ 1 h |

Tip: use 4 walls and 25 % gyroid infill for the bracket and link
parts — the servo cradles see the most load and the extra walls add
substantial stiffness for very little weight.

---

## 6. Assembly sequence

Allow ~ 4 hours for a first build, ~ 90 min for a second.

### 6.1 Per-leg sub-assembly (do all 6 in parallel)

1. **Coxa bracket + yaw servo:** drop the yaw servo into the bracket
   from the +Z (top) side, so the output gear pokes UP through the
   open top of the cradle. Bolt through the four M3 tab holes. The
   servo body now hangs below the chassis edge plane.
2. **Horn adapter on the yaw servo:** centre the servo, push a stock
   plastic 4-arm horn onto the spline at 0°, then bolt
   `servo_horn_adapter.stl` to the horn with 4 × M2.5 self-tappers
   (the screws that ship with the servo) plus the M3 horn-attach
   screw.
3. **Coxa link:** bolt the link's hub flange to the horn adapter with
   4 × M3 × 8 mm + nuts.
4. **Hip-pitch servo:** drop into the cradle in the coxa link. Bolt
   through the four M3 tab holes. Fit a stock horn + adapter on the
   output, perpendicular to the leg arm so the femur swings up and
   down (not fore-and-aft).
5. **Femur:** bolt the femur's hip-end flange to the hip horn adapter
   (4 × M3).
6. **Knee servo:** drop into the cradle in the femur. Tab-bolt + horn
   + adapter as before, pointing perpendicular to the femur spar.
7. **Tibia:** bolt the tibia's knee-end flange to the knee horn
   adapter (4 × M3).
8. **Foot pad:** push-fit into the tibia's foot socket, glue with CA
   if it's loose.

You now have a complete leg dangling from a coxa bracket. Repeat 6
times.

### 6.2 Final assembly

9. **Bottom chassis plate:** lay flat. Bolt the 6 leg sub-assemblies
   to the 6 outer-edge bolt patterns (4 × M3 × 16 mm + nylocs each).
10. **Stand-off posts:** screw 4 × M3 × 25 mm M-F standoffs into the
    inner bolt pattern.
11. **Battery holder + electronics tray:** bolt to the bottom plate
    using the same inner bolt pattern (the battery holder feet share
    the standoff bolt pattern, the tray sits adjacent).
12. **Wire it up:** see §7.
13. **Top chassis plate:** screw down onto the M3 standoff tops.

---

## 7. Wiring

```
                    +--------------+
                    |  Arduino     |
                    |  Mega 2560   |
                    +-+-+----+--+--+
                      | |    |  |
                  SDA/SCL   D2-D5
                      | |    |  |
            +---------+ |    |  +-- IMU MPU-6050 (I2C, addr 0x68)
            |           |    |
       +----+----+ +----+----+
       | PCA9685 | | PCA9685 |   I2C addr 0x40 + 0x41 (jumper)
       |  #1     | |  #2     |
       | ch 0-15 | | ch 0-1  |
       +-+-+-+...+ +-+-+-+...+
         | | |       | | |
        18 PWM lines to the servos:
            #1 ch  0-2  -> leg 0 (yaw, hip, knee)
            #1 ch  3-5  -> leg 1
            #1 ch  6-8  -> leg 2
            #1 ch  9-11 -> leg 3
            #1 ch 12-14 -> leg 4
            #1 ch 15    -> leg 5 yaw
            #2 ch  0-1  -> leg 5 hip + knee

  Power side (separate from logic):
            +-----------+
            | 3S LiPo   |  (XT60 -> on/off switch -> XT60 splitter)
            +-----+-----+
                  |
        +---------+---------+
        |                   |
     5V 5A BEC          5V 5A BEC
   (servo rail #1)    (servo rail #2)
        |                   |
        +-> PCA9685 #1 V+   +-> PCA9685 #2 V+
            (powers 9 servos)   (powers 9 servos)
```

Two BECs / two PCA9685s let you split the 18-servo current draw
across two regulators. A single 5 A BEC will brown out under
synchronous-walk current spikes (peak ~ 6 – 8 A across all 18 servos).

The Arduino gets its 5 V from one BEC's auxiliary output (or USB
during development) — DO NOT power the Arduino off the LiPo's raw
11.1 V via the barrel jack, the on-board regulator can't handle the
combined logic + sensor load.

---

## 8. Software

A starter Arduino sketch (~ 200 lines) is enough to walk the
prototype. You will need:

```cpp
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm1(0x40);
Adafruit_PWMServoDriver pwm2(0x41);

// Pulse-width mapping (DS3225: 500 us = -90 deg, 2500 us = +90 deg)
const float PWM_MIN_US = 500.0;
const float PWM_MAX_US = 2500.0;

// Per-joint trim (calibrate after first power-up)
float trim_deg[18] = { /* ... */ };

void writeJoint(int joint_idx, float angle_deg) {
    Adafruit_PWMServoDriver& drv = (joint_idx < 16) ? pwm1 : pwm2;
    int chan = joint_idx % 16;
    float us = PWM_MIN_US
             + (angle_deg + 90 + trim_deg[joint_idx]) / 180.0
             * (PWM_MAX_US - PWM_MIN_US);
    drv.writeMicroseconds(chan, (int)us);
}
```

Then add a ~ 50 Hz loop that runs the inverse kinematics for each leg
(closed-form 3R IK — coxa yaw, hip pitch, knee pitch — there's a
classical solution; see e.g. the "Phantom X / Lynxmotion AX hexapod"
references) and a tripod-gait scheduler.

If you want to use ROS instead of bare-metal Arduino, swap the Arduino
for a Raspberry Pi 4 running `ros2_control` + the same PCA9685
driver, and the same gait code (Python or C++) you intend to use on
the full-size walker.

---

## 9. Tuning notes

* **Per-joint trim:** the first thing you do after assembly is set
  every joint to its "neutral" angle (femur and tibia horizontal,
  pointing radially out) and adjust the `trim_deg` array until each
  link physically lines up. Mechanical horn-spline mounting introduces
  ~ 14 ° of quantization, so you'll see ~ 7 ° trim on each joint.
* **Pulse-width range:** check the actual mechanical end-stops by
  sweeping each joint slowly from 500 µs to 2500 µs. Cheap servos
  often only achieve ~ 160 ° of travel, not the advertised 180 °, so
  clamp `angle_deg` to ±80 ° in software.
* **Gait period:** start at 2 s per cycle (very slow, easy to debug),
  speed up to 1 s once the IK and trim are dialled.
* **Power supply sag:** if the robot collapses momentarily during
  swing-to-stance transitions, the BECs are sagging. Switch to a
  beefier 5 A switching BEC (D-Link D24V50F5 or similar) or split
  the 18 servos across 3 BECs instead of 2.

---

## 10. Migration to the full-size walker

The prototype's value is that **everything you build above the joint
level transfers unchanged** to the full-size walker:

* The IK math is identical (same coxa / femur / tibia ratio).
* The gait scheduler is identical.
* The per-leg state machine (stance / swing / lift / re-plant) is
  identical.
* The body kinematics + IMU fusion are identical.

What changes when you scale up:

* The actuators talk EtherCAT or CAN instead of PWM, so the joint
  driver layer changes.
* Joint torque limits are 100× higher, so the gait scheduler needs to
  add a torque-aware planner (don't command a stride that requires
  more than 70 % of peak knee torque).
* Safety: a 280 kg vehicle needs an E-stop, brake-on-fault behaviour,
  and a "freewheel only when stationary" supervisor — none of which
  matter on a 1.3 kg tabletop unit.

Plan to spend ~ 3 months walking on the prototype, ironing the gait
and IK out, before you cut metal for the full-size build.

---

## 11. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot "twitches" but won't move | One BEC is browning out | Add a second BEC, verify 5 V steady on a scope under load |
| One leg drags | Per-joint trim is off, or that servo's output dead-band is unusually wide | Re-trim, or swap that servo (cheap servos have ± 10 ° unit-to-unit variation) |
| Knee servos overheat | Gait is over-loading (too high a chassis, too long a stride) | Lower chassis by re-targeting `STANCE_FEMUR_DEG = -15`, shorter stride |
| Robot drifts sideways | Tripod legs not lifting in sync | Increase swing duration; verify `trim_deg` for hip-pitch is consistent across all 6 legs |
| Foot slips on floor | TPU pad too smooth | Cut a small section of bicycle inner tube, glue inside the pad |

---

## 12. License & Disclaimer

Same as the parent project — personal exploration, no license declared
yet. The prototype is mechanically benign (small servos, small
batteries, no rider) but **3S LiPos can fight back if you puncture
them** — charge in a fire-safe bag and don't leave them charging
unattended.

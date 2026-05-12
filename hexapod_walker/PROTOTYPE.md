# Hexapod Walker — Tabletop Prototype Build Guide

> A scaled-down sibling of the [full-size walker](ASSEMBLY.md) intended
> for proving out the geometry, kinematics, and gait controller before
> you commit to industrial servomotors. Same architecture: regular hex
> chassis, six identical 3-DOF legs, alternating-tripod gait — but
> everything is shrunk roughly 6× and every joint is driven by a
> generic 25 kg·cm hobby servo (DS3225 case/geometry) instead of a
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

**Knee torque margin:** the design target is a DS3225 25 kg·cm metal-gear
digital servo.  At 6.8 V it gives a ~ 4× safety factor over the
worst-case knee torque.  The printed wells, tab pilots, horn adapters
and RL servo torque limits are all tuned around this DS3225-class case.

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
| `coxa_bracket.stl` | Horizontal flange + servo well (yaw motor hangs below). 4 vertical M3 bolts clamp the flange between the two chassis plates. | Flange on bed, well opening up |
| `coxa_link.stl` | U-arm driven by the yaw servo's horn; carries the hip-pitch servo in a side-loaded well. | Hub face down, well opening up |
| `femur_link.stl` | I-beam thigh with a slot through the spar so the knee servo body can slide past it during assembly. Top + bottom flange bridges connect the spar to the well. | Spar's flat face on bed, well opening up |
| `tibia_link.stl` | Shin link with knee pad and foot socket at the far end. | Flat on bed |
| `foot_pad.stl` | Compliant foot — print in TPU for grip | Hub up, no supports |

### 3.3 Generic hardware (print 18 + spares)

| File | Function | Print orientation |
|---|---|---|
| `servo_horn_adapter.stl` | Bolts to a stock plastic horn, presents a 4 × M3 bolt pattern that the link plate clamps to | Flat |

### 3.4 Visualization (do not print)

| File | Function |
|---|---|
| `assembly_preview.stl` | All parts placed in standing pose. Open in MeshLab or Cursor's STL viewer to sanity-check before printing. |

### 3.5 Don't have a 3D printer? Order from a print service

Run `./run.sh hexapod_walker/prepare_xometry_upload.py` to build a
self-contained order package in `xometry_upload/`. The script
re-orients each part for printing (hollow servo pockets opening
toward +Z, broadest flat face on the build plate), consolidates the
two identical chassis plates into a single file with `qty=2`, and
emits a `manifest.csv` listing the recommended material, color, and
finish for every part. See `xometry_upload/README.md` for the full
upload-and-checkout flow on Xometry, Shapeways, JLCPCB, etc.

A complete bundle runs **~ $580 in MJF PA12** (Xometry, mid-2026)
versus **~ $20 in PLA filament** if you have access to an FDM
printer.

---

## 4. Bill of materials

### 4.1 Actuators (the long pole)

| Item | Spec | Qty | Approx. cost |
|---|---|---|---|
| Hobby servo | **DS3225 25 kg·cm metal-gear digital servo**, standard 40 × 20 × 38 mm case, 54 mm tab span, ~49.5 mm tab-hole spacing, output offset ~10 mm from case centre. Buy all 20 from the same listing/batch. | 18 + 2 spare | ~$13 each on AliExpress, ~$18 each on Amazon ($260 – $360 total) |

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

| Bucket | DS3225 build |
|---|---:|
| Actuators | ~$300 |
| Power | ~$70 |
| Electronics | ~$50 |
| Fasteners + filament | ~$20 |
| **Total** | **~ $440** |

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

1. **Coxa bracket + yaw servo:** drop the yaw servo straight DOWN
   through the body cutout in the bracket flange and into the well
   below. The servo's mounting tabs land flush on the well rim, with
   the gear stack and output spline poking UP above the flange.
   Drive 4 × M3 self-tapping screws through the tab clearance holes
   into the four pilot holes drilled vertically through the well's
   side walls. The servo body now hangs below the chassis edge plane.
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

9. **Bottom chassis plate:** lay flat. Each coxa bracket's flange
   bolts to the chassis edge with **4 × M3 × 16 mm** caps — drive
   them straight down through the flange and through the matching
   bolt pattern in the chassis plate, capture with a nyloc on the
   underside. The two outboard bolts sit just inside the chassis
   perimeter; the two inboard bolts sit 16 mm further in. The flange
   sandwiches between the bottom plate and the standoff ring later
   in step 13.
10. **Stand-off posts:** screw 4 × M3 × 25 mm M-F standoffs into the
    inner bolt pattern.
11. **Battery holder + electronics tray:** bolt to the bottom plate
    using the same inner bolt pattern (the battery holder feet share
    the standoff bolt pattern, the tray sits adjacent).
12. **Wire it up:** see §7.
13. **Top chassis plate:** screw down onto the M3 standoff tops.

---

## 7. Wiring

### 7.1 One-servo bench test first

Before assembling the robot, test **one DS3225 + one PCA9685 + Arduino**
on the bench. This proves the exact servo listing you bought fits the
electrical and software assumptions before you print/bolt all 18 joints.

Bench wiring:

```text
Raspberry Pi / laptop  --USB serial-->  Arduino Mega
Arduino Mega SDA/SCL  ------------->  PCA9685 SDA/SCL
Arduino Mega 5V/GND   ------------->  PCA9685 VCC/GND  (logic only)
5-6 V BEC + / -       ------------->  PCA9685 V+ / GND (servo power)
DS3225 signal/+/-     ------------->  PCA9685 channel 0
```

Important: **all grounds must be common**: Pi USB ground, Arduino ground,
PCA9685 ground, and BEC/servo ground. Do **not** power the DS3225 from
the Arduino 5 V pin.

Upload the Arduino bridge sketch:

```bash
hexapod_walker/firmware/prototype_servo_bridge/prototype_servo_bridge.ino
```

Then from the Raspberry Pi or laptop:

```bash
python -m pip install pyserial
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 centre
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 0
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 joint 0 20 --sweep
```

On macOS the port will look like `/dev/cu.usbmodem...`; on Raspberry Pi /
Linux it is usually `/dev/ttyACM0` or `/dev/ttyUSB0`.

Once one servo works, plug the same servo into channels 1 and 2 and run:

```bash
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 1
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 2
```

That validates the yaw / hip / knee channel order for one leg before
you connect all 18 servos.

### 7.2 Full robot wiring

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

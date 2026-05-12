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
| 1 | Servo extensions | 20-pack, 3-pin male-female, 30 cm. | [Amazon: servo extension cable 30cm 20 pack](https://www.amazon.com/s?k=servo+extension+cable+30cm+20+pack) |
| 1 | Dupont jumper kit | Mixed M-F / F-F / M-M jumper wires for I2C and logic wiring. | [Amazon: dupont jumper wires 120 pcs](https://www.amazon.com/s?k=dupont+jumper+wires+120+pcs) |
| 1 | USB cable, Pi to Arduino | USB-A to USB-B if using Arduino Mega with full-size USB-B. If your clone uses USB-C or micro-USB, buy that cable instead. | [Amazon: USB A to B cable Arduino](https://www.amazon.com/s?k=USB+A+to+B+cable+Arduino) |
| 1 | M3 screw assortment | M3 socket-head screws, nuts, washers, lengths 6/8/10/12/16/20 mm. | [Amazon: M3 stainless screw kit assortment](https://www.amazon.com/s?k=M3+stainless+screw+kit+assortment) |
| 1 | M3 nyloc nuts | 100-pack nylon-insert lock nuts. Use on vibration-prone joints. | [Amazon: M3 nyloc lock nut 100 pack](https://www.amazon.com/s?k=M3+nyloc+lock+nut+100+pack) |
| 1 | M3 standoffs | M3 x 25 mm male-female brass standoffs, pack of 20. | [Amazon: M3 25mm standoffs male female brass](https://www.amazon.com/s?k=M3+25mm+standoffs+male+female+brass) |
| 1 | M2.5 screw pack | M2.5 x 8 mm screws, useful for horn/adapter work if the servo-included screws are bad. | [Amazon: M2.5 8mm screw 50 pack](https://www.amazon.com/s?k=M2.5+8mm+screw+50+pack) |
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

Files are in `hexapod_walker/stl_prototype/`.

| Qty | STL |
|---:|---|
| 1 | `chassis_top.stl` |
| 1 | `chassis_bottom.stl` |
| 1 | `battery_holder.stl` |
| 1 | `electronics_tray.stl` |
| 6 | `coxa_bracket.stl` |
| 6 | `coxa_link.stl` |
| 6 | `femur_link.stl` |
| 6 | `tibia_link.stl` |
| 6 | `foot_pad.stl` |
| 18 | `servo_horn_adapter.stl` |

For a one-leg test, print only:

| Qty | STL |
|---:|---|
| 1 | `coxa_bracket.stl` |
| 1 | `coxa_link.stl` |
| 1 | `femur_link.stl` |
| 1 | `tibia_link.stl` |
| 1 | `foot_pad.stl` |
| 3 | `servo_horn_adapter.stl` |

## Bench Test Order

1. Buy **one DS3225** first if you want to de-risk fit.
2. Print `coxa_bracket.stl`, `coxa_link.stl`, `femur_link.stl`, and one `servo_horn_adapter.stl`.
3. Confirm the DS3225 slides into all three wells and the tab holes line up.
4. Wire one DS3225 to PCA9685 channel 0 and run:

```bash
python hexapod_walker/pi_control/servo_bridge_client.py --port /dev/ttyACM0 wiggle --joint 0
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

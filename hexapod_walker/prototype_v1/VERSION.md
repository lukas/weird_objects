# Hexapod prototype — **v1 (FROZEN)**

This directory is a **frozen snapshot** of the hexapod prototype design built
around the **old DS3225-class PWM hobby servo**. It is intentionally stable:
treat it as a read-only baseline. Active development of the next revision
happens elsewhere (see *Directory map* below).

Frozen from git commit `60f228b`
("servo horn: finish disc-horn migration") on 2026-06-01.

## What v1 is

- **Actuator:** generic 25 kg·cm digital servo (DS3225 / MG996R class) —
  rectangular body with protruding mounting **tabs**, 25T spline output.
- **Horn:** 20 mm aluminium 25T **disc horn** on every joint.
- **Mounting:** servo drops into a printed **well/cradle** and is retained by
  4 vertical M3 screws (2 heat-set inserts + 2 self-tap pilots per cradle,
  "Design E" mixed mode).
- **Control:** **Arduino Mega** I²C master driving **2× PCA9685** PWM drivers;
  per-joint trims persisted in the Arduino's **EEPROM**.
  - Firmware: `firmware/prototype_servo_bridge/prototype_servo_bridge.ino`
  - Host client: `pi_control/servo_bridge_client.py`
  - Wiring/bench bring-up: `firmware/WIRING.md`
- **Power:** 3S LiPo → 5–6 V BECs for the servos, separate 5 V BEC for the Pi.
- **No external joint encoders** (the AS5600 experiment is not part of v1).

Builds clean: `python build_prototype_assembly.py` → 120,154 triangles.

## Directory map

| Directory | Servo | Status |
|-----------|-------|--------|
| `prototype_v1/` (this) | DS3225 PWM + Arduino/PCA9685 | **Frozen v1 baseline** |
| `prototype/` | (in flux) | Working tree, mid-migration |
| `prototype_sts3215/` | FEETECH STS3215 serial bus | New redesign (in progress) |

## Why it's frozen

The next revision replaces the DS3225 + PCA9685 + Arduino stack (and the
external-encoder experiment) with **FEETECH STS3215** smart serial-bus servos,
which carry their own 12-bit position feedback and bolt via **4× M2.5
case-face holes** instead of tabs. That is a large mechanical refit, so this
DS3225 design is preserved here as the last-known-good v1.

## How to build (reference)

```bash
cd hexapod_walker/prototype_v1
python build_prototype_assembly.py     # full assembly STLs
python _verify_prototype.py            # geometric verification suite
```

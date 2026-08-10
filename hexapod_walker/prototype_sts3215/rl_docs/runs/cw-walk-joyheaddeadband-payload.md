# cw-walk-joyheaddeadband-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:40:40+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband

**wandb_id**: t3tuzqug

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: chassis payload (1.0-1.4x mass, the joyheadfric-payload/joyfric-payload PASS envelope) x servo deadband (1.0-3.0x) on the widest +-90deg driving package (joyheaddeadband, seed-confirmed 2/2 this cycle). Payload has composed for free onto every friction-hardened +-90 driving package tried so far (joyheadfric-payload PASS); deadband is a different degraded-actuator axis never tried with payload together. If-true: JOYSTICK GATE @90 0 falls; own-cfg (DR0.5+lat+deadband+mass) det+sto 6/6 gv, 0 term, prog med>=0.80; DR0 nominal retention clean vs joyheaddeadband's own band (prog 0.90/0.93, slip ~1.5-1.8). If-false: the extra mass interacts with the dead-zone servo response to blow the retention caps, mirroring the joyhead90-payload-r1 FAIL (payload was NOT free without friction hardening).

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x+mass1.0-1.4x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 nominal retention det 6/6 gv, prog med and slip within noise of joyheaddeadband's own retention band; frames watched det

**verdict**: PASS: chassis payload (1.0-1.4x mass) composes onto the widest +-90deg deadband-hardened driving package. JOYSTICK GATE @90deg 0 in-envelope falls (all scenarios+flip-stress). Own-cfg (DR0.5+lat+deadband+mass) det/sto gv 6/6, 0 term, prog med 0.89/0.92 (>=0.80 gate). DR0 nominal retention det/sto gv 6/6, 0 term, prog 0.94/0.90, slip 1.44/1.51 -- inside joyheaddeadband's own retention band (prog 0.90-0.93, slip 1.5-1.8). Refutes the if-false branch (payload was NOT free without friction hardening in joyhead90-payload-r1) -- deadband hardening also protects payload composability, matching the friction-hardened lineages.


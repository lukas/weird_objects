# cw-walk-joyquad30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T04:54:34+00:00

**pod**: hexapod-mjx-train-10

**steps**: 12000000

**parent**: cw-walk-joylat25

**wandb_id**: bslsafxo

**hypothesis**: QUAD INTO THE MAINLINE (operator 08-10 00:4x: 'four leg trick in the main line so I can hit it with the joystick'): compose quad-hold (30% mix; 50% eroded walk in quad-hold1-r2, if-false branch) onto the DRIVING champion joylat25 (ruling-7 panel complete, DR0.5+latency+abrupt flips) instead of the plain walk champion. Command = two-hot goal one-hot, obs unchanged, warm start exact. One checkpoint that drives AND lifts fronts on a joystick button (drive_policy.py key 4 wired). If false: quad mix erodes the joystick gate -> ladder mix down to 0.2 or train quad-entry only from stops.

**gate**: JOYSTICK GATE @DR0.2 retained (0 in-envelope falls incl. flip-stress) AND own-cfg walk-mode det slip/m within joylat25 band (<=1.55) AND quad-mode: fronts_off >= 0.9, mean clear >= 20 mm, planted_frac >= 0.95 over final 10 s in >= 10/12 eps, |roll|,|pitch| <= 4 deg, 0 term


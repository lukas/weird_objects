# cw-uni-blend1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:05:53+00:00

**pod**: hexapod-mjx-train-4

**steps**: 18000000

**parent**: cw-uni-blend1

**wandb_id**: 4bwtkbhx

**hypothesis**: Same as cw-uni-blend1 (unified joystick policy line: driving champion + goal-mix walk=0.7/hold=0.1/rise=0.1/lower=0.1) but on sim >=273ebde where femur/tibia/knee-servo now collide with the floor. blend1 was killed 25min in because its rise/lower could exploit shins passing through the ground (operator caught it in MuJoCo). If-true: rise/lower learned here are floor-respecting and hardware-plausible.

**gate**: JOYSTICK GATE retention AND rise/lower >= 5/6 AND quiet hold; VIDEO: no leg-through-floor in rise/lower


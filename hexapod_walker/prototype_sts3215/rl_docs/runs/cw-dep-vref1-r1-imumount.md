# cw-dep-vref1-r1-imumount

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T07:33:28+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: ezy1o5g1

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's 25deg tilt-termination safety envelope is only as good as the IMU reading it's computed from, but the checkpoint has never trained with IMU mounting misalignment (10deg, validated elsewhere) -- directly relevant since the real IMU is hand-mounted and won't be perfectly axis-aligned with the chassis. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; IMU-mount offset composes free like it does elsewhere. If-false: a systematic tilt-reading bias interacts with the 25deg safety threshold in a new way (early/late trips, or the policy exploits the bias) -- a real pre-hardware risk to flag.

**gate**: Own-cfg (DR0.35+imu_mount10deg) det+sto 6/6 @15s: gait_valid 6/6, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto); DR0 no-offset retention clean; frames watched det

**verdict**: PASS -- IMU mounting-rotation misalignment (10deg residual after calib) composes free onto vref1-r1. Own-cfg (DR0.35+offset) det+sto gv 6/6, 0 term, det slip/m med 0.97 sto med 0.92 -- inside vref1-r1's own band. DR0 no-offset retention gv 6/6, 0 term, det slip/m med 0.88 sto med 0.99 (same known lineage fixed-draw sto/4 crater, slip 4.93 vs parent's 5.97, same draw index). Own-cfg det/5+sto/0+sto/1 crater cluster reproduces at the same indices/magnitudes as the gyronoise and latency siblings -- a shared DR0.35+seed0 lineage draw, not specific to IMU-mount offset. No early/late safety-trip exploit seen: 0 terminations in either pass, so the 25deg tilt threshold isn't being gamed by the rotated tilt reading. Frames clean six-leg creep, no flag leg/fall. Training finished clean (reward quarters 600/687/663/651).


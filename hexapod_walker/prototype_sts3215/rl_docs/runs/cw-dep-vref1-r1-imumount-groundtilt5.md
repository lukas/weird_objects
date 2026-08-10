# cw-dep-vref1-r1-imumount-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T17:53:58+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: gp0slp1b

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's two individually-PASSed tilt-related axes -- IMU mount-rotation offset (10 deg residual from -imumount) and real floor slope (5 deg from -groundtilt5) -- have never been exposed TOGETHER. This pairing is safety-critical: both axes bias the SAME tilt reading the 25 deg deploy trip relies on, and a real install can plausibly have both a slightly-misaligned IMU mount AND an unlevel floor at once. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band, 0 spurious/missed 25 deg trips -- the two tilt-biasing axes don't compound into an unsafe or broken tilt reading. If-false: combined mount+floor tilt bias pushes the effective reading enough to mistrigger or fail to trigger the safety trip, or breaks gait -- a real pre-attempt-#2 safety risk to flag before deployment.

**gate**: own-cfg (DR0+imumount10deg+groundtilt5deg) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36), 0 unexpected safety-trip terminations; video frames watched det+sto for flag-leg/skate

**verdict**: PASS -- the two individually-PASSed tilt-BIASING axes (10 deg IMU-mount rotation residual + 5 deg real floor slope) that both corrupt the SAME tilt reading the 25 deg safety trip relies on compose free together. Own-cfg (DR0+imumount10deg+groundtilt5deg) det+sto gv 6/6 each, 0 term, 0 safety_flags, 0 sacrificed legs in EITHER pass; det slip/m med 1.15 (~2% over vref1-r1's own 1.13 det ceiling, inside +-20% tol), sto slip/m med 0.98 (comfortably inside 1.13-1.36 band). The one degraded draw (det/4, prog 0.12) is the lineage's known fixed-seed march-in-place crater seen in every DR0.35 sibling compose tonight -- video-checked (det/0 clean six-leg gait; det/4 crater still six legs cycling, level body, no flag-leg/drag). 0 spurious or missed 25 deg trips across 12 episodes -- the combined mount+floor tilt bias does not mistrigger or defeat the safety threshold.


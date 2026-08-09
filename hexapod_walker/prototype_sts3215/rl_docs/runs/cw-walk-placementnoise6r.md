# cw-walk-placementnoise6r

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T22:41:30+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: Retry of cw-walk-placementnoise6 (lost a launch-collision race amid a concurrent-cycle drain storm, worker EOFError at init, 0 steps -- no science result). Same hypothesis: new 13b axis, per-joint hand-placement/assembly slop (dr.placement_noise_deg=6.0, 3x default) off champion at DR0. Champion baseline already measured (NOT free: det prog 0.72, slip 2.40 vs clean 0.94-0.96 band). If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.1m, DR0 no-noise retention clean. If-false: axis not trainable at this magnitude.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.placement_noise_deg=6.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; plus DR0 no-noise retention det 6/6 gv, det slip/m med <=1.24; frames watched det


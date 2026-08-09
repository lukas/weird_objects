# cw-walk-imupos15-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T22:42:51+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-imupos15

**hypothesis**: RETRY #2 (both prior attempts crashed: first likely resource contention, second lost a launch-collision race amid a concurrent-cycle drain storm — worker EOFError at init, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: isolated dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02 axis off champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det


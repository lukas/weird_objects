# cw-walk-gyrobias3-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:40:17+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-gyrobias3

**wandb_id**: s5fcpoet

**hypothesis**: RETRY (first attempt lost a launch-collision race amid a concurrent-cycle drain storm — worker EOFError at init, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: isolated dr.gyro_bias_deg_s=3.0 axis off champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.gyro_bias_deg_s=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det


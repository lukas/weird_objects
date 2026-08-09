# cw-walk-tiltnoise-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:43:32+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-tiltnoise

**hypothesis**: RETRY (first attempt lost a launch-collision race amid a concurrent-cycle drain storm — worker EOFError at init, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: isolated dr.tilt_noise_deg=1.5 axis off champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


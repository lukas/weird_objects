# cw-walk-tiltnoise-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:07:52+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-tiltnoise-r3

**hypothesis**: 5th launch attempt (base+r1+r2+r3 all died 0-step to launch-collision storms). Same hypothesis: IMU tilt-noise exposure, dr.tilt_noise_deg=1.5 isolated.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


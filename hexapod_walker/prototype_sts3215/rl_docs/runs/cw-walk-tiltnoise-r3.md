# cw-walk-tiltnoise-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:57:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-tiltnoise-r2

**hypothesis**: OPERATOR WISHLIST 13c untested axis (IMU tilt-reading NOISE): per-step measurement noise on attitude estimate, dr.tilt_noise_deg=1.5 isolated. 3rd launch attempt (base + r1 + r2 all died to launch-collision storms, 0 steps, no science). Same hypothesis as base.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


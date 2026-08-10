# cw-walk-groundtilt5-payload-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:29:47+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-walk-groundtilt5-payload-r2

**hypothesis**: 4th launch attempt (1st-3rd all died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: slope(5deg)+payload(1.0-1.5x) compose off cw-walk-groundtilt5.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal (no tilt, no payload) retention det 6/6 gv, det slip/m <=1.24; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


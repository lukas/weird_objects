# cw-walk-groundtilt8-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:18:57+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-groundtilt8

**hypothesis**: Retry of cw-walk-groundtilt8 (lost the fleet-wide launch-collision race, gotcha 13b, 0 steps trained — no science result). Same spec unchanged: 8deg tilt ladder rung off groundtilt5's own checkpoint.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to groundtilt5 at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


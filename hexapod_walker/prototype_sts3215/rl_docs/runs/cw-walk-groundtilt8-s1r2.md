# cw-walk-groundtilt8-s1r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:57:01+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-groundtilt8-s1r1

**hypothesis**: 3rd launch attempt of the groundtilt8 seed-1 twin (s1 FAILED to launch-collision gotcha13b, s1r1 FAILED to same EOFError with 27 leaked hexmjx shm segments on train-0 -- cleaned before this retry). Same hypothesis unchanged: seed-1 twin of groundtilt8-r3 to confirm/refute the 3/6 crater fraction as a real physics effect vs seed luck.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare crater fraction (currently 3/6) to seed0

**failed_reason**: run never appeared as 'running' in W&B within 240s


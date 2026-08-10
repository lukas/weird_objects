# cw-walk-multiaxis-dr05-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:31:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis-dr05-r1

**hypothesis**: Retry of cw-walk-multiaxis-dr05-r1 (lost the fleet-wide launch-collision race, gotcha 13b, 0 steps trained -- no science result). Same spec unchanged: +14M step continuation of multiaxis-dr05 (own-cfg 4-axis+DR0.5 panel was clean but DR0 retention slip missed the 1.24 cap at 1.27, testing whether more steps close the gap or it's a hard ceiling).

**gate**: Own-cfg harness DR0.5+4-axis panel det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m; DR0 nominal retention det 6/6 gv, det slip/m <=1.24 (the cap missed at 1.27 with fewer steps); frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


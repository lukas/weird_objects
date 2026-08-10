# cw-walk-multiaxis2-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:49:59+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis2-r2

**hypothesis**: 3rd attempt to continue multiaxis2 for +14M steps (same 5-axis DR0 compose, own-cfg clean but DR0 retention missed cap at 1.30 vs 1.24). r1 and r2 both died at init in the fleet launch-collision storm (gotcha 13b) with 0 steps -- this is a mechanical retry of the same spec, not a new hypothesis.

**gate**: Own-cfg DR0+5axis panel det+sto 6/6: gait_valid 12/12, 0 term, det med fwd>=1.2m (retain); PRIMARY: DR0 nominal retention det slip/m med <=1.24 (the exact cap missed at 1.30 before more steps) AND det med fwd within 0.1m of multiaxis1's clean 1.54m band.

**failed_reason**: run never appeared as 'running' in W&B within 240s


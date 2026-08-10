# cw-walk-multiaxis-dr05-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:17:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis-dr05

**hypothesis**: Continuation of the ONE near-miss (mirrors the multiaxis2-r1 idea a concurrent cycle already queued for multiaxis2): multiaxis-dr05 (4-axis stack + generic DR0.5) passed its own-cfg exposure gate clean but missed the pre-registered DR0 nominal-retention cap by a small real margin (det slip med 1.27 vs 1.24 cap) at 20M steps. Continue training the SAME checkpoint +14M steps, same config, no new variable, to test if the miss is a step-budget issue or a genuine ceiling. If-true: DR0 retention slip drops back under 1.24 -- just needed more steps. If-false: stays >1.24 -- axis-stacking-plus-generic-DR past 4 axes is a genuine capacity ceiling at this model size, ship multiaxis1 as the base and stop growing this particular stack.

**gate**: Own-cfg DR0.5+4axis panel det+sto 6/6: gait_valid 12/12, 0 term, det med fwd>=1.1m (retain); PRIMARY: DR0 nominal retention det slip/m med <=1.24 (the exact cap missed at 1.27 before more steps) AND det med fwd within 0.1m of multiaxis1's clean band.

**failed_reason**: run never appeared as 'running' in W&B within 240s


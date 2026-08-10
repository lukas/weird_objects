# cw-walk-multiaxis2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:04:52+00:00

**pod**: hexapod-mjx-train-0

**steps**: 14000000

**parent**: cw-walk-multiaxis2

**hypothesis**: Continuation of the ONE near-miss (not a new axis): multiaxis2 (5-axis DR0 compose: multiaxis1's 4 + ground_tilt_deg=5) passed its own-cfg exposure gate clean but missed the pre-registered DR0 nominal-retention cap by a small real margin (det slip med 1.30 vs 1.24 cap) at 18M steps -- same pattern as multiaxis-dr05 (4-axis + generic DR0.5, also missed at 1.27). Question: is the retention miss a genuine axis-count ceiling, or just under-trained consolidation? Continue training the SAME checkpoint +14M steps, same config, no new variable. If-true: DR0 retention slip drops back under 1.24 with more steps -- axis-stacking just needed a bigger step budget, not a hard ceiling. If-false: retention stays >1.24 or gets worse -- axis-stacking past 4 (or adding generic DR) is a genuine capacity ceiling at this model size, not a step-budget issue; stop growing the multiaxis stack and ship multiaxis1 as the base.

**gate**: Own-cfg DR0+5axis panel det+sto 6/6: gait_valid 12/12, 0 term, det med fwd>=1.2m (retain); PRIMARY: DR0 nominal retention det slip/m med <=1.24 (the exact cap missed at 1.30 before more steps) AND det med fwd within 0.1m of multiaxis1's clean 1.54m band.

**verdict**: Never trained: died at init with EOFError (0 steps) - the known fleet-wide launch-collision storm under heavy concurrent-drain contention (COMMANDS.md gotcha, c67 pattern), not a science result. Infra fault, hypothesis NOT TESTED. Requeued as -r2.

**failed_reason**: run never appeared as 'running' in W&B within 240s


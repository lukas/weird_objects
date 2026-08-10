# cw-uni-mix0

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:34:11+00:00

**pod**: hexapod-mjx-train-11

**steps**: 18000000

**parent**: cw-uni-mix20-r1

**hypothesis**: Extreme rung of the inverse mix-ladder (c69): ZERO walk share, pure rise/lower+hold skill acquisition off cw-uni-blend1-r2. Isolates whether rise/lower CAN be learned by this reward/env at all, decoupled from any walk-share competition (mix20/mix40 still confound partial walk share with under-training). Prediction-if-true: rise/lower success climbs off zero within 18M steps (removes doubt that income-scale, not walk-competition, is the blocker). Prediction-if-false: rise/lower success stays flat 0 even with 100% of gradient budget -- points to a reward-shaping/income defect in the rise/lower terms themselves, not a mixing-ratio problem.

**gate**: own-cfg DR0.5 rise/lower success >=5/6 det each by 18M steps (checked via reward curve + eval at intermediate ckpt if time allows); hold quiet (height_err_end<=8mm); VIDEO: no leg-through-floor

**failed_reason**: run never appeared as 'running' in W&B within 240s


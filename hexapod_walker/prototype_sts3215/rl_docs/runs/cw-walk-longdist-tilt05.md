# cw-walk-longdist-tilt05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T14:28:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 1tnmspx7

**hypothesis**: 0-c(i) tilt lever folded into the CHAMPION line, A/B partner of cw-walk-longdist-dr05 (identical 30s/0.05-0.06 DR0.5 config, one variable: reward.k_roll=50,k_pitch=50). cw-walk-dr05-tilt50 showed tilt pricing eliminates det slip blowouts + flag legs at DR0.5 on the dr05-r1 line but cannot touch the draw-specific sto stalls. If-true: vs the A/B partner's same-eval baseline, det DR0.5 has no ep slip/m>3, gv 12/12, det slip median <= partner's minus noise, DR0 retention intact. If-false (no delta vs partner beyond noise, or det slip inflates): tilt pricing does not transfer to the champion line and the 0-c(i) tilt lever closes; stability then rides purely on DR training.

**gate**: own-cfg DR0.5 6+6 vs cw-walk-longdist-dr05 same-eval baseline: 0 term, gv 12/12, det no ep slip/m>3, det slip/m median <= partner's; DR0 det retention gv 6/6 prog med 0.75-1.25; frames watched det


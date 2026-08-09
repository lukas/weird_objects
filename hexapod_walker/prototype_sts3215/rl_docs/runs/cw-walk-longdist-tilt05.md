# cw-walk-longdist-tilt05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T14:28:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 1tnmspx7

**hardware_ready**: no

**hypothesis**: 0-c(i) tilt lever folded into the CHAMPION line, A/B partner of cw-walk-longdist-dr05 (identical 30s/0.05-0.06 DR0.5 config, one variable: reward.k_roll=50,k_pitch=50). cw-walk-dr05-tilt50 showed tilt pricing eliminates det slip blowouts + flag legs at DR0.5 on the dr05-r1 line but cannot touch the draw-specific sto stalls. If-true: vs the A/B partner's same-eval baseline, det DR0.5 has no ep slip/m>3, gv 12/12, det slip median <= partner's minus noise, DR0 retention intact. If-false (no delta vs partner beyond noise, or det slip inflates): tilt pricing does not transfer to the champion line and the 0-c(i) tilt lever closes; stability then rides purely on DR training.

**gate**: own-cfg DR0.5 6+6 vs cw-walk-longdist-dr05 same-eval baseline: 0 term, gv 12/12, det no ep slip/m>3, det slip/m median <= partner's; DR0 det retention gv 6/6 prog med 0.75-1.25; frames watched det

**verdict**: FAIL / NO-EFFECT on pre-registered if-false. Tilt pricing (k_roll=k_pitch=50) folded into the champion line does nothing vs the A/B partner longdist-dr05 at same-eval DR0.5: det slip/m med 1.051 vs 1.123 (per-ep ranges fully overlap = inside noise), agg 1.091 vs 1.088, attitude err med 0.96 vs 1.06 deg, same sto ep3 draw degrades both (2.84/0.55 vs 2.75/0.56). Gate scalars technically met (gv 12/12, 0 term, no det ep>3, DR0 retention gv 6/6 prog med 0.95) but the decision clause was 'median <= partner minus noise' — not met. Frames det watched: level six-leg cycling, indistinguishable from partner. 0-c(i) tilt lever CLOSED on the champion line; stability rides on DR training. hardware-ready: no.


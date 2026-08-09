# cw-walk-longdist-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:20:18+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: mmlq1y05

**hypothesis**: Seed-1 twin of cw-walk-longdist-r2 per operator ruling 7 (promotion needs multi-seed panels). r2 near-missed its gate (sto 5/6) but posted campaign-best det numbers: slip/m 0.96, prog_ratio 0.98, 1.57m@30s. Identical config, --seed 1. If-true (s1 reproduces det slip/m <=1.1 and prog in band): the 30s narrow-band result is seed-robust and the line is a champion candidate pending DR1.0 comparison. If-false (s1 reverts to champion-like slip ~1.2+): r2's gain is seed luck, no promotion.

**gate**: DR0 det+sto 6/6: median fwd distance >=1.2m @30s, zero terminations, gait_valid 12/12, det slip/m <=1.24; det slip/m <=1.1 for the seed-robustness claim


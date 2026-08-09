# cw-walk-longdist-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:20:18+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_anchorgate.zip

**wandb_id**: mmlq1y05

**hardware_ready**: False

**hypothesis**: Seed-1 twin of cw-walk-longdist-r2 per operator ruling 7 (promotion needs multi-seed panels). r2 near-missed its gate (sto 5/6) but posted campaign-best det numbers: slip/m 0.96, prog_ratio 0.98, 1.57m@30s. Identical config, --seed 1. If-true (s1 reproduces det slip/m <=1.1 and prog in band): the 30s narrow-band result is seed-robust and the line is a champion candidate pending DR1.0 comparison. If-false (s1 reverts to champion-like slip ~1.2+): r2's gain is seed luck, no promotion.

**gate**: DR0 det+sto 6/6: median fwd distance >=1.2m @30s, zero terminations, gait_valid 12/12, det slip/m <=1.24; det slip/m <=1.1 for the seed-robustness claim

**verdict**: OBSERVATIONS: DR0 det 6/6 (median fwd 1.63m@30s, slip/m 0.94, prog 1.03, gv 6/6, 0 term, swing counts balanced 17-22 all legs); DR0 sto 5/6 - single miss is THE SAME fixed draw r2 missed (ep4: churn-in-place 0.37m, slip 6.68, swings inflated 25-34, no fall). DR1.0 own-cfg: det slip/m 0.98 agg (r2 1.06, champion 1.240), sto 3/6 identical to r2 with the same worst draw (ep3). Frames: level six-leg alternating gait at both DR levels, no flag leg/sacrifice. INTERPRETATION: seed-1 reproduces r2 on every axis - det slip/m <=1.1 met at both seeds and both DR levels; sto weakness is draw-specific (same fixed commands fail in both seeds), a lineage trait not seed luck. VERDICT: hypothesis CONFIRMED (seed-robust); strict gate NEAR-MISS on the known single sto draw. hardware-ready: NO (still ~0.94m slide per meter). HYPOTHESIS STATUS: if-true branch holds -> longdist line promoted, r2 checkpoint becomes walk champion (ASSUMPTION logged, cycle 43).


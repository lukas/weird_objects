# cw-walk-multiaxis1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T21:34:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 18000000

**parent**: cw-walk-multiaxis1

**wandb_id**: a5nxh280

**hypothesis**: Seed twin of the c63 multiaxis1 PASS (ruling-7 practice before leaning on a recipe): identical 4-axis compose config (payload 1.0-1.4x + latency 0.5-2.5x + deadband 1-3x + CoM +30mm at dr-scale 0) off the champion, seed 1 instead of 0. The compose recipe is slated as robustness-champion base - a twin tells us it is the recipe, not seed luck. Prediction-if-true: seed 1 matches - own-cfg 4-axis panel gv 12/12, 0 term, det med fwd >=1.2m, DR0 nominal retention clean, heavy tail no worse than 2/6 slow-shuffle draws. Prediction-if-false: seed 1 collapses on mid-range draws or charges nominal retention - compose is seed-fragile, panel before promotion. Strongest alternative: both seeds pass with different tail signatures - compare per-episode fwd spread. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at dr-scale 0 + all four dr overrides, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det


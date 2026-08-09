# cw-walk-payload50-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T19:37:18+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: l5crlij0

**hardware_ready**: no

**hypothesis**: Seed twin of the c57 payload50 PASS (ruling-7 seed-panel practice): identical config, seed 1 instead of 0. The payload axis PASSed with a marginal heavy tail (2/6 det draws squat-shuffle at 1.4-1.5x mass) - a seed twin tells us whether 'solid to +40%' is the recipe or seed luck before we lean on it (payload-dr05 compose already in flight). Prediction-if-true: seed 1 matches - own-cfg mass 1.0-1.5x gv 12/12, 0 term, det med fwd >=1.2m, heavy-tail no worse. Prediction-if-false: seed 1 collapses on mid-range mass draws or retention erodes - payload exposure is seed-fragile, panel before promotion. Strongest alternative: both seeds pass but with different tail signatures - compare per-episode fwd spread. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 no-payload retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: PASS — seed twin CONFIRMS the payload recipe (ruling-7): own-cfg mass 1.0-1.5x panel gv 12/12, 0 term, det med fwd 1.32m (seed0 1.31); DR0 no-payload retention gv 6/6, det slip/m 1.17<=1.24 (seed0 1.15); heavy tail identical to seed0 (2/6 det draws prog 0.39-0.47, slip 3.6-3.9 vs 3.4-3.8) — same signature, not worse. "Solid to ~+40%, 1.4-1.5x marginal" is the recipe, not seed luck. hardware-ready: no (paddle lineage).


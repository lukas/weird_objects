# cw-walk-wander30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T13:50:30+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander.zip

**wandb_id**: e4wuad6b

**hypothesis**: Driving endurance: cw-walk-wander PASSED start/steer/stop transitions over 15s (~3 command changes). One variable off it: horizon 15->30s (~6 changes/ep). Plain: prove the drive-it-around skill doesn't decay over longer drives. Prediction-if-true: 30s eps hold prog_ratio ~1.0 with gv and no term across all command changes. Prediction-if-false: degradation accumulates late-episode (height sag, slip growth, parked segments after later changes) -> transitions are only locally stable. Strongest alternative: fine at 30s but slip on change-segments stays ~2x straight-line (contact-pricing root, not fixable here).

**gate**: own-cfg DR0 30s 6+6: gv 12/12, 0 term, prog_ratio median 0.85-1.15, no ep prog<0.5; change-segment slip no worse than parent (~2.1 slip/m ep-level); frames watched det+sto


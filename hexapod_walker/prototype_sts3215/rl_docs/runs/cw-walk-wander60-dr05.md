# cw-walk-wander60-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:28:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander_dr05.zip

**wandb_id**: g83cguon

**hypothesis**: Steering line convergence rung: minute-long drives (wander60 c56 PASS at DR0: no endurance decay over ~12 changes/ep) x physics robustness (wander-dr05 c50 PASS at 15s, seed-robust via s1 c56). One variable off the wander-dr05 checkpoint: episode-seconds 15 -> 60 at the same DR 0.5. Plain: the operator's joystick sessions are minutes long on a real, imperfect robot - this is the closest sim rung to that. Prediction-if-true: 60s eps at DR0.5 hold gv 12/12, 0 term, prog med >=0.85 with no ep <0.5, slip/m med <=2.4 and no second-half degradation - minute-long robust driving banked. Prediction-if-false: DR-induced errors accumulate over long horizons (late-episode stalls/parks or slip growth) - robustness IS horizon-limited and the line needs recovery-focused training, a distinct finding neither parent shows. Strongest alternative: passes but with the known 1/6 sto brittleness worsening (>2/12 low-prog eps) - report the corner, keep wander-dr05 as line champion.

**gate**: own-cfg DR0.5 60s 6+6 resampled cmds: gv 12/12, 0 term, prog_ratio med >=0.85, no ep prog <0.5, slip/m med <=2.4, worst-ep <=2.6; DR0 det retention 6 eps gv 6/6; frames watched det full horizon


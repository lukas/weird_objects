# cw-walk-wander60-dr05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T18:28:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander_dr05.zip

**wandb_id**: g83cguon

**hardware_ready**: no

**hypothesis**: Steering line convergence rung: minute-long drives (wander60 c56 PASS at DR0: no endurance decay over ~12 changes/ep) x physics robustness (wander-dr05 c50 PASS at 15s, seed-robust via s1 c56). One variable off the wander-dr05 checkpoint: episode-seconds 15 -> 60 at the same DR 0.5. Plain: the operator's joystick sessions are minutes long on a real, imperfect robot - this is the closest sim rung to that. Prediction-if-true: 60s eps at DR0.5 hold gv 12/12, 0 term, prog med >=0.85 with no ep <0.5, slip/m med <=2.4 and no second-half degradation - minute-long robust driving banked. Prediction-if-false: DR-induced errors accumulate over long horizons (late-episode stalls/parks or slip growth) - robustness IS horizon-limited and the line needs recovery-focused training, a distinct finding neither parent shows. Strongest alternative: passes but with the known 1/6 sto brittleness worsening (>2/12 low-prog eps) - report the corner, keep wander-dr05 as line champion.

**gate**: own-cfg DR0.5 60s 6+6 resampled cmds: gv 12/12, 0 term, prog_ratio med >=0.85, no ep prog <0.5, slip/m med <=2.4, worst-ep <=2.6; DR0 det retention 6 eps gv 6/6; frames watched det full horizon

**verdict**: PASS — minute-long robust driving banked: own-cfg DR0.5 60s panel (±45° resamples/5s, 15% stops) gv 12/12, 0 term, prog med 0.94 det/0.93 sto (gate ≥0.85, min ep 0.90 — no ep <0.5), slip/m med 1.39/1.44 (gate ≤2.4), worst-ep 1.57 (gate ≤2.6); DR0 det retention gv 6/6, prog 0.98, slip 1.32. No second-half degradation and the known 1/6 sto brittleness did NOT worsen (0 low-prog eps). Frames det full 60s watched: level six-leg cycling through command changes and stops, no flag leg, no late sag. DR-induced errors do NOT accumulate over long horizons. Steering-line convergence rung LANDED; seed twin queued (ruling 7). Paddle slip root — not hardware-ready.


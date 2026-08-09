# cw-walk-velsag30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:44:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: 18ai44t8

**hypothesis**: WISHLIST 13b/13c axis (servo speed sag): STS3215 profile speed drops under load and battery voltage sag - the fitted sim speed is a fresh-battery bench number. ISOLATED axis off the no-DR champion: dr-scale 0.0 + ONLY dr.vel_scale=0.70,1.10 (default DR low side is 0.85; real sag can be deeper). Plain: the walk must survive servos up to 30% slower than calibrated. If-true: gait tolerates slow servos (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - voltage-sag robustness banked. If-false: slow servos break swing timing (feet land late, gait degrades to dragging or terminations) - deployment needs a battery-state input or conservative speed calibration. Strongest alternative: policy just walks slower and under-tracks the command band - prog_ratio will show it without gait damage.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.vel_scale=0.70,1.10, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 nominal-speed retention det 6/6 gv, det slip/m <=1.24; frames watched det


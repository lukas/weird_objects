# cw-walk-nv

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-long5m

**parent**: ppo_goal_cw_walk_dr04b.zip

**wandb_id**: 8g6mggws

**checkpoint**: pod long5m: rl_move/sim/policies/ppo_goal_cw_walk_nv.zip

**hypothesis**: goal.walk_obs_body_vel=0: tracking re-learnable from proprioception+IMU only (deployable obs)

**gate**: sto walk >=4/6 @ vel_err <=0.035

**verdict**: FAIL at 4M (round 8): sto walk 1/6 @ 0.035; full flag-leg pathology, rise broken; raise unexpectedly 5-6/6 (noted, not evidence). Continuation cw-walk-nv2 to 8M is the fixed-budget baseline for the aac comparison. NOT HARDWARE-READY.


# cw-walk-flagw

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-s3

**parent**: ppo_goal_cw_walk_dr04b.zip

**wandb_id**: 8n5eo6zj

**checkpoint**: pod s3: rl_move/sim/policies/ppo_goal_cw_walk_flagw.zip (md5 c99d9be2d70a1b57a96b8ded5bfcffe7)

**hypothesis**: walk-only flag routing (reward.flag_leg_walk_only=1, k_flag_leg=5.0) suppresses the flag leg without the rise/raise collapse of all-modes routing

**gate**: sto walk >=4/6 @ vel_err <=0.030 AND video: six-foot gait, no flag leg AND sto rise >=4/6

**verdict**: FAIL (cycle 10): sto walk 0/6 gait-valid @ vel_err 0.032 (det 0/6 @ 0.040); two exploit modes - tripod anchor (legs 1/3/5 airborne, duty ~1.0 on 0/2/4) and 5-leg shuffle (leg 3 parked); reward_flag_leg still -0.75/step at 4M. Retention half SUPPORTED: rise 6/6 det+sto, raise 6/5. Suppression half REFUTED. NOT HARDWARE-READY. Champion unchanged. Flag-leg penalty dead as a gait fix.


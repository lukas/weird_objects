# cw-walk-flag

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**pod**: hexapod-sweep-s3

**parent**: ppo_goal_cw_walk_dr04b.zip

**wandb_id**: 2r0jj2qq

**checkpoint**: pod s3: rl_move/sim/policies/ppo_goal_cw_walk_flag.zip

**hypothesis**: all-modes reward.k_flag_leg=5.0 removes the lineage flag leg without losing tracking

**gate**: sto walk >=4/6 @ vel_err <=0.030 AND video: no foot >50mm except swing AND sto rise >=4/6

**verdict**: FAIL (round 8): sto walk 2/6 @ 0.032, rise collapsed (bridge 0/5), raise 0/6; video still a 5-leg shuffle. Hypothesis REFUTED - all-modes routing fights rise's transient swings. NOT HARDWARE-READY.


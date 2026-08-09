# cw-walk-aac-s1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T19:38:59+00:00

**pod**: hexapod-sweep-s3

**steps**: 3720000

**parent**: ppo_goal_cw_walk_aac_s1_25044096_steps.zip (own lineage; ultimate parent ppo_goal_cw_walk_dr04b)

**wandb_id**: gmbwrp4v

**hypothesis**: Rebalance continuation of cw-walk-aac-s1 (asym-critic seed-1 twin): privileged critic preserves the value signal so a hardware-obs actor learns tracking; killed on starving 30-core lower (75 fps), continued on 56-core s3

**gate**: sto walk >=4/6 @ vel_err <=0.035 with hardware-legal actor AND beat cw-walk-nv 4M mark; final vs nv2 @ 8M; blunt video verdict + gait-validity gate

**verdict**: FAIL on walk at 4M (cycle 11b): det/sto 0/6 gait-valid @ vel_err 0.036, leg 3 parked 12/12 - tracking EQUAL to blind nv at matched steps (0.036 vs 0.035, within eval noise); retention half SUPPORTED: rise 11/12 vs nv 3/12, raise 4-5/6. Video: same 5-leg scoot, NOT WALKING, NOT HARDWARE-READY. Fixed-budget comparison completes at 8M via continuation cw-walk-aac-s1c. ckpt md5 0642955c07b2d2e59dc81dd4f8eb5c8c.


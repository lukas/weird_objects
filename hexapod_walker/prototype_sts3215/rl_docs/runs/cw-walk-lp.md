# cw-walk-lp

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED_BY_OPERATOR

**pod**: hexapod-sweep-walk

**parent**: ppo_goal_cw_walk_dr04b.zip

**wandb_id**: 0ff8idlz

**hypothesis**: LP-weighted speed-bucket sampling (goal.walk_lp_curriculum=1, 8 buckets 0.02-0.12) widens tracked speed without the manual-widening regressions

**gate**: sto walk >=4/6 @ vel_err <=0.030 on 0.02-0.06 AND sto mean vel_err <=0.045 uniform 0.02-0.12 AND blunt video review


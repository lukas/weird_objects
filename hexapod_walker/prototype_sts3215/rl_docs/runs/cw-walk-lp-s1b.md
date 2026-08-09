# cw-walk-lp-s1b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T19:46:52+00:00

**pod**: hexapod-sweep-s4

**steps**: 4570000

**parent**: ppo_goal_cw_walk_lp_s1_25188096_steps.zip (own lineage; ultimate parent ppo_goal_cw_walk_dr04b)

**wandb_id**: 4qe1gv3z

**hypothesis**: Continuation of cw-walk-lp-s1 (LP speed-curriculum seed-1 twin, died silently on lower @25.19M): LP-weighted sampling of 8 speed buckets 0.02-0.12 widens tracked speed without manual-widening regressions

**gate**: retention sto walk >=4/6 @ vel_err <=0.030 on 0.02-0.06 AND sto mean vel_err <=0.045 over uniform 0.02-0.12 AND blunt video review + gait-validity gate

**verdict**: FAIL both halves (cycle 11b): retention sto 0/6 gait-valid @ 0.032 (leg 3 parked / tripod anchor); uniform 0.02-0.12 sto vel_err 0.059 (gate <=0.045), speed pinned ~0.03 m/s at all commands, tripod tower in video. det rise eroded to 3/6. LP curriculum REFUTED - no bucket above the shuffle ceiling improves, matching speedhi FALSE branch. Line closed. NOT HARDWARE-READY. ckpt md5 972966bee997cd722bc24083a693d416.


# cw-walk-aac-s1c

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-08T21:00:40+00:00

**pod**: hexapod-sweep-s3

**steps**: 4000000

**parent**: ppo_goal_cw_walk_aac_s1b.zip (md5 0642955c, in place on s3)

**wandb_id**: ij6xowjy

**hypothesis**: Continuation of cw-walk-aac-s1b to complete the fixed-budget deployable-obs comparison at 8M (review 6): nv baseline ran its full 8M and was called (sto 0/6 gait-valid @ 0.035, rise 3/12); aac at 4M ties on tracking (0.036) but retains rise 11/12. If-true (asym critic matters): at 8M aac tracking pulls ahead of the called nv numbers or retention gap persists at matched budget -> temporal actor builds on aac. If-false: aac at 8M identical to nv at 8M on all axes -> critic obs was not the binding constraint, temporal actor builds on whichever ckpt retains rise better. Same settings as aac-s1b, plain warm start of the asym ckpt.

**gate**: at 32.77M cum (8M asym): beat called nv 8M baseline (sto walk 0/6 gait-valid @ 0.035, rise sto 3/6) on tracking OR retention at matched budget; gait-validity gate + blunt video; sto rise >=4/6 retained

**verdict**: SPLIT at 8M (cycle 12): retention half SUPPORTED - rise height-only det 6/6 / sto 5/6 vs nv 3/12 at matched budget; tracking half REFUTED - walk det 0/6 gait-valid @ 0.038 / sto 0/6 @ 0.031 vs nv 0.035, inside the calibrated noise band. Gate as written (beat nv on tracking OR retention) technically passes on retention, but end_posture (new eval gate) is 0/6 in EVERY stance-ending mode, worst clear 342-385mm: every skill ends with a ~35cm vertical antenna leg; det walk is a full tripod park (legs 1/3/5 duty 0.02-0.06). Video: three-leg tower scoot. NOT WALKING, NOT HARDWARE-READY. Keep --asym-critic as retention tool for warm continuations; dr04b lineage retired as warm-start source for GAIT experiments (0-for-9 on gait validity). Walk champion unchanged. ckpt md5 f7ed9a6c48cbd196d2d25b1d382da41f.


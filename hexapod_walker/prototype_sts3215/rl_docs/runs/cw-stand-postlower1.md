# cw-stand-postlower1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-14T22:19:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 6000000

**parent**: cw-stand-footlow2-hard1

**wandb_id**: 694a7hx1

**hypothesis**: Teach the stance policy to stand back up out of the exact pose its own sit-down leaves it in - the single boundary the 1,800-session joystick gate left failing (100% of the product baseline's det session failures, sto 0.801). This arm tests whether exposing 35% of rise episodes to harvested real lower-endpoint start states (goal.rise_start_bank, the policy's own settled post-lower poses: knees ~+113deg off the flat-zero pose rise training has ever seen) fixes the post-lower stand-up without eroding cold-start rise, hold, or lower. Prediction-if-true: sto post-lower rise >=0.90 on the fresh c2 session cohort with det retention intact. Prediction-if-false: bank starts trade against cold-start rise (the zero-sum pattern transdagger3 showed on demo mixes), naming exposure-fraction vs mechanism as the fork.

**gate**: Pre-registered in rl_docs/tracks/hw/SESSION_BULK_GATE.md 'Cohort c2': PASS iff (1) sto post-lower rise >=0.90 with Wilson CI lower >0.842 (parent CI upper) on fresh banks det 920000..920049 / sto 930000..930049; (2) det session zero-fall >=0.95; (3) det post-lower rise >=0.967; (4) det first-rise strata each >=0.95 and lower >=0.99; (5) standard stance gate + eval_session hard gates pass with roll_tail/drag/slip quoted vs footlow2_hard1. FAIL on any miss; two exposure misses on this boundary = change mechanism.


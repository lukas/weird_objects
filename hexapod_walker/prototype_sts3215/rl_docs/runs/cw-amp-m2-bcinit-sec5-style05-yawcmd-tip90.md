# cw-amp-m2-bcinit-sec5-style05-yawcmd-tip90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T22:45:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05-yawcmd

**wandb_id**: nqs2fby4

**hypothesis**: Plain English: the maximum-exposure dose twin of tip50-r2 -- if turn-in-place learning needs command DENSITY, making 90% of episodes whole-episode turns should show a clear dose-response over the 0.5 arm; the accepted cost is translation forgetting, which is fine for a curriculum STAGE (a later stage re-mixes translation back in; measured baseline: yawcmd det dir_err 32.9 / prog 1.22). Same substrate and single lever as tip50-r2: continue from the yawcmd checkpoint (74-dim obs + full bank-verified yaw pricing + teacher_v2 real turn clips), only goal.walk_turn_in_place_frac differs (0.9 vs 0.5). Prediction-if-true: tip-left/right err drops below tip50-r2's (dose-response) with command-signed wz both directions. Prediction-if-false: still parks at 90% exposure -- together with a tip50-r2 park this REFUTES command exposure as the unlock on this substrate and the next lever is structural (mirror-symmetry regularizer / turn-specific gait phase), not more exposure or pricing. Strongest alternative: turns emerge but gait degrades to a shuffle (style/gait conflict under rotation) -- read the tip-episode videos, not just wz.

**gate**: Discovery (2M, DR-0). PASS = eval_yaw (manual, on the pod, --speed 0.08 --wz-max 0.3 + run cfg) tip-left AND tip-right err <= 0.20 with achieved wz sign matching wz_ref both directions, zero falls, gait_valid >=5/6 on the standard DR-0 gate (translation erosion TOLERATED on this arm -- stage read, compare dir_err vs yawcmd det 32.9/sto 49.0 and report it, but it does not fail the arm). FAIL-park = tip errs ~0.30 both arms -> command exposure refuted on the AMP substrate, next lever structural. Read jointly with tip50-r2 for the dose-response.


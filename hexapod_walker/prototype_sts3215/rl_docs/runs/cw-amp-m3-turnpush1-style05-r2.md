# cw-amp-m3-turnpush1-style05-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T01:09:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: v7orof4k

**hypothesis**: Plain English (retry of the r1 obs-mismatch crash: pushsmoke1-style05's cfg lacked the yaw-cmd obs column so the 75-dim turn-capable checkpoint didn't load into a 74-dim env; fix is adding the SAME yaw cfg block turnclone-yawcmd0-r2 trained with). M3 push-recovery has only ever been trained/tested on the yaw-blind headingsfull substrate. cw-amp-m2-turnclone-yawcmd0-r2 showed a single checkpoint can combine full-circle heading translation AND turn-in-place (eval_yaw tip err 0.15/0.16, DR-0 walk gate gait_valid 6/6, slip med 2.24). Does that combined substrate still tolerate mid-episode shoves the way the plain headingsfull-style05 substrate did in pushsmoke1 (topple rate falling 2-3x over 2M, gait_valid staying high, no crouch-statue), or does the added yaw-command channel make push recovery harder? Single lever vs pushsmoke1-style05: --init-from swapped to the turn-capable checkpoint + the matching yaw cfg block (obs-contract requirement, not a reward change) + dr.ext_push_prob=1.0 unchanged.

**gate**: Discovery (2M, DR-0). Compare against pushsmoke1-style05's own shape (terminations falling toward ~15 pitch/~7 roll by end of window; DR-0 gate topples ~1/6 det + ~3/6 sto, gait_valid >=5/6, no crouch-statue). Prediction-if-true: terminations fall similarly, gate topples comparable, eval_yaw tip err stays within ~0.05 of 0.15/0.16 pre-push. Prediction-if-false: terminations stay high/flat, or turn tracking collapses toward the park fingerprint (0.28-0.33) under push disturbance. Video-check before any claim.


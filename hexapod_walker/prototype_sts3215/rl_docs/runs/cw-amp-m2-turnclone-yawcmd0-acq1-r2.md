# cw-amp-m2-turnclone-yawcmd0-acq1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T00:27:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: e9aa1c4z

**hypothesis**: Retry of cw-amp-m2-turnclone-yawcmd0-acq1 (REFUSED at launch time on a snapshot-tag collision, nothing trained). Same question: does a 3x budget acquisition continuation off yawcmd0-r2 (best-rounded M2 arm: tip err 0.15/0.16, both <=0.20, clean full-heading translation 6/6 det+sto) tighten turn-in-place accuracy toward eval_yaw's own strict 0.10 rad/s gate while holding translation quality, matching the M3 push-track's discovery->acquisition pattern (pushacq1 PASSED an identical 2M->6M continuation)?

**gate**: Acquisition (6M continuation, DR-0). Manual eval_yaw (own cfg): PASS-full = tip-left AND tip-right err <=0.10 with correct sign, zero falls, translation still gait_valid 6/6 det+sto with slip/m and progress_ratio not degraded >20% vs yawcmd0-r2. PASS-partial = tip errs improve but stay in the 0.10-0.20 band. FAIL = regresses toward yawcmd0-r2's own 0.15/0.16 or worse, or translation erodes.


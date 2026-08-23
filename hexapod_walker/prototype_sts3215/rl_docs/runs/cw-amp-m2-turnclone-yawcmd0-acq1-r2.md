# cw-amp-m2-turnclone-yawcmd0-acq1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T00:27:47+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-amp-m2-turnclone-yawcmd0-r2

**wandb_id**: e9aa1c4z

**hypothesis**: Retry of cw-amp-m2-turnclone-yawcmd0-acq1 (REFUSED at launch time on a snapshot-tag collision, nothing trained). Same question: does a 3x budget acquisition continuation off yawcmd0-r2 (best-rounded M2 arm: tip err 0.15/0.16, both <=0.20, clean full-heading translation 6/6 det+sto) tighten turn-in-place accuracy toward eval_yaw's own strict 0.10 rad/s gate while holding translation quality, matching the M3 push-track's discovery->acquisition pattern (pushacq1 PASSED an identical 2M->6M continuation)?

**gate**: Acquisition (6M continuation, DR-0). Manual eval_yaw (own cfg): PASS-full = tip-left AND tip-right err <=0.10 with correct sign, zero falls, translation still gait_valid 6/6 det+sto with slip/m and progress_ratio not degraded >20% vs yawcmd0-r2. PASS-partial = tip errs improve but stay in the 0.10-0.20 band. FAIL = regresses toward yawcmd0-r2's own 0.15/0.16 or worse, or translation erodes.

**verdict**: REFUTED: budget escalation makes turning WORSE, not better -- the 6M acquisition continuation off yawcmd0-r2 landed in the run's own pre-registered FAIL branch. eval_yaw (identical panel/cfg to the parent's, run's own pod, artifact logs/ckpt_eval/cw_amp_m2_turnclone_yawcmd0_acq1_r2_yawgate.json): turn |wz_err| med 0.264 vs parent 0.155 and raw turn-clone 0.104; tip-left/right err 0.399/0.347 vs parent 0.153/0.161 -- WORSE THAN PARKING (park=0.30), i.e. RL actively unlearned the clone's turn pattern. Hold segments stayed perfect (hold |wz| med 0.007, stop-hold 0.001) and translation held or improved (DR-0 gate 6/6 det+sto, det prog med 1.10 vs 0.94; only det slip eroded 2.24->2.99). Reward rose the whole run (quarters 146/291/325/350) while the gate metric halved in accuracy -- textbook misalignment per the 08-21 ruling, NOT undertraining; no further continuations. Joint story with yawprice3x (3x yaw income => err 0.208) and this run: turn-skill erosion is MONOTONE in both training steps (raw 0.104 -> 2M 0.155 -> 6M 0.264) and income multiplier -- every pricing/budget/exposure lever is now refuted. The yaw stack's optimum is provably not accurate wz tracking (holds pay reliably, turn income is outbid or farmable). Next lever is an INCOME AUDIT + bank case (extend probe_walk_income/test_task_semantics to prove an accurate-turner out-earns the erode-to-park policy, then reprice to make it true) or accept yawcmd0-r2 (2M, tips 0.15/0.16 <=0.20 milestone bar) as the M2-yaw champion and keep RL doses SHORT on this substrate. yawcmd0-r2 REMAINS champion.


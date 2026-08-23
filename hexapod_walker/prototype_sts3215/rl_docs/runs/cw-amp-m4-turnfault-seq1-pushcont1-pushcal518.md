# cw-amp-m4-turnfault-seq1-pushcont1-pushcal518

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T10:17:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**wandb_id**: lgra8gib

**hypothesis**: Plain English: is the fall risk in the fault+push composition (found this cycle: pushcont1's own hazard-baked own-cfg DR-0 gate has 4/12 REAL falls via raw terminated field, not the gait_valid metric which never zeros on TERM) fixable by simply recalibrating the trained push-force range downward, or does the checkpoint still fall even at a narrower trained range? Eval-only bisection this cycle on the SAME pushcont1 checkpoint (no retraining) found: dr.ext_push_n held at eval-time to 5-12N or 12-18N both land 0/12 real falls; the original trained 10-25N range produces 4/12 falls, so the failure threshold sits in roughly 18-25N. This arm actually RETRAINS (same recipe/seed=7/2M/init-from turnfault_seq1 as pushcont1, single lever: dr.ext_push_n 10,25 -> 5,18) to test whether training under the narrower, eval-confirmed-safe range produces a policy whose OWN own-cfg DR-0 gate is genuinely fall-free (raw terminated check), not just whether the OLD checkpoint tolerates weaker pushes at eval time.

**gate**: Own-cfg DR-0 walk gate (12 episodes, own baked cfg incl. dr.fault_prob=1.0/dr.ext_push_prob=1.0 at the new dr.ext_push_n=5-18N range): read the RAW per-episode terminated/term_reason field via ops.sh report (NOT gait_valid, which never zeros on TERM per this cycle's harness-reading correction). PASS = 0/12 real falls (matches the eval-time bisection at this force range). PARTIAL = falls reduced vs pushcont1's own 4/12 but not zero (recalibration helps but isn't sufficient alone; recovery-training/pricing dig-in still needed). FAIL = still >=3/12 falls (recalibration alone does not transfer to a fresh retrain; the root defect is in recovery training/reward, not just force magnitude — do not spend further recalibration arms, escalate straight to reward/training-exposure fix).


# cw-recover-any14-retentiongate-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DEAD

**created**: 2026-08-17T18:57:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any11-rsi-scratch1

**wandb_id**: 5usjwbjq

**hypothesis**: Make the fallen robot's get-up ladder honest: from now on the curriculum may only advance to a harder starting position after re-proving, in ONE fresh deterministic same-round assay, that the robot still gets up from EVERY easier position it already learned (all >=0.8), and starting positions the robot keeps fumbling during ordinary training automatically get more practice time. This arm continues the operator-selected best recovery policy (canonical ppo_goal_cw_recover_any11_rsi_scratch1.zip) under the retention-gated promotion + training-error-weighted replay landed at d25fcbe, keeping any11's proven recipe with the default 0.50/0.25/0.15/0.10 replay mix, the new default 0.10 training-error overlay, and RSI OFF (recover_rsi_frac=0 - the checkpoint is already past the flat-belly wall RSI existed to prevent). Training terminal error is graded potential shortfall (safety=1, success=0), globally aggregated, sampler-only, never certification. Direct operator order (MCP lane, GPT-5 Codex for Lukas, 20260817T185428Z); supersedes the SIM SPRINT no-new-launch banner for this one run.

**gate**: Advancement is legitimate ONLY when the fresh full retention suite passes: every CERT/recover_promoted=1 event must coincide with CERT/recover_retention_suite_passed=1 in the same cert round (fresh same-round assay of every unlocked earlier bucket, each >=0.8); any promotion without that is a FAIL of the mechanism. Report which retained bucket blocks promotion when the suite fails, and whether that bucket's TRAIN/recover_bucket_N_training_error_mean feeds a rising RECOVER_SCORE/bucket_NN_training_error_priority and _sample_probability. Video-verify any earned frontier (no flag/stilt/park).

**verdict**: Trainer died silently at ~0.52M/40M steps ~113s after launch (PID 1799785 zombie, no traceback, cgroup oom_kill=0, GPU freed) — launch-infra death tied to the timed-out kubectl exec session teardown, not science. Retried once as cw-recover-any14-retentiongate-cont1-r1 per DEAD protocol.

**failed_reason**: W&B global_step not advancing (458752 -> 0) after 210s (n_steps=128, cpu-time flat for 2 polls)


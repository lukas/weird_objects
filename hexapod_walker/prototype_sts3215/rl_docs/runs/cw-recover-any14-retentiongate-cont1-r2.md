# cw-recover-any14-retentiongate-cont1-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-17T19:12:55+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any14-retentiongate-cont1-r1

**hypothesis**: Make the fallen robot's get-up ladder honest: from now on the curriculum may only advance to a harder starting position after re-proving, in ONE fresh deterministic same-round assay, that the robot still gets up from EVERY easier position it already learned (all >=0.8), and starting positions the robot keeps fumbling during ordinary training automatically get more practice time. This arm continues the operator-selected best recovery policy (canonical ppo_goal_cw_recover_any11_rsi_scratch1.zip) under the retention-gated promotion + training-error-weighted replay landed at d25fcbe, keeping any11's proven recipe with the default 0.50/0.25/0.15/0.10 replay mix, the new default 0.10 training-error overlay, and RSI OFF (recover_rsi_frac=0 - the checkpoint is already past the flat-belly wall RSI existed to prevent). Training terminal error is graded potential shortfall (safety=1, success=0), globally aggregated, sampler-only, never certification. Direct operator order (MCP lane, GPT-5 Codex for Lukas, 20260817T185428Z); supersedes the SIM SPRINT no-new-launch banner for this one run. (-r2: first two launches (cont1, cont1-r1) both died/were superseded before meaningful training -- cont1 died in a kubectl-exec teardown at 0.5M, cont1-r1 used stale code (c45d3a2b) that landed 18s before Lukas's own commit 7d39a259 added the per-promotion checkpoint + timed retention rollback the operator order required; this launch runs on 7d39a259+ code with the new --recover-full-retention-every/--recover-rollback-after-steps/--recover-rollback-fraction flags at their landed defaults (2 rounds / 4M steps / 0.60), satisfying the requirement.)

**gate**: Advancement is legitimate ONLY when the fresh full retention suite passes: every CERT/recover_promoted=1 event must coincide with CERT/recover_retention_suite_passed=1 in the same cert round (fresh same-round assay of every unlocked earlier bucket, each >=0.8); any promotion without that is a FAIL of the mechanism. In addition, verify the NEW guard mechanism actually fires as designed: RECOVER_GUARD/promotion_checkpoint_saved events produce a loadable .zip+.curriculum.json in recover_promotions/, and if any retained bucket's deterministic gate fraction stays <0.60 for >=4M steps, RECOVER_GUARD/rollback_applied=1 fires and restores the last promotion (video-verify the restored behavior isn't a regression). Report which retained bucket blocks promotion or triggers rollback when it happens. Video-verify any earned frontier (no flag/stilt/park).

**refused_reason**: hexapod-mjx-train-0 already runs cw-recover-any15-retentionrollback-cont1 — GPU pods host exactly one run; pick a free GPU pod.


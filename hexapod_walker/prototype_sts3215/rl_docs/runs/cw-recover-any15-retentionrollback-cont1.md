# cw-recover-any15-retentionrollback-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-17T19:11:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any11-rsi-scratch1

**wandb_id**: xqcqvb3u

**hypothesis**: Make the fallen robot's get-up training self-healing: every time the curriculum promotes to a harder starting position it now saves a full checkpoint of that moment, and if the robot later FORGETS an already-mastered position (same retained bucket below 0.60 for 4M steps of certification time), training automatically rolls back to the last promotion checkpoint instead of grinding on with a damaged policy. This continues the operator-selected best recovery policy (canonical ppo_goal_cw_recover_any11_rsi_scratch1.zip) with its proven any11 recipe INCLUDING goal.recover_rsi_frac=0.5, default 0.50/0.25/0.15/0.10 replay mix plus the new default 0.10 training-error overlay, retention-gated promotion (every promotion requires a fresh same-round pass of all easier buckets >=0.8), full retention suite every 2 cert rounds, 1M cert cadence / 16 cert envs / 40M budget as in any14. Implementation at exact main SHA 7d39a25 (Checkpoint and roll back recovery promotions), 45 recovery tests green. Direct operator order (MCP lane, GPT-5 Codex relaying Lukas's checkpoint/rollback amendment, 20260817T190903Z); supersedes the SIM SPRINT no-new-launch banner for this one run. Prediction-if-true: ladder climbs with every CERT/recover_promoted=1 paired with recover_retention_suite_passed=1, promotion checkpoints appear under policies/recover_promotions and on W&B, and any retention collapse triggers a logged rollback that restores the frontier instead of a permanent regression. Prediction-if-false: promotions stall behind the retention gate at the same bucket any11 plateaued, or rollback thrashes (repeated restore/regress loop) without net ladder progress. Strongest alternative: forgetting is caused by the replay mix itself, in which case rollback merely oscillates and the fix belongs in replay weighting, not checkpointing.

**gate**: PASS requires the guard mechanism proven live: (1) at first promotion, RECOVER_GUARD/promotion_checkpoint_saved fires with count/latest_checkpoint_* metrics AND a unique policy ZIP + curriculum JSON exist under policies/recover_promotions and are uploaded to W&B; (2) every CERT/recover_promoted=1 coincides with CERT/recover_retention_suite_passed=1 in the same round; CERT/recover_retention_suite_* logged on the every-2-rounds cadence; (3) adaptive training-error priority/sample-probability metrics (RECOVER_SCORE/bucket_*_training_error_*) respond to fumbled buckets; (4) if any retained bucket stays <0.60 for 4M env steps, rollback fires: pending then applied/target/trigger metrics, policy+optimizer+curriculum restored before next rollout with training-error debt retained. Behavioral: ladder frontier >= any11's without any unretained promotion; video-verify any earned frontier (no flag/stilt/park).


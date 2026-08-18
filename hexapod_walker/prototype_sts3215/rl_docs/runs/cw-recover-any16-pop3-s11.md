# cw-recover-any16-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID

**created**: 2026-08-17T22:23:09+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**wandb_id**: b10k74mr

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by training three fresh policies in lockstep and always keeping the best one; this arm is one of three identical-recipe seeds (11/12/13) that share progress: the first seed to earn a retention-clean curriculum promotion uploads its policy+optimizer+curriculum, member 0 elects the earliest valid candidate, and all three adopt and ACK the same weights before any further promotion — testing whether best-of-three seed selection at each rung beats the single-seed line, now that the env-0-only admission bug (which invalidated any15) is fixed at 4d1b45d. From scratch, NO init-from; population recover-any16-pop3, roster s11,s12,s13. Member 0, seed 11.

**gate**: Integration gate (operator, fb_20260817T221115_78b688): all 3 cohort ledger rows RUNNING with distinct W&B IDs and seeds; logs show 'synchronized cohort armed'; first cert reports CERT/recover_training_envs_synchronized=512 on all members; at first promotion exactly one B1 winner is elected and all 3 runs log RECOVER_POPULATION adopted_bucket=1/ack_bucket=1 with the same winner hash before any B2 election; transient candidate upload/poll failures retry rather than forking the roster. Behavioral: any earned frontier video-verified (genuine six-foot recoveries, no flag/stilt/park).

**verdict**: INVALID_INTEGRATION_CANARY — population sync broken: member 0 elected/ADOPTED its own B1 (step 1,966,080) then published B2/B3, but s12/s13 never adopted it (cached wandb summaries in _peer_rows + no release barrier after all-ACK). Stopped per operator MCP note fb_20260817T223644_c8bc48; checkpoints preserved on train-0; do NOT relaunch until Codex pushes corrected SHA + relaunch directive. Fleet broadcast fix itself verified working (CERT/recover_training_envs_synchronized=512).


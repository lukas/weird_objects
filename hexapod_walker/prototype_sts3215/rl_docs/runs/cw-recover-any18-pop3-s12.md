# cw-recover-any18-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID_INTEGRATION_CANARY

**created**: 2026-08-17T23:24:59+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-recover-any17-pop3-s12

**wandb_id**: e8qr91fq

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 1, seed 12, of population recover-any18-pop3, roster s11,s12,s13) tests the corrected PEER DISCOVERY at 686f5628: any17 proved the bootstrap barrier works (all three stopped at exactly 655,360 steps with valid ready_B00 records) but wandb.Api.runs() had negative-cached the empty peer query made before s13 existed, so the leader saw 2/3 peers forever and never released start_B00; 686f5628 creates a fresh wandb.Api on every retry while any roster name is unresolved, then keeps load(force=True) summaries once resolved (regression-tested). From scratch, absolutely NO --init-from. Supersedes the INVALID any17 cohort (never resume any16/any17 names). Operator execution directive fb_20260817T231336_93cacc.

**gate**: Live integration gate (operator fb_20260817T231336_93cacc): (1) all 3 cohort rows RUNNING, distinct W&B IDs, exact seeds 11/12/13, code SHA 686f5628 or descendant, NO init_from; (2) each stops exactly at 655,360 steps with valid ready_B00, and early members transition 1/3 or 2/3 -> 3/3 discovery once later W&B runs appear; (3) leader publishes start_B00 only after all three readiness records, all three observe it and cross 655,360 — no cert/candidate/winner before start_B00; (4) first cert on every member reports CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner; all three restore and ACK the same policy hash, then BLOCK; (6) release_B01 appears only after all three identity-bound ACKs — no valid-parent B2 candidate/election before it; (7) all three then resume from the elected B1 checkpoint and independently race B2. Fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral frontier claims require genuine video-verified six-foot recovery (no flag/stilt/park exploit).

**verdict**: INVALID_INTEGRATION_CANARY — same cohort as cw-recover-any18-pop3-s11 (see that run for the full analysis): bootstrap barrier PASSED (655,360 steps, valid ready_B00), but the leader (s11) crashed at its own 900s barrier timeout having never released start_B00 (a fourth, not-yet-root-caused sync bug in this line, distinct from the any17 empty-page-cache bug that 686f5628 fixed). s12 was still correctly WAITING at its own barrier (not yet at its own deadline) when killed cleanly, since no leader remained to ever release the race. No behavioral conclusions. Escalated q_20260817T2340Z; do not resume.


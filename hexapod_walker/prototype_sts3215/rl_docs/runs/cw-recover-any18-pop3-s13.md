# cw-recover-any18-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID_INTEGRATION_CANARY

**created**: 2026-08-17T23:26:28+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-recover-any17-pop3-s13

**wandb_id**: 1z5ejwe4

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this launch (member 2, seed 13, of population recover-any18-pop3, roster s11,s12,s13) is the operator-directed append-only replacement for the INVALID any17 cohort, executed per fb_20260817T231336_93cacc on code SHA 686f5628 ("Refresh unresolved recovery peers") or a descendant, fixing the THIRD synchronization bug in this line: any16 let member 0 run ahead on a stale Run.summary cache; any17 stopped clean at the bootstrap barrier (all three hit exactly 655,360 steps with a valid ready_B00 record) but start_B00 was never released because wandb.Api.runs() negative-cached the empty peer page queried before s13 (the third roster member) existed, freezing s11/s12 at 2/3 peer discovery forever; 686f5628 uses a fresh wandb.Api object on every retry while any roster ID remains unresolved and load(force=True) for resolved summaries, with a regression test proving member 2 resolves on the second fresh lookup after appearing post-first-empty-query. Exact unchanged recipe: 40,000,000 steps, ppo/gpu-mjx, joint_walk task, goal-mix recover=1.0 only, bc_anchor_recover + bc_anchor_foot_z(3mm) + bc_anchor_min_h_ahead_mm(15, 0.5s lookahead) teacher on the footlow2_hard1 recover_start_bank, admit_n=4/retreat_n=6/ema_beta=0.25/rsi_frac=0.5 curriculum, 10-rollout bootstrap, 900s barrier timeout — absolutely NO --init-from, from scratch. Supersedes the INVALID any16/any17 cohorts (never resume either).

**gate**: Live integration gate (operator fb_20260817T231336_93cacc, corrected peer discovery at 686f5628): (1) all 3 cohort rows RUNNING, distinct W&B IDs, exact seeds 11/12/13, code 686f5628 or descendant, NO init_from. (2) each stops exactly at 655,360 steps with a valid ready_B00 record; early members transition from 1/3 or 2/3 to 3/3 peer discovery once later W&B runs appear (this is exactly what any17 failed). (3) only after all three readiness records does the leader publish start_B00; all three observe it and cross 655,360. No cert/candidate/winner before start_B00. (4) first cert on every member reports CERT/recover_training_envs_synchronized=512. (5) exactly one B1 winner is elected; all three restore and ACK the same policy hash, then block. (6) release_B01 appears only after all three identity-bound ACKs; no valid-parent B2 candidate/election before it. (7) all three then resume from the elected B1 checkpoint and independently race B2. Fail closed and preserve evidence on any breach. Behavioral frontier claims still require genuine video-verified six-foot recovery, no flag/stilt/park exploit.

**verdict**: INVALID_INTEGRATION_CANARY — same cohort as cw-recover-any18-pop3-s11 (see that run for the full analysis): bootstrap barrier PASSED (655,360 steps, valid ready_B00), but the leader (s11) crashed at its own 900s barrier timeout having never released start_B00 (a fourth, not-yet-root-caused sync bug in this line, distinct from the any17 empty-page-cache bug that 686f5628 fixed). s13 was still correctly WAITING at its own barrier (not yet at its own deadline) when killed cleanly, since no leader remained to ever release the race. No behavioral conclusions. Escalated q_20260817T2340Z; do not resume.


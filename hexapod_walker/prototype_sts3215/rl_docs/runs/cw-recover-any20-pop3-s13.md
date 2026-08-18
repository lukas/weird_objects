# cw-recover-any20-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: STOPPED_SUPERSEDED

**created**: 2026-08-18T00:23:03+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-recover-any19-pop3-s13

**wandb_id**: 0f713040

**hardware_ready**: False

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this relaunch (member 2, seed 13, of population recover-any20-pop3, roster s11,s12,s13, predeclared W&B ids 7ab95ed5,96e80076,0f713040, this member 0f713040) tests the CACHE-FREE RENDEZVOUS at 8fbb7b2: peer existence and summary reads go through InternalApi.run_resume_status (fresh GraphQL each call, live-probed in the pod's wandb 0.28.1 to recover from an early miss) and the Public Api is never constructed during rendezvous/election/ACK/release — any19 proved even direct-by-id Public Api lookups poison the in-process negative cache ('not found' persisted after the run existed), and any18 proved display-name search lags past the 900s barrier. From scratch, absolutely NO --init-from. Never reuse any16/17/18/19 names. Operator execution directive fb_20260818T001206_0ee733.

**gate**: Live integration gate (operator fb_20260818T001206_0ee733): (1) commands at 8fbb7b2 or descendant, exact names/seeds/ID roster, no init_from; W&B page IDs exactly 7ab95ed5/96e80076/0f713040, not generated; (2) all 3 stop at exactly 655,360 with valid ready_B00 — no cert/candidate/winner beforehand; (3) once the final run+ready exists, leader publishes start_B00; all 3 observe and cross 655,360; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) exactly one B1 winner; all 3 adopt+ACK identical hash and block; (6) release_B01 only after all 3 identity-bound ACKs — no valid-parent B2 candidate/election before release; (7) all 3 resume from B1 and independently race B2. Fail closed + preserve evidence on any breach. Behavioral claims require genuine video-verified six-foot recovery (no flag/stilt/park exploit).

**verdict**: Stopped by SIGINT at 00:39:17Z ~2min before its hard 900s B0 barrier timeout, per operator fallback fb_20260818T002830 (partial cohort: s11/s12 never started). s13 itself HEALTHY: 8fbb7b2 cache-free rendezvous worked (clean 60s WAITING beats at 1/3 peers, ready_B00 published, no errors). Superseded by cw-recover-any21-pop3 (fresh predeclared ids).


# cw-recover-any19-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-17T23:52:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-recover-any18-pop3-s11

**hypothesis**: Teach the robot to stand back up from any fallen position by racing three fresh identical-recipe seeds in lockstep and always adopting the first retention-clean winner at each curriculum bucket; this launch (member 0, seed 11, of population recover-any19-pop3, roster s11,s12,s13) replaces eventually-consistent display-name peer discovery (the mechanism behind FOUR distinct any16/17/18 sync bugs) with predeclared, immutable W&B run IDs (6907573e,1c67c001,79ef86ae) assigned before training: wandb.init(id=<own id>, resume='never') aborts the process if W&B assigns a different id, and every peer lookup is a direct api.run(project/id) call, never a filtered list query. Exact unchanged training recipe otherwise (40M steps, joint_walk recover=1.0 only, bc_anchor_recover + bc_anchor_foot_z(3mm) + bc_anchor_min_h_ahead_mm(15,0.5s) teacher on footlow2_hard1 recover_start_bank, admit_n=4/retreat_n=6/ema_beta=0.25/rsi_frac=0.5, 10-rollout bootstrap, 900s barrier timeout). Absolutely NO --init-from, from scratch. Supersedes the INVALID any16/17/18 cohorts (never resume those names).

**gate**: Live integration gate (operator fb_20260817T234449_bcdcce, predeclared-id peer discovery): (1) commands contain exact SHA 3cc62a2 or descendant, exact names/seeds/id roster (6907573e/1c67c001/79ef86ae), no init_from -- W&B page IDs must literally equal the predeclared ids, not W&B-generated ones. (2) all 3 stop exactly at 655,360 steps with valid ready_B00; no cert/candidate/winner before that. (3) once all three exist, member 0 publishes start_B00 and all 3 observe+cross 655,360. (4) first cert on all members reports CERT/recover_training_envs_synchronized=512. (5) exactly one B1 winner; all 3 adopt+ACK identical hash and block. (6) release_B01 only after all 3 ACKs; no valid-parent B2 candidate/election before release. (7) all 3 resume from B1 and independently race B2. Fail closed + preserve evidence on any breach. Behavioral claims require genuine six-foot recovery video (no flag/stilt/park exploit).


# cw-recover-any21c2-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-18T06:32:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 100000000

**parent**: cw-recover-any21-pop3-s12

**wandb_id**: cc54b647

**hypothesis**: Keep the robot's get-back-up training rumbling: the operator ordered the finished three-seed recovery cohort continued for 100M more steps per member from its exact final state, to see whether more training breaks the tangle wall (B15) or further hardens the B14 frontier ('keep it rumbling lets see what happens', Lukas via MCP operator lane 08-18). Member 1 (seed 12, the B14 winner's own seed) of population recover-any21c2-pop3, roster s11,s12,s13, predeclared single-use W&B ids 5ecd335b,cc54b647,11892a73, this member cc54b647. Resumes from ppo_goal_cw_recover_any21_pop3_s12.zip (this member's own exact final checkpoint: policy+optimizer via SB3 load) plus --recover-init-curriculum on its own B14 promotion sidecar (md5 4ebd3fa4, bit-identical to the copy the other two members adopted). Availability note reported, not hidden: the trainer only persists curriculum state at promotions, so post-adoption per-cert-round counters as of step 40M are not on disk anywhere; the sidecar is this member's own promotion state, not a substitute checkpoint. No changes to rewards, curriculum, sampling, learning rate, or promotion/retention rules; synchronized-population protocol preserved with fresh ids. COHORT RE-RUN (attempt 2): the any21c attempt was lost to launch skew (a wrong member-index recording artifact in the source ledger entry delayed s12 ~17 min; leader s11 hit the 900s bootstrap barrier timeout and failed closed per protocol; ids b24a5f7c/5d131e10/120ad2f8 consumed). This cohort pins every member index explicitly and raises the rendezvous barrier timeout to 3600s -- an orchestration-robustness parameter only; rewards, curriculum, sampling, LR, and promotion/retention rules unchanged.

**gate**: Live integration gate (operator-ordered continuation): (1) commands at a1a01b27 or descendant, exact names/seeds (11/12/13), id roster 5ecd335b/cc54b647/11892a73 exact on every member, W&B page IDs exactly the predeclared ids; --init-from = each member's own any21 final zip, --recover-init-curriculum = the md5-4ebd3fa4 B14 sidecar; (2) startup logs '[recover-init] curriculum restored ... active_n=15' and population arms at initial B14 on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B14, leader releases start_B14, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) any promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral claims require video-verified genuine six-foot recovery (no flag/stilt/park exploit).


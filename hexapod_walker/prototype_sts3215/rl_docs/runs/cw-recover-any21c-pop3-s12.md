# cw-recover-any21c-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-18T06:22:03+00:00

**pod**: hexapod-mjx-train-1

**steps**: 100000000

**parent**: cw-recover-any21-pop3-s12

**wandb_id**: 5d131e10

**hardware_ready**: no

**hypothesis**: Keep the robot's get-back-up training rumbling: the operator ordered the finished three-seed recovery cohort continued for 100M more steps per member from its exact final state, to see whether more training breaks the tangle wall (B15) or further hardens the B14 frontier ('keep it rumbling lets see what happens', Lukas via MCP operator lane 08-18). Member 1 (seed 12, the B14 winner's own seed) of population recover-any21c-pop3, roster s11,s12,s13, predeclared single-use W&B ids b24a5f7c,5d131e10,120ad2f8, this member 5d131e10. Resumes from ppo_goal_cw_recover_any21_pop3_s12.zip (this member's own exact final checkpoint: policy+optimizer via SB3 load) plus --recover-init-curriculum on its own B14 promotion sidecar (md5 4ebd3fa4, bit-identical to the copy the other two members adopted). Availability note reported, not hidden: the trainer only persists curriculum state at promotions, so post-adoption per-cert-round counters as of step 40M are not on disk anywhere; the sidecar is this member's own promotion state, not a substitute checkpoint. No changes to rewards, curriculum, sampling, learning rate, or promotion/retention rules; synchronized-population protocol preserved with fresh ids.

**gate**: Live integration gate (operator-ordered continuation): (1) commands at a1a01b27 or descendant, exact names/seeds (11/12/13), id roster b24a5f7c/5d131e10/120ad2f8 exact on every member, W&B page IDs exactly the predeclared ids; --init-from = each member's own any21 final zip, --recover-init-curriculum = the md5-4ebd3fa4 B14 sidecar; (2) startup logs '[recover-init] curriculum restored ... active_n=15' and population arms at initial B14 on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B14, leader releases start_B14, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) any promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral claims require video-verified genuine six-foot recovery (no flag/stilt/park exploit).

**verdict**: Killed mid-bootstrap as a mechanically doomed partial-cohort member, not a training result: leader s11 had already failed closed on the 900s bootstrap barrier timeout (caused by this member's own ~17-min launch delay: wrong --recover-population-member 0 recording artifact in the source ledger entry -> argparse fail-closed; tag collision + stale pod marker on retries) and s11's single-use id b24a5f7c was consumed, so no start_B14 release could ever arrive. Never-continue-partial rule applied. Cohort re-run as recover-any21c2-pop3 (members pinned, barrier timeout 3600s, fresh ids).


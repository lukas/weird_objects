# cw-recover-any21c-pop3-s11

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-18T06:05:03+00:00

**pod**: hexapod-mjx-train-0

**steps**: 100000000

**parent**: cw-recover-any21-pop3-s11

**wandb_id**: b24a5f7c

**hardware_ready**: no

**hypothesis**: Keep the robot's get-back-up training rumbling: the operator ordered the finished three-seed recovery cohort continued for 100M more steps per member from its exact final state, to see whether more training breaks the tangle wall (B15) or further hardens the B14 frontier ('keep it rumbling lets see what happens', Lukas via MCP operator lane 08-18). Member 0 (seed 11) of population recover-any21c-pop3, roster s11,s12,s13, predeclared single-use W&B ids b24a5f7c,5d131e10,120ad2f8, this member b24a5f7c. Resumes from ppo_goal_cw_recover_any21_pop3_s11.zip (this member's own exact final checkpoint: policy+optimizer via SB3 load) plus --recover-init-curriculum on the adopted B14-winner sidecar (md5 4ebd3fa4, bit-identical on all three pods -- the exact frontier/cert state every member held from 15M to 40M). Availability note reported, not hidden: the trainer only persists curriculum state at promotions, so post-adoption per-cert-round counters as of step 40M are not on disk anywhere; the sidecar is each member's own lineage state, not a substitute checkpoint. No changes to rewards, curriculum, sampling, learning rate, or promotion/retention rules; synchronized-population protocol preserved with fresh ids.

**gate**: Live integration gate (operator-ordered continuation): (1) commands at a1a01b27 or descendant, exact names/seeds (11/12/13), id roster b24a5f7c/5d131e10/120ad2f8 exact on every member, W&B page IDs exactly the predeclared ids; --init-from = each member's own any21 final zip, --recover-init-curriculum = the md5-4ebd3fa4 B14 sidecar; (2) startup logs '[recover-init] curriculum restored ... active_n=15' and population arms at initial B14 on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B14, leader releases start_B14, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) any promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral claims require video-verified genuine six-foot recovery (no flag/stilt/park exploit).

**verdict**: Fail-closed rendezvous loss, not a training result: restored curriculum to B14 correctly ([recover-init] active_n=15), waited at the bootstrap barrier, but s12's launch was delayed ~17 min by three mechanical faults (wrong --recover-population-member 0 recording artifact in the source ledger entry -> argparse fail-closed; retry hit the pre-existing exp/ tag; second retry hit a stale pod code marker), so s11 hit the 900s barrier timeout and exited per protocol. Id b24a5f7c consumed. Same failure class as any20 (launch skew racing the barrier window). Cohort re-run as recover-any21c2-pop3: members pinned, barrier timeout 3600s, fresh ids.


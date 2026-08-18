# cw-recover-any21c2-pop3-s12

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS (partial)

**created**: 2026-08-18T06:32:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 100000000

**parent**: cw-recover-any21-pop3-s12

**wandb_id**: cc54b647

**hardware_ready**: False

**hypothesis**: Keep the robot's get-back-up training rumbling: the operator ordered the finished three-seed recovery cohort continued for 100M more steps per member from its exact final state, to see whether more training breaks the tangle wall (B15) or further hardens the B14 frontier ('keep it rumbling lets see what happens', Lukas via MCP operator lane 08-18). Member 1 (seed 12, the B14 winner's own seed) of population recover-any21c2-pop3, roster s11,s12,s13, predeclared single-use W&B ids 5ecd335b,cc54b647,11892a73, this member cc54b647. Resumes from ppo_goal_cw_recover_any21_pop3_s12.zip (this member's own exact final checkpoint: policy+optimizer via SB3 load) plus --recover-init-curriculum on its own B14 promotion sidecar (md5 4ebd3fa4, bit-identical to the copy the other two members adopted). Availability note reported, not hidden: the trainer only persists curriculum state at promotions, so post-adoption per-cert-round counters as of step 40M are not on disk anywhere; the sidecar is this member's own promotion state, not a substitute checkpoint. No changes to rewards, curriculum, sampling, learning rate, or promotion/retention rules; synchronized-population protocol preserved with fresh ids. COHORT RE-RUN (attempt 2): the any21c attempt was lost to launch skew (a wrong member-index recording artifact in the source ledger entry delayed s12 ~17 min; leader s11 hit the 900s bootstrap barrier timeout and failed closed per protocol; ids b24a5f7c/5d131e10/120ad2f8 consumed). This cohort pins every member index explicitly and raises the rendezvous barrier timeout to 3600s -- an orchestration-robustness parameter only; rewards, curriculum, sampling, LR, and promotion/retention rules unchanged.

**gate**: Live integration gate (operator-ordered continuation): (1) commands at a1a01b27 or descendant, exact names/seeds (11/12/13), id roster 5ecd335b/cc54b647/11892a73 exact on every member, W&B page IDs exactly the predeclared ids; --init-from = each member's own any21 final zip, --recover-init-curriculum = the md5-4ebd3fa4 B14 sidecar; (2) startup logs '[recover-init] curriculum restored ... active_n=15' and population arms at initial B14 on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B14, leader releases start_B14, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) any promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral claims require video-verified genuine six-foot recovery (no flag/stilt/park exploit).

**verdict**: Operator-ordered +100M/member continuation ("keep it rumbling") finished clean (100,000,007 steps, no traceback, all 6 integration-gate clauses held: barrier crossed at B14, CERT/recover_training_envs_synchronized=512 every round, member ACKed adopting winner s11's B15 with matching policy hash, RECOVER_GUARD/rollback_count=0 the whole run, no partial-cohort breach). Behavioral read answers the operator's own question but not cleanly either way: the population DID push the frontier past the old B14 tangle wall to B15 (video-verified genuine tangle-untangle + bank recoveries, no flag/stilt/park exploit), but this member's OWN matched gate-eval score FELL vs its pre-continuation checkpoint (det 8/18 DR-0, 10/18 DR-0.1 vs the prior 16/18 / 14/18) because previously-solved EASY buckets (crouch_shallow/mid/deep, partial_mid/low, zero, plant_catch under DR-0.1) now fail on this snapshot -- over_current stalls and short-of-height timeouts, not new falls or exploits. This is the SAME oscillating-retention wall CURRENT_TRUTHS already named (any15/any21 corrections), caught at a worse phase of the oscillation by chance of exactly-100M-steps landing, not a new mechanism and not proof either that more training hardens or breaks the frontier -- rollback never fired despite the dips, confirming the guard still can't see oscillation. Not hardware-ready. Verdict scoped to s12 only; s11/s13 are sibling members still training, off-limits to this cycle. No follow-up queued -- recover/tangle redesign stays operator-gated and outside the SIM SPRINT.


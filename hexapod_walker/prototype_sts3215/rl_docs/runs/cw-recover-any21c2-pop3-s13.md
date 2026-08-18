# cw-recover-any21c2-pop3-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-18T06:33:41+00:00

**pod**: hexapod-mjx-train-3

**steps**: 100000000

**parent**: cw-recover-any21-pop3-s13

**wandb_id**: 11892a73

**hardware_ready**: False

**hypothesis**: Keep the robot's get-back-up training rumbling: the operator ordered the finished three-seed recovery cohort continued for 100M more steps per member from its exact final state, to see whether more training breaks the tangle wall (B15) or further hardens the B14 frontier ('keep it rumbling lets see what happens', Lukas via MCP operator lane 08-18). Member 2 (seed 13) of population recover-any21c2-pop3, roster s11,s12,s13, predeclared single-use W&B ids 5ecd335b,cc54b647,11892a73, this member 11892a73. Resumes from ppo_goal_cw_recover_any21_pop3_s13.zip (this member's own exact final checkpoint: policy+optimizer via SB3 load) plus --recover-init-curriculum on the adopted B14-winner sidecar (md5 4ebd3fa4, bit-identical on all three pods -- the exact frontier/cert state every member held from 15M to 40M). Availability note reported, not hidden: the trainer only persists curriculum state at promotions, so post-adoption per-cert-round counters as of step 40M are not on disk anywhere; the sidecar is each member's own lineage state, not a substitute checkpoint. No changes to rewards, curriculum, sampling, learning rate, or promotion/retention rules; synchronized-population protocol preserved with fresh ids. COHORT RE-RUN (attempt 2): the any21c attempt was lost to launch skew (a wrong member-index recording artifact in the source ledger entry delayed s12 ~17 min; leader s11 hit the 900s bootstrap barrier timeout and failed closed per protocol; ids b24a5f7c/5d131e10/120ad2f8 consumed). This cohort pins every member index explicitly and raises the rendezvous barrier timeout to 3600s -- an orchestration-robustness parameter only; rewards, curriculum, sampling, LR, and promotion/retention rules unchanged.

**gate**: Live integration gate (operator-ordered continuation): (1) commands at a1a01b27 or descendant, exact names/seeds (11/12/13), id roster 5ecd335b/cc54b647/11892a73 exact on every member, W&B page IDs exactly the predeclared ids; --init-from = each member's own any21 final zip, --recover-init-curriculum = the md5-4ebd3fa4 B14 sidecar; (2) startup logs '[recover-init] curriculum restored ... active_n=15' and population arms at initial B14 on all 3; (3) all 3 stop at the bootstrap barrier (655,360) with valid ready_B14, leader releases start_B14, all 3 cross; (4) first cert on all members reports CERT/recover_training_envs_synchronized=512; (5) any promotion follows the unchanged election protocol (one winner, identical-hash adoption+ACK on all 3 before release); (6) fail closed + preserve evidence on any breach; never silently continue a partial cohort. Behavioral claims require video-verified genuine six-foot recovery (no flag/stilt/park exploit).

**verdict**: Operator-ordered +100M/member continuation ("keep it rumbling") finished clean (100,007,936 steps, no traceback, all 6 integration-gate clauses held: barrier crossed at B14, CERT/recover_training_envs_synchronized=512 every round, frontier confirmed at B15 via the shared population adoption, no partial-cohort breach). Unlike sibling s12 (own gate score FELL), this member's OWN matched gate score IMPROVED vs its pre-continuation baseline: det 16/18 DR-0 + 16/18 DR-0.1 (was 13/18 + 10/18), video-verified genuine tangle_mid/tangle/bank recoveries (full untangle-to-stand, no flag/stilt/park exploit); the only clean miss is tangle_deep (over_current, non-exploit) plus the still-unreached flip bucket (fully inverted, beyond frontier, expected). NEW datum: RECOVER_GUARD/rollback_count=1 this run (fired once at step 72,024,064, restored a B15-frontier checkpoint) -- the guard IS capable of firing (not dead code), but training-time bucket stats at the final cert still show oscillation on easy buckets (crouch_shallow/mid/deep 0.125/0.375/0.25) even after that rollback, so one firing did not cure the wall -- consistent with, not a refutation of, the any15 dig-in's age-vs-rate critique. Cohort-level: s12 regressed, s13 improved on the identical mechanism/budget -- answers the operator's question with genuine both-directions evidence, not a clean win or loss. Not hardware-ready. Verdict scoped to s13 only; s11 is a sibling member off-limits to this cycle (still training at prompt time). No follow-up queued -- recover/tangle redesign stays operator-gated, outside the SIM SPRINT.


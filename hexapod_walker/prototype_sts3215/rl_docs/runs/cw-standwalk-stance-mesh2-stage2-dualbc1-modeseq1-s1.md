# cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-26T11:59:11+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**wandb_id**: v38ba434

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1 (identical recipe, seed 1 only) -- same run for the cross-seed pass-rate reading the joint-call convention requires before promoting a stage-2 recipe.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as -modeseq1 seed 0; joint call reads both seeds together.

**verdict**: CANARY FAIL - MECHANISM (joint call with seed0 twin, cw-standwalk-stance-mesh2-stage2-dualbc1-modeseq1): same regression, same signature, cross-seed replicated. Evidence (report.json, DR-0 + own-DR(0.5), det+sto): hold TERMINATES in 24/24 episodes (det: 5x hold_low_height + 1x hold_min_load; sto: 4x hold_low_height + 2x hold_min_load), never settles (settled 0/6 every read). lower 0/6 success everywhere, worst_clear 96-152mm, up to 4/6 over_current terms in sto. walk: no falls (gait_valid 5-6/6, roll settled) but prog_ratio only 0.24-0.26 det / ~0.00-0.02 sto, dir_err 56-90deg (cap ~40deg), slip/m 4-34 -- walk_det contact sheet is a near-static splayed stance, no visible translation across the 30s strip, matching seed0s video. Reward: ep_rew_mean quarters [40.4, -0.0, -534.9, -202.1], same collapse-then-weak-partial-recovery shape as seed0, ending deeply negative -- aligned bad-and-stuck (08-21 ruling), not undertrained. CORRECTION to this runs own ledger history: the original launch was mis-recorded status=FAILED/CANARY-FAIL-INFRASTRUCTURE by a checkup false positive (SUSPECT at a slow periodic-eval/video-render round -- the process was NOT actually hung/killed; its train log shows a clean finish at 2,031,616 steps with a real exported checkpoint and W&B state=finished). This verdict evaluates that real checkpoints real eval data (matches the seed0 read exactly) and supersedes the infra-failure framing; status corrected to the mechanism verdict below. (The identical-seed retry -s1r that the false positive triggered is a separate ledger entry with its own eval read -- not double-counted here.) Routed to the pre-registered fallback jointly with seed0: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor1-s1 (2M canary, VERIFIED RUNNING) -- same recipe + the cw-arch-gru-dual1-proven stance-only/walk-off bc_anchor bundle (coef=3.0 + state_aligned/stratified/foot_z/flat_time_indexed/lower=1.0, min_h_ahead_mm=8, bc_anchor_walk=0.0).


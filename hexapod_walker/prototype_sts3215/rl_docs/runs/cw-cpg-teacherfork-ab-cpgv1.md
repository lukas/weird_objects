# cw-cpg-teacherfork-ab-cpgv1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T05:31:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: y90uxjtc

**hypothesis**: Plain English: does the CPG-search gait (a 120-iter GP-optimized, closed-loop-yaw-trimmed SE2FootGait, cpg/STATUS.md 08-23 -- passes its own held-out contextual gate with better slip than the scripted teacher) work at least as well as the hand-scripted tripod teacher as the AMP style-reward's source motion? Single lever vs the already-PASSED cw-amp-m2-bcinit-sec5-style05 (BC-clone init + section-5 minimal reward + AMP style, 2M discovery, DR-0): --amp-motion-lib swapped from teacher_v2.npz (scripted tripod, slip/m 0.5-3.1 per-clip) to cpg_v1.npz (CPG winner via the SAME build_motion_library.py generator + closed-loop yaw trim on turn clips, 45/45 clips accepted, slip/m 0.27-2.17 -- as good or better per-clip than teacher_v2 on this exact metric). This is cpg/STATUS.md's own pre-registered Next item 3 (teacher_v2/motion-library fork from the CPG winner, equal AMP budget, do not swap teacher_v1 without this result).

**gate**: Read JOINTLY against style05's own recorded result (det video shows sustained cyclic six-leg walking, net fwd travel >=0.10m/15s at the DR-0 gate, no crouch collapse, disc unsaturated). PASS/AT-LEAST-AS-GOOD = this arm clears the SAME bars (gait_valid, fwd travel, height_err trend) at parity or better than style05's own numbers. WORSE-BUT-WALKING = clears the bars but with worse margins -- informative, not a lever refutation. FAIL-collapse = crouch/statue like every non-BC-init sec5 arm -- names the CPG clip's own obs_style distribution (not the reward) as the confound, since style05 with the SAME reward+init did not collapse.

**verdict**: CPG-search motion library WORKS as an AMP style source but with WORSE margins than the scripted teacher -- lands cleanly in the pre-registered WORSE-BUT-WALKING branch, not a lever refutation. Single lever vs the PASSED cw-amp-m2-bcinit-sec5-style05 (--amp-motion-lib teacher_v2.npz -> cpg_v1.npz, same BC-clone init, same section-5 reward, same 2M budget, same seed): DR-0 gate gait_valid 6/6 det+sto BOTH arms, ZERO sacrificed legs either way -- the core question (does swapping the style-source library collapse the walker back into the crouch-statue basin) is answered NO, decisively; video confirms genuine multi-leg cyclic gait on this arm too (contact sheet + walk_sto_0 strip watched, no dragging/statue/pathology). Margins: det WORSE across the board (prog med 0.99 vs 1.16, slip med 2.46 vs 1.88, fwd med 0.59 vs 0.69m/15s); sto MIXED (prog med 0.58 identical, slip med 4.61 vs 4.71 marginally better, fwd med 0.30 vs 0.23m actually better). Reward trajectory comparable to style05's own shape (quarters 38.9/101.9/157.4/185.8 here vs style05's own rising curve). Reads as: the AMP mechanism is robust to the choice of demonstration library (teacher_v2 and cpg_v1 both organize real six-leg walking from the same BC-clone init + minimal reward), but the scripted teacher's clips are currently the BETTER style target on the deterministic axis -- plausibly because cpg_v1's SE2FootGait clips have a different swing/stance duty and cadence than the BC-clone's own tripod-derived init, so the discriminator has to pull the actor further from its start point than teacher_v2 (which shares the same tripod lineage) does. Answers cpg/STATUS.md Next item 3's core question (compare against the current teacher at equal AMP budget): do NOT swap teacher_v2 for cpg_v1 as the default AMP motion-prior on this evidence -- cpg_v1 is a validated, usable alternative library (useful for future style-source diversity/robustness experiments) but not currently a straight upgrade. Evidence: logs/ckpt_eval/cw_cpg_teacherfork_ab_cpgv1_gate/, cf. logs/ckpt_eval/cw_amp_m2_bcinit_sec5_style05_gate/report.json.


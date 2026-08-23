# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-turnclip1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T18:07:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**hypothesis**: Plain English: instead of swapping the WHOLE demo library (which regressed slip, cpgdemo1 FAIL) or only slightly speeding up the scripted teacher's own turn clips (tips unmoved, turnlib3 FAIL), replace ONLY the two turn-in-place demo clips (turn_ccw/turn_cw -- the exact family the M5 tip-left/tip-right bar scores) with clips recorded from the CPG-search controller, which a same-cycle eval-only probe just confirmed actually achieves ~0.29 rad/s at commanded 0.30 (vs the scripted teacher's raw ~0.134-0.144 and teacher_v3's tripod-rescaled ~0.174) -- nearly the full physically-reachable rate. Every other family (forward/lateral/diagonal/forward_turn/accel/decel) stays byte-identical to teacher_v2, so a slip regression (cpgdemo1's failure mode) is far less likely, and any tip-tracking change isolates to the turn-in-place demo shape alone. teacher_v4.npz built+tested this cycle (rl_move/sim/merge_motion_library.py, 11/11 new unit tests, verified loads clean under MotionLibrary's per-clip-neutral hard-fail check). This is the SEVENTH yaw mechanism class (first UN-refuted one) after pricing/demo-ceiling/style-ablation/densification/cpg-full-swap/discriminator-obs-masking all closed FAIL on the wzmask2 n=5 grid (q_20260823T1750Z) -- and it targets the one thing that was never yet isolated: swap ONLY the exact demo family the bar scores, at its real achieved-capability ceiling, nothing else. Prediction-if-true: tip errs move meaningfully toward 0.20 (>=0.03 either side, mirroring the noamp1/turnlib3 gate convention) with walk slip staying within the family's own noise band (+-0.3 of pushcal518's 3.47-3.83) since non-turn clips are untouched. Prediction-if-false: tips unmoved (+-0.02, now calibrated by the wzmask2 grid) -- closes demo-side yaw mechanisms entirely, forcing either the bar-amendment (already answered NO this cycle, q_20260823T0130Z: 0.30 rad/s is plant-reachable) or a genuinely NEW curriculum/exposure mechanism (e.g. explicit turn-in-place practice via goal.walk_turn_in_place_frac, already baked in at 0.5 on this lineage, so exposure itself is not the gap).

**gate**: PASS: both tip_left_err and tip_right_err improve by >=0.03 vs pushcal518's own pooled reads (0.2168-0.249 family band, use the nearest same-seed comparison, seed7 0.2157/0.2351) with >=1 side <=0.22, AND walk det_slip_med stays within 3.2-4.0 (no new slip regression), AND 0 raw falls / gait_valid>=11/12 every section (safety override, matches the wzmask2 grid's own safety bar). PARTIAL: one tip clears but the other doesn't, or tips move but slip regresses >0.3 (the cpgdemo1 trade pattern) -- would need a second seed before promotion either way. FAIL: tips unmoved (+-0.02, wzmask2-grid-calibrated noise floor) -- closes the surgical-splice mechanism, leaving only curriculum/exposure-side (non-demo) mechanisms or the already-answered bar-amendment as open forks.


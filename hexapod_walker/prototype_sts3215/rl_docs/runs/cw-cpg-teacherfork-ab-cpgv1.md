# cw-cpg-teacherfork-ab-cpgv1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T05:31:29+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m2-bcinit-sec5-style05

**wandb_id**: y90uxjtc

**hypothesis**: Plain English: does the CPG-search gait (a 120-iter GP-optimized, closed-loop-yaw-trimmed SE2FootGait, cpg/STATUS.md 08-23 -- passes its own held-out contextual gate with better slip than the scripted teacher) work at least as well as the hand-scripted tripod teacher as the AMP style-reward's source motion? Single lever vs the already-PASSED cw-amp-m2-bcinit-sec5-style05 (BC-clone init + section-5 minimal reward + AMP style, 2M discovery, DR-0): --amp-motion-lib swapped from teacher_v2.npz (scripted tripod, slip/m 0.5-3.1 per-clip) to cpg_v1.npz (CPG winner via the SAME build_motion_library.py generator + closed-loop yaw trim on turn clips, 45/45 clips accepted, slip/m 0.27-2.17 -- as good or better per-clip than teacher_v2 on this exact metric). This is cpg/STATUS.md's own pre-registered Next item 3 (teacher_v2/motion-library fork from the CPG winner, equal AMP budget, do not swap teacher_v1 without this result).

**gate**: Read JOINTLY against style05's own recorded result (det video shows sustained cyclic six-leg walking, net fwd travel >=0.10m/15s at the DR-0 gate, no crouch collapse, disc unsaturated). PASS/AT-LEAST-AS-GOOD = this arm clears the SAME bars (gait_valid, fwd travel, height_err trend) at parity or better than style05's own numbers. WORSE-BUT-WALKING = clears the bars but with worse margins -- informative, not a lever refutation. FAIL-collapse = crouch/statue like every non-BC-init sec5 arm -- names the CPG clip's own obs_style distribution (not the reward) as the confound, since style05 with the SAME reward+init did not collapse.


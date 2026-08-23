# cw-cpg-teacherfork-ab-cpgv1-budget2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: SUPERSEDED

**created**: 2026-08-23T06:04:49+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-cpg-teacherfork-ab-cpgv1

**wandb_id**: qs9dmqra

**hypothesis**: Second data point for cpg/STATUS.md Next item 3 (teacher_v2-vs-CPG-winner AMP style-source adoption): does the CPG library (cpg_v1.npz) close style05's 2M gap (progress_ratio med 0.99 vs 1.16, fwd dist 0.59 vs 0.69m/15s) given matched extra budget, or is the gap structural? Single lever vs the SAME source arm: warm-start from cw-cpg-teacherfork-ab-cpgv1's own 2M checkpoint, +6M more steps (8M total), identical reward/env/motion-lib. Matched control cw-cpg-teacherfork-ab-style05-budget2 continues the teacher_v2 twin the same +6M so both sides get equal extra budget, not just one.

**gate**: Read jointly against the matched style05-budget2 control at the SAME 8M total budget. CLOSES-GAP = det progress_ratio/fwd-dist median within ~5% of style05-budget2's own numbers, gait_valid 6/6, zero falls both modes. STILL-WORSE = gap persists at similar or same magnitude as the 2M read -- confirms a structural (not budget-limited) style-source quality gap, closes the adoption question without swapping teacher_v2. Either branch is a real answer; no further cpg/amp swap decision without this pair.

**verdict**: SUPERSEDED (self-killed, no verdict): duplicate of the concurrent cycle's cw-cpg-teacherfork-ab-cpgv1-acq1b, which reached VERIFIED RUNNING first (same source checkpoint, same +6M steps, same motion-lib/reward/env, identical hypothesis) -- discovered both were live simultaneously (train-0 acq1b @2.8M, train-1 budget2 @1.0M) via a benign name-collision race (this run started life as -cont1, which also collided with their commit 855a54d6 before either process actually launched). Killed this run's pod-side PIDs (2338432/2338433 on hexapod-mjx-train-1) rather than let two identical 6M continuations burn double GPU time; acq1b is the run of record for this side of the comparison. Redirected this cycle's contribution to the genuinely-missing piece: the matched style05 control at the same +6M budget (cw-cpg-teacherfork-ab-style05-budget2), which neither cycle had queued yet.


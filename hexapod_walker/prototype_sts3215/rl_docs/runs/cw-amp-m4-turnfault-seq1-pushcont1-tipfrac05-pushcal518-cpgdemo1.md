# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-cpgdemo1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T15:55:49+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: zbjprvmd

**hypothesis**: Plain English: the noise-floor anneal dose grid just closed (stdanneal50 FAIL: probe plateaued 9.55->9.6mm, and slip/tips got WORSE not better) — next lever per that gate is the demo-anchor fork, swapping WHAT the AMP discriminator imitates instead of pricing behavior against it. Single lever: --amp-motion-lib teacher_v2.npz -> cpg_v1.npz (the CPG-search-optimized library, already built for the cpg-track teacher-fork A/B but never tried on this harder turn+fault+push composed lineage) on the unchanged pushcal518 recipe. cpg_v1's own recorded clip slip is measurably lower than teacher_v2's at the SAME operating speed (forward_0.08 slip/m 0.324 vs 0.527, -39%); turn clips are similar magnitude but asymmetric (ccw achieves 80% of commanded wz vs teacher's own ~50-60%, cw only ~55%, roughly on par). Prediction-if-true: m5 walk det slip med drops toward <=3.5 without new falls/fault regressions; tip_left may improve (ccw-favorable asymmetry), tip_right likely does not. Prediction-if-false: slip/tips stay within +-0.15 of pushcal518's own 3.67/0.2157/0.2351 (matching the just-refuted reward-side and noise-floor levers) -- the discriminator's obs_style features (joint pos/vel, base angvel, foot pos) don't actually transmit the clip's recorded ground-contact slip trace, closing the demo-anchor mechanism too. Strongest alternative: swapping the anchor library destabilizes an already-converged basin unrelated to the slip question (fresh 2M retrain against a different discriminator target could cost fall-safety or fault-gait-validity independent of slip).

**gate**: eval_amp_m5 full suite (walk/yaw/push/fault) + probe_stance_slip_dist (hazard-free own-cfg, seed 0, 6 eps) vs the pushcal518_ctrl matched control (median 14.03) and the stdanneal45/-50 rungs (9.55/9.6) + weight-movement precheck vs turnfault_seq1. PASS = 0/12 raw falls AND walk gait_valid 12/12 AND m5 walk det slip med <=3.5 AND tips within 0.25 band both signs AND fault gait_valid >=10 with <=1 sacrificed leg. PARTIAL = walk det slip med improves >=0.15 toward 3.5 (<=3.55) without clearing it, or one tip clears 0.20 while the other doesn't, with 0 falls preserved -- mechanism real but insufficient alone, worth composing with the stdanneal45 rung next. FAIL = slip/tips unmoved (within +-0.15 of pushcal518's own numbers) or any new fall/fault regression -- demo-anchor mechanism refuted, escalate to the q_20260823T0700Z bar-amendment ruling as the last open lever.


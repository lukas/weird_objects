# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema-yawonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T07:58:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: 9rv88wr2

**hypothesis**: Decomposition arm vs the bundled kernelema1 respec (same cycle): isolates the YAW half of the kernel-noise-tax fix (reward.walk_kernel_yaw_ema only, translation kernel untouched) to attribute any tip-tracking effect to the yaw axis specifically rather than the bundle. Same single-lever discipline as kernelema1: seed=7, 2M, byte-identical tipfrac05 recipe otherwise.

**gate**: Read jointly with kernelema1 and kernelema-velonly: if yawonly alone reproduces most of kernelema1's delta, the yaw kernel is the driver (funds a dedicated yaw-kernel-ema-only production arm, cheaper than the bundle); if yawonly is flat but kernelema1 moved, the vel-only arm carries it instead; if neither single-axis arm moves but the bundle does, the mechanisms interact and must ship together.

**verdict**: Result: yaw-axis-only kernel-noise-tax fix does NOT improve turn-tracking -- eval_amp_m5 tips 0.2285/0.2053, WORSE than tipfrac05's clean-pass 0.162/0.184 and over the 0.20 bar (m5_pass=false: yaw FAIL, walk FAIL-narrow on slip 3.65 vs bar 3.5/parent 3.36, push PASS, fault PASS gv 10/10). Evidence: this cycle ran eval_amp_m5 by hand (prestage only covers DR-0 gate, not m5) on train-0; video-clean six-leg upright gait, zero falls, no sacrificed legs in walk section. Why: per the pre-registered joint read with sibling arms kernelema1 (bundle, tips 0.2264/0.2302) and kernelema-velonly2 (translation-only, tips 0.2064/0.2286, read this cycle for context only, not verdicted here) -- ALL THREE 2M fresh-retrain arms land in the SAME ~0.21-0.23 band regardless of which axis got the EMA fix, and that band is statistically indistinguishable from the tipfrac05 lineage's OWN already-measured seed-variance band (seed23 0.207/0.228, seed13 0.218/0.228, same recipe no kernel change at all). So the joint read's own IMPROVED/FLAT/WORSE test cannot cleanly attribute causation to the yaw kernel specifically -- velonly regressed nearly as much despite never touching the yaw kernel -- the more likely explanation is basin-selection noise at a fresh 2M retrain swamps this lever's effect size, not a genuine yaw-kernel/achieved-rotation-gate interaction. What's next: do NOT fund a dedicated yaw-kernel-ema-only production arm off this result (the pre-registered IMPROVED branch did not fire, and the WORSE branch's causal story doesn't survive the 3-way comparison either). The diagnostic lever that actually controls for basin is applying the EMA fix(es) as a CONTINUATION on the tipfrac05 checkpoint itself (fixed basin, same question q_20260823T0240Z item b was originally asked about: does repricing rescue budget-continuation, i.e. the acq1 erosion failure) rather than another fresh from-scratch retrain -- flagging for the next AMP dig-in cycle rather than launching blind.


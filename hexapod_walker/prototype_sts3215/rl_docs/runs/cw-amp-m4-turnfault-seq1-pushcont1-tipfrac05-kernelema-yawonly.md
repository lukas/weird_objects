# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema-yawonly

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T07:58:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**hypothesis**: Decomposition arm vs the bundled kernelema1 respec (same cycle): isolates the YAW half of the kernel-noise-tax fix (reward.walk_kernel_yaw_ema only, translation kernel untouched) to attribute any tip-tracking effect to the yaw axis specifically rather than the bundle. Same single-lever discipline as kernelema1: seed=7, 2M, byte-identical tipfrac05 recipe otherwise.

**gate**: Read jointly with kernelema1 and kernelema-velonly: if yawonly alone reproduces most of kernelema1's delta, the yaw kernel is the driver (funds a dedicated yaw-kernel-ema-only production arm, cheaper than the bundle); if yawonly is flat but kernelema1 moved, the vel-only arm carries it instead; if neither single-axis arm moves but the bundle does, the mechanisms interact and must ship together.


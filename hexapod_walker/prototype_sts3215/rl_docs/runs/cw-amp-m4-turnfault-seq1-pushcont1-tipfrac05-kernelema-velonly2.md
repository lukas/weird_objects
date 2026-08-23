# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema-velonly2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T08:02:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: y73206rj

**hypothesis**: Decomposition arm vs the bundled kernelema1 respec (same cycle; -velonly2 because the first -velonly launch attempt raced a pod collision with -yawonly and never trained, only leaving a stray git tag behind): isolates the TRANSLATION half of the kernel-noise-tax fix (reward.walk_kernel_vel_ema only, the already-built phasedir7/7b/8 joystick-track lever never applied to this AMP lineage; yaw kernel untouched) to attribute any tip-tracking effect to the linear-velocity axis specifically rather than the bundle. Same single-lever discipline: seed=7, 2M, byte-identical tipfrac05 recipe otherwise.

**gate**: Read jointly with kernelema1 and kernelema-yawonly (see kernelema-yawonly's gate text for the joint read).


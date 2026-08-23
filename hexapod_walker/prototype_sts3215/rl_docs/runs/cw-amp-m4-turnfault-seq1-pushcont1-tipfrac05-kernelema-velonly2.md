# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-kernelema-velonly2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T08:02:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05

**wandb_id**: y73206rj

**hypothesis**: Decomposition arm vs the bundled kernelema1 respec (same cycle; -velonly2 because the first -velonly launch attempt raced a pod collision with -yawonly and never trained, only leaving a stray git tag behind): isolates the TRANSLATION half of the kernel-noise-tax fix (reward.walk_kernel_vel_ema only, the already-built phasedir7/7b/8 joystick-track lever never applied to this AMP lineage; yaw kernel untouched) to attribute any tip-tracking effect to the linear-velocity axis specifically rather than the bundle. Same single-lever discipline: seed=7, 2M, byte-identical tipfrac05 recipe otherwise.

**gate**: Read jointly with kernelema1 and kernelema-yawonly (see kernelema-yawonly's gate text for the joint read).

**verdict**: Result: translation-axis-ONLY kernel-noise-tax fix (reward.walk_kernel_vel_ema=1, tau=0.75s, yaw kernel left untouched) still lands in the pre-registered WORSE branch on tip-tracking, nearly matching the bundled kernelema1 regression. Evidence: eval_amp_m5 tip-left/right err 0.2064/0.2286 (parent tipfrac05: 0.1620/0.1838, bar-clean; bundled kernelema1: 0.2264/0.2302) -- both now OVER the 0.20 bar despite the yaw kernel itself never being touched by this arm. Walk section PASS (det_slip_med 3.484<=3.5, prog 1.127 vs parent 1.0955, gait_valid 12/12, 0 terms), push PASS (1 term, gait_valid 12/12), fault section actually IMPROVED and now clears its own bar cleanly (gait_valid 10/12, meets the min-10 bar the parent itself missed at 9/12) -- video-clean six-leg cycling on both walk and fault contact sheets, zero falls anywhere, so this is a tracking-quality regression isolated to yaw, not a stability cost. Why: this is the single most informative arm in the 3-way decomposition -- because the yaw kernel was NEVER modified here, the original gate-text hypothesis (an EMA'd yaw kernel fighting the still-raw achieved-rotation gates walk_yaw_kernel_gate/walk_yaw_hold_prog_gate) CANNOT be the whole story, since velonly2 reproduces ~90% of the bundle's yaw regression (0.2064 vs 0.2264 tip-left) with zero yaw-side changes. The translation-kernel EMA itself (or its interaction with the walk<->turn-in-place transition, where commanded translation speed drops to ~0 during a turn-in-place segment and the EMA'd velocity-error state carries stale pre-transition error into the segment) is now the leading suspect. What's next: read jointly with the still-pending -kernelema-yawonly sibling (owned by a concurrent cycle) -- if yawonly ALSO regresses on tip err despite never touching the vel kernel, the mechanism is a general 'any-kernel-EMA near a turn-in-place transition' interaction, not axis-specific; if yawonly is clean (~parent's 0.16-0.18), this result stands alone as confirming the translation axis is the culprit and the yaw-EMA half of the bundle is exonerated (and could be re-tried solo once the translation-EMA's transition-boundary interaction is separately fixed, e.g. resetting the EMA state at walk/turn-in-place mode transitions). Do not stack kernelema1/velonly2 onto tipfrac05; the hold/forward income-repricing prerequisite for M5-candidate promotion remains OPEN. SKILLS.md M4 turn-erosion row to be amended once the joint read (all 3 arms) is complete.


# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-phasehz05-cpglib

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-23T19:21:19+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: sbdnpdwr

**hypothesis**: Plain English: give the robot BOTH the slow metronome AND matching slow-gait demonstrations, so nothing in its training is still telling it to take fast small steps. The CPG controller that actually achieves the graded 0.29 rad/s turn on this plant walks at 0.5 Hz with 13-15 deg strides; this arm sets goal.walk_phase_hz 1.333->0.5 AND swaps the AMP anchor teacher_v2.npz->cpg_v1.npz (uniformly 0.5 Hz clips) so clock and style agree -- the coherent CPG-gait package, a 2-change bundle whose coupling is necessary: the earlier single-lever cells each failed with the OTHER half contradicting them (cpgdemo1 = 0.5 Hz demos under a 1.33 Hz clock regressed slip, plausibly a cadence-conflict artifact; sibling -phasehz05 = 0.5 Hz clock under 1.33 Hz demos tests the reverse). Prediction-if-true: tips improve >=0.03 toward 0.20 (achieved tip wz breaks the 0.15-0.16 teacher saturation) and walk slip moves TOWARD the CPG clips' -39% recorded slip rather than cpgdemo1's +0.7 regression -- which would also reopen the closed slip fork. Prediction-if-false: tips unmoved +-0.02 -- even a fully coherent slow-big-stride curriculum cannot pull the warm-started policy out of its 1.33 Hz small-step basin; phase-clock lever closes (with sibling), fork narrows to stance-geometry code work / bar amendment. Strongest alternative: retraining against a different discriminator target destabilizes the basin (falls/fault regressions) independent of the cadence question -- FAIL branches on safety are judged separately from tips-unmoved.

**gate**: eval_amp_m5 suite. PASS: both tip_left_err and tip_right_err improve >=0.03 vs parent seed7 pooled reads (0.2157/0.2351) with >=1 side <=0.22, AND m5 walk det_slip_med within 3.2-4.0 read at n_translating>=6 (if <6 translating episodes rerun with --walk-per-mode 24 before judging slip), AND 0 raw falls, gait_valid >=11/12 walk/push and >=10/12 fault. BONUS finding (record even on tip-FAIL): walk det_slip_med <=3.5 with clean safety reopens the slip fork with cadence-coherent demos as the mechanism. PARTIAL: one tip clears or tips move >=0.03 but slip regresses >0.3 or fault gait_valid <10 -- mechanism real, needs dose/repair or a reseed. FAIL: tips unmoved (+-0.02 floor) -- with sibling -phasehz05 also unmoved the phase-clock cadence lever CLOSES and the joint-space fork narrows to stance-geometry code work / the bar-amendment question; a safety regression (falls>0 or fault gait_valid <10) is a separate FAIL branch (basin destabilization), does not close the cadence lever by itself.

**verdict**: The coherent slow-gait package (0.5 Hz clock + matching CPG demos) confirms the sibling's finding -- turning is fixed, slip is excellent, but forward speed halves; and comparing the two arms shows the CLOCK, not the demos, is the mechanism. Evidence (m5 + walk-per-mode-24 re-read, n_translating=28): yaw tips 0.1458/0.1367 vs parent 0.2157/0.2351 (improve 0.070/0.098, both clear the strict 0.20 bar; yaw PASS), walk det_slip_med 2.676 (band 3.47-3.83; BONUS branch of this arm's gate FIRES: slip fork REOPENED -- but sibling phasehz05 with UNCHANGED teacher_v2 demos hit 2.443, so the reopened fork's mechanism is CADENCE, not cadence-coherent demos; cpgdemo1's earlier slip regression was indeed the clock/demo cadence conflict), gait_valid 48/48, 0 terms, fault 11/12, push 12/12 with 1 det term. BUT walk det_prog_med 0.465 vs bar 0.75 (family ~0.94) -- PARTIAL, same translation-speed cost as sibling; demo swap bought ~0.01-0.02 on tips (inside noise) and nothing on slip/prog vs sibling. Why: with clock and demos both slow the policy converges to the same slow-stride basin; demo source is second-order once the clock agrees. Next: cadence dose sweep runs on the SIMPLER teacher_v2 recipe (sibling lineage); this coupled bundle is not the preferred continuation base since it adds a variable without measurable gain. Video: walk det strips clean six-leg cycling, no pathology.


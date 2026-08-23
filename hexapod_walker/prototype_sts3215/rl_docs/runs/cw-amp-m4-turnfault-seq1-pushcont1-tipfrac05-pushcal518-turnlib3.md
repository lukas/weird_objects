# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-turnlib3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T12:35:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: apan30st

**hypothesis**: Plain English: the robot turns at only half the commanded rate because the AMP style term punishes rotating faster than its demo clips, and the demos themselves only rotate at 0.134 rad/s (mislabeled 0.25) - retraining with rebuilt turn demos that actually rotate at 0.174 rad/s (teacher_v3: turn clips at stride_scale 1.4/period_scale 1.2, the scripted gait's measured max; every other clip bit-exact vs v2) should let the policy turn fast enough to clear the tip bar while KEEPING the style term's slip benefit (noamp1: AMP-off improved tips to 0.178/0.215 but regressed slip to 3.92 - so remove the demo ceiling, not the discriminator). Single lever vs pushcal518: --amp-motion-lib teacher_v2->v3. Measured root cause: untrimmed tripod achieves ratio 0.48-0.54 of commanded wz (saturates ~0.15), exactly the price-invariant ratio the 4/4-FAIL income-dose grid could not move. Prediction-if-true: both tip errs <=0.20 (or >=0.03 improvement both sides) with slip <=3.8 and 0/12 falls. Prediction-if-false: tips unmoved - demo ceiling refuted, next mechanism is turn-state reset densification (gait-spawn omega passthrough). Strongest alternative: the policy's turn authority is exploration-limited (log_std -2.0), not style-limited.

**gate**: Own-cfg DR-0 gate + eval_amp_m5 yaw+walk sections. PASS = 0/12 raw falls AND both tip errs <=0.20 AND walk det slip med <=3.8. PARTIAL = both tips improve >=0.03 vs 0.2157/0.2351 but one misses 0.20 (demo ceiling confirmed, re-dose stance scales). FAIL = tips unmoved (+-0.02) or slip >3.8 or any fall - demo-ceiling hypothesis refuted, escalate to turn-state reset densification (walk gait-spawn omega passthrough), not library re-dosing.


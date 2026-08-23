# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T17:15:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2

**hypothesis**: Plain English: two identical-config runs of the full-gyro discriminator mask gave OPPOSITE yaw-tracking readings (tips 0.239/0.237 vs 0.174/0.202), so this 3-seed grid (s23/s13/s17, exact wzmask2 recipe) decides whether hiding the robot's body-rotation sense from the AMP style critic is a real turn-tracking lever or the good read was basin luck — and calibrates the tip metric's replicate noise for the first time. This arm: seed 13, matching parent pushcal518-seed13's existing m5 read (tips 0.2168/0.2269) for a same-seed paired comparison. Prediction-if-true (mask is a real lever): pooled mask draws (n=5 incl. the seed-7 pair) median tips improve >=0.02 both sides vs parent pooled (0.2168/0.2351), >=2/5 draws with a side <=0.20. Prediction-if-false: pooled medians within +-0.02 (gyroxyz was a tail draw; gyro-channel closes per wzmask1's original FAIL branch). Strongest alternative: the tip distribution is so wide (dleft 0.065 between same-seed draws) that n=5 vs n=3 still cannot resolve 0.02 — then the grid's noise calibration itself is the deliverable and the yaw fork escalates to the bar-amendment/turn-curriculum question with measured error bars.

**gate**: Grid-level (evaluated when all 3 arms have m5 reads, pooled with wzmask2+gyroxyz): PASS = pooled n=5 median tip_left <=0.197 AND median tip_right <=0.215 AND >=2/5 draws with a side <=0.20 AND 0 falls AND walk gv >=11/12 per draw AND slip deltas <0.3 treated unmoved. FAIL = pooled medians within +-0.02 of parent pooled 0.2168/0.2351 -> gyro-channel hypothesis CLOSED for real, no further mask arms, yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z) armed with the measured tip noise floor. Safety override: any fall or gv<10 in any section = that draw FAILs regardless of tips.


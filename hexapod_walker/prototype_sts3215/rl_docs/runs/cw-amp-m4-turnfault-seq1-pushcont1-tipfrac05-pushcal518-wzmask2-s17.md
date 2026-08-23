# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T17:18:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s23

**hypothesis**: Plain English: two identical-config runs of the full-gyro discriminator mask gave OPPOSITE yaw-tracking readings (tips 0.239/0.237 vs 0.174/0.202), so this 3-seed grid (s23/s13/s17, exact wzmask2 recipe) decides whether hiding the robot's body-rotation sense from the AMP style critic is a real turn-tracking lever or the good read was basin luck. This arm: seed 17, third leg of the decider grid alongside s23 (this cycle) and s13 (concurrent cycle). Prediction-if-true: pooled mask draws (n=5 incl. seed-7 pair) median tips improve >=0.02 both sides vs parent pooled (0.2168/0.2351), >=2/5 draws with a side <=0.20. Prediction-if-false: pooled medians within +-0.02 (gyro-channel closes).

**gate**: Grid-level (evaluated when all 3 arms have m5 reads, pooled with wzmask2+gyroxyz): PASS = pooled n=5 median tip_left <=0.197 AND median tip_right <=0.215 AND >=2/5 draws with a side <=0.20 AND 0 falls AND walk gv >=11/12 per draw AND slip deltas <0.3 treated unmoved. FAIL = pooled medians within +-0.02 of parent pooled 0.2168/0.2351 -> gyro-channel hypothesis CLOSED for real, no further mask arms, yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z) armed with the measured tip noise floor. Safety override: any fall or gv<10 in any section = that draw FAILs regardless of tips.


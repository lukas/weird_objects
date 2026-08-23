# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED_FAIL

**created**: 2026-08-23T17:18:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s23

**wandb_id**: jw4fkxbf

**hypothesis**: Plain English: two identical-config runs of the full-gyro discriminator mask gave OPPOSITE yaw-tracking readings (tips 0.239/0.237 vs 0.174/0.202), so this 3-seed grid (s23/s13/s17, exact wzmask2 recipe) decides whether hiding the robot's body-rotation sense from the AMP style critic is a real turn-tracking lever or the good read was basin luck. This arm: seed 17, third leg of the decider grid alongside s23 (this cycle) and s13 (concurrent cycle). Prediction-if-true: pooled mask draws (n=5 incl. seed-7 pair) median tips improve >=0.02 both sides vs parent pooled (0.2168/0.2351), >=2/5 draws with a side <=0.20. Prediction-if-false: pooled medians within +-0.02 (gyro-channel closes).

**gate**: Grid-level (evaluated when all 3 arms have m5 reads, pooled with wzmask2+gyroxyz): PASS = pooled n=5 median tip_left <=0.197 AND median tip_right <=0.215 AND >=2/5 draws with a side <=0.20 AND 0 falls AND walk gv >=11/12 per draw AND slip deltas <0.3 treated unmoved. FAIL = pooled medians within +-0.02 of parent pooled 0.2168/0.2351 -> gyro-channel hypothesis CLOSED for real, no further mask arms, yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z) armed with the measured tip noise floor. Safety override: any fall or gv<10 in any section = that draw FAILs regardless of tips.

**verdict**: Third and final arm of the wzmask2 decider grid, and it lands exactly inside the range the other 4 draws had already mathematically locked: m5 yaw tip_left/right_err 0.2082/0.2442 (parent pooled 0.2168/0.2351). FULL n=5 pooled grid now closed: tip_left [0.239,0.174,0.2092,0.215,0.2082] median 0.2092 (delta -0.0076), tip_right [0.2371,0.202,0.252,0.2234,0.2442] median 0.2371 (delta +0.0020) -- both inside the +-0.02 FAIL band around parent, and only 1/5 draws (gyroxyz) has any side <=0.20, short of the >=2/5 PASS requirement. Grid gate: FAIL, confirming the wzmask2-s23 verdict's pre-completion math exactly. Gyro-channel yaw hypothesis (masking dims 36-38 from the AMP discriminator) is CLOSED FOR REAL -- combined with wzmask1 (gyro-z alone), no discriminator-obs rotation-channel mask moves yaw tracking. This run's own behavior: 0 falls/terms all sections, walk gait_valid 12/12 (det_slip_med 3.5195, at the family's ~3.5 sampling-noise floor), push PASS (slip 3.099), fault PASS but with cost -- gait_valid 10/12 with 2 sacrificed legs [1,2] (bar min 10, so it clears, but notably worse than s23's clean 12/12 fault read; same recipe, different seed, another data point for how noisy the fault section is seed-to-seed). Yaw fork has now exhausted all 6 discriminator/pricing/demo/densification mechanism classes; escalates to the 0.20-bar amendment (q_20260823T0130Z) or stance-geometry/turn-curriculum next, not another mask/obs arm.


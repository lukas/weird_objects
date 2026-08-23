# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED_FAIL

**created**: 2026-08-23T17:06:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2

**wandb_id**: xrw2lsb6

**hypothesis**: Plain English: two identical-config runs of the full-gyro discriminator mask gave OPPOSITE yaw-tracking readings (tips 0.239/0.237 vs 0.174/0.202), so this 3-seed grid (s23/s13/s17, exact wzmask2 recipe) decides whether hiding the robot's body-rotation sense from the AMP style critic is a real turn-tracking lever or the good read was basin luck — and calibrates the tip metric's replicate noise for the first time. This arm: seed 23, matching parent pushcal518-seed23's existing m5 read (tips 0.2493/0.2393) for a same-seed paired comparison. Prediction-if-true (mask is a real lever): pooled mask draws (n=5 incl. the seed-7 pair) median tips improve >=0.02 both sides vs parent pooled (0.2168/0.2351), >=2/5 draws with a side <=0.20. Prediction-if-false: pooled medians within +-0.02 (gyroxyz was a tail draw; gyro-channel closes per wzmask1's original FAIL branch). Strongest alternative: the tip distribution is so wide (dleft 0.065 between same-seed draws) that n=5 vs n=3 still cannot resolve 0.02 — then the grid's noise calibration itself is the deliverable and the yaw fork escalates to the bar-amendment/turn-curriculum question with measured error bars.

**gate**: Grid-level (evaluated when all 3 arms have m5 reads, pooled with wzmask2+gyroxyz): PASS = pooled n=5 median tip_left <=0.197 AND median tip_right <=0.215 AND >=2/5 draws with a side <=0.20 AND 0 falls AND walk gv >=11/12 per draw AND slip deltas <0.3 treated unmoved. FAIL = pooled medians within +-0.02 of parent pooled 0.2168/0.2351 -> gyro-channel hypothesis CLOSED for real, no further mask arms, yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z) armed with the measured tip noise floor. Safety override: any fall or gv<10 in any section = that draw FAILs regardless of tips.

**verdict**: The 3-seed decider grid's answer is now MATHEMATICALLY LOCKED regardless of the still-running 3rd arm: this run (seed 23) reads m5 yaw tip_left/right_err 0.2092/0.252 (vs parent-pooled 0.2168/0.2351) -- pooled with the two prior draws (wzmask2 0.239/0.2371, gyroxyz 0.174/0.202) and s13 (concurrent cycle, 0.215/0.2234) gives n=4 pooled medians tip_left 0.2121, tip_right 0.2303, both within the +-0.02 FAIL band of parent. Because a pooled-5 median is bounded by the sorted 4-draw's 2nd/3rd order statistics (tip_left in [0.2092,0.215], tip_right in [0.2234,0.2371]) no matter what the pending s17 reads, neither pooled median can leave the FAIL band -- the grid's PASS branch is now IMPOSSIBLE, independent of s17. Per its own pre-registered gate: the gyro-channel yaw hypothesis (mask dims 36-38, and by extension dim-38-alone from wzmask1) is CLOSED FOR REAL; the earlier straddling reads (wzmask2 FAIL / gyroxyz apparent-PASS) were both draws from one noisy-but-flat distribution, not a real lever. This run's own behavior is clean: walk section 0 falls/terms, gait_valid 12/12, det_slip_med 3.527 (at the family's known 3.5 bar per the x12 sampling-noise ruling, not a regression); push PASS (slip 3.064); fault PASS (0 falls, gv 12/12, fwd 0.479m). Next: the yaw fork has now exhausted FIVE structural/lever classes (pricing, demos, style-ablation, reset-densification, discriminator-obs incl. two gyro-mask widths) plus this replicate-noise calibration -- per the STATUS's own escalation path it moves to the 0.20-bar amendment (q_20260823T0130Z) or stance-geometry/turn-curriculum, NOT another discriminator-obs arm. s17 will still be let finish and gets its own verdict for the record, but does not gate this conclusion.


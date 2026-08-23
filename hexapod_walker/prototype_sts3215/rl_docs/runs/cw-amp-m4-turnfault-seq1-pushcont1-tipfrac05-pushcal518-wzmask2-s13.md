# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-s13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T17:15:53+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2

**wandb_id**: 5d7ntfnt

**hypothesis**: Plain English: two identical-config runs of the full-gyro discriminator mask gave OPPOSITE yaw-tracking readings (tips 0.239/0.237 vs 0.174/0.202), so this 3-seed grid (s23/s13/s17, exact wzmask2 recipe) decides whether hiding the robot's body-rotation sense from the AMP style critic is a real turn-tracking lever or the good read was basin luck — and calibrates the tip metric's replicate noise for the first time. This arm: seed 13, matching parent pushcal518-seed13's existing m5 read (tips 0.2168/0.2269) for a same-seed paired comparison. Prediction-if-true (mask is a real lever): pooled mask draws (n=5 incl. the seed-7 pair) median tips improve >=0.02 both sides vs parent pooled (0.2168/0.2351), >=2/5 draws with a side <=0.20. Prediction-if-false: pooled medians within +-0.02 (gyroxyz was a tail draw; gyro-channel closes per wzmask1's original FAIL branch). Strongest alternative: the tip distribution is so wide (dleft 0.065 between same-seed draws) that n=5 vs n=3 still cannot resolve 0.02 — then the grid's noise calibration itself is the deliverable and the yaw fork escalates to the bar-amendment/turn-curriculum question with measured error bars.

**gate**: Grid-level (evaluated when all 3 arms have m5 reads, pooled with wzmask2+gyroxyz): PASS = pooled n=5 median tip_left <=0.197 AND median tip_right <=0.215 AND >=2/5 draws with a side <=0.20 AND 0 falls AND walk gv >=11/12 per draw AND slip deltas <0.3 treated unmoved. FAIL = pooled medians within +-0.02 of parent pooled 0.2168/0.2351 -> gyro-channel hypothesis CLOSED for real, no further mask arms, yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z) armed with the measured tip noise floor. Safety override: any fall or gv<10 in any section = that draw FAILs regardless of tips.

**verdict**: Third of five pooled draws for the wzmask2 gyro-mask replicate-noise grid; individually safety-clean but does not show the mask's PASS signature. Result -> eval_amp_m5: walk PASS (0 raw terms det+sto, gait_valid 12/12, det_slip_med 3.4735 <= bar 3.5), push PASS (0 terms, gv 12/12), fault PASS (0 terms, gv 12/12, sacrificed [] -- clean, no carried-fault-leg sacrifice this draw), yaw FAIL (tip_left 0.215 / tip_right 0.2234, both above the 0.20 bar); m5_pass=false on yaw alone. Evidence: this seed's own unmasked ancestor pushcal518-seed13 read tips 0.2168/0.2269 (paired same-seed comparison per the arm's hypothesis) -- masked draw is flat/unmoved on tips (delta -0.0018/+... actually left +0.0018 worse, right -0.0035 better, both inside noise), i.e. this draw reads closer to the wzmask2 base FAIL signature (0.239/0.2371) than to the gyroxyz PASS signature (0.174/0.202). Walk slip did improve more than the <0.3 unmoved threshold (3.4735 vs seed13's own unmasked 3.82, delta 0.35) -- a side note, not gate-deciding. Why: this is exactly the grid's own pre-registered question -- whether the mask's one good draw (gyroxyz) was a real lever or a tail draw of a replicate distribution wide enough (dleft 0.065) that n=3 still cannot resolve 0.02. Current n=3 pooled (wzmask2, gyroxyz, this run): tip_left {0.239,0.174,0.215} median 0.215, tip_right {0.2371,0.202,0.2234} median 0.2234 -- both medians sit ABOVE the grid's own PASS bar (<=0.197/<=0.215) already at n=3, though the formal call needs the full n=5 (s17 + s23, other cycles' pods) per the gate text. What's next: no action here -- pool with s17/s23 when both land, then close the grid per its own PASS/FAIL text; do not launch further mask arms before that pooled read.


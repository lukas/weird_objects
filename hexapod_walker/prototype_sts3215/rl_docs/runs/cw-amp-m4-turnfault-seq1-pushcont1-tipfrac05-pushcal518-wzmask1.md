# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T15:47:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: w624mne1

**hypothesis**: Plain English: the AMP discriminator currently punishes the robot for rotating faster than its turn demos (which are geometry-capped at ~0.134 rad/s while eval commands 0.30), so we blind the discriminator to the yaw-rate channel (obs_style dim 38, zeroed on both real and fake sides via new --amp-style-mask-dims, default-off/bit-exact, test_amp_style_mask.py 6/6 + full amp banks 21/21) and keep every other style pressure intact. Measured basis: the noamp1 full ablation improved tips by 0.038/0.020 (tip_left CLEARED the 0.20 bar) but regressed walk slip +0.25 -- the style term is load-bearing for foot cleanliness, so remove ONLY the rotation-speed signal instead of the whole term. Single lever vs pushcal518 (same seed 7, 2M). Prediction-if-true: m5 yaw tip errs move >=0.03 toward the 0.20 bar (like noamp1) while walk det_slip_med stays within +-0.15 of parent 3.67 (unlike noamp1). Prediction-if-false: tips unmoved -- the discriminator reads rotation speed off joint_vel/foot patterns, not the gyro channel, and the mask must widen (36-38 or joint_vel) or the yaw axis needs the stance-geometry/curriculum fork instead. Strongest alternative: tips improve but slip regresses anyway (rotation style WAS the slip-relevant channel), reproducing the noamp1 trade and closing this lever.

**gate**: eval_amp_m5 yaw+walk on own cfg vs parent pushcal518 (tips 0.2157/0.2351, walk det_slip_med 3.67): PASS = tip_left AND tip_right each improve >=0.02 with at least one <=0.20, walk det_slip_med within +-0.15 of 3.67, 0 falls, walk gait_valid 12/12. PARTIAL = tips improve >=0.02 both sides but slip regresses >0.15 (noamp1 trade reproduced -> lever closed, yaw fork goes structural). FAIL = tips unmoved (<0.02) -> widen mask to 36,37,38 in ONE follow-up arm; if that also fails, gyro-channel hypothesis closed, yaw axis goes to stance-geometry/turn-curriculum.


# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-23T15:47:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518

**wandb_id**: w624mne1

**hypothesis**: Plain English: the AMP discriminator currently punishes the robot for rotating faster than its turn demos (which are geometry-capped at ~0.134 rad/s while eval commands 0.30), so we blind the discriminator to the yaw-rate channel (obs_style dim 38, zeroed on both real and fake sides via new --amp-style-mask-dims, default-off/bit-exact, test_amp_style_mask.py 6/6 + full amp banks 21/21) and keep every other style pressure intact. Measured basis: the noamp1 full ablation improved tips by 0.038/0.020 (tip_left CLEARED the 0.20 bar) but regressed walk slip +0.25 -- the style term is load-bearing for foot cleanliness, so remove ONLY the rotation-speed signal instead of the whole term. Single lever vs pushcal518 (same seed 7, 2M). Prediction-if-true: m5 yaw tip errs move >=0.03 toward the 0.20 bar (like noamp1) while walk det_slip_med stays within +-0.15 of parent 3.67 (unlike noamp1). Prediction-if-false: tips unmoved -- the discriminator reads rotation speed off joint_vel/foot patterns, not the gyro channel, and the mask must widen (36-38 or joint_vel) or the yaw axis needs the stance-geometry/curriculum fork instead. Strongest alternative: tips improve but slip regresses anyway (rotation style WAS the slip-relevant channel), reproducing the noamp1 trade and closing this lever.

**gate**: eval_amp_m5 yaw+walk on own cfg vs parent pushcal518 (tips 0.2157/0.2351, walk det_slip_med 3.67): PASS = tip_left AND tip_right each improve >=0.02 with at least one <=0.20, walk det_slip_med within +-0.15 of 3.67, 0 falls, walk gait_valid 12/12. PARTIAL = tips improve >=0.02 both sides but slip regresses >0.15 (noamp1 trade reproduced -> lever closed, yaw fork goes structural). FAIL = tips unmoved (<0.02) -> widen mask to 36,37,38 in ONE follow-up arm; if that also fails, gyro-channel hypothesis closed, yaw axis goes to stance-geometry/turn-curriculum.

**verdict**: FAIL on its pre-registered gate: blinding the AMP discriminator to the yaw-rate gyro channel (obs_style dim 38) does NOT recover the tip-tracking gain that turning the whole style term off (noamp1) produced — the discriminator's rotation-speed signal does not live (only) in the gyro channel. Evidence (m5 suite, matched parent pushcal518 at matched 2M budget): tips 0.202/0.2371 vs parent 0.2157/0.2351 — tip_left -0.014, tip_right +0.002, both inside the ±0.02 unmoved band (noamp1 for scale: 0.1778/0.2151); walk det slip med 3.8485 vs parent 3.67 (+0.18, at the edge of the ±0.15 band but within the n=2 translating-episode sampling noise documented on q_20260823T0700Z this hour — read as unmoved, not a regression claim). Safety clean: 0 falls all sections, walk gait_valid 12/12, push section PASS (slip 3.22), fault PASS (gv 11/12, single carried leg), upright six-leg det strip. Reward-vs-eval: training healthy (ep_rew 40->251, full 2M, 12.5k fps), eval unmoved vs matched control — mechanism refuted, not undertrained; not a continue case. Next per the pre-registered FAIL branch: exactly ONE widen-mask follow-up masking the FULL gyro triple (dims 36-38) — launched as -wzmask2; if that too reads unmoved, the gyro-channel hypothesis closes and the yaw axis rests on the command-conditioned discriminator (cmdcond1, finished, own triage pending) or the stance-geometry/turn-curriculum fork.


# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-gyroxyz

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-23T16:35:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask1

**wandb_id**: gsss1sqt

**hypothesis**: Plain English: hiding only the yaw-rate gyro channel from the AMP discriminator did nothing (wzmask1 FAIL: tips 0.202/0.237 vs parent 0.216/0.235), so this last obs-side arm hides the ENTIRE body-rotation sense (gyro x/y/z, obs_style dims 36-38, zeroed on both real and fake sides) -- if the discriminator's anti-turn pressure lives anywhere in the body-rotation channels, this removes all of it while keeping every joint/foot style pressure intact. This is the pre-registered single widen-mask follow-up from wzmask1's own FAIL branch; cmdcond1 (cmd-conditioning, 63-dim obs) also failed this cycle, so obs-side structural fixes are one arm from closure. Single lever vs pushcal518 (same seed 7, 2M, same recipe as wzmask1 except mask width). Prediction-if-true: m5 yaw tip errs improve >=0.03 on at least the worse side (toward the 0.20 bar, like noamp1's -0.038/-0.020) with walk det_slip_med within +-0.15 of parent 3.67. Prediction-if-false: tips stay inside the +-0.03 band of 0.2157/0.2351 -- the discriminator reads rotation off joint_vel/foot patterns, the gyro-channel hypothesis CLOSES, and yaw moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z). Strongest alternative: tips improve but slip regresses past +-0.15 (rotation style was load-bearing for foot cleanliness after all) -- noamp1's trade reproduced, lever closed.

**gate**: eval_amp_m5 full suite + weight-movement precheck vs turnfault_seq1 + discriminator health (d_real/d_fake separated, not collapsed). PASS = tip improvement >=0.03 on the worse side AND >=1 tip <=0.20 AND walk det_slip_med within +-0.15 of 3.67 AND 0 falls AND fault gait_valid>=10. PARTIAL = tips improve >=0.03 but slip outside +-0.15 (noamp1 trade reproduced -> lever closed, no continuation). FAIL = tips within +-0.03 of parent 0.2157/0.2351 -> gyro-channel hypothesis CLOSED; no further mask arms; yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment. Falls/fault regression overrides to FAIL.

**verdict**: This run was recorded as a launch-race duplicate and killed — but the kill never took effect: the trainer ran to its full 2M budget (W&B gsss1sqt state=finished, 2,031,616 steps), and far from producing the byte-identical trajectory the kill verdict assumed, GPU non-determinism diverged it from its identical-config twin -wzmask2 (final ep_rew 262.4 vs 233.7). Its m5 read is the best yaw result ever measured in this family: tips 0.174/0.202 (parent 0.2157/0.2351; the 11-read family floor was 0.198), turn_err_med 0.154, 0 falls, 0 terms, walk gait_valid 12/12, det slip 3.83 (unmoved under the <0.3 sampling caveat), video-clean six-leg gait. It cannot be promoted as a gyro-mask effect: its identical twin read 0.239/0.2371, so this is one draw from a recipe whose replicate spread (0.065 on tip_left) exceeds every tip gate band used this campaign. Value delivered: (1) second replicate establishing the tip noise-floor finding; (2) checkpoint retained append-only as the family's yaw-best artifact — if the 3-seed decider grid (launching now) confirms the mask lever, this checkpoint is the promotion candidate; if not, it is a documented tail draw. OPS DEFECT: a kill was recorded in the ledger while the trainer kept running to completion on train-1 — kill paths must verify by process death + W&B state, not command exit; flagged in amp STATUS.


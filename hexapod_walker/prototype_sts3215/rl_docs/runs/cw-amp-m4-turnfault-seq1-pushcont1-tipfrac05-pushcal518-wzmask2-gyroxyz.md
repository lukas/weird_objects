# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2-gyroxyz

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-23T16:35:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask1

**wandb_id**: gsss1sqt

**hypothesis**: Plain English: hiding only the yaw-rate gyro channel from the AMP discriminator did nothing (wzmask1 FAIL: tips 0.202/0.237 vs parent 0.216/0.235), so this last obs-side arm hides the ENTIRE body-rotation sense (gyro x/y/z, obs_style dims 36-38, zeroed on both real and fake sides) -- if the discriminator's anti-turn pressure lives anywhere in the body-rotation channels, this removes all of it while keeping every joint/foot style pressure intact. This is the pre-registered single widen-mask follow-up from wzmask1's own FAIL branch; cmdcond1 (cmd-conditioning, 63-dim obs) also failed this cycle, so obs-side structural fixes are one arm from closure. Single lever vs pushcal518 (same seed 7, 2M, same recipe as wzmask1 except mask width). Prediction-if-true: m5 yaw tip errs improve >=0.03 on at least the worse side (toward the 0.20 bar, like noamp1's -0.038/-0.020) with walk det_slip_med within +-0.15 of parent 3.67. Prediction-if-false: tips stay inside the +-0.03 band of 0.2157/0.2351 -- the discriminator reads rotation off joint_vel/foot patterns, the gyro-channel hypothesis CLOSES, and yaw moves to stance-geometry/turn-curriculum or the 0.20-bar amendment (q_20260823T0130Z). Strongest alternative: tips improve but slip regresses past +-0.15 (rotation style was load-bearing for foot cleanliness after all) -- noamp1's trade reproduced, lever closed.

**gate**: eval_amp_m5 full suite + weight-movement precheck vs turnfault_seq1 + discriminator health (d_real/d_fake separated, not collapsed). PASS = tip improvement >=0.03 on the worse side AND >=1 tip <=0.20 AND walk det_slip_med within +-0.15 of 3.67 AND 0 falls AND fault gait_valid>=10. PARTIAL = tips improve >=0.03 but slip outside +-0.15 (noamp1 trade reproduced -> lever closed, no continuation). FAIL = tips within +-0.03 of parent 0.2157/0.2351 -> gyro-channel hypothesis CLOSED; no further mask arms; yaw fork moves to stance-geometry/turn-curriculum or the 0.20-bar amendment. Falls/fault regression overrides to FAIL.

**verdict**: KILLED as an exact duplicate, not a result: a concurrent cycle launched the same pre-registered wzmask1 FAIL-branch follow-up (-wzmask2, train-0, 16:33:24Z) two minutes before this launch (16:35:12Z) -- same seed 7, same --amp-style-mask-dims=36,37,38, same parent recipe, so the two runs would produce byte-identical trajectories and zero extra information. Killed within minutes of launch to free train-1. The scientific question (full-gyro discriminator mask) lives in -wzmask2.


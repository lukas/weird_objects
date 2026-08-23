# cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T16:33:24+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1-tipfrac05-pushcal518-wzmask1

**wandb_id**: aqh3h4io

**hypothesis**: Plain English: hiding only the yaw-rate sensor from the AMP style critic (wzmask1) did nothing, so this last gyro-channel arm hides the robot's ENTIRE body-rotation sense (gyro x/y/z, obs_style dims 36-38) from the discriminator to test whether ANY direct rotation-rate channel is what suppresses turning speed. Pre-registered as wzmask1's single FAIL-branch follow-up. Single lever vs pushcal518 (same seed 7, 2M): --amp-style-mask-dims 38 -> 36,37,38; default-off masking mechanism already tested (test_amp_style_mask.py 6/6). Prediction-if-true: m5 yaw tips move >=0.03 toward 0.20 (noamp1 read 0.1778/0.2151) with walk det_slip_med within the family band (3.55-3.85). Prediction-if-false: tips unmoved (±0.02 of parent 0.2157/0.2351) — the discriminator reads rotation speed off joint_vel/foot-trajectory dims, the gyro-channel hypothesis CLOSES for good, and the yaw axis rests on cmdcond1's command-conditioning or the stance-geometry/turn-curriculum fork. Strongest alternative: masking roll/pitch rates (36/37) also degrades the style term's stabilizing pressure and costs fault/push robustness or slip — reproducing a partial noamp1 trade.

**gate**: eval_amp_m5 full suite vs matched parent pushcal518 (tips 0.2157/0.2351, walk det slip med 3.67) and vs wzmask1 (0.202/0.2371, slip 3.8485). PASS = both tips improve >=0.02 with >=1 side <=0.20 AND walk det slip med within ±0.15 of 3.67 AND 0/12 falls AND fault gait_valid >=10 with <=1 sacrificed. PARTIAL = tips improve >=0.02 but slip/fault regress beyond band (noamp1 trade reproduced through the gyro triple — lever closed as a trade, informative for cmdcond1). FAIL = tips within ±0.02 of parent — gyro-channel hypothesis CLOSED; no further mask arms; yaw axis moves to cmdcond1 (already trained, own triage) or stance-geometry/turn-curriculum fork. Sampling caveat recorded on q_20260823T0700Z applies: treat slip deltas <0.3 as unmoved.


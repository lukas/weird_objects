# cw-walk-imubias3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T19:24:59+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: yu38zceq

**hypothesis**: OPERATOR WISHLIST 13c top axis (IMU bias walk): roll/pitch calibration bias is a REAL defect (imu_calibrate leaves residual error; the IMU reads level when the body is not). ISOLATED axis: dr-scale 0.0 with ONLY dr.imu_bias_deg=3.0 (per-episode roll/pitch bias u(-3,3) deg; full-DR default 1.0, champion trained at 0). Plain: the robot should walk level even when its tilt sensor is calibrated a few degrees wrong. Prediction-if-true: gait holds across the bias spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - IMU bias robustness is trainable by exposure. Prediction-if-false: biased draws lean/veer or trip attitude terms (tilted stance, progress collapse) - tilt feedback is trusted too literally and needs bias estimation. Strongest alternative (torquedroop lesson): champion ALREADY tolerates 3deg IMU bias free - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_bias_deg=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det


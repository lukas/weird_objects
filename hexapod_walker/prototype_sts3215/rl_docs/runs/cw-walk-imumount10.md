# cw-walk-imumount10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T18:35:00+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: xq79idv3

**hypothesis**: OPERATOR WISHLIST 13c-class sensor axis via the 13b one-axis recipe: IMU MOUNT MISCALIBRATION. The policy's only orientation signal is the IMU; a mount rotated a few degrees biases every tilt/gyro reading the same way all episode - the classic cheap-robot calibration residual. ISOLATED: dr-scale 0.0 with ONLY dr.imu_mount_deg=10.0 (per-episode mount rotation up to 10deg, the full-DR default; champion trained at 0) - one variable off the no-DR champion. Plain: keep walking straight even when the robot's sense of 'level' is rotated a few degrees. Prediction-if-true: gait holds under biased tilt (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - sensor-bias robustness is trainable by exposure. Prediction-if-false: biased tilt corrupts balance responses (veer, tilt terminations, prog craters) - IMU calibration error needs explicit estimation, not exposure. Strongest alternative: policy learns to ignore tilt obs entirely - check DR0 retention and any tilt-canary erosion. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_mount_deg=10.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det


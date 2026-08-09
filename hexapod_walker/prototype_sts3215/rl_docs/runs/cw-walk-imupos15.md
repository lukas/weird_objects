# cw-walk-imupos15

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:11:20+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: OPERATOR WISHLIST 13c untested axis (IMU mounting POSITION, distinct from imu_mount rotation already landed via imumount10): the physical IMU board sits offset from the chassis CoM by some assembly tolerance, which changes the lever-arm on gyro/accel readings during body motion (rotation-induced linear accel at the sensor site). ISOLATED axis: dr-scale 0.0 with ONLY dr.imu_pos_xy_m=0.015 and dr.imu_pos_z_m=0.0,0.02 (15mm lateral offset spread, 0-20mm vertical - a generous assembly-tolerance stress margin; full-DR default imu_pos_xy_m 0.0 in this cfg key convention scaled by dr-scale so isolating here). Plain: does a mis-placed IMU sensor (reading rotational effects at the wrong point) perturb the gait? Prediction-if-true: gait holds (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - closes as another free axis. Prediction-if-false: the lever-arm error corrupts attitude estimate under motion, causing veer/instability. Strongest alternative: champion already tolerates this free at this magnitude - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.imu_pos_xy_m=0.015 + dr.imu_pos_z_m=0.0,0.02, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


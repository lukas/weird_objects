# cw-dep-vref1-r1-imupos

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T17:02:57+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has only been hardened against IMU MOUNT ROTATION (imumount, PASSED) and gyro/tilt noise, never IMU MOUNT POSITION -- the real IMU could be bolted anywhere on the chassis, and an off-center mount feels lever-arm accelerations during body rotation that corrupt the accel-derived tilt exactly while leaning (untested dimension, distinct dr field: imu_pos_xy_m/imu_pos_z_m vs imu_mount_deg). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- position-induced lever-arm noise composes free like every other sensor axis so far. If-false: lever-arm-corrupted tilt during turns/leans breaks tracking in a way rotation-only noise did not -- flag as a real pre-attempt-#2 sensor-mounting risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate


# cw-dep-vref1-r1-imupos

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T17:02:57+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1 has only been hardened against IMU MOUNT ROTATION (imumount, PASSED) and gyro/tilt noise, never IMU MOUNT POSITION -- the real IMU could be bolted anywhere on the chassis, and an off-center mount feels lever-arm accelerations during body rotation that corrupt the accel-derived tilt exactly while leaning (untested dimension, distinct dr field: imu_pos_xy_m/imu_pos_z_m vs imu_mount_deg). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- position-induced lever-arm noise composes free like every other sensor axis so far. If-false: lever-arm-corrupted tilt during turns/leans breaks tracking in a way rotation-only noise did not -- flag as a real pre-attempt-#2 sensor-mounting risk.

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**verdict**: PASS -- IMU mount POSITION offset (0.07m xy lever arm, -0.02..0.10m z), a new axis distinct from the already-PASSed mount-ROTATION case, composes free onto the hardware candidate. Own-cfg det+sto gv 6/6 each, 0 term, 0 safety_flags, 0 sacrificed legs either pass; det slip/m med 1.04, sto slip/m med 0.93 -- both inside vref1-r1's own band (0.89-1.13 det / 1.13-1.36 sto). The one degraded draw (det/4, prog 0.08) is the lineage's known fixed-seed march-in-place crater, video-checked clean six-leg cycling, level body, no flag-leg. Lever-arm-corrupted tilt during leans/turns does not break tracking any worse than rotation-only mount noise did.


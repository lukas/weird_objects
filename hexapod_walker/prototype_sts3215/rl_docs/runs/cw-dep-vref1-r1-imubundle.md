# cw-dep-vref1-r1-imubundle

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T16:47:49+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly if the real IMU has several realistic quirks AT ONCE instead of one at a time. vref1-r1 already PASSED latency drift (0.5-2.5x), a tilted IMU mount (10deg), and gyro noise (1.5deg/s) INDIVIDUALLY (3 separate PASSed respecs tonight) -- the real IMU chip will have all three simultaneously (a slightly crooked mount, sensor noise, AND some read/transport delay), so this bundles the three already-PASSED sensor-realism axes onto the same base recipe as its siblings (respec of cw-dep-vref1-r1, not a warm start from any single-axis checkpoint, to avoid compounding a single lineage's drift). Per P0 rule 3, k_current=0. If-true: own-cfg (DR0.35 + all 3 axes) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- individually-benign sensor axes stay benign combined, same as the already-PASSED comshift+deadband and fric+groundtilt5 bundles. If-false: the combined sensor uncertainty (mount offset shifting the noise's effective bias, on top of latency) breaks tracking in a way no single axis did -- flag as a real pre-attempt-#2 sensor risk, not assume single-axis DR passes compose for free.

**gate**: own-cfg (DR0.35 + dr.latency_scale=0.5,2.5 + dr.imu_mount_deg=10.0 + dr.gyro_noise_deg_s=1.5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1 own band; DR0 no-sensor-DR retention clean; frames watched det

**refused_reason**: hexapod-mjx-train-0 already runs cw-dep-vref1-r1-tiltnoise — GPU pods host exactly one run; pick a free GPU pod.


# cw-dep-vref1-r1-imupos-imubias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T19:34:32+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-dep-vref1-r1-imupos

**wandb_id**: oh8i5kpa

**hardware_ready**: False

**hypothesis**: Plain English: does the hardware checkpoint headed for tonight's attempt still walk cleanly if the IMU chip is BOTH mounted off-center/off-axis (imu_pos, already PASSed alone) AND has a residual calibration bias (imu_bias_deg 3deg, already PASSed alone) at the same time -- the realistic combination for a hand-installed IMU (position error and calibration error come from the same install, not independent events), never tested together. Per P0 rule 3, k_current=0 (inherited from imupos's args). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- IMU-mounting realism composes free like every other axis pairing tonight. If-false: the lever-arm position error and the calibration bias compound (plausible: position error affects the sensed angular accel term the bias-correction path also touches) -- flag as a real pre-flight IMU risk before the operator's hardware attempt.

**gate**: own-cfg (DR0.35 + imu_pos_xy_m=0.07 + imu_pos_z_m + imu_bias_deg=3.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: PASS: IMU lever-arm offset AND calibration bias TOGETHER (realistic same-install pairing, both individually PASSed alone) compose free. DR0-gate 12/12 gv, 0 term, slip in band, only the pre-allowed known catastrophic crater (det/4). Own-cfg DR0.35 12/12 gv, 0 term, in band; the 3 degraded episodes (det/5,sto/0,1) match the pre-registered lineage fixed-seed fingerprint exactly, no new pathology. Video clean six-leg creep, no flag-leg.


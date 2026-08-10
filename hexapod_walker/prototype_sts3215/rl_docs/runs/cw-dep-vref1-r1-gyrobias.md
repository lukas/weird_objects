# cw-dep-vref1-r1-gyrobias

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T18:58:43+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: test whether the hardware candidate still walks cleanly if the IMU's roll/pitch calibration is biased (a fixed sensor error from imperfect calibration, distinct from noise) rather than just noisy. vref1-r1 already PASSED tilt-angle NOISE (tiltnoise) and gyro-rate NOISE (gyronoise) individually and together; gyro BIAS (a steady rate offset that integrates into a drifting attitude estimate) is a different, untested failure mode on the same complementary filter. If-true: own-cfg (DR0.35 + dr.gyro_bias_deg_s=1.5) det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- the filter's periodic accel correction bounds the drift enough to stay benign. If-false: the biased rate integrates into a persistent attitude error that either trips the 25deg tilt safety or degrades gait beyond the tolerance -- flag as a real pre-attempt-#2 sensor-calibration risk.

**gate**: own-cfg (DR0.35 + dr.gyro_bias_deg_s=1.5) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4, prog~0.1 slip~22) is pre-allowed as baseline, not a new pathology

**refused_reason**: hexapod-mjx-train-4 already runs cw-dep-vref1-r1-gyrobias — GPU pods host exactly one run; pick a free GPU pod.


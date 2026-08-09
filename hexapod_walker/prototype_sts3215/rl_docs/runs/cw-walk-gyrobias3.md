# cw-walk-gyrobias3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:29:31+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: OPERATOR WISHLIST 13c untested axis (gyro RATE bias, distinct from imu_bias_deg=attitude bias already landed via imubias3): a cheap IMU's gyroscope has a residual rate bias after calibration (drifts the integrated angle if ever used, and corrupts any rate-feedback term). ISOLATED axis: dr-scale 0.0 with ONLY dr.gyro_bias_deg_s=3.0 (per-episode constant rate bias u(-3,3) deg/s on all 3 axes; full-DR default 0.5). Plain: does a miscalibrated gyro reading (roll/pitch/yaw RATE, not angle) perturb the gait? Prediction-if-true: gait holds (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - closes as another free axis like torquedroop. Prediction-if-false: biased rate feedback destabilizes recovery reflexes under perturbation. Strongest alternative: champion already tolerates this free at this magnitude - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.gyro_bias_deg_s=3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


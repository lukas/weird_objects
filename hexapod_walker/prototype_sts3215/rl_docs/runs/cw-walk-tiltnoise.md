# cw-walk-tiltnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:32:40+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hypothesis**: OPERATOR WISHLIST 13c untested axis (IMU tilt-reading NOISE, distinct from imu_bias_deg=constant bias already landed): per-step measurement noise on the attitude estimate (vibration/ADC noise), separate from a fixed calibration bias. ISOLATED axis: dr-scale 0.0 with ONLY dr.tilt_noise_deg=1.5 (full-DR default 0.3; 5x stress margin). Plain: does noisy (not biased) tilt sensing perturb the gait? Prediction-if-true: gait holds (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds. Prediction-if-false: noisy attitude feedback induces jitter/instability the policy can't filter. Strongest alternative: champion already tolerates this free - triage MUST eval parent longdist-r2 under the same spread BEFORE verdicting (c59 rule).

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.tilt_noise_deg=1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; parent baseline under same spread compared at triage; frames watched det

**verdict**: Launch failure (gotcha 13b EOFError collision storm), 0 steps, no science result.


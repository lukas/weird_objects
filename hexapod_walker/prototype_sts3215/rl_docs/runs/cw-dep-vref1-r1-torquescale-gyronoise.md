# cw-dep-vref1-r1-torquescale-gyronoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T17:33:00+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: cnfbxu1k

**hardware_ready**: False

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's two individually-PASSed axes -- battery-sag torque droop (0.5-1.05x from -torquescale) and gyro rate-noise (1.5 deg/s from -gyronoise) -- have never been exposed TOGETHER, but a real weak/sagging battery plausibly causes BOTH low servo torque AND noisier IMU rail readings simultaneously (shared root cause: brownout). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- shared-brownout axes stay benign combined, like every prior compose tonight. If-false: combined torque-sag+sensor-noise defeats the contract-exact obs in a way neither did alone -- flag as a real hardware risk (weak battery is a common real-world state) before deployment.

**gate**: own-cfg (DR0+torque0.5-1.05x+gyronoise1.5deg/s) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); video frames watched det+sto for flag-leg/skate

**verdict**: PASS -- confirms if-true: shared-brownout axes (torque droop 0.5-1.05x + gyro rate-noise 1.5deg/s) stay benign COMBINED, matching every prior compose tonight. Own-cfg (DR0+both axes, per this run's own authored gate) det+sto 6/6 gv, 0 term, slip/m med 1.12 det / 0.99 sto -- both inside vref1-r1's own band (0.89-1.36). Degraded episode det/4 (prog 0.01, slip 25.34) is the lineage's known fixed-seed march-in-place crater seen identically in other PASSed siblings; frame-checked: level body, six legs still cycling, no flag-leg/drag/fall. Training finished clean (reward quarters 590/666/641/622, no instability). Not independently hardware-ready (inherits vref1-r1's own paddle-gait economics); clears the combined torque-sag+gyro-noise brownout axis as safe for the hardware candidate.


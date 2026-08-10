# cw-dep-vref1-r1-comshift-deadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T16:00:22+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: vref1-r1's two individually-PASSed real-world axes (off-center CoM 0.03m from -comshift; servo deadband 1.0-3.0x from -deadband) have never been exposed TOGETHER, but the real robot has both simultaneously (uncentered battery/wiring AND worn dead-zone servos). Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band (0.71-1.13 det, 0.9-1.36 sto +-20%) -- the two individually-benign axes stay benign combined, as every prior compose on other lineages (comshift+payload, comshift+deadband on driving/crouch lines) showed. If-false: the combination surfaces an interaction neither axis alone did (contract-exact velocity obs amplifying combined timing/load-bias error) -- flag as a real hardware risk before deployment, do not rely on single-axis DR passes alone.

**gate**: own-cfg (DR0.35 + comshift0.03 + deadband1-3x) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto


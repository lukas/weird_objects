# cw-dep-vref1-r1-actionnoise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T19:56:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: 4ynyz9ra

**hardware_ready**: False

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly if the servo's OUTPUT command itself is noisy (not just its position READING, already tested via encoder noise)? Real STS3215 servos have write jitter/quantization on the command side distinct from encoder read noise, and this axis has never been isolated on this line. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the lineage's known fixed-draw crater), 0 term, slip/m within vref1-r1's own band -- composes free like every sensing/actuator axis tested tonight. If-false: noisy commands degrade tracking beyond encoder noise alone -- flag as a real pre-attempt-#2 risk distinct from sensing error.

**gate**: own-cfg (DR0.35 + dr.action_noise=0.06, 3x nominal) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (~0.89-1.13 det, ~1.13-1.36 sto) +-20%; the lineage's known fixed-draw crater (det/4) is pre-allowed as baseline

**verdict**: Action-command noise (3x nominal write jitter, dr.action_noise=0.06) composes free onto the hardware candidate: own-cfg DR0.35 gv 12/12 (det+sto), 0 term, 0 sacrificed legs; slip med 1.16 det / 1.15 sto sit right at/just inside vref1-r1's own band (0.89-1.13/1.13-1.36); the fixed-eval crater episode shifts from the usual det/4 to det/5 (expected -- action noise itself injects RNG unlike the pure static DR axes) but is the same benign march-in-place stall on video, no flag-leg/drag/fall. DR0-gate (still carries the fixed action-noise injection) matches. Closes the last individually-untested actuator-command axis on the dep-line protect-the-candidate sweep.


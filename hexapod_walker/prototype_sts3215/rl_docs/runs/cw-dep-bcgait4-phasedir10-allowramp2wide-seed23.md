# cw-dep-bcgait4-phasedir10-allowramp2wide-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T16:35:29+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir10-allowramp-seed23

**hypothesis**: Plain English: the early foot-drag allowance was sized (48mm) from measurements on the one seed it ended up helping; this arm tests whether the hurt seeds need a WIDER starting allowance because their honest-but-noisy stance travel is fatter than that calibration. Single change vs cw-dep-bcgait4-phasedir10-allowramp-seed23 (FAIL, 0.792x/1.217x vs no-ramp longrun23 0.818x/1.175x): reward.drag_stance_allow_ramp_mm 48->64 (schedule unchanged at 1.2M, still lockstep with the log-std anneal; anneals to the same validated 24mm target). 'Wider start' half of the untried branch in joystick STATUS Next item 3. Grid launched as one batch per operator 08-22.

**gate**: Watcher DR-0 gate report, det mode, clone-relative rung-A: PASS = progress >=0.9x clone AND slip <=1.2x clone, zero falls, gait_valid 6/6. Compare against the no-ramp baseline (longrun23: 0.818x/1.175x) and the 1.2M/48mm ramp reading (0.792x/1.217x). Prediction-if-true: both axes improve vs both baselines (the 48mm start was under-sized for this seed's noisy-honest tail). Prediction-if-false: flat/worse vs no-ramp — start-width dose refuted on this seed; a 0-for-4 grid CLOSES the ramp lever class for failing seeds and redirects to the matched-timing stance-slip mechanism (STATUS item b).


# cw-dep-bcgait4-phasedir10-allowramp2wide-seed29-b

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T16:45:01+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir10-allowramp-seed29

**wandb_id**: ysmr3ng9

**hypothesis**: Plain English: the early foot-drag allowance was sized (48mm) from measurements on the one seed it ended up helping; this arm tests whether the WORST seed needs a WIDER starting allowance because its honest-but-noisy stance travel is fatter than that calibration. Single change vs cw-dep-bcgait4-phasedir10-allowramp-seed29 (FAIL, 0.725x/1.466x vs no-ramp longrun29 0.740x/1.296x): reward.drag_stance_allow_ramp_mm 48->64 (schedule unchanged at 1.2M, lockstep with the log-std anneal; anneals to the same validated 24mm target). 'Wider start' half of the untried branch in joystick STATUS Next item 3, on the harder seed; read with the seed23 sibling. Grid launched as one batch per operator 08-22. (Name suffix -b: the plain and -a names' snapshot tags were consumed by stale-pod-code-marker REFUSED attempts, same precedent as allowramp-a.)

**gate**: Watcher DR-0 gate report, det mode, clone-relative rung-A: PASS = progress >=0.9x clone AND slip <=1.2x clone, zero falls, gait_valid 6/6. Compare against the no-ramp baseline (longrun29: 0.740x/1.296x) and the 1.2M/48mm ramp reading (0.725x/1.466x). Prediction-if-true: both axes improve vs both baselines. Prediction-if-false: flat/worse vs no-ramp — start-width dose refuted on this seed; a 0-for-4 grid CLOSES the ramp lever class for failing seeds and redirects to the matched-timing stance-slip mechanism (STATUS item b).


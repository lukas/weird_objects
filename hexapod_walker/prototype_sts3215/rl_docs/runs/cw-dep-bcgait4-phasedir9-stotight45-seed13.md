# cw-dep-bcgait4-phasedir9-stotight45-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:28:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: 6w0jmnc6

**hypothesis**: Plain English: the robot just passed the full 60-second joystick test for the first time, but only on one training seed — this arm re-runs the exact same recipe on a second seed to learn whether the pass is the recipe or a lottery ticket. Identical stack to stotight45 (longrun17 recipe, fresh re-init, --log-std-final -4.5), only the seed changes (17->13). At -3.2 the det rung pass rate was 1/4 (seed13 failed at 0.79x prog); the -4.5 noise floor mechanically fixed sto slip on seed17 (monotone dose-response). Prediction-if-true (recipe-robust): this seed's sto slip also lands ~2.5-2.9 and the session gate passes or near-misses only on det progress. Prediction-if-false (seed lottery): sto slip improves (mechanism is seed-independent) but the session gate fails on det axes like seed13 always has. Strongest alternative: noise floor interacts with basin selection and even sto slip stays >2.9.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true (zero falls, slip<=2.9, dir<=40, gait_valid all). Secondary read: own-DR sto slip vs the seed17 dose curve (3.00/2.87/2.48).


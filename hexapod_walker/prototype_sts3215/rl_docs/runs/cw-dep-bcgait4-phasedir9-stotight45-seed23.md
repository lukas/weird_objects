# cw-dep-bcgait4-phasedir9-stotight45-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:32:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: 67r5g96d

**hypothesis**: Plain English: second of three seed-reproduction arms for the first-ever 60-second joystick-gate pass — same recipe, new seed, to measure whether the pass survives the seed lottery. Identical to stotight45 except seed 17->23. Seed23's det history at -3.2: 0.818x prog no-ramp, worse under every ramp dose. Prediction-if-true: sto slip lands ~2.5-2.9 (mechanism seed-independent) and the session gate passes or misses only on det. Prediction-if-false: session gate fails wide like seed23's rung history. Strongest alternative: noise floor changes basin selection and this seed lands somewhere new entirely.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true. Secondary: own-DR sto slip vs seed17 dose curve.


# cw-dep-bcgait4-phasedir9-stotight45-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:30:27+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: svd95ftl

**hypothesis**: Plain English: third seed-reproduction arm for the first-ever 60-second joystick-gate pass — same recipe on the lineage's historically worst seed, the hardest honest test of whether the recipe generalizes. Identical to stotight45 except seed 17->29 (worst det history: 0.740x no-ramp, worse under every ramp dose). Prediction-if-true: sto slip still lands ~2.5-2.9 (the noise-floor mechanism is seed-independent) even if det axes fail. Prediction-if-false: everything fails wide — the -4.5 dose does nothing without a good det basin. With seeds 13/23 this measures the DONE-gate pass rate at n=4 seeds for the promotion question q_20260822T1730Z.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true. Secondary: own-DR sto slip vs seed17 dose curve (3.00/2.87/2.48).


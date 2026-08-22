# cw-dep-bcgait4-phasedir9-stotight50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T17:55:07+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: fqnodxo5

**hypothesis**: Plain English: the joystick gate now passes on two seeds but with thin margins (seed17 own-DR sto slip 2.859 vs cap 2.9; seed23 own-DR-alone dir median 40.36 vs allow 40) — this arm asks whether pushing the trained noise floor one dose lower keeps widening those margins. Sto slip fell monotonically across the -3.6/-4.0/-4.5 log-std-final dose grid (own-DR 3.00/2.87/2.48); single change vs the stotight45 gate-passer: --log-std-final -4.5 -> -5.0 (final std ~0.007), same seed 17, fresh reinit per the lineage rule. Prediction-if-true: session sto slip/dir margins widen further (sto slip < 2.6) with det still under every cap — a fatter-margin champion candidate. Prediction-if-false: the dose curve is past its knee — det softens below a cap (det slip was already drifting 2.30->2.55 across doses) or the basin flips and the session gate fails. Strongest alternative: margins move inside eval noise either way (dose saturated at -4.5).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip <= seed17-stotight45's 2.671 (margins actually widened). INFORMATIVE = pass=true with margins inside noise of -4.5 (dose saturated — stop the dose ladder). FAIL = pass=false (dose overshot; -4.5 stays the recipe).


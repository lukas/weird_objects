# cw-dep-bcgait4-phasedir9-stotight50-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T18:50:41+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: 53ylwzcb

**hypothesis**: Plain English: the -5.0 noise floor widened seed17's gate margins but HURT seed13 (slip 2.407->2.63), so the dose ladder may be seed17-specific -- this arm tests transfer on seed23, the passer whose own-DR-alone dir median (40.36 at -4.5) is the thinnest named hardening gap. Single change vs the stotight50 PASS: seed 17 -> 23 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate pass=true with dir err below seed23@-4.5's 39.4 and own-DR dir comfortably under 40 (deep floor closes the gap; ladder transfers to some non-17 basins). Prediction-if-false: margins at or worse than seed23@-4.5 (ladder is seed17-only; hardening via dose is closed and seed13@-4.5 stays champion with no further dose arms).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined dir < seed23@-4.5's 39.4 AND own-DR-alone dir <= 40 (gap closed). INFORMATIVE = pass=true but dir/slip within noise of or worse than seed23@-4.5 (no transfer; dose hardening closed for seed23). FAIL = pass=false (dose flips the basin).


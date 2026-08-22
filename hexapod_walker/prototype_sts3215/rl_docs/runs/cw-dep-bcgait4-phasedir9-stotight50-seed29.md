# cw-dep-bcgait4-phasedir9-stotight50-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T18:48:56+00:00

**pod**: hexapod-mjx-train-3

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: 8n50emls

**hypothesis**: Plain English: the -5.0 noise floor widened seed17's gate margins but HURT seed13 (slip 2.407->2.63), so this arm tests transfer on seed29, the passer with the thinnest own-DR sto slip margin (2.736 vs cap 2.9 at -4.5) -- the other named hardening gap. Single change vs the stotight50 PASS: seed 17 -> 29 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate pass=true with combined slip below seed29@-4.5's 2.704 and own-DR slip pulled off the 2.736 edge (ladder transfers; own-DR hardening gap closes). Prediction-if-false: margins at or worse than seed29@-4.5 (together with seed13's regression this establishes the ladder as seed17-only; dose hardening is closed and champion selection rests on the -4.5 readings).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip < seed29@-4.5's 2.704 AND own-DR slip < 2.736 (gap closed). INFORMATIVE = pass=true but slip within noise of or worse than seed29@-4.5 (no transfer; dose hardening closed for seed29). FAIL = pass=false (dose flips the basin).


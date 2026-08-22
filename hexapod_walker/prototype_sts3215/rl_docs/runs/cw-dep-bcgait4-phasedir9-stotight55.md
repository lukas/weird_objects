# cw-dep-bcgait4-phasedir9-stotight55

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T18:19:11+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**hypothesis**: Plain English: every step down the trained-noise-floor ladder (-3.6/-4.0/-4.5/-5.0) has widened the joystick session-gate margins with no det cost yet — this arm takes one more rung (-5.5, final std ~0.004) on the same seed to find where the ladder stops paying. Single change vs the stotight50 PASS (joygate slip 2.569/dir 38.02, det prog 0.69): --log-std-final -5.0 -> -5.5, same seed 17, fresh reinit per the lineage rule. Prediction-if-true: margins widen again (combined slip < 2.5) with det still under every cap. Prediction-if-false: the knee finally appears — det softens below a cap, exploration starves and the basin flips, or the session gate fails. Strongest alternative: margins move inside eval noise of -5.0 (ladder saturated; -5.0 is the recipe endpoint).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip <= stotight50's 2.569 (ladder still paying). INFORMATIVE = pass=true with margins inside noise of -5.0 (ladder saturated — stop at -5.0). FAIL = pass=false or det softens below a cap (knee found; -5.0 stays the recipe).


# cw-dep-bcgait4-phasedir9-stotight60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T18:41:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight55

**wandb_id**: 8jfqonne

**hypothesis**: Plain English: five rungs down the trained-noise-floor ladder (-3.6/-4.0/-4.5/-5.0/-5.5) have each widened the joystick session-gate margins with no det cost — this arm takes one more rung (-6.0, final std ~0.0025) on the same seed to find where the ladder stops paying. Single change vs the stotight55 PASS (joygate slip 2.515/dir 34.97, 15s det prog 0.74): --log-std-final -5.5 -> -6.0, same seed 17, fresh reinit per the lineage rule. Prediction-if-true: margins widen again (combined slip < 2.46 or dir < 33) with det still under every cap. Prediction-if-false: the knee appears — det softens below a cap, exploration starves and the basin flips, or the session gate fails. Strongest alternative: margins move inside eval noise of -5.5 (slip step was already shrinking 0.102->0.054/rung; ladder saturated — -5.5 is the recipe endpoint).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND (combined slip <= stotight55's 2.515 OR dir < 34.0) (ladder still paying). INFORMATIVE = pass=true with margins inside noise of -5.5 (ladder saturated — stop at -5.5). FAIL = pass=false or det softens below a cap (knee found; -5.5 stays the recipe).


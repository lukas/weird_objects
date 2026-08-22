# cw-dep-bcgait4-phasedir9-stotight60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T18:41:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight55

**wandb_id**: 8jfqonne

**hypothesis**: Plain English: five rungs down the trained-noise-floor ladder (-3.6/-4.0/-4.5/-5.0/-5.5) have each widened the joystick session-gate margins with no det cost — this arm takes one more rung (-6.0, final std ~0.0025) on the same seed to find where the ladder stops paying. Single change vs the stotight55 PASS (joygate slip 2.515/dir 34.97, 15s det prog 0.74): --log-std-final -5.5 -> -6.0, same seed 17, fresh reinit per the lineage rule. Prediction-if-true: margins widen again (combined slip < 2.46 or dir < 33) with det still under every cap. Prediction-if-false: the knee appears — det softens below a cap, exploration starves and the basin flips, or the session gate fails. Strongest alternative: margins move inside eval noise of -5.5 (slip step was already shrinking 0.102->0.054/rung; ladder saturated — -5.5 is the recipe endpoint).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND (combined slip <= stotight55's 2.515 OR dir < 34.0) (ladder still paying). INFORMATIVE = pass=true with margins inside noise of -5.5 (ladder saturated — stop at -5.5). FAIL = pass=false or det softens below a cap (knee found; -5.5 stays the recipe).

**verdict**: The knee is found: -6.0 is past the ladder's optimum for seed17 — margins regress well beyond noise, closing the dose ladder at -5.5. 60s joystick DONE-gate evaluator still passes (0 falls/48, gait 48/48, no sacrificed legs, clean six-leg sheet) but combined slip 2.823 vs -5.5's 2.515 (worse than every rung back to -4.0) and dir 37.68 vs 34.97; own-DR slip 2.873 grazes the 2.9 cap. Neither pre-registered PASS condition met (slip <=2.515 / dir <34). Det did not collapse (15s DR-0 det prog 0.70/slip 2.01, 6/6 ok) so this is not the FAIL branch — it is the knee: five monotone rungs then a clear regression at -6.0 (final std ~0.0025 likely starves exploration late). Reward rose and converged (quarters -80/175/417/453) — honest answer, not undertraining. RECIPE ENDPOINT: seed17@-5.5 (stotight55, slip 2.515/dir 34.97) is the best seed17 policy; no -6.5 arm. Combined with seed13 (knee at -4.5) and seed29 (knee at -4.5): every seed's knee is now bracketed.


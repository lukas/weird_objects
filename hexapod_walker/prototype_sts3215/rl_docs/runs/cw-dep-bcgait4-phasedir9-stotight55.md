# cw-dep-bcgait4-phasedir9-stotight55

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T18:19:11+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: ty8ud4f3

**hypothesis**: Plain English: every step down the trained-noise-floor ladder (-3.6/-4.0/-4.5/-5.0) has widened the joystick session-gate margins with no det cost yet — this arm takes one more rung (-5.5, final std ~0.004) on the same seed to find where the ladder stops paying. Single change vs the stotight50 PASS (joygate slip 2.569/dir 38.02, det prog 0.69): --log-std-final -5.0 -> -5.5, same seed 17, fresh reinit per the lineage rule. Prediction-if-true: margins widen again (combined slip < 2.5) with det still under every cap. Prediction-if-false: the knee finally appears — det softens below a cap, exploration starves and the basin flips, or the session gate fails. Strongest alternative: margins move inside eval noise of -5.0 (ladder saturated; -5.0 is the recipe endpoint).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip <= stotight50's 2.569 (ladder still paying). INFORMATIVE = pass=true with margins inside noise of -5.0 (ladder saturated — stop at -5.0). FAIL = pass=false or det softens below a cap (knee found; -5.0 stays the recipe).

**verdict**: The noise-floor ladder is STILL paying at -5.5: this run passes the full 60s randomized joystick DONE-gate with the best direction-following of any passer and no det cost. Plain result: pass=true, 0 falls/48, gait_valid 48/48, no sacrificed legs; combined slip 2.515 (parent stotight50 2.569, cap 2.9), dir err 34.97deg (parent 38.02, best yet; allow 40); per-pass DR-0 slip 2.449/dir 35.36, own-DR 2.542/33.7. The feared det trade did NOT appear again: 15s DR-0 det prog 0.74/slip 1.63 vs parent 0.69/1.72 — det IMPROVED. Videos watched (det+sto strips, DR-0 gate pass): clean upright six-leg gait, no flag leg, worst sto episode still upright. Reward rose all run (quarters -102->454) with std annealed to 0.004 — reward and gate agree. Pre-registered PASS branch (slip <= 2.569) is met; slip step is shrinking (0.102 -> 0.054) but dir jumped 3deg, outside the prior step's 0.6deg scale — not saturated. Next: one more rung (stotight60, -6.0, seed17) to find the knee, plus seed13 x -5.5 (best basin, champion-candidate margins).


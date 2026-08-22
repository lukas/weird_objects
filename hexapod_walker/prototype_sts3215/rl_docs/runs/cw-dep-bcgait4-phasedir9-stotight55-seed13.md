# cw-dep-bcgait4-phasedir9-stotight55-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T18:43:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight55

**wandb_id**: 77hdpxqp

**hypothesis**: Plain English: seed 13 is the lineage's fattest-margin basin (slip 2.407 at -4.5, the family best) — this arm runs the newly-validated -5.5 dose on that seed to test whether the ladder's gains stack with the best basin and produce the strongest champion candidate yet. Single change vs the stotight55 PASS (seed17, joygate slip 2.515/dir 34.97): seed 17 -> 13, fresh reinit per the lineage rule. Prediction-if-true: combined slip < 2.407 (beats every existing passer) with det under every cap. Prediction-if-false: seed13's basin does not benefit from the deeper floor (margins at or worse than its -4.5 result) or det softens below a cap. Strongest alternative: passes but lands between 2.407 and 2.515 — dose helps but basin and dose gains do not stack.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip < seed13@-4.5's 2.407 (new family-best; champion-candidate margins). INFORMATIVE = pass=true with slip in [2.407, 2.515] (dose helps this seed but gains do not stack). FAIL = pass=false or det softens below a cap.

**verdict**: Dose and basin gains do NOT stack: -5.5 on the best-basin seed13 lands worse than seed13's own -4.5 on both axes. 60s joystick DONE-gate evaluator passes (0 falls/48, gait 48/48, no sacrificed legs, clean six-leg sheet) but combined slip 2.594 vs the pre-registered PASS bar <2.407 and even outside the INFORMATIVE band [2.407,2.515]; dir 39.72 vs 36.4; own-DR-alone dir 40.4 slightly over the 40 allowance (combined passes). Det stayed healthy (15s DR-0 det prog 0.67/slip 1.92) so not the FAIL branch. Reward rose and converged (quarters -112/153/482/457). With stotight50-seed13 (also worse at -5.0) this brackets seed13's knee AT -4.5 — two rungs past it both regress monotonically (2.407 -> 2.63 -> 2.594±noise). CHAMPION CANDIDATE CONFIRMED UNCHANGED: seed13@-4.5 (stotight45-seed13, slip 2.407/dir 36.4) keeps the fattest margins; the strongest-champion-yet hypothesis is refuted. Per-seed dose search is complete for all four seeds; no further stotight arms.


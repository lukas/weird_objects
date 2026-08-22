# cw-dep-bcgait4-phasedir9-stotight50-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T18:28:45+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**wandb_id**: bk8ld9a7

**hypothesis**: Plain English: the widest-margin joystick gate passer so far is seed13 at the -4.5 noise floor (slip 2.407/dir 36.4), and deepening the floor to -5.0 just widened seed17's margins with no det cost — this arm combines best seed with deeper dose to ask whether the effects stack into the fattest-margin champion candidate. Single change vs the stotight50 PASS: seed 17 -> 13 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate combined slip < 2.407 (beats every existing passer) with det under every cap — the promotion candidate. Prediction-if-false: seed13's basin does not benefit (margins at or inside noise of its own -4.5 reading, or the basin flips and the gate fails) — dose gains are seed17-specific, champion stays seed13@-4.5. Strongest alternative: pass with margins between 2.407 and 2.569 (stacking is partial; still informative for promotion choice).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip <= seed13@-4.5's 2.407 (new family-best; promotion candidate). INFORMATIVE = pass=true with slip in (2.407, 2.569] (dose helps seed13 less than seed17; champion choice unchanged). FAIL = pass=false or det softens below a cap (dose overshoots seed13's basin).

**verdict**: Deepening the noise floor does NOT stack with the best seed: seed13 at -5.0 still passes the 60s joystick DONE-gate but with WORSE margins than its own -4.5 reading -- the run's prediction-if-false landed. Plain result: pass=true, 0 falls/48, gait_valid 48/48, no sacrificed legs, but combined slip 2.63 vs seed13@-4.5's 2.407 (pre-registered PASS bar missed; even outside the 2.569 INFORMATIVE ceiling) and dir 38.21deg vs 36.4. Per-pass: DR-0 slip 2.577/dir 38.0, own-DR 2.763/39.59. No FAIL trigger: det did not soften (15s DR-0 det prog 0.79/slip 1.61, strong), videos watched det+sto -- clean upright six-leg gait, worst sto episode (slip 2.70) still walking. Reward rose all run (quarters -134->499, std annealed 0.0067) -- reward and gate agree; no misalignment, no continuation case. Conclusion: the -5.0 rung's gains are seed17-specific; dose response is per-basin, not universal. Champion candidate unchanged: seed13@-4.5 (slip 2.407/dir 36.4) keeps the fattest margins. Next: test whether the deeper floor transfers to the two seeds with the named hardening gaps (seed23 own-DR dir 40.36, seed29 own-DR slip 2.736) -- stotight50-seed23/seed29 batch.


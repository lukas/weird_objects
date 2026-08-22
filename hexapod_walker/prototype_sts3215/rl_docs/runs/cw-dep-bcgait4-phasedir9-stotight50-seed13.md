# cw-dep-bcgait4-phasedir9-stotight50-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-22T18:28:45+00:00

**pod**: hexapod-mjx-train-4

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50

**hypothesis**: Plain English: the widest-margin joystick gate passer so far is seed13 at the -4.5 noise floor (slip 2.407/dir 36.4), and deepening the floor to -5.0 just widened seed17's margins with no det cost — this arm combines best seed with deeper dose to ask whether the effects stack into the fattest-margin champion candidate. Single change vs the stotight50 PASS: seed 17 -> 13 (identical -5.0 recipe, fresh reinit per the lineage rule). Prediction-if-true: joygate combined slip < 2.407 (beats every existing passer) with det under every cap — the promotion candidate. Prediction-if-false: seed13's basin does not benefit (margins at or inside noise of its own -4.5 reading, or the basin flips and the gate fails) — dose gains are seed17-specific, champion stays seed13@-4.5. Strongest alternative: pass with margins between 2.407 and 2.569 (stacking is partial; still informative for promotion choice).

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined slip <= seed13@-4.5's 2.407 (new family-best; promotion candidate). INFORMATIVE = pass=true with slip in (2.407, 2.569] (dose helps seed13 less than seed17; champion choice unchanged). FAIL = pass=false or det softens below a cap (dose overshoots seed13's basin).


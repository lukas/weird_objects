# cw-dep-bcgait4-phasedir9-cmdmix45-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T19:33:16+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight45-seed29

**wandb_id**: 93y14tr0

**hypothesis**: Plain English: fourth basin of the cmdmix batch, completing seed coverage -- seed29 at its best dose (-4.5, joygate slip 2.704/dir 39.05) is the champion set's THINNEST margin (own-DR sto slip 2.736, just 0.164 under the 2.9 cap) and, like its 3 siblings (seed13/17/23, launched this same batch), was only ever trained on ONE fixed forward command though the gate grades turns/stops/reverses. This arm trains the identical recipe on the gate's own changing-command family to test whether on-distribution transition practice rescues seed29's tight own-DR slip margin specifically, completing the batch's read across all 4 champion-candidate seeds instead of leaving the worst-margin basin untested. Single change vs stotight45-seed29 (fresh reinit, same seed 29, same -4.5 dose): goal.walk_cmd_mode=stress_mix + walk_cmd_resample_s=4.0 + jitter 0.5 (the exact gate command family; training rng only, gate's held-out seed base 90000 untouched). Prediction-if-true: own-DR sto slip drops measurably below 2.736 (margin widens) with dir held near 39. Prediction-if-false: churn vs the BC anchor pushes slip closer to or over the 2.9 cap, or falls appear -- seed29's basin is too fragile for added command variety. Strongest alternative: within noise of parent -- emergent transfer already saturates, matching a 0/3 or mixed read from the other 3 basins.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND own-DR sto slip <= 2.6 (parent 2.736, outside rung noise) with combined dir held <= 41 (parent 39.05 + noise). INFORMATIVE = pass=true within noise of parent (on-distribution training adds nothing to this basin either). FAIL = pass=false or falls>0 (command churn breaks seed29's fragile basin). Read jointly with cmdmix45-seed13/cmdmix50-seed23/cmdmix55-seed17: 4/4 PASS = general lever; mixed = per-basin lottery (matches every other lever's fate this lineage); 0/4 with any FAIL = command churn is anti-productive on BC-anchored recipes.

**verdict**: FAIL per pre-registered gate: joygate pass=false — slip 3.043 > 2.9 cap (parent stotight45-seed29 2.704), dir 39.45 within allow, zero falls 48/48, gait_valid 48/48, no sacrificed legs. Same pure slip-margin regression as all three siblings; 15s rungs also degraded (det slip 3.09, own-DR sto 3.45 vs parent ~2.5-2.7). Reward rose (quarters -167.7 -> 460.6) while gate worsened = misalignment per 08-21 ruling, already root-caused by the batch: training on the gate's own stress_mix command churn fights the BC anchor; emergent transfer from fixed-command training beats practicing transitions. Finalizes the cmdmix batch at 0/4 PASS (3 FAIL, 1 INFORMATIVE) — lever stays CLOSED, no continuation. Champion unchanged: stotight45-seed13 (slip 2.407 / dir 36.4).


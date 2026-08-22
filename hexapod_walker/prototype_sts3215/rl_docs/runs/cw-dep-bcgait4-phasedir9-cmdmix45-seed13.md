# cw-dep-bcgait4-phasedir9-cmdmix45-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-22T19:20:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight45-seed13

**wandb_id**: 1uovy2ce

**hypothesis**: Plain English: every 60s-joystick-gate passer so far was trained on ONE fixed forward command, yet the gate grades turns, stops and reverses — this arm trains the champion recipe on the gate's own family of changing commands so the policy practices transitions instead of relying on emergent generalization. Single change vs stotight45-seed13 (the champion candidate, joygate slip 2.407/dir 36.4): goal.walk_cmd_mode=stress_mix + walk_cmd_resample_s=4.0 + jitter 0.5 (the exact gate command family; training rng, gate's held-out seed base 90000 untouched). Dose ladder is CLOSED (per-seed knees mapped 08-22: seed13@-4.5, seed17@-5.5, seed23@-5.0, seed29@-4.5) — command distribution is the last untried margin lever. Prediction-if-true: direction error drops toward/below the 35-deg teacher floor (transitions practiced, not improvised) with slip held. Prediction-if-false: mid-episode command churn fights the BC anchor and the basin degrades or falls appear. Strongest alternative: within noise of parent — emergent transfer already saturates what on-distribution training can teach.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined dir_err <= 34.4 (parent 36.4 minus 2 deg, outside rung noise) AND combined slip <= 2.51 (parent 2.407 + 0.1). INFORMATIVE = pass=true with dir/slip within noise of parent (on-distribution training adds nothing). FAIL = pass=false or falls>0 (command churn breaks the basin).


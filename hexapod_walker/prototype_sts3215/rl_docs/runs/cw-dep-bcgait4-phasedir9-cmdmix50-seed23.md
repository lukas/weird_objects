# cw-dep-bcgait4-phasedir9-cmdmix50-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-22T19:21:58+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight50-seed23

**wandb_id**: 0dzb357r

**hypothesis**: Plain English: third basin of the batch — seed23 at its best dose (-5.0, joygate slip 2.543/dir 35.31) also only ever trained on one fixed forward command; this arm trains it on the gate's own changing-command family (turns, stops, reverses) to test whether transition practice is a general margin lever across independent basins or another per-seed lottery (the fate of every prior lever here: ramp 1/3 seeds, dose knees all different). Single change vs stotight50-seed23: goal.walk_cmd_mode=stress_mix + walk_cmd_resample_s=4.0 + jitter 0.5 (gate command family, training rng only; held-out gate seed base untouched). Prediction-if-true: dir_err <= 33.3 with slip held. Prediction-if-false: churn vs BC anchor degrades the basin. Strongest alternative: within noise — emergent transfer already saturates.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined dir_err <= 33.31 (parent 35.31 minus 2 deg) AND combined slip <= 2.64 (parent 2.543 + 0.1). INFORMATIVE = pass=true within noise of parent. FAIL = pass=false or falls>0. Batch read across the 3 cmdmix arms: 3/3 PASS = general lever; 1/3 = basin lottery again; 0/3 with any FAIL = command churn is anti-productive on BC-anchored recipes.

**verdict**: Worst of the batch: on-distribution command training badly degraded seed23's basin — joygate FAIL, slip 3.595 (parent stotight50-seed23: 2.543), dir 43.67 (parent 35.31), own-DR slip 3.692. Zero falls, gait 48/48. Reward rose (Q4 ~430) while the gate regressed far beyond noise — pre-registered FAIL branch: mid-episode command churn fights the BC anchor. Evidence: logs/ckpt_eval/cw_dep_bcgait4_phasedir9_cmdmix50_seed23_joygate/gate_verdict.json. Next: with 2/3 FAIL the batch read is 'churn anti-productive on BC-anchored recipes' — command-distribution lever CLOSED.


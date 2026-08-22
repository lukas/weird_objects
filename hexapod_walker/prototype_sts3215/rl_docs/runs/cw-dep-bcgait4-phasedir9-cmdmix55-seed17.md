# cw-dep-bcgait4-phasedir9-cmdmix55-seed17

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INFORMATIVE

**created**: 2026-08-22T19:25:44+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-stotight55

**wandb_id**: 78f8lw5t

**hypothesis**: Plain English: the best direction-follower so far (seed17 at -5.5, dir 34.97 deg) still only ever trained on one fixed forward command — this arm trains that same recipe on the gate's own changing-command family (turns, stops, reverses) to test whether practicing transitions pushes direction error below the 35-deg teacher floor. Single change vs stotight55 (joygate slip 2.515/dir 34.97): goal.walk_cmd_mode=stress_mix + walk_cmd_resample_s=4.0 + jitter 0.5 (gate command family, training rng only; held-out gate seed base untouched). Second of a 3-basin batch (seed13/17/23 best doses) asking whether on-distribution command training is a general margin lever or another basin lottery. Prediction-if-true: dir_err < 33 with slip held. Prediction-if-false: churn vs BC anchor degrades the basin. Strongest alternative: within noise — emergent transfer already saturates.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, held-out seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true AND combined dir_err <= 32.97 (parent 34.97 minus 2 deg) AND combined slip <= 2.62 (parent 2.515 + 0.1). INFORMATIVE = pass=true within noise of parent. FAIL = pass=false or falls>0.

**verdict**: Only arm of the 3-basin cmdmix batch to keep the joygate (pass=true, 0 falls/48, gait 48/48) but it MISSED the pre-registered PASS bar and lost margin vs parent stotight55: dir 34.97->36.73 (bar was <=32.97), slip 2.515->2.817 (bar 2.62; own-DR 2.897 grazing the 2.9 cap). Fixed-heading det panel also softened (prog 0.70->0.55, slip 2.01->2.84). Reward rose while every gate margin shrank: on-distribution command training teaches nothing the fixed-command basin didn't already generalize to, and costs slip. Batch read (0/3 PASS, 2 FAIL): command churn is anti-productive on BC-anchored recipes — lever CLOSED. Champion unchanged: stotight45-seed13 (2.407/36.4). Evidence: logs/ckpt_eval/cw_dep_bcgait4_phasedir9_cmdmix55_seed17_joygate/gate_verdict.json + _gate/_owncfg reports.


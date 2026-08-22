# cw-dep-bcgait4-phasedir9-stotight45-seed29

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T17:30:27+00:00

**pod**: hexapod-mjx-train-2

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: svd95ftl

**hypothesis**: Plain English: third seed-reproduction arm for the first-ever 60-second joystick-gate pass — same recipe on the lineage's historically worst seed, the hardest honest test of whether the recipe generalizes. Identical to stotight45 except seed 17->29 (worst det history: 0.740x no-ramp, worse under every ramp dose). Prediction-if-true: sto slip still lands ~2.5-2.9 (the noise-floor mechanism is seed-independent) even if det axes fail. Prediction-if-false: everything fails wide — the -4.5 dose does nothing without a good det basin. With seeds 13/23 this measures the DONE-gate pass rate at n=4 seeds for the promotion question q_20260822T1730Z.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true. Secondary: own-DR sto slip vs seed17 dose curve (3.00/2.87/2.48).

**verdict**: The recipe passes the joystick DONE gate on the lineage's historically WORST seed — the strongest possible seed-reproduction evidence. Evidence: eval_joystick_gate pass=true, n=48 held-out episodes, ZERO falls, gait_valid 48/48, no sacrificed legs, slip/m med 2.704 (cap 2.9; DR-0 2.687 / own-DR-0.35 2.736), dir_err med 39.05deg (allow 40; 38.45/39.38) — thinner margins than seed13 (2.407/36.4) and the original stotight45 pass (2.671/38.6), consistent with seed29's history as the worst basin of the n=4 -3.2-dose sample (0.740x prog), but still under every cap. Training healthy: std annealed to 0.011 on schedule, ep_rew rose every quarter (-150 -> +498). Videos watched (video-joygate rerun, det+sto, DR-0 + own-DR): clean upright six-leg alternating gait, no drag/flag/paddle pathology. Why it matters: with seed17 (original), seed13, and now seed29 all passing, the stotight45 recipe (longrun17 stack + --log-std-final -4.5 noise floor) is 3/3 on tested seeds vs 1/4 at the -3.2 dose — the -4.5 noise floor converted a seed lottery into a reproducible recipe. Next: seed23 finishes the n=4 sample (still training, another cycle owns it); q_20260822T1730Z's implied independent-seed-reproduction bar is met — promotion of stotight45 as joystick champion candidate stands, gate-green declaration still operator-confirmed. Caveat honestly recorded: seed29's own-DR sto margins are the thinnest of the three passers (slip 2.736/2.9, dir 39.4/40).


# cw-dep-bcgait4-phasedir9-stotight45-seed13

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T17:28:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: 6w0jmnc6

**hypothesis**: Plain English: the robot just passed the full 60-second joystick test for the first time, but only on one training seed — this arm re-runs the exact same recipe on a second seed to learn whether the pass is the recipe or a lottery ticket. Identical stack to stotight45 (longrun17 recipe, fresh re-init, --log-std-final -4.5), only the seed changes (17->13). At -3.2 the det rung pass rate was 1/4 (seed13 failed at 0.79x prog); the -4.5 noise floor mechanically fixed sto slip on seed17 (monotone dose-response). Prediction-if-true (recipe-robust): this seed's sto slip also lands ~2.5-2.9 and the session gate passes or near-misses only on det progress. Prediction-if-false (seed lottery): sto slip improves (mechanism is seed-independent) but the session gate fails on det axes like seed13 always has. Strongest alternative: noise floor interacts with basin selection and even sto slip stays >2.9.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true (zero falls, slip<=2.9, dir<=40, gait_valid all). Secondary read: own-DR sto slip vs the seed17 dose curve (3.00/2.87/2.48).

**verdict**: The first-ever joystick DONE-gate pass is NOT a lottery ticket — the same recipe on a second seed passes the full 60s randomized joystick session gate again. Evidence: eval_joystick_gate pass=true, n=48 held-out episodes, ZERO falls, gait_valid 48/48, no sacrificed legs (per-leg sacrificed_frac all 0.0), slip/m med 2.407 (cap 2.9; DR-0 2.274 / own-DR-0.35 2.566), dir_err med 36.4deg (allow 40; 35.4/37.9 per pass) — comfortably wider margins than the original stotight45 pass (2.671 slip / 38.6 dir). Training healthy: std annealed exactly to 0.011, ep_rew rose every quarter (-166 -> +507). Videos watched (video-joygate rerun, det+sto, DR-0 + own-DR): clean upright six-leg alternating gait, body level, no drag/flag/paddle pathology. Why it matters: seed13 was this lineage's historically WORSE seed (failed the 15s rung at every prior dose) — the -4.5 log-std noise floor recipe lifts even a bad-basin seed over the full session gate, so the recipe (not seed luck) is doing the work. Next: with seed29 also passing (same cycle), recipe pass rate is 3/3 known seeds; update q_20260822T1730Z — the operator's implied independent-seed-reproduction bar is now met.


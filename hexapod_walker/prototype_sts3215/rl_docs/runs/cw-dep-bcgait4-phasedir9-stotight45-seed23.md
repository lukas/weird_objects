# cw-dep-bcgait4-phasedir9-stotight45-seed23

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-22T17:32:13+00:00

**pod**: hexapod-mjx-train-1

**steps**: 4000000

**parent**: cw-dep-bcgait4-phasedir9-longrun17-stotight45

**wandb_id**: 67r5g96d

**hypothesis**: Plain English: second of three seed-reproduction arms for the first-ever 60-second joystick-gate pass — same recipe, new seed, to measure whether the pass survives the seed lottery. Identical to stotight45 except seed 17->23. Seed23's det history at -3.2: 0.818x prog no-ramp, worse under every ramp dose. Prediction-if-true: sto slip lands ~2.5-2.9 (mechanism seed-independent) and the session gate passes or misses only on det. Prediction-if-false: session gate fails wide like seed23's rung history. Strongest alternative: noise floor changes basin selection and this seed lands somewhere new entirely.

**gate**: eval_joystick_gate 60s randomized session (stress_mix, seed base 90000, DR-0 + own-DR 0.35, det+sto, n=12/pass): PASS = evaluator pass=true. Secondary: own-DR sto slip vs seed17 dose curve.

**verdict**: SECOND SEED PASSES THE JOYSTICK DONE GATE — the stotight45 recipe reproduces across seeds. Plain English: a brand-new seed (23) of the exact recipe that produced the first-ever 60s joystick gate pass (seed17) also passes the full randomized session gate: evaluator pass=true, ZERO falls 48/48, gait_valid 48/48, no sacrificed legs, slip/m 2.78 (cap 2.9), dir_err 39.4deg (allow 40). Honest margin notes: thinner than seed17's 2.671/38.6, and the own-DR-alone dir median is 40.36deg — a hair over the allowance on its own even though the gate's combined aggregation passes; sto own-DR worst-episode slip 4.08. 15s rung numbers strong and uniform: DR-0 det prog 0.58/slip 2.57/fwd 0.62m, own-DR det 0.62/2.53/0.66m, zero terminations anywhere. Video (DR-0 + own-DR det strips watched): clean level six-leg alternating gait, no flag leg, no dragging. Training reward rose all run (-163->504) with std annealed to 0.011 — reward and gate agree. Prediction-if-true fired exactly (sto slip landed 2.5-2.9, session passes). WHY IT MATTERS: seed17's pass is now NOT a 1-in-N lottery — 2/2 stotight45 seeds measured so far pass (vs the -3.2 longrun recipe's 1/4); seed13/29 (concurrent cycle) complete the n=4 pass-rate reading. Hardware-ready: sim-gate yes; promotion/champion question already filed q_20260822T1730Z.


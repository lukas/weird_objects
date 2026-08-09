# cw-walk-legmass25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T20:46:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: wmu1agzv

**hypothesis**: OPERATOR WISHLIST 13b, per-leg mass asymmetry axis (last unexposed dr.<field>; real robot has uneven wiring + a damaged L5 knee). Champion baseline FIRST per c59/c60 rule: NOT free - longdist_r2_legmass_base craters 2/6 det draws (prog 0.38-0.53, slip 2.9-4.2) under dr.leg_mass_jitter_pct=0.25. ISOLATED: dr-scale 0.0 with only leg-mass jitter 25% per leg-link, one variable off the no-DR champion. Plain: each leg weighing a bit different (wiring, repairs) should not break the gait. If-true: exposure hardens it like mass/friction did - own-cfg gv 12/12, 0 term, det med fwd >=1.2m, and DR0 nominal retention clean (slip <=1.24) - axis joins the recipe. If-false: asymmetric inertia is not trainable by exposure (worst draws stay near-in-place or nominal pays) - flag for the estimator rung like torque. Strongest alternative: passes by slowing/shortening stride on hard draws - per-episode fwd will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.leg_mass_jitter_pct=0.25, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det; baseline logs/ckpt_eval/longdist_r2_legmass_base


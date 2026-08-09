# cw-walk-legmass25

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: NO-EFFECT

**created**: 2026-08-09T20:46:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: wmu1agzv

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13b, per-leg mass asymmetry axis (last unexposed dr.<field>; real robot has uneven wiring + a damaged L5 knee). Champion baseline FIRST per c59/c60 rule: NOT free - longdist_r2_legmass_base craters 2/6 det draws (prog 0.38-0.53, slip 2.9-4.2) under dr.leg_mass_jitter_pct=0.25. ISOLATED: dr-scale 0.0 with only leg-mass jitter 25% per leg-link, one variable off the no-DR champion. Plain: each leg weighing a bit different (wiring, repairs) should not break the gait. If-true: exposure hardens it like mass/friction did - own-cfg gv 12/12, 0 term, det med fwd >=1.2m, and DR0 nominal retention clean (slip <=1.24) - axis joins the recipe. If-false: asymmetric inertia is not trainable by exposure (worst draws stay near-in-place or nominal pays) - flag for the estimator rung like torque. Strongest alternative: passes by slowing/shortening stride on hard draws - per-episode fwd will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.leg_mass_jitter_pct=0.25, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det; baseline logs/ckpt_eval/longdist_r2_legmass_base

**verdict**: NO-EFFECT: per-leg mass jitter (0.25) exposure training did not change the champion's response to the axis. Own-cfg eval (dr=0+leg_mass override) is episode-identical to the unexposed champion baseline (logs/ckpt_eval/longdist_r2_legmass_base): same 2/6 det draws crater near-in-place with matching severity (legmass25 prog 0.54/0.38 slip 2.77/3.93 vs baseline prog 0.53/0.38 slip 2.93/4.22); the other 4/6 draws also track baseline within noise (med prog 0.94 vs 0.91, slip 1.20 vs 1.20). DR0 nominal retention is clean (det gv 6/6, slip/m med 0.92 <=1.24, fwd 1.59m = champion band) -- exposure did no harm, but also no good. Frames confirm the crater draws are stationary paddling, not a new pathology. Per the c60/c62 test-champion-first rule: leg-mass asymmetry joins contact-stiffness/torque-droop/servo-gain as an axis NOT fixable by naive single-axis DR-0 exposure -- last unexposed 13b axis closed this way.


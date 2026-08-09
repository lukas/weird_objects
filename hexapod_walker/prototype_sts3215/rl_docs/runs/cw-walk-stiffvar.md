# cw-walk-stiffvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: NO-EFFECT

**created**: 2026-08-09T17:19:28+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**hardware_ready**: False

**hypothesis**: OPERATOR WISHLIST 13b via c49 dr.<field> overrides, next isolated axis after mass/latency/torque/friction: CONTACT COMPLIANCE (solref timeconst). The real robot's rubber feet on unknown floors are the least-calibrated contact quantity we have, and contact is the ruled root of the skating defect - exposure to varied stiffness is the cheapest transfer hedge while the operator's hardware calibration waits. ISOLATED: dr-scale 0.0 with ONLY dr.contact_stiff_scale=0.5,2.5 randomized (wider than the 0.7-2.0 standard envelope), one variable off the no-DR champion. Prediction-if-true: gait absorbs compliance variation (own-cfg gv 12/12, 0 term, det med fwd >=1.2m @30s) and DR0 nominal retention holds - compliance becomes a keeper robustness rung. Prediction-if-false: soft/stiff contact breaks the paddle gait (terminations, prog craters, or slip explodes at the soft end) - the paddle transport is compliance-fragile, strengthening the case that corrected contact physics will need a retrain (readiness review P0). Strongest alternative: policy passes by slowing/parking on soft contact - frames + prog_ratio will show it.

**gate**: own-cfg harness at --dr-scale 0.0 + dr.contact_stiff_scale=0.5,2.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det

**verdict**: NO-EFFECT: contact-compliance jitter (0.5-2.5x solref) exposure training did not meaningfully change the champion's response to the axis. Own-cfg eval (dr=0+contact_stiff override) tracks the unexposed champion baseline (logs/ckpt_eval/longdist_r2_stiffvar_base) episode-by-episode: same 2/6 det draws crater near-in-place with similar-or-worse severity (stiffvar prog 0.49/0.42 slip 3.47/3.44 vs baseline prog 0.51/0.39 slip 3.14/4.65); median moves inside noise (prog 0.93 vs 0.90, slip 1.29 vs 1.45, fwd 1.38 vs 1.46m) -- deltas too small/inconsistent in direction to call an improvement. DR0 nominal retention is clean (det gv 6/6, slip/m med 0.97 <=1.24, fwd 1.52m = champion band) -- exposure did no harm, but did not fix the extremes either. Frames confirm the crater draws are stationary paddling, not a new pathology (no flag leg, no fall). Contact compliance joins leg-mass/torque-droop/servo-gain as an axis NOT fixable by naive single-axis DR-0 exposure -- consistent with friction's finding that the real fix is the operator's contact/current pricing calibration, not more exposure training.

**refused_reason**: hexapod-mjx-train-7 already runs cw-walk-head90-s1 — GPU pods host exactly one run; pick a free GPU pod.


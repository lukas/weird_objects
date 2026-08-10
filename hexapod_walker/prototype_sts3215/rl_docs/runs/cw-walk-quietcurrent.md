# cw-walk-quietcurrent

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T22:25:42+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 3j7l9vib

**hardware_ready**: no

**hypothesis**: WISHLIST item 13 [READY]: quiet gait -- minimize hot-current concentration as an explicit objective, hardware-friendliness. ONE variable off champion: enable the existing (default-OFF) per-servo hot-current penalty reward.k_current_hot=0.2 above current_hot_a=1.5A (same coefficients that halved sustained hot-time 3.7->1.5-2.4s in a stance-mode probe, cw-stance-even, without code changes -- now tried on the WALK task for the first time). Plain: charge concentration directly (quadratic above a soft per-servo threshold) so 6 legs at 0.4A cost nothing but one knee riding at 1.8A hurts, discouraging the tripod/hot-knee load pattern visual eval has flagged before. If-true: own-cfg det gv 12/12, 0 term, det median fwd >=1.2m retained (walk gate unchanged) AND mean/peak servo current measurably lower than champion baseline on the same draws -- quiet gait is a free win, stack it into the champion. If-false: current drops but distance/slip erodes (gait redistributes load by slowing down, not by walking better) or current is unchanged (concentration is not gradient-reachable during locomotion, unlike static stance) -- shelve until the current-pricing operator calibration lands. Strongest alternative: the champion already avoids sustained hot-knee riding while walking (unlike static holds) so this term is a no-op paid for nothing -- triage MUST compare mean/peak per-servo current against a champion baseline run under IDENTICAL draws before verdicting, not just eyeball frames.

**gate**: Own-cfg DR0 harness det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m (walk gate unchanged from champion); champion baseline run on IDENTICAL draws for mean/peak servo current comparison at triage; if-true needs a measurable (not noise-band) current reduction; frames watched det for tripod/hot-knee posture change

**verdict**: FAIL (if-false confirmed). Enabling the hot-current-concentration penalty (k_current_hot=0.2 above 1.5A) on the WALK task for the first time: gait itself is unharmed (own-cfg det gv 12/12, 0 term, det med fwd 1.50m >=1.2 gate, DR0 retention clean) but per-servo current is NOT measurably lower than a champion baseline run on IDENTICAL draws (logs/ckpt_eval/cw_walk_longdist_r2_currentbaseline, same seed/cfg): cur_max_a 2.49 vs champion 2.53, cur_p95_a 1.38 vs 1.34, cur_s_above_soft 10.9 vs 9.6 (det, slightly WORSE not better), leg_imbalance 1.09 vs 1.14 -- every delta is inside run-to-run noise, none is a reduction. Matches the pre-registered strongest-alternative: the champion already avoids sustained hot-knee riding while walking (unlike the static-stance probe where this term halved hot-time), so the penalty is a no-op paid for nothing -- reward quarters show the expected small hit (1460 final vs unpenalized bands) with zero behavioral change. Quiet-gait-via-this-penalty is CLOSED for the walk task; shelve until the current-pricing operator calibration lands (per plan).


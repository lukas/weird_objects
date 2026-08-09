# cw-walk-quietcurrent

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T22:25:42+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 3j7l9vib

**hypothesis**: WISHLIST item 13 [READY]: quiet gait -- minimize hot-current concentration as an explicit objective, hardware-friendliness. ONE variable off champion: enable the existing (default-OFF) per-servo hot-current penalty reward.k_current_hot=0.2 above current_hot_a=1.5A (same coefficients that halved sustained hot-time 3.7->1.5-2.4s in a stance-mode probe, cw-stance-even, without code changes -- now tried on the WALK task for the first time). Plain: charge concentration directly (quadratic above a soft per-servo threshold) so 6 legs at 0.4A cost nothing but one knee riding at 1.8A hurts, discouraging the tripod/hot-knee load pattern visual eval has flagged before. If-true: own-cfg det gv 12/12, 0 term, det median fwd >=1.2m retained (walk gate unchanged) AND mean/peak servo current measurably lower than champion baseline on the same draws -- quiet gait is a free win, stack it into the champion. If-false: current drops but distance/slip erodes (gait redistributes load by slowing down, not by walking better) or current is unchanged (concentration is not gradient-reachable during locomotion, unlike static stance) -- shelve until the current-pricing operator calibration lands. Strongest alternative: the champion already avoids sustained hot-knee riding while walking (unlike static holds) so this term is a no-op paid for nothing -- triage MUST compare mean/peak per-servo current against a champion baseline run under IDENTICAL draws before verdicting, not just eyeball frames.

**gate**: Own-cfg DR0 harness det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m (walk gate unchanged from champion); champion baseline run on IDENTICAL draws for mean/peak servo current comparison at triage; if-true needs a measurable (not noise-band) current reduction; frames watched det for tripod/hot-knee posture change


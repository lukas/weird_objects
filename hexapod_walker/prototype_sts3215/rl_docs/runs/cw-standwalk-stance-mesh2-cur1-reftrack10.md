# cw-standwalk-stance-mesh2-cur1-reftrack10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T07:05:39+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-cur1

**hypothesis**: Plain English: cur1's full-mix rise topples instead of following the demonstrated rise reference (k_rise_ref_track=2.0). Companion to riseonly1's curriculum-share test: this arm keeps the ORIGINAL full goal-mix (hold=.1/rise=.45/lower=.45) but multiplies the tracking weight 5x (k_rise_ref_track=10.0), testing the other candidate fix (the weight is too weak vs other shaping terms, not that rise is undertrained by curriculum share). Prediction-if-true: rise's tilt_pitch/over_current-at-rise rate drops and DR-0 rise episodes end closer to valid plant even with hold/lower still competing for gradient. Prediction-if-false: still topples -- rules out a pure tracking-weight fix; combine with riseonly1's read to decide whether the fix is curriculum (split), pricing (weight), or budget (neither) at all.

**gate**: Discovery/canary read at 2M: pod_eval stance panel, all 3 modes, n=6 det DR-0. Read jointly with riseonly1: if reftrack10 fixes rise but riseonly1 doesn't (or vice versa), that names the correct lever for the real rung-3 launch; if neither fixes it, escalate to budget/teacher-signal instead of more reward-weight iteration.


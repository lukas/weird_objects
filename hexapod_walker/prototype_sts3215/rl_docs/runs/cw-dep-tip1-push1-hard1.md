# cw-dep-tip1-push1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-12T03:49:58+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-tip1-push1

**hypothesis**: Give the torque-shove (walk_push) training recipe the budget it was still using at the 2M discovery buzzer, on the identical one-variable recipe (warm from tip1-push1 itself, same dr.walk_push_prob=0.5/nm=2.0-3.0/s=0.8-1.5, same dep-line stack) -- the same pattern that turned cw-dep-bcgait1's partial existence-proof into cw-dep-bcgait1-hard1's decisive PASS. Prediction-if-true: the matched-parent probe_walk_push.py fall-rate gap widens to >=2x (the pre-registered bar the discovery run just missed at 1.8x) with nominal DR0 retention unchanged. Prediction-if-false: the gap stays flat or narrows under more steps -- meaning the 2M signal was budget noise, not a strengthening mechanism, and the torque-DR family closes for real this time (3rd-and-final arm).

**gate**: PASS if matched-parent probe_walk_push.py (n>=12 seeds/side, same forced 2.6Nm/1.5s dose) shows child fall rate >=2x lower than frozen tip1 AND nominal DR0 walk retention (gait_valid, slip/m, prog_ratio) matches this run's own 2M discovery band with zero new falls. FAIL if the fall-rate gap does not widen past 1.8x (flat or worse) -- torque-DR family CLOSED for good, remaining lever is contact/pinning modeling only.

**refused_reason**: --steps belongs to the launcher, not the passthrough args


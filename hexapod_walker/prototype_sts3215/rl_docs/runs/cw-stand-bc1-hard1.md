# cw-stand-bc1-hard1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-11T04:37:30+00:00

**pod**: hexapod-mjx-train-6

**steps**: 10000000

**parent**: cw-stand-bc1

**hypothesis**: The BC-anchor mechanism that fixed the flag-leg cheat in discovery (cw-stand-bc1) is trained for real: does the SAME recipe at a hardening budget (10M vs 2M) consolidate the honest six-foot plant (raise flat-start valid_plant off its current 0/10 footprint-only miss) and recover the hold/track precision and raise/tipped-recovery probes that looked worse in cw-stand-bc1's own training diagnostic (n=2 samples, weak evidence) - or does more training re-drift back toward the cheat now that the anchor's pull is diluted over more updates? Continuation of the SAME arm (init-from-source), one variable (steps), per the phase system's hardening rule (already visibly works, now prove it holds and improves with budget).

**gate**: harness at 10M (all of hold/track/rise/raise/lower/tipped, det+sto, DR0 + own-DR): rise valid_plant >=8/18 det (44%, vs today's 13/30=43% RSI-off / 3/6=50% RSI-on) held or improved on ALL start kinds including >=2/10 flat (today 0/10); hold/track_err_mean_deg back under 2.0 deg (today 3.0); raise and tipped_recovery each >=1/2 in the training periodic-eval at every checkpoint from 4M on (today's parent-vs-child regression, if real, must resolve). Kill early if rise_feet_factor collapses back under 0.4 for 2 consecutive periodic-eval windows (the pre-anchor collapse signature) - would mean the anchor's fix doesn't survive a longer horizon.


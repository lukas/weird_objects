# cw-standwalk-stance-mesh2-stancemix-bcchain3-slowchain

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T16:04:27+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal

**hypothesis**: Does the rise pace-halving lever validated on the ISOLATED rise lineage (riseonly-bcchain3-slowchain: bc_anchor_lookahead_s 0.5->0.25s, min_h_ahead_mm 15->8mm, which cut deep-start cur_p95 2.64A->1.85A and raised DR-0 det valid_plant 2/6->3/6) ALSO unpin rise's deep-start press-up when ported into the FULL hold+rise+lower mix, warm-started from the just-PASSED stancemix-bcchain3-stdanneal checkpoint (hold 6/6+6/6 zero-term @0.78A, lower 6/6+6/6 @<=1.4mm, rise det 2/6 with deep starts pinned 2.64A) -- without disturbing the hold/lower parity that checkpoint just achieved? Warm-start (not from-scratch) so hold/lower keep their solved weights; log-std pinned at -4 (no re-anneal, mirroring the isolated cont8 continuation) since noise re-injection was already shown NOT to be the rise blocker. Prediction-if-true: rise DR-0 det valid_plant rises above 2/6 and/or deep-start cur_p95 median falls below 2.64A toward slowchain's 1.85A, while hold stays >=5/6+5/6 zero-term <=1.0A and lower stays >=4/6 honest (<=10mm) -- pace transfers cleanly into the mix. Prediction-if-false: rise stays pinned at 2.64A/2-6 valid_plant despite the pace change (mix dynamics differ from isolated), or hold/lower regress (the mix trades one mode's gain for another's loss under shared policy capacity) -- either result means the pace fix must be re-derived in-mix, not simply ported.

**gate**: 8M continuation (same budget class), DR-0 + own-DR(0.2) det+sto n=6+6 per mode. PASS: rise DR-0 det valid_plant >=3/6 (beats stdanneal's 2/6) AND deep-start cur_p95 median <2.64A AND hold det+sto stay >=5/6 valid_plant with zero hold_min_load/over_current terms (cur_p95<=1.0A) AND lower det stays >=4/6 honest descents (height_err_end<=10mm) -- pace lever transfers into the mix without cross-mode cost. PARTIAL: rise improves (valid_plant count up and/or cur_p95 median falls) without crossing the full bar, hold/lower unchanged or only mildly softer -- pace helps but doesn't finish the job in-mix either. FAIL: rise unchanged (still <=2/6, cur_p95 still ~2.64A) OR any of hold/lower regresses vs the stdanneal parent (loses its just-achieved zero-term/clean-descent parity) -- the isolated fix does not transfer, the mix needs its own lever.


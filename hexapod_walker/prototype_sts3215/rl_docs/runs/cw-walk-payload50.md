# cw-walk-payload50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:22:18+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: kvhl6uv7

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 11 (payload) via new dr.<field> cfg overrides (cycle 49, tag exp/cycle49-dr-overrides): champion walks while carrying up to +50% chassis mass. ISOLATED axis: dr-scale 0.0 (nominal sim + sensor noise floors) with ONLY dr.mass_scale=1.0,1.5 randomized - one variable off the no-DR champion. If-true: gait absorbs payload (own-cfg payload harness gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 no-payload retention holds - payload becomes a robustness rung to keep. If-false: mass shifts collapse height/gait (terminations or prog craters) - payload needs a curriculum or explicit height/effort adaptation, not exposure. Strongest alternative: policy survives by crouching/parking rather than walking loaded - frames + height_err/duty will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 no-payload retention det 6/6 gv, det slip/m within champion band (<=1.24); frames watched det

**verdict**: PASS (pre-registered gate met): payload harness (mass 1.0-1.5x) gv 12/12, 0 term, det median fwd 1.31m>=1.2; DR0 no-payload retention gv 6/6, det slip/m 1.15<=1.24. Caveat named from frames+scalars: at the heaviest draws (2/6 det eps) the robot squats and shuffles - prog 0.39-0.47, slip/m 3.4-3.8, ~half speed - but never parks, sacrifices a leg, or falls. Payload up to ~+40% is solid; the 1.4-1.5x top end degrades tracking. Not hardware-ready (lineage paddle-slide unchanged).


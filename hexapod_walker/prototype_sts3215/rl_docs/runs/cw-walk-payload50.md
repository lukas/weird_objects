# cw-walk-payload50

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:22:18+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: kvhl6uv7

**hypothesis**: OPERATOR WISHLIST 11 (payload) via new dr.<field> cfg overrides (cycle 49, tag exp/cycle49-dr-overrides): champion walks while carrying up to +50% chassis mass. ISOLATED axis: dr-scale 0.0 (nominal sim + sensor noise floors) with ONLY dr.mass_scale=1.0,1.5 randomized - one variable off the no-DR champion. If-true: gait absorbs payload (own-cfg payload harness gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 no-payload retention holds - payload becomes a robustness rung to keep. If-false: mass shifts collapse height/gait (terminations or prog craters) - payload needs a curriculum or explicit height/effort adaptation, not exposure. Strongest alternative: policy survives by crouching/parking rather than walking loaded - frames + height_err/duty will show it.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2 m; plus DR0 no-payload retention det 6/6 gv, det slip/m within champion band (<=1.24); frames watched det


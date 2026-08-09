# cw-walk-deadband30

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:11:27+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 1sbenkad

**hypothesis**: OPERATOR WISHLIST 13b/13c: servo DEADBAND, the classic cheap-servo sim2real gap (13c names backlash/deadband first). STS3215 deadband widens with wear and load; the champion trained at nominal only. ISOLATED axis via dr overrides: dr-scale 0.0 with ONLY dr.deadband_scale=1.0,3.0 randomized - worse-than-nominal only (narrower deadband is free), one variable off the no-DR champion. Plain: keep walking when the servos ignore small corrections. Prediction-if-true: gait absorbs up to 3x deadband (own-cfg gv 12/12, 0 term, det med fwd >=1.2m @30s) with DR0 nominal retention - deadband tolerance is a keeper rung. Prediction-if-false: wide deadband kills the small-amplitude paddle strokes (prog craters or jitter/limit-cycling as the policy overdrives to punch through) - the paddle gait depends on fine corrections real servos won't deliver, a concrete hardware-readiness defect to record. Strongest alternative: policy compensates with larger, jerkier strokes - passes scalars but frames show lurching (verdict must lead with it).

**gate**: own-cfg harness at --dr-scale 0.0 + dr.deadband_scale=1.0,3.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m med <=1.24; frames watched det


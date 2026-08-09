# cw-walk-velsag

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T17:50:53+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip

**wandb_id**: pl37zvwj

**hypothesis**: OPERATOR WISHLIST 13b/13c: servo VELOCITY DROOP (voltage sag) - the battery-side axis none of the running physics arms cover (friction/compliance/deadband/CoM/latency are contact/servo-timing axes). STS3215 max speed drops with bus voltage as the battery drains; the champion trained at nominal vel limits only. ISOLATED axis via dr overrides: dr-scale 0.0 with ONLY dr.vel_scale=0.70,1.00 (worse-than-nominal only - up to 30 percent slower servos; absolute override applied after scaling), one variable off the no-DR champion. Plain: keep walking when the servos get slow on a draining battery. Prediction-if-true: gait absorbs 30 percent speed droop - own-cfg gv 12/12, 0 term, det med fwd >=1.2m @30s, slip/m within champion band + noise. Prediction-if-false: the paddle gait's stroke timing depends on nominal servo speed (cadence collapses, prog craters, or jitter as strokes clip) - a concrete battery-life readiness defect to record. Strongest alternative: policy passes by slowing cadence uniformly - passes scalars, check cadence/stride vs champion and say so.

**gate**: own-cfg 30s 6+6 with dr.vel_scale=0.70,1.00 override active (dr-scale 0.0 like training): gv 12/12, 0 term, det med fwd >=1.2m, det slip/m med <=1.2 (champion band ~0.94-1.1 + noise); nominal retention det 6 eps gv 6/6; frames watched det

**verdict**: KILLED at ~4.2M/20M steps (not ~0 as first recorded): duplicate of cw-walk-velsag30 (concurrent cycle, launched 6 min earlier, same isolated dr.vel_scale voltage-sag axis off the same champion parent). No training verdict — the velocity-droop question is owned by cw-walk-velsag30. Partial checkpoint not evaluated.


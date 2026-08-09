# cw-walk-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:43:26+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: riyz9vdh

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run) + readiness review reliability-via-targeted-axes: floor slope is the 'terrain-lite' axis every real room has (ramps, thresholds, carpet edges). ISOLATED axis via dr.<field> overrides: dr-scale 0.0 with ONLY dr.ground_tilt_deg=5.0 (tilt u(0,5deg), random azimuth; full-DR default is 2deg, champion trained at 0) - one variable off the no-DR champion. Plain: the robot should walk across a mildly sloped floor without stalling uphill or sliding downhill. Prediction-if-true: gait holds across the slope spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - slope robustness is trainable by exposure and joins the transfer recipe. Prediction-if-false: uphill draws collapse progress or downhill draws slide/fall - slopes need a curriculum or explicit incline estimation, not plain exposure. Strongest alternative: policy survives by crabbing across-slope or slowing cadence - check heading err + cadence vs champion. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det


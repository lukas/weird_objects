# cw-walk-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:43:26+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: riyz9vdh

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run) + readiness review reliability-via-targeted-axes: floor slope is the 'terrain-lite' axis every real room has (ramps, thresholds, carpet edges). ISOLATED axis via dr.<field> overrides: dr-scale 0.0 with ONLY dr.ground_tilt_deg=5.0 (tilt u(0,5deg), random azimuth; full-DR default is 2deg, champion trained at 0) - one variable off the no-DR champion. Plain: the robot should walk across a mildly sloped floor without stalling uphill or sliding downhill. Prediction-if-true: gait holds across the slope spread (own-cfg gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - slope robustness is trainable by exposure and joins the transfer recipe. Prediction-if-false: uphill draws collapse progress or downhill draws slide/fall - slopes need a curriculum or explicit incline estimation, not plain exposure. Strongest alternative: policy survives by crabbing across-slope or slowing cadence - check heading err + cadence vs champion. Parent: rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: PASS - floor-slope axis lands by exposure: own-cfg (tilt u(0,5deg), random azimuth) gv 12/12, 0 term, det med fwd 1.40m; DR0 retention det 6/6 gv, slip/m med 1.03, prog 0.96 (= champion band). Honest tail: 2/6 steepest det draws slow to a shuffle (fwd 0.52-0.67m, slip/m 3.4-4.5) without falling - solid to ~3-4deg, 5deg marginal; no crabbing (track err <2deg). Frames watched det 6/6: level body, six legs cycling, lineage paddle-slide.


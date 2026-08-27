# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-27T03:50:45+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**hypothesis**: Plain sentence: the integrated stand+walk policy has only ever been commanded straight-ahead walking, and the new 60s mixed-session gate measured it ignoring direction commands (median direction error ~52 deg under the joystick stress_mix script, anchor2 smoke 08-27); this arm opens a modest +/-45 deg heading command cone in the walk segments of the same stage-2 recipe so the walk core learns to follow commanded directions -- required by the track DONE gate (directions followed) and the operator 08-27 integrated-policy priority. Single lever vs anchor2's own recipe: goal.walk_heading_max_rad 0.0 -> 0.7854 (heading rides the existing vx_ref/vy_ref obs channels, so the checkpoint is layout-compatible), 2M continuation off anchor2's own checkpoint. Prediction-if-true: walk survives (det gait_valid >=5/6) AND direction error drops well below the ~52 deg untrained baseline (<=35 deg full bar). Prediction-if-false: heading commands destabilize the walk core or direction error stays at baseline (course-reward pricing at nonzero headings, dig-in scope). Strongest alternative: needs the staged walk_cmd_stage curriculum, not a static cone. RETRY of the 03:37 queue attempt: that drain pass hit a transient dirty-code-marker refusal (a concurrent cycle's then-uncommitted WIP), not a design problem.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill maturity or close a class. WALK-SURVIVES: DR-0 det walk gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=0.2. DIRECTION-LEARNS: walk eval WITH goal.walk_heading_max_rad=0.7854: median direction_err_mean_deg materially below the anchor2 baseline (~52 deg, 08-27 mixed-session smoke; <=35 deg full bar, >=10 deg drop partial). STANCE-UNHARMED: hold/lower panels within the anchor2 band (hold/sto 6/6 hold_min_load is the KNOWN baseline failure, not a regression). JOINT call with -s1: both seeds direction-learn + walk survives -> promote the heading cone into the stage-2 recipe and open the next command rung (stops/reverses); walk collapses either seed -> close the static-cone lever, route direction coverage through walk_cmd_stage curriculum instead.

**refused_reason**: hexapod-mjx-train-0 code marker 099671613e43532328e9b659095d1b5e6a0d8719-dirty != local HEAD 099671613e43532328e9b659095d1b5e6a0d8719 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-0 (and snapshot/commit before that if the tree is dirty).


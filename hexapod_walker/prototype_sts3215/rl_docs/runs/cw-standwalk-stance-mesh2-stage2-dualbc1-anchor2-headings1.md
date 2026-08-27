# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T04:00:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: 2t05s1ng

**hypothesis**: Plain sentence: the integrated stand+walk policy ignores direction commands (median dir err ~52 deg on the 08-27 mixed-session smoke; it only ever trained straight-ahead); this arm opens a +/-45 deg heading cone (goal.walk_heading_max_rad 0 -> 0.7854, heading rides the existing vx_ref/vy_ref obs channels) as a 2M continuation off anchor2's own checkpoint. Prediction-if-true: walk survives (det gait_valid >=5/6) and dir err drops well below ~52 deg (<=35 full bar). Prediction-if-false: walk destabilizes or dir err unchanged (course-reward pricing at nonzero heading, dig-in). Alternative: needs the staged walk_cmd_stage curriculum. Retry: prior attempts REFUSED on a transient dirty-code-marker, then PARKED after racing another launch for the same pod; seed-1 twin is already RUNNING on train-0.

**gate**: MECHANISM-HEALTH CANARY ONLY. WALK-SURVIVES: DR-0 det walk gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=0.2. DIRECTION-LEARNS: walk eval WITH goal.walk_heading_max_rad=0.7854 median direction_err_mean_deg well below ~52 deg baseline (<=35 full bar, >=10 deg drop partial). STANCE-UNHARMED: hold/lower within anchor2 band (hold/sto 6/6 hold_min_load is the KNOWN baseline failure, not regression). JOINT 2-seed call: both learn+survive -> promote cone, open stops/reverses rung; either collapses -> close static cone, use walk_cmd_stage curriculum.


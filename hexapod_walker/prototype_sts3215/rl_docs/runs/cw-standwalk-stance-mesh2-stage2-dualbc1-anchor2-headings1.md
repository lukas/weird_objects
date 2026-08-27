# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T04:00:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: 2t05s1ng

**hypothesis**: Plain sentence: the integrated stand+walk policy ignores direction commands (median dir err ~52 deg on the 08-27 mixed-session smoke; it only ever trained straight-ahead); this arm opens a +/-45 deg heading cone (goal.walk_heading_max_rad 0 -> 0.7854, heading rides the existing vx_ref/vy_ref obs channels) as a 2M continuation off anchor2's own checkpoint. Prediction-if-true: walk survives (det gait_valid >=5/6) and dir err drops well below ~52 deg (<=35 full bar). Prediction-if-false: walk destabilizes or dir err unchanged (course-reward pricing at nonzero heading, dig-in). Alternative: needs the staged walk_cmd_stage curriculum. Retry: prior attempts REFUSED on a transient dirty-code-marker, then PARKED after racing another launch for the same pod; seed-1 twin is already RUNNING on train-0.

**gate**: MECHANISM-HEALTH CANARY ONLY. WALK-SURVIVES: DR-0 det walk gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=0.2. DIRECTION-LEARNS: walk eval WITH goal.walk_heading_max_rad=0.7854 median direction_err_mean_deg well below ~52 deg baseline (<=35 full bar, >=10 deg drop partial). STANCE-UNHARMED: hold/lower within anchor2 band (hold/sto 6/6 hold_min_load is the KNOWN baseline failure, not regression). JOINT 2-seed call: both learn+survive -> promote cone, open stops/reverses rung; either collapses -> close static cone, use walk_cmd_stage curriculum.

**verdict**: CANARY FAIL - MECHANISM (seed0; JOINT call with headings1-s1/seed1 below). Result: WALK-SURVIVES PASSES cleanly (det gait_valid 6/6 DR-0 + 6/6 own-DR, no sacrifice, prog_ratio 0.29-0.51). DIRECTION-LEARNS is a genuine but FRAGILE partial: DR-0 det dir_err_mean_deg 30-49 (median ~36-37), a real ~15deg drop from the ~52deg no-heading baseline and close to the <=35deg full bar -- some episodes (30.5, 34.9, 35.9deg) actually clear it. But this signal does NOT hold up: own-DR(0.5) det washes back out to median ~50deg (31-65 range, no longer clearly better than baseline), and STOCHASTIC mode never improves at either DR (64-88deg, worse than baseline in every read). STANCE-UNHARMED holds (hold/sto 6/6 term both DR, same as anchor2 baseline). JOINT CALL WITH SEED1 (headings1-s1, verdicted this same cycle): seed1 shows ZERO direction-learning signal in any condition (55-82deg everywhere). Cross-seed picture: the static +/-45deg heading-cone opener produces AT BEST an inconsistent, DR-fragile, deterministic-only partial signal on one seed and nothing on the other -- this does not meet the gate's 'both learn+survive -> promote' bar (neither seed clears it robustly) but is milder than the gate's 'either collapses' framing (walk never breaks on either seed, it's a pure non-learning/fragile-learning miss). Net: JOINT CALL CLOSED, do not promote the static cone as-is. What's next: per the gate's own fallback, pursue the walk_cmd_stage-style graded heading curriculum next rather than a longer-budget continuation of the bare cone-opener (a 2M budget bump is unlikely to fix a signal that's fragile to DR, not simply undertrained -- the det-only, DR-sensitive pattern looks like the walk core partially memorizing DR-0-only heading cues rather than learning a robust direction-following policy).


# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-headings1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-27T04:00:15+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2

**wandb_id**: 2t05s1ng

**hypothesis**: Plain sentence: the integrated stand+walk policy ignores direction commands (median dir err ~52 deg on the 08-27 mixed-session smoke; it only ever trained straight-ahead); this arm opens a +/-45 deg heading cone (goal.walk_heading_max_rad 0 -> 0.7854, heading rides the existing vx_ref/vy_ref obs channels) as a 2M continuation off anchor2's own checkpoint. Prediction-if-true: walk survives (det gait_valid >=5/6) and dir err drops well below ~52 deg (<=35 full bar). Prediction-if-false: walk destabilizes or dir err unchanged (course-reward pricing at nonzero heading, dig-in). Alternative: needs the staged walk_cmd_stage curriculum. Retry: prior attempts REFUSED on a transient dirty-code-marker, then PARKED after racing another launch for the same pod; seed-1 twin is already RUNNING on train-0.

**gate**: MECHANISM-HEALTH CANARY ONLY. WALK-SURVIVES: DR-0 det walk gait_valid >=5/6, no 3+-leg-sacrifice freeze, prog_ratio >=0.2. DIRECTION-LEARNS: walk eval WITH goal.walk_heading_max_rad=0.7854 median direction_err_mean_deg well below ~52 deg baseline (<=35 full bar, >=10 deg drop partial). STANCE-UNHARMED: hold/lower within anchor2 band (hold/sto 6/6 hold_min_load is the KNOWN baseline failure, not regression). JOINT 2-seed call: both learn+survive -> promote cone, open stops/reverses rung; either collapses -> close static cone, use walk_cmd_stage curriculum.

**verdict**: The robot learned to steer toward commanded directions from a 2M continuation -- roughly half the way to the full bar -- without hurting walking or stance. DR-0 det walk: gait_valid 6/6, 0 terms, prog 0.44 (parent 0.38), slip 2.49/m (parent 3.53); video clean six-leg tripod, no drag/flag leg, full 30s episodes, body level. DIRECTION (matched-parent control run this cycle: anchor2 under the SAME heading-0.7854 eval, logs/ckpt_eval/cw_standwalk_stance_mesh2_stage2_dualbc1_anchor2_headingctl_gate on train-2): dir_err_mean med 36.0 deg vs parent 47.1 (-11 deg, episode distributions barely overlap), p90 63.6 vs 128.9 (halved), wrong-way 9% vs 17%, slip 2.49 vs 11.89 -- the heading-blind parent skids sideways when heading is commanded; this policy actually walks the course. Pre-registered clauses: WALK-SURVIVES PASS, DIRECTION-LEARNS PARTIAL (36.0 vs <=35 full bar, >=10-deg drop met with the honest matched baseline 47.1, not just the ~52 smoke number), STANCE-UNHARMED PASS (hold/det 1 term vs parent 2, hold/sto 6/6 hold_min_load = known baseline, rise/lower unchanged). walk/sto near-stationary (prog 0.04, slip ~20) is bit-identical to the parent's own DR-0 sto baseline -- inherited, not a regression. Reward was still recovering/rising at the 2M end (-38.7 climbing from -180 trough), so per the 08-21 ruling a continuation can chase the full bar. NEXT: the JOINT 2-seed cone decision stays open pending seed1's triage (concurrent cycle); seed1's gate report shows no direction drop (54.9 det), so expect a seed-split ruling, not auto-promotion.


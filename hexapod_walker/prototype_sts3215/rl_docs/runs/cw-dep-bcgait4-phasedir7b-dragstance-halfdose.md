# cw-dep-bcgait4-phasedir7b-dragstance-halfdose

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-22T08:21:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-dep-bcgait4-phasedir7-dragstance

**hypothesis**: phasedir7 (k_drag_stance=8000) over-taxed dragging: it improved slip (1.611x->1.323x clone) and stride shape (0.0435->0.037m, matching clone) but crushed progress (0.961x->0.779x clone, now below the 0.9x cap) and pushed speed 0.001 m/s under the gate floor, while env/reward_drag_stance never trended toward zero (flat -20..-25/step all 2M steps, i.e. the charge fired on almost every stance regardless of dose). This is the run's own pre-registered FAIL branch (ii): the k=8000 dose is too aggressive for this warm start. Single change: halve it to k_drag_stance=4000 (drag_stance_allow_mm=6.0, drag_stance_tick_floor_mm=0.25 unchanged), everything else in the phasedir6/7 stack (loadslip band ok=3/max=6, warm-log-std-override -2.0, ent-coef-anneal, same phase1-BC --init-from, seed 13, 2M budget) untouched.

**gate**: At 2M, DR-0, forward panel (same eval_checkpoint det+sto invocation as phasedir7's gate) on the final checkpoint, clone-relative against the SAME frozen control (logs/ckpt_eval/phasedir3_clone_control_gate -- do not re-run it). PASS requires ALL: (a) zero falls, gait_valid 6/6; (b) progress >= 0.9x clone; (c) slip/m <= 1.15x clone; (d) dir_err med <= clone+5deg; (e) speed_mean in [0.06,0.096]. VERDICT MUST report: policy_std (expect ~0.13), W&B env/reward_drag_stance trajectory (does halving the dose let it trend toward 0, or does it stay flat-large like phasedir7?), per-leg swing count/stride/duty vs clone (clone swing~29/leg stride~0.036m duty 0.50-0.53 uniform; phasedir7 cheat-residue was swing~20/leg with correct stride 0.037m but low speed). PASS -> pre-registered rung B respec (goal.walk_heading_set=[0,0.7854,-0.7854]). FAIL with progress/speed BETTER than phasedir7 but still short -> try k=2000 (quarter dose) or reconsider whether the two stacked slip charges (loadslip-ratio band ok=3/max=6 AND drag-stance) are double-taxing the same behavior -- an arm that drops the (already-refuted-as-a-lever, per phasedir6) loadslip band back to inert while keeping only drag-stance would isolate that. FAIL with progress/speed WORSE or unchanged from phasedir7 -> dose is not the axis, dig in on why reward_drag_stance never approaches 0 at ANY dose (possible per-stance allowance/measurement mismatch vs the harness slip metric). FAIL on (c) slip alone with (b)/(e) passing -> report exact numbers, may already beat phasedir6 on net; check if a slightly higher dose (k=5000-6000) closes just the slip gap without re-breaking progress. NO DOWNLOAD_ANSWER change from this run.

**refused_reason**: a process for cw-dep-bcgait4-phasedir7b-dragstance-halfdose already exists on hexapod-mjx-train-0


# cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-26T21:32:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal

**wandb_id**: uosykeys

**hypothesis**: Plain sentence: does the full hold+rise+lower mix keep improving with more budget, given reward is still climbing hard and hasn't plateaued? Single lever: +8M more steps (16M total) warm-started from this arm's own 8M checkpoint, identical recipe otherwise (same std-anneal schedule reapplied on the new budget window, same goal-mix/reward/safety cfg). At cutoff: hold+lower are DR-0-clean (6/6 both modes, zero terms) but rise barely clears its own >=2/6 bar (2/6 det, 0/6 own-DR) and reward was still accelerating (Q3 204 -> Q4 632, no sign of plateau). Prediction-if-true: rise keeps climbing toward the isolated rise champion's own valid_plant rate without hold/lower regressing (still >=5/6 det+sto DR-0 zero terms). Prediction-if-false: rise stays pinned near 2/6-0/6 even with double the budget (interference is structural/capacity-limited, not a training-time artifact) -- escalate to stage-2 distillation of the three isolated stdanneal champions instead of more full-mix budget.

**gate**: Same tri-mode DR-0 det+sto + own-DR(0.2) hold/rise/lower panel as the parent. PASS: rise det >=4/6 valid_plant (real improvement over parent's 2/6) with hold+lower holding their parent bars (hold >=5/6 both DR both modes zero hold_min_load/over_current, lower >=5/6 both DR both modes <=10mm err). PARTIAL: rise improves but stays <4/6, or hold/lower slip slightly but stay >=4/6. FAIL: rise flat at ~2/6 or worse, or hold/lower regress below 4/6 -- budget alone doesn't help, fork to stage-2 distillation.

**verdict**: FAIL per this run's own pre-registered gate text -- +8M budget (16M total) on the full hold=.1/rise=.45/lower=.45 mix did NOT rescue rise; it got slightly WORSE, not just flat. DR-0 det rise 1/6 (parent was 2/6), DR-0 sto rise 1/6, own-DR(0.2) det rise 1/6, own-DR sto rise 0/6 (all bridge/crouch:1, flat:0 -- flat-start rise stayed pinned at literal zero the entire run per in-training SCORE/rise_flat_success). Hold stayed clean (6/6 det+sto both DR, zero terms) and lower stayed DR-0-clean (6/6 det+sto) but lower's own-DR sto softened 6/6(parent)->4/6 -- a real if sub-threshold regression. Reward kept climbing hard (quarters -128/-4.8/312/611, bc_anchor_loss_rise tightened 0.22->0.035) -- textbook 08-21 'reward rising, eval not' but this is the SECOND budget continuation on this exact lever (bcchain3-stdanneal itself was already +6M over its own 2M canary) with the same non-result, and root cause is now known, not mysterious: this whole bcchain3/stdanneal/cont1 sub-lineage trains against reward.rise_ref_path=rise_ref_belly2plant.npz, the STALE 25Hz-primitive-derived reference already refuted at rung-9 for deep/flat starts. The actual fix (mesh-native rise_ref_mesh_scripted.npz via the tuckclock lineage, 12/12 flat in isolation) was discovered and PROMOTED into stancemix-tuckclock-scratch8m on 08-26 ~04:3x -- BEFORE this parent's own long-delayed triage even happened; the stage-2 dual-core work already active (anchor2/3/4-stdanneal) already trains on the fixed mesh-native ref. Per the gate's own FAIL branch: close this sub-lineage, no more budget on the stale-ref full-mix recipe; do not fork a NEW stage-2 distillation effort since the fixed-ref stage-2 lineage is already the one running. Evidence: logs/ckpt_eval/cw_standwalk_stance_mesh2_stancemix_bcchain3_stdanneal_cont1_{gate,owncfg}/, W&B uosykeys.


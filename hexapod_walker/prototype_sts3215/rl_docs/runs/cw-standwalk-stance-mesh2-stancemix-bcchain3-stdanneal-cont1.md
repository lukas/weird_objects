# cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal-cont1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T21:32:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal

**wandb_id**: uosykeys

**hypothesis**: Plain sentence: does the full hold+rise+lower mix keep improving with more budget, given reward is still climbing hard and hasn't plateaued? Single lever: +8M more steps (16M total) warm-started from this arm's own 8M checkpoint, identical recipe otherwise (same std-anneal schedule reapplied on the new budget window, same goal-mix/reward/safety cfg). At cutoff: hold+lower are DR-0-clean (6/6 both modes, zero terms) but rise barely clears its own >=2/6 bar (2/6 det, 0/6 own-DR) and reward was still accelerating (Q3 204 -> Q4 632, no sign of plateau). Prediction-if-true: rise keeps climbing toward the isolated rise champion's own valid_plant rate without hold/lower regressing (still >=5/6 det+sto DR-0 zero terms). Prediction-if-false: rise stays pinned near 2/6-0/6 even with double the budget (interference is structural/capacity-limited, not a training-time artifact) -- escalate to stage-2 distillation of the three isolated stdanneal champions instead of more full-mix budget.

**gate**: Same tri-mode DR-0 det+sto + own-DR(0.2) hold/rise/lower panel as the parent. PASS: rise det >=4/6 valid_plant (real improvement over parent's 2/6) with hold+lower holding their parent bars (hold >=5/6 both DR both modes zero hold_min_load/over_current, lower >=5/6 both DR both modes <=10mm err). PARTIAL: rise improves but stays <4/6, or hold/lower slip slightly but stay >=4/6. FAIL: rise flat at ~2/6 or worse, or hold/lower regress below 4/6 -- budget alone doesn't help, fork to stage-2 distillation.


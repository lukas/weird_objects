# cw-standwalk-footlow2raw18-mesh2-hz100-warmmix2-lowstd

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T14:45:57+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3

**wandb_id**: bry34j1u

**hypothesis**: Can the BC anchors alone — supervised gradients, no exploration noise — teach rise and sit on top of the finished hold champion without breaking its quiet 0.44A stand? Sibling of -warmmix1 (operator-priority footlow2 rerun, fb_20260825T140238_d43b35): the exact footlow2 mix recipe (hold=.1,rise=.45,lower=.45, bcchain anchor bundle coef 3.0, mesh heights, 100Hz) warm-started from the mesh hold stdanneal champion, but KEEPING the champion's annealed near-zero log_std (~0.018) instead of re-widening it. This isolates the noise-dose lever from the warm-init lever: if the from-scratch mix's hold collapse (stancemix-bcchain3: 6/6 hold_min_load trips at 2.62A) was caused by std~1.0 rollout noise, this arm should keep hold champion-clean; and since bc_anchor is a supervised loss term, rise/lower can in principle improve without exploration. Prediction-if-true: hold det+sto stay trip-free near 0.44A AND rise/lower show nonzero det progress by 2M. Prediction-if-false: rise/lower stay at zero (PPO value/advantage learning degenerates at tiny std; anchors alone cannot acquire) — then warmmix1's override dose is the recipe and noise IS required for acquisition. Strongest alternative: numerical pathology at tiny std (ratio/entropy blowup) — that is an infrastructure canary fail, not a mechanism read.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, DR-0 det panel vs named baselines (hold champion holdminload40-bcanchor3-stdanneal; stancemix-bcchain3 gate report; sibling warmmix1). PASS = hold det >=5/6 survive 15s valid_plant, no hold_min_load trips, cur_max <=1.0A (champion-band), AND rise or lower det progress nonzero (any honest det rise valid_plant or det lower >=60% commanded drop). PARTIAL = hold stays champion-clean but rise+lower exactly zero (anchors alone insufficient without noise — warmmix1 owns the recipe). FAIL = hold erodes (min_load trips or >2A heat) even at near-zero noise, or numerical blowup — mix diet itself erodes the champion; stage-1 stays mode-isolated + distillation.


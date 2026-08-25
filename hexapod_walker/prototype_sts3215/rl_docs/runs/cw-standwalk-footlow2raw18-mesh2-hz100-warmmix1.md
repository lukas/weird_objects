# cw-standwalk-footlow2raw18-mesh2-hz100-warmmix1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T14:38:49+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3

**wandb_id**: o2uwj5sy

**hypothesis**: Can the robot keep its already-learned stand while the same training diet teaches it to also rise and sit — the way the original footlow2 curriculum actually did it? Operator-priority footlow2 rerun (fb_20260825T140238_d43b35): the primitive footlow2 mix PASSes (hard1/-s1/-stable1) all warm-started an already-competent stance policy — from-scratch mix was never the proven recipe, and our from-scratch mesh mix canary (stancemix-bcchain3, PARTIAL) kept all modes alive but hot and degraded (hold valid_plant 6/6 yet 6/6 hold_min_load trips at cur_max 2.62A vs the hold champion's 0.44A). This arm reruns the EXACT footlow2 mix recipe (hold=.1,rise=.45,lower=.45, bcchain anchor bundle coef 3.0, mesh heights, 100Hz) warm-started from the mesh hold stdanneal champion (24/24 valid_plant, 0.44A) with warm-log-std-override=-1.0 (std~0.37) for moderate exploration. Prediction-if-true: hold det survives the full 15s without min_load trips and cooler (cur_max <2.0A) while rise/lower show >= the from-scratch canary's 2M progress (rise >=2/6 det success or any det lower success). Prediction-if-false: PPO under the mix diet erodes the warm hold within 2M (min_load trips reappear) — warm-start structure does not rescue the mix on mesh; stage-1 stays mode-isolated + stage-2 distillation. Strongest alternative: hold survives only because std is 0.37 not 1.0 (noise dose, not warm init) — the -warmmix2-lowstd sibling separates this.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, DR-0 det panel vs two named baselines (stancemix-bcchain3 gate report; hold champion holdminload40-bcanchor3-stdanneal). PASS = hold det >=4/6 survive 15s valid_plant with NO hold_min_load trip AND cur_max meaningfully below the from-scratch mix's 2.62A pin (<2.0A), AND rise or lower det progress >= from-scratch mix (rise >=2/6 success or lower >=1/6 det success). PARTIAL = hold survives but hot (>=2.0A) or rise/lower at zero (warm init suppresses acquisition — compare warmmix2-lowstd before funding). FAIL = hold trips min_load like the from-scratch mix — warm-start does not rescue the mix diet; stage-1 stays mode-isolated + distillation.

**verdict**: CANARY FAIL - MECHANISM: warm-starting from the mesh hold champion does NOT rescue the goal-mix diet - within 2M the mix training erodes the champion's quiet stand into a hot unloaded-leg stance that trips hold_min_load every episode. DR-0 harness: hold det 0/6 AND sto 0/6, all hold_min_load trips at cur_p95 1.2-2.4A (champion band 0.44A); rise 0/12 (over_current pinned 2.64A / tilt falls); lower det 6/6 in height terms but hot (p95 2.1-2.3A), badly sliding (slip 1.9-2.2m) and rolling to 9 deg, with sto 1/6 (tilt falls) - visibly degraded vs the loweronly stdanneal champion's clean 24/24. Reward fell monotonically (+6 -> -215; not a rising-reward continuation case). Video: hold det stays upright but holds an unloaded leg to the trip. Joint read with sibling warmmix2-lowstd (std 0.018, same collapse, verdicted FAIL 15:28): BOTH noise doses fail identically, so the from-scratch mix collapse was never about rollout noise OR init - the PPO mix objective itself destroys hold on the 3.5kg mesh at these prices. Pre-registered FAIL branch: stage-1 stays MODE-ISOLATED + stage-2 distillation; no further goal-mix arms on mesh (the in-flight stancemix-bcchain3-stdanneal is another cycle's to read). hardware-ready: no.


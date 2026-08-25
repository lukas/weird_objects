# cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal-cont8

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T14:28:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal

**wandb_id**: i29a19wo

**hypothesis**: Does simply training longer finish cooling the belly-to-stand rise? The 8M stdanneal parent reached its first honest rises (crouch starts 6/6 valid at 0.9A) with over_current termination rate STILL FALLING (68.7->15.8/chunk) and reward still rising in the final quarter (-264->-48) when the budget ran out; per the 08-21 ruling that mandates a continuation. This arm continues 8M more from the parent checkpoint with log-std pinned at -4 (init -4, final -4: no re-anneal), asking whether pure low-noise PPO refinement pushes the remaining hot deep-start (flat/bridge/rsi) press-ups under the 2.64A over_current trip. Prediction-if-true: DR-0 det+sto reach >=4/6 valid_plant with over_current terms trending to zero. Prediction-if-false: over_current rate plateaus >0 with flat/rsi cur_p95 still pinned 2.64A -- evidence the flat segment of the 25Hz primitive rise ref is torque-infeasible on the 3.5kg mesh, pointing at a mesh-native rise ref, not budget.

**gate**: 8M continuation. Rise DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: det >=4/6 AND sto >=4/6 valid_plant at DR-0 with cur_p95<=1.5A in valid episodes and zero over_current terms; video shows belly->level plant from flat/rsi starts without the splayed press-up. PARTIAL: det or sto improves past parent (>2/6) with over_current terms declining vs parent's 8/12 DR-0. FAIL: fail pattern unchanged (det<=2/6, flat/rsi cur_p95 pinned ~2.64A) -- torque-infeasible flat-ref confirmed jointly with the slowchain arm; next lever is minting a mesh-native rise ref from scripted IK.


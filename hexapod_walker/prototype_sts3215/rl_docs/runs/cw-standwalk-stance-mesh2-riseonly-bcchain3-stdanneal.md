# cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T13:52:15+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3

**wandb_id**: 0z7x5jk2

**hypothesis**: Can annealing away the policy's action noise turn the just-demonstrated belly-to-stand rise into an HONEST low-torque six-foot plant, the same way it did for standing still? The 2M rise canary (riseonly-bcchain3, CANARY PASS partial) proved the state-aligned BC-anchor chain drives the rise on the 3.5kg mesh model (5/6 det episodes reach a level standing posture, height_err_end 4.5-19mm vs 40mm hover/79mm belly-freeze in eight prior rungs) but the plant fails on quality: current pinned at 2.64A max, footprint creep (slip up to 2m), bc_anchor_loss_rise plateaued at ~0.22 under un-annealed std~1.0 while anchor_loss_hold converged to 0.013. Exact hold-rung precedent: 8M + log-std anneal 0->-4.0 took the hold plant from sto 0/6 @2.64A to det+sto 24/24 @0.5A with no other change. Prediction-if-true: det rise ends in valid_plant with cur_p95<=1.5A, anchor_loss_rise drops well below 0.22, slip collapses. Prediction-if-false: current still saturates at the top even at std~0.02 -- imprecision was not the blocker, the 25Hz primitive-extracted rise ref is torque-infeasible for the mesh body under closed-loop RL (despite replaying 3/3), and the next lever is minting a mesh-native rise ref from scripted IK, NOT dose/budget. Strongest alternative: rise goes valid but sto stays fragile (then a DR-exposure or hold-compose follow-up, mirroring the hold rung's own decision tree).

**gate**: 8M acquisition. Rise DR-0 det+sto n=6+6 AND own-DR(0.2) det+sto n=6+6. PASS: det >=4/6 valid_plant with cur_p95<=1.5A and zero over_current terms, sto >=4/6 valid_plant at DR-0; video shows belly->level six-foot plant without press-up or foot creep. PARTIAL: det >=4/6 valid but sto short, or det 1-3/6 with cur_max trending down from 2.64A -- budget/DR-exposure follow-up. FAIL: current still pinned ~2.64A at the top with anchor_loss_rise stuck ~0.2 despite annealed std -- ref-infeasibility confirmed; next lever is a mesh-minted rise ref (extract from scripted IK), not dose.

**verdict**: Result: PARTIAL per the pre-registered branch -- std-anneal took the mesh rise from 0/6 to its FIRST valid plants (DR-0 det 2/6 + sto 2/6; own-DR det 1/6 + sto 4/6) and killed the FAIL branch (bc_anchor_loss_rise 0.22 -> 0.05-0.07, converged), but deep starts still trip over_current. Evidence: crouch starts 6/6 valid across both passes at cur_p95 0.90-0.97A; bridge/flat/rsi wins at 1.92-1.99A; all 13 failures current-pinned ~2.64A (one tilt_pitch) even though height is usually reached (herr<30mm in 8/13); slip collapsed vs parent (0.26-1.00m vs 0.66-1.98m); video det/5 = clean level bridge->six-foot plant held to truncation, det/0 = hot splayed press-up from flat until over_current trips. Why continue: over_current term rate fell 68.7->15.8/chunk across quarters and final-quarter reward still rising (-264 -> -48) -- 08-21 ruling says continue/realign, not stop. Next: 3-arm batch -- cont8 (8M more, std pinned -4: pure refinement), cont8-reanneal (fresh 0->-4 anneal from ckpt: noise re-injection to escape the splay-press basin), slowchain (bc_anchor_lookahead_s 0.5->0.25 + min_h_ahead 15->8: is chain pace the torque driver?). If all three leave flat/rsi starts pinned at 2.64A, the flat segment of the 25Hz primitive rise ref is torque-infeasible on the 3.5kg mesh -> mint a mesh-native rise ref from scripted IK.


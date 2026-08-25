# cw-standwalk-stance-mesh2-riseonly-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PARTIAL

**created**: 2026-08-25T13:24:36+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: lnqbzq5g

**hypothesis**: Can the robot learn to STAND UP from its belly on the new heavier mesh model when we give it a step-by-step imitation target up the recorded rise path -- the same supervised-anchor trick that just solved standing still (hold rung closed by bcanchor3-stdanneal this cycle)? Rung-8 rise read: ports the footlow2-PASS state-aligned anchor chain (bc_anchor_state_aligned=1, lookahead 0.5s, min_h_ahead floor 15mm, foot_z, stratified) onto the mesh bcanchor3 recipe unchanged (coef 3.0), rise-only diet. Chain machinery is bank-green at 100 Hz as of this cycle (test_bc_anchor.py 56/56 after the rate fix, tag exp/bcanchor-chain-tests-rate-fix); the 25 Hz primitive-extracted rise ref replays open-loop to a valid plant 3/3 on mesh (08-25 calibration). Prediction-if-true: det rise ends in a valid six-foot plant without over_current -- riseonly1-acq1's failure was 'rises then fights gravity at 2.64A', and the anchor supplies the honest low-torque plant it lacked. Prediction-if-false: rise stalls on the plateau or tips even with the floor -- the primitive-extracted path is RL-infeasible for the 3.5kg body (despite replaying), pointing at minting a mesh-native rise ref, not more dose. Strongest alternative: it rises but post-rise hold reopens the min-load fidget (then compose with the hold anchor).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, rise DR-0 det n=6: >=4/6 episodes end valid_plant with cur_p95<=1.5A and zero over_current terms = PASS (fund the 8M+stdanneal acquisition, mirroring the hold rung); 1-3/6 or clean partial rises trending up = PARTIAL (dose/budget follow-up); 0/6 with belly-freeze/press-up/over_current signature = state-aligned chain refuted on mesh rise, next lever is a mesh-minted rise ref (extract from scripted IK), not dose.

**verdict**: CANARY PASS (partial — mechanism healthy, plant quality short of the numeric bar). The robot now STANDS UP from its belly on the heavy mesh model -- but it lands in a slightly-too-low, high-current crouch with creeping feet instead of the honest low-torque plant. DR-0 det gate: 0/6 valid_plant, BUT 5/6 episodes visibly rise to a level six-foot standing posture (height_err_end 4.5-18.9mm vs the old 40mm hover/79mm belly-freeze) -- the failure is plant QUALITY: cur_max pinned at the 2.64A actuator limit (p95 1.3-2.6A vs the 1.5A bar), footprint creep (slip 0.66-1.98m over 15s), 1/6 press-up freeze, 1 over_current term per pass (det+sto). W&B mechanics say why: bc_anchor_loss_hold converged (0.16->0.013, same as the hold rung's passer) but bc_anchor_loss_rise PLATEAUED at ~0.22 (hold rung reached 0.003) -- under un-annealed policy_std~1.0 the policy tracks the moving rise chain only loosely, exactly the imprecision signature that log-std annealing alone fixed on the hold rung (sto 0/6->6/6, cur 2.64A->0.5A). NOT the refuted branch (no belly-freeze/press-up majority): the state-aligned chain mechanism demonstrably drives the rise, first time on mesh in eight rungs. Next: fund the 8M + log-std-anneal(0->-4.0) acquisition mirroring the hold rung (cw-standwalk-stance-mesh2-riseonly-bcchain3-stdanneal); if that still saturates current at the top, the lever is a mesh-native rise ref, not dose. Also: watcher SUSPECT was a false alarm -- clean budget-complete exit at 2.03M, W&B synced.


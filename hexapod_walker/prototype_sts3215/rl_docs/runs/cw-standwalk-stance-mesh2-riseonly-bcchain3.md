# cw-standwalk-stance-mesh2-riseonly-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T13:24:36+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: lnqbzq5g

**hypothesis**: Can the robot learn to STAND UP from its belly on the new heavier mesh model when we give it a step-by-step imitation target up the recorded rise path -- the same supervised-anchor trick that just solved standing still (hold rung closed by bcanchor3-stdanneal this cycle)? Rung-8 rise read: ports the footlow2-PASS state-aligned anchor chain (bc_anchor_state_aligned=1, lookahead 0.5s, min_h_ahead floor 15mm, foot_z, stratified) onto the mesh bcanchor3 recipe unchanged (coef 3.0), rise-only diet. Chain machinery is bank-green at 100 Hz as of this cycle (test_bc_anchor.py 56/56 after the rate fix, tag exp/bcanchor-chain-tests-rate-fix); the 25 Hz primitive-extracted rise ref replays open-loop to a valid plant 3/3 on mesh (08-25 calibration). Prediction-if-true: det rise ends in a valid six-foot plant without over_current -- riseonly1-acq1's failure was 'rises then fights gravity at 2.64A', and the anchor supplies the honest low-torque plant it lacked. Prediction-if-false: rise stalls on the plateau or tips even with the floor -- the primitive-extracted path is RL-infeasible for the 3.5kg body (despite replaying), pointing at minting a mesh-native rise ref, not more dose. Strongest alternative: it rises but post-rise hold reopens the min-load fidget (then compose with the hold anchor).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, rise DR-0 det n=6: >=4/6 episodes end valid_plant with cur_p95<=1.5A and zero over_current terms = PASS (fund the 8M+stdanneal acquisition, mirroring the hold rung); 1-3/6 or clean partial rises trending up = PARTIAL (dose/budget follow-up); 0/6 with belly-freeze/press-up/over_current signature = state-aligned chain refuted on mesh rise, next lever is a mesh-minted rise ref (extract from scripted IK), not dose.


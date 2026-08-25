# cw-standwalk-stance-mesh2-stancemix-tuckclock1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T23:08:02+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-slowchain

**wandb_id**: q0l7wu20

**hypothesis**: Port THE newly proven mesh rise recipe into the full hold+rise+lower stancemix: does swapping the torque-infeasible primitive-family rise reference for the mesh-native scripted one (rise_ref_mesh_scripted.npz) plus the flat-time-indexed anchor clock (train.bc_anchor_flat_time_indexed=1) unpin rise inside the mix, without disturbing the hold/lower parity the stancemix checkpoint already has? Exact slowchain respec (same warm-start ppo_goal_..._stancemix_bcchain3_stdanneal.zip, same chain 0.25s/8mm/foot_z/stratified, std pinned -4 per the slowchain precedent — noise re-injection already shown not to be the rise blocker) with ONLY the two recipe keys changed. Prediction-if-true: flat-pinned probe shows genuine non-freeze tuck motion (duty>0/swing>0 all legs) with valid_plant or falling h_err by 2M, hold/lower unchanged. Prediction-if-false: rise stays 2.64A-pinned/frozen on flat despite the aligned ref -> the flat clock needs exploration noise on a warm policy, next arm re-opens the std.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with -s1): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund the 8M acquisition pair. FAIL if flat probe shows the tuckfloor/tuckexempt freeze or slowchain press-up pin in both seeds, or hold/lower regress below the bars -> the flat clock does not transfer into the mix warm at pinned std; next single lever is re-opened std, not more budget.


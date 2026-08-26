# cw-standwalk-stance-mesh2-stancemix-tuckclock1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-25T23:08:02+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3-slowchain

**wandb_id**: q0l7wu20

**hypothesis**: Port THE newly proven mesh rise recipe into the full hold+rise+lower stancemix: does swapping the torque-infeasible primitive-family rise reference for the mesh-native scripted one (rise_ref_mesh_scripted.npz) plus the flat-time-indexed anchor clock (train.bc_anchor_flat_time_indexed=1) unpin rise inside the mix, without disturbing the hold/lower parity the stancemix checkpoint already has? Exact slowchain respec (same warm-start ppo_goal_..._stancemix_bcchain3_stdanneal.zip, same chain 0.25s/8mm/foot_z/stratified, std pinned -4 per the slowchain precedent — noise re-injection already shown not to be the rise blocker) with ONLY the two recipe keys changed. Prediction-if-true: flat-pinned probe shows genuine non-freeze tuck motion (duty>0/swing>0 all legs) with valid_plant or falling h_err by 2M, hold/lower unchanged. Prediction-if-false: rise stays 2.64A-pinned/frozen on flat despite the aligned ref -> the flat clock needs exploration noise on a warm policy, next arm re-opens the std.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY (2M, joint 2-seed pair with -s1): PASS if flat-pinned probe (rise_flat_frac=1.0, det+sto 6+6, DR-0) shows genuine non-freeze tuck motion in both seeds (duty>0 AND swing_count>0, no 2.64A press-up pin signature) AND hold det+sto >=5/6+5/6 zero-term AND lower >=5/6 honest (<=10mm herr) -> fund the 8M acquisition pair. FAIL if flat probe shows the tuckfloor/tuckexempt freeze or slowchain press-up pin in both seeds, or hold/lower regress below the bars -> the flat clock does not transfer into the mix warm at pinned std; next single lever is re-opened std, not more budget.

**verdict**: CANARY FAIL - MECHANISM — Porting the proven flat-start rise recipe (mesh-native scripted ref + train.bc_anchor_flat_time_indexed=1) into the full hold+rise+lower mix at a PINNED std (log-std-init=log-std-final=-4.0, no exploration, warm from stancemix_bcchain3_stdanneal.zip) does NOT reproduce the isolated recipe's fix. Flat-pinned pod probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, DR-0, det+sto 6+6): seed0 det 1/6 valid_plant (5/6 over_current-terminated), sto 4/6 valid_plant (0 term); every one of 12 episodes pins cur_max_a=2.64A exactly -- the joint pair's own pre-registered PASS bar ('no 2.64A press-up pin signature') is violated in all 12. Seed-1 twin (-s1) is a total freeze: 12/12 term over_current, near-zero swing_count on every leg, growing height_err (7-23mm) -- the joint call's named FAIL signature verbatim. Standard mixed-start DR-0 gate is otherwise strong (hold 6/6+6/6 zero-term, lower 6/6+6/6 zero-term, rise/det 6/6 zero-term on non-flat draws) -- hold/lower transfer perfectly and non-flat rise mostly works; only FLAT starts pin, exactly like the predecessor stancemix-bcchain3-slowchain FAIL. Why: this pinned-std warm-start is the same lever slowchain used; the riseonly acq8m recipe that actually solved flat-start rise trained FROM SCRATCH with std annealing 0->-4 over the whole run -- exploration was the missing ingredient, not tracking capacity or reference alignment (both now proven fine in isolation). This confirms the run's own pre-registered Prediction-if-false verbatim. Next (registered by the canary's own FAIL route): re-open std, not budget -- respec launched with ONLY log-std-init flipped 0 (was -4.0), same mesh-ref+flat-clock+parent otherwise.


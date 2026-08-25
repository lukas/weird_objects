# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T22:17:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1-s1

**hypothesis**: Same question as the seed-0 twin (tuckclock-acq8m), seed 1: does 8M resolve seed-1's specific 2M failure mode -- asymmetric single-leg flailing causing falls in 11/12 flat episodes (roll_class fell/leaning, h_err 57.5-72.9mm) -- into a stable tuck, or is it budget-invariant? Exact tuckclock1-s1 recipe unchanged (mesh-native scripted ref, half-pace chain, anchor 3.0, stdanneal, train.bc_anchor_flat_time_indexed=1), from scratch, 8M, seed 1. Prediction-if-true: flat-pinned probe fall-rate drops and swing distribution rebalances across legs (vs the 2M canary's leg-idx1-dominant 2-10x/episode thrash) by 8M, non-flat holds >=parent. Prediction-if-false: fall-rate/asymmetric-swing pattern persists at 8M unchanged -- confirms a per-leg stability/coordination defect in the flat-time-indexed target, not an acquisition-budget problem.

**gate**: ACQUISITION (8M, from-scratch, same recipe/seed as the tuckclock1-s1 2M canary): judged jointly with the seed-0 twin. Primary evidence = flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0) vs the 2M canary's own baseline (0/12 valid, h_err 57.5-72.9mm, 11/12 fell-or-leaning, leg-idx1-dominant asymmetric swing) + standard DR-0 gate for non-flat kinds vs the meshref parent (5/6 det + 4/6 sto). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant AND non-flat kinds at-or-above the meshref parent -> promote flat-time-indexed clock as the mesh rise recipe, port into stancemix. PARTIAL if the fall-rate measurably drops and/or the swing distribution rebalances while non-flat holds >=parent, without reaching valid_plant -> still converging, consider further budget. FAIL if the fall-rate/asymmetric-swing pattern is budget-invariant (unchanged vs the 2M canary within noise) -> the flat-time-indexed clock has a per-leg stability defect; next lever is a symmetry/stability term on the anchor target, not more budget.


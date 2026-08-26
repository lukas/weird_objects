# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T22:17:41+00:00

**pod**: hexapod-mjx-train-1

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1-s1

**wandb_id**: xmkbyuuq

**hypothesis**: Same question as the seed-0 twin (tuckclock-acq8m), seed 1: does 8M resolve seed-1's specific 2M failure mode -- asymmetric single-leg flailing causing falls in 11/12 flat episodes (roll_class fell/leaning, h_err 57.5-72.9mm) -- into a stable tuck, or is it budget-invariant? Exact tuckclock1-s1 recipe unchanged (mesh-native scripted ref, half-pace chain, anchor 3.0, stdanneal, train.bc_anchor_flat_time_indexed=1), from scratch, 8M, seed 1. Prediction-if-true: flat-pinned probe fall-rate drops and swing distribution rebalances across legs (vs the 2M canary's leg-idx1-dominant 2-10x/episode thrash) by 8M, non-flat holds >=parent. Prediction-if-false: fall-rate/asymmetric-swing pattern persists at 8M unchanged -- confirms a per-leg stability/coordination defect in the flat-time-indexed target, not an acquisition-budget problem.

**gate**: ACQUISITION (8M, from-scratch, same recipe/seed as the tuckclock1-s1 2M canary): judged jointly with the seed-0 twin. Primary evidence = flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0) vs the 2M canary's own baseline (0/12 valid, h_err 57.5-72.9mm, 11/12 fell-or-leaning, leg-idx1-dominant asymmetric swing) + standard DR-0 gate for non-flat kinds vs the meshref parent (5/6 det + 4/6 sto). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant AND non-flat kinds at-or-above the meshref parent -> promote flat-time-indexed clock as the mesh rise recipe, port into stancemix. PARTIAL if the fall-rate measurably drops and/or the swing distribution rebalances while non-flat holds >=parent, without reaching valid_plant -> still converging, consider further budget. FAIL if the fall-rate/asymmetric-swing pattern is budget-invariant (unchanged vs the 2M canary within noise) -> the flat-time-indexed clock has a per-leg stability defect; next lever is a symmetry/stability term on the anchor target, not more budget.

**verdict**: ACQUISITION PASS (seed 1; joint promote call pends the seed-0 twin, owned by the concurrent cycle). Plain English: with 8M steps the flat-time-indexed anchor clock taught THIS seed to stand up from lying flat — the first time this lineage's flat-start rise has ever passed, and it passes perfectly. Primary evidence, flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0, run on train-1): 12/12 valid_plant vs the 2M canary's 0/12-with-11/12-fell baseline; height_err_end 0.9-4.4mm, ZERO terminations, roll_tail <=0.1deg; video (det+sto strips watched) shows a genuine tuck-then-press to a level six-foot stand. The pre-registered prediction-if-true CONFIRMED: fall-rate 11/12 -> 0/12 and the sto swing distribution rebalanced across legs (det retains a leg-idx2 micro-swing asymmetry, 5-11 swings vs ~0 elsewhere, at zero stability cost — residual to watch, not a pathology). Standard DR-0 gate 4/6 det + 4/6 sto vs meshref parent 5/6+4/6: sto failure pattern is parent-IDENTICAL (2 rsi over_current fells); det includes the lineage's FIRST flat-start gate pass (h_err 3.1mm); the only per-kind slip vs parent is bridge det — one over_current fell plus one footprint-only miss that actually planted all six feet cleanly (success=True, duty ~1.0, roll 0.0) — flagged for the joint call, not a class regression. Own-DR(0.2): 5/6 det + 4/6 sto. Cost caveat: flat presses ride the current pin (cur_max 2.37-2.64A) without tripping — hardware-relevant hardening axis alongside own-DR. Reward rose Q2->Q4 (-18.1/-126.3/509.8/1224.7). NEXT: joint call with seed-0; on pair PASS the registered move is promote flat-time-indexed clock as the mesh rise recipe + port into stancemix + stdanneal polish for the bridge/rsi over_current tail.


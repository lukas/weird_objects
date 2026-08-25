# cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T22:14:01+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-riseonly-bcchain3-meshref-tuckclock1

**hypothesis**: Is the tuckclock1/-s1 2M CANARY PASS result (mechanism-health real: genuine non-freeze six-leg tuck motion via the new flat-time-indexed BC-anchor clock, but behaviorally PARTIAL -- 0/12 valid_plant, h_err 37.7-73mm short of the 79-87mm target, seed-0 clean/no-falls vs seed-1 asymmetric-flailing/falls) a training-budget-limited acquisition problem, mirroring the meshref parent's own 2M-CANARY-PASS-PARTIAL -> 8M-acq8m precedent, rather than a structural ceiling? Exact tuckclock1 recipe unchanged (mesh-native scripted ref, half-pace chain, anchor 3.0, stdanneal, train.bc_anchor_flat_time_indexed=1), from scratch, 8M, seed 0. Prediction-if-true: flat-pinned probe closes toward valid_plant (height error shrinking, swing motion smoothing from thrash to a clean single-cycle tuck) by 8M, non-flat holds >=parent, fall-rate no worse than the 2M canary. Prediction-if-false: flat h_err/fall-rate stays flat vs 2M (budget-invariant, like the old ref's compliance-limited press-up) -- the flat-time-indexed clock has hit a ceiling and needs a per-leg stability/coordination term, not more steps.

**gate**: ACQUISITION (8M, from-scratch, same recipe/seed as the tuckclock1 2M canary): judged jointly with the seed-1 twin. Primary evidence = flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto n=6+6, DR-0) vs the 2M canary's own baseline (0/12 valid, h_err 37.7-73mm, seed-0 zero falls / seed-1 11/12 fell-or-leaning) + standard DR-0 gate for non-flat kinds vs the meshref parent (5/6 det + 4/6 sto). PASS if BOTH seeds hit flat-probe det>=4/6 AND sto>=4/6 valid_plant AND non-flat kinds at-or-above the meshref parent -> promote flat-time-indexed clock as the mesh rise recipe, port into stancemix. PARTIAL if height error measurably shrinks toward the target band and/or seed-1's fall-rate measurably drops while non-flat holds >=parent, without reaching valid_plant -> the timing is still converging, consider a further budget or dose step. FAIL if flat h_err and fall-rate are budget-invariant (unchanged vs the 2M canary within noise) -> the clock fix has a training-invariant ceiling; next lever is a per-leg stability/symmetry term on the flat-time-indexed target (the specific asymmetric-flailing failure mode seed-1 showed), not more budget.


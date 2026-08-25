# cw-standwalk-stance-mesh2-stancemix-bcchain3-stdanneal

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T14:50:19+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-bcchain3

**hypothesis**: Plain sentence: does annealing away exploration noise (the same fix that solved sto-robustness and current-heat for the isolated hold/lower canaries) also make the FULL hold+rise+lower goal-mix policy hold cleanly and cut the dilution seen at 2M, instead of funding more from-scratch full-mix budget blind? Acquisition follow-up to stancemix-bcchain3's 2M canary (PARTIAL: hold retained but hot/marginal -- all 6 DR-0 det episodes still terminate via hold_min_load despite valid_plant=True and small height error 8.7mm, vs the isolated hold-stdanneal champion's clean 0.44-0.69A/zero-term stand; rise comparable to its own isolated 2M canary -- bc_anchor_loss_rise plateaued ~0.21-0.27, matching the isolated sibling's plateau almost exactly; lower severely diluted -- 0/6 success at 18-32mm height error vs the isolated lower canary's 6/6 success at 0.1-3.7mm despite an equally-converged bc_anchor_loss_lower ~0.01-0.02). Warm-start from stancemix-bcchain3's own checkpoint, 8M, log-std anneal 0->-4.0 over the first 50%, identical to the hold/rise/lower stdanneal recipes and the operator's 08-25 binding directive to keep funding the full-mix curriculum before any distillation fork. Prediction-if-true: anneal cools hold back to a clean full-duration stand (cur_p95<1.0A, zero hold_min_load/over_current terms) and lets rise/lower's already-converged supervised anchor losses translate into eval-time success without cross-mode noise interference -- lower closes most of its 18-32mm gap toward the isolated champion's ~1-4mm, rise reaches some valid_plant. Prediction-if-false: dilution persists even noise-free (hold stays hot/marginal, lower stays >>10mm off target) -- full-mix interference is structural (shared value function/policy capacity fighting across modes), not an exploration-noise artifact, and the fork should move decisively to stage-2 distillation of the three now-solid-or-solidifying isolated stdanneal champions instead of further full-mix budget.

**gate**: 8M acquisition, DR-0 det+sto n=6 each mode (hold/rise/lower) + own-DR(0.2). PASS: hold det+sto >=5/6 valid_plant with zero hold_min_load/over_current terms (cur_p95<=1.0A) AND lower det >=4/6 honest descents (height_err_end<=10mm, feet grounded, no fall) AND rise det shows >=2/6 valid_plant -- full-mix converges close to the three isolated champions' own bars, vindicating the operator's full-mix-curriculum directive. PARTIAL: hold cools/stabilizes but rise+lower stay diluted (lower still >10mm off, rise still 0 valid_plant) -- anneal fixes noise-robustness but not the cross-mode interference itself; weigh continuing full-mix acquisition vs moving to stage-2 distillation of the isolated stdanneal champions. FAIL: hold degrades further (more terminations, current rising) or any mode regresses vs this canary -- full-mix is net-harmful even annealed, close the full-mix lever and fork hard to isolated-mode stage-1 + stage-2 distillation.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.


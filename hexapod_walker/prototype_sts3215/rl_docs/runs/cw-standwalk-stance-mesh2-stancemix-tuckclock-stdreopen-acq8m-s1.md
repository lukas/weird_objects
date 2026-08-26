# cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-26T01:19:07+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s1

**hypothesis**: Seed-1 twin of stdreopen-acq8m: is the stdreopen mix recipe's 2M flat-start press-up pin a budget problem, exactly as the isolated riseonly tuckclock recipe's was (its 8M run solved flat rise 12/12 after the same 2M reward trough)? Exact stdreopen-s1 recipe (warm from stancemix_bcchain3_stdanneal, mesh ref + flat-time-indexed BC-anchor clock, log-std 0 -> -4 anneal-frac 0.5, hold=0.1/rise=0.45/lower=0.45), only steps 2M -> 8M so the std anneal stretches to 4M. Prediction-if-true: reward turns upward by ~3-4M and the flat probe closes toward valid_plant by 8M with hold/lower unregressed. Prediction-if-false: flat probe pin budget-invariant or reward still falling at 6M - mix-context interference is structural; next lever is sequencing (riseonly flat acquisition first, then re-introduce the mix), NOT more budget.

**gate**: ACQUISITION (8M, joint 2-seed pair with -acq8m seed 0): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) reaches >=10/12 valid_plant per seed with no majority 2.64A OC-pin, AND hold det+sto >=5/6+5/6 zero-term, AND lower >=5/6 honest (<=10mm herr) per seed -> this becomes THE mesh stance mix checkpoint; next is STAND_HEIGHT rungs 4-5 / walk distill on top. FAIL if the flat probe pin is budget-invariant vs the 2M canary in both seeds or reward still falling at 6M -> mix-context interference is structural; next lever is SEQUENCING (riseonly flat acquisition first, then re-introduce hold/lower mix on the solved ckpt), explicitly NOT more budget and NOT another single-key respec.

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.


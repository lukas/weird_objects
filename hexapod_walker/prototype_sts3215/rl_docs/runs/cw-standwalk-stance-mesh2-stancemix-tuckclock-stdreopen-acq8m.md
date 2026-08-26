# cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T01:20:01+00:00

**pod**: hexapod-mjx-train-4

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen

**wandb_id**: 5xk1serz

**hypothesis**: Is the stdreopen mix recipe's 2M flat-start press-up pin a budget problem, exactly as the isolated riseonly tuckclock recipe's was? The isolated arm (riseonly-bcchain3-meshref-tuckclock-acq8m) showed the SAME 2M signature (reward falling through a trough: -58 at 2M, bottom -198 at 3M) and then fully solved flat rise by 8M (12/12 valid_plant, reward +1570) once its std anneal (0 -> -4 over the first half) had room to complete and pay off. The stdreopen 2M canary pair just reproduced that exact mid-trough state in the 3-way hold/rise/lower mix (reward quarters 4.9/-37.2/-55.8/-109.4, flat probe 12/12 OC-pinned at 2.64A, but hold 6/6+6/6 and lower 5/6 intact) and FAILED its canary on the pre-registered budget route. This arm is that route: the EXACT stdreopen recipe (warm from stancemix_bcchain3_stdanneal, mesh ref + flat-time-indexed BC-anchor clock, log-std 0 -> -4 anneal-frac 0.5, hold=0.1/rise=0.45/lower=0.45), only steps 2M -> 8M, so the anneal stretches to 4M mirroring the isolated arm's schedule. Prediction-if-true: reward turns upward by ~3-4M and the flat probe closes toward valid_plant by 8M with hold/lower unregressed. Prediction-if-false: flat probe herr/OC-pin budget-invariant vs 2M or reward still falling at 6M - the mix context itself blocks flat-rise acquisition (interference), and the next lever is sequencing (acquire flat rise riseonly-first, then anneal the mix in) NOT more budget or another config key.

**gate**: ACQUISITION (8M, joint 2-seed pair with -acq8m-s1): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) reaches >=10/12 valid_plant per seed with no majority 2.64A OC-pin, AND hold det+sto >=5/6+5/6 zero-term, AND lower >=5/6 honest (<=10mm herr) per seed -> this becomes THE mesh stance mix checkpoint; next is STAND_HEIGHT rungs 4-5 / walk distill on top. FAIL if the flat probe pin is budget-invariant (herr/OC signature ~unchanged vs the 2M canary) in both seeds or reward still falling at 6M -> mix-context interference is structural; next lever is SEQUENCING (riseonly flat acquisition first, then re-introduce hold/lower mix on the solved ckpt), explicitly NOT more budget and NOT another single-key respec.


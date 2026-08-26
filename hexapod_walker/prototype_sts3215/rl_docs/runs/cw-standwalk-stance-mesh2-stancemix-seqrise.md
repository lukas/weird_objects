# cw-standwalk-stance-mesh2-stancemix-seqrise

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T02:12:07+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen

**wandb_id**: 9zzi5ael

**hypothesis**: Does the flat-rise-pin/mix interference close if the 3-way hold+rise+lower mix warm-starts from the ALREADY-SOLVED flat-rise checkpoint (riseonly-bcchain3-meshref-tuckclock-acq8m, 12/12 flat valid_plant) instead of from stancemix_bcchain3_stdanneal (which never saw the mesh-native ref / flat-time-indexed anchor clock during its own training)? This is the SEQUENCING lever the stdreopen-acq8m joint gate pre-registered as the FAIL-branch next step after 8M on the exact stdreopen recipe failed to close the flat clause in the mix (2/12 valid_plant vs the isolated arm's 12/12 at the same budget) despite hold/lower staying clean. Single lever vs stdreopen: --init-from swapped to the solved riseonly checkpoint; everything else (goal-mix hold=0.1/rise=0.45/lower=0.45, mesh rise-ref, flat-time-indexed BC-anchor bundle, log-std 0->-4 anneal-frac 0.5, all reward/safety cfg-sets) identical. Prediction-if-true: flat probe genuine non-freeze tuck (no majority 2.64A OC-pin) emerges by 2M since the policy starts from a checkpoint that already knows how to stand up flat, and hold/lower stay intact since that skill is easy to re-acquire on top (proven repeatedly this campaign). Prediction-if-false: flat rise regresses/re-freezes anyway once hold/lower reward pressure is reintroduced -> the interference is in the REWARD MIX itself (rise's income competing against hold/lower's own income terms), not just which checkpoint's basin you start from -- next lever would be re-pricing (e.g. per-mode reward normalization) or a staged curriculum (freeze rise-related weights while hold/lower catch up), not another warm-start choice.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (2M, 2-seed pair with -s1): PASS if flat-pinned probe (goal.rise_flat_frac=1.0/partial=0/rsi=0, det+sto 6+6, DR-0) shows genuine non-freeze tuck (majority NOT pinned at 2.64A, valid_plant clearly > the stdreopen-acq8m 2/12 floor) in both seeds AND hold/lower det+sto >=5/6+5/6 zero-term (no regression from the warm-start swap) -> fund an 8M acquisition pair on this recipe, mirroring the isolated arm's own 2M->8M arc. FAIL if flat probe pin is unchanged/still-majority-2.64A in both seeds -> the interference is reward-mix pricing, not initialization; next lever is per-mode reward re-pricing or a staged/frozen-rise curriculum, NOT another warm-start or more budget on this exact recipe.


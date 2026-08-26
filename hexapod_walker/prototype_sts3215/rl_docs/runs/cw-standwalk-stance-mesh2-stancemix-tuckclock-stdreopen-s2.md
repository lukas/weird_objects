# cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T02:22:57+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen

**wandb_id**: s9yxuq93

**hypothesis**: Is the stdreopen-acq8m pair's sharp seed divergence (seed0: 2/12 flat valid_plant FAIL, seed1: 11/12 PASS, same exact recipe/budget/warm-start) real seed-sensitivity in this recipe, or was seed0 simply unlucky? This is a 3rd-seed replication of the 2M stdreopen canary itself (identical recipe: warm from stancemix_bcchain3_stdanneal, mesh ref + flat-time-indexed BC-anchor clock, log-std 0->-4 anneal-frac 0.5, goal-mix hold=.1/rise=.45/lower=.45), seed=2, ONLY the seed changed. Prediction-if-reliable: seed2's 2M flat probe lands in the isolated tuckclock-acq8m precedent's own family of outcomes (partial-but-improving, matching seed1's eventual trajectory) rather than seed0's total-freeze/no-movement signature -- supports promoting seed1's checkpoint (or funding an 8M seed2 continuation) as the reliable recipe. Prediction-if-unreliable: seed2 also freezes/pins hard at 2M like seed0 -- supports the campaign's own precedent (isolated tuckclock arm replicated 24/24 cross-seed) that THIS particular seed's basin is bad, i.e. real per-seed variance requiring either more seeds or an initialization fix (e.g. warm-start from the solved riseonly ckpt, the parallel seqrise arm already testing that lever).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (2M, single seed, 3rd datapoint for the stdreopen recipe's cross-seed reliability question): reads alongside seed0 (2/12, FAIL) and seed1 (11/12, PASS) already posted. If seed2 pattern-matches seed1 (partial-to-good tuck, clear improvement vs a fresh-seed 2M baseline, no majority OC-pin) -> recipe is seed-robust-enough (2/3), seed0 was the unlucky outlier -> promote seed1 ckpt / fund seed0-reseed's own 8M; if seed2 pattern-matches seed0 (total freeze, 0 movement) -> recipe is genuinely ~1/3 reliable -> DO NOT promote on a single seed, fund either a broader seed sweep or adopt the seqrise (solved-checkpoint warm-start) lever as the fix instead of this recipe as-is.


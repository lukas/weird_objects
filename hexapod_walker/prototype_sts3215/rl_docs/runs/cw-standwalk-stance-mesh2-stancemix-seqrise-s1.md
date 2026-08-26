# cw-standwalk-stance-mesh2-stancemix-seqrise-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T02:17:11+00:00

**pod**: hexapod-mjx-train-5

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stancemix-tuckclock-stdreopen-s1

**wandb_id**: 6x4ivpd1

**hypothesis**: Seed-1 twin of cw-standwalk-stance-mesh2-stancemix-seqrise: does the flat-rise/mix interference close if the 3-way hold+rise+lower mix warm-starts from the ALREADY-SOLVED flat-rise checkpoint (riseonly-bcchain3-meshref-tuckclock-acq8m-s1, 12/12 flat valid_plant on seed 1) instead of stancemix_bcchain3_stdanneal? Same recipe/hypothesis as the seed-0 arm, seed 1 lineage throughout (source config + warm-start ckpt) to answer the joint 2-seed pass-rate question in one batch.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (2M, 2-seed pair with seqrise seed0): PASS if flat-pinned probe shows genuine non-freeze tuck (majority NOT pinned at 2.64A, valid_plant clearly > the stdreopen-acq8m-s1 floor) AND hold/lower det+sto >=5/6+5/6 zero-term in both seeds -> fund 8M acquisition pair. FAIL if flat probe pin unchanged/still-majority-2.64A in both seeds -> interference is reward-mix pricing, not initialization; next lever is per-mode reward re-pricing or a staged/frozen-rise curriculum.


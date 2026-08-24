# cw-arch-tf64-joyfullcurr13-v7-hz100-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-24T22:09:16+00:00

**pod**: hexapod-mjx-train-6

**steps**: 38000000

**parent**: cw-arch-tf64-joyfullcurr13-v7-hz100-canary2

**hypothesis**: Plain English: give the transformer the same full budget the MLP needed before judging it -- the 08-24 dig-in showed the two 2M transformer 'collapse' canaries were statistically identical to the known-good MLP sibling at matched step counts (all ~-739 at 2M; the MLP fell to ~-1460 by 7M, crossed zero only ~12-14M, and finished at +746 in 40M), so the tf line was never actually shown to be worse than the architecture that passed. This arm continues the 2L/d128/8h/ff256 canary2 checkpoint (proven pre-100Hz geometry, tf-r1-hard1) for 38M more steps (40M total, matching the MLP sibling's budget) on the identical V7/hist64/100Hz stack, judged this time against the MLP's OWN matched-step trajectory as the control. Prediction-if-true (transformer is fine at 100Hz): reward tracks the MLP sibling's valley shape within noise, turns upward by ~10-12M, crosses zero by ~15M, walkcurr frontier unlocks past b0, and by 40M reaches MLP-equivalent health. Prediction-if-false (real architecture deficit, now with a valid control): at 15M reward is still monotonically declining or sits far below the MLP's matched-step value (~-980 at 10M, positive by ~14M) with no upturn and frontier still pinned at b0. Strongest alternative: seed-0 luck differences in valley depth -- judged by shape/turn timing, not exact values.

**gate**: Matched-step control gate vs cw-arch-hist64-joyfullcurr13-v7-hz100-scratch-s0-r1 (the MLP sibling's own W&B trajectory, cached): PASS requires (a) reward upturn (sustained rise over >=2M steps) beginning by 15M total steps, (b) reward crossing 0 by ~18M (MLP crossed ~12-14M; +30% slack), (c) walkcurr/frontier promoted past b0 once positive, (d) by 40M total: ep_rew_mean >= +400, env/walk_direction_valid >= 0.9, env/walk_loadslip_ratio <= 1.5, plus standard DR-0/own-DR/joygate reads reviewed on video. FAIL-architecture (the claim the overturned canaries tried to make, now with a valid control): still monotonically declining at 15M or >2x below the MLP's matched-step reward with frontier pinned at b0 -- that closes the hist64@100Hz transformer line for real. No mid-run kill on bad evals while reward tracks the MLP valley shape (08-21 ruling + the matched-step lesson).

**refused_reason**: acquisition runs require --evidence: name the healthy canary and a comparable full-budget learning precedent.


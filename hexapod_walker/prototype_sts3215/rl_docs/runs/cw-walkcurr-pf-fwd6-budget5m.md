# cw-walkcurr-pf-fwd6-budget5m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T20:30:14+00:00

**pod**: hexapod-mjx-train-0

**steps**: 5000000

**parent**: cw-walkcurr-pf-fwd3-chargeramp

**hypothesis**: Plain English: fwd1-fwd5 (8 arms) all froze identically at the same 2M-step budget every time; a cheap W&B-history read shows train/clip_fraction hits EXACTLY 0 within the first third of the run and never recovers, so PPO essentially stops updating -- but this specific hypothesis (does more optimizer steps alone, with the IDENTICAL mechanism, eventually escape the frozen basin) was never tested because every prior arm used the same 2M cap. Single lever: only --steps 2000000 -> 5000000 on the exact fwd3-chargeramp recipe, everything else byte-identical, fresh init. Prediction-if-true: env/walk_freeprog_score eventually leaves its flat [-0.10,-0.05] band and clip_fraction becomes nonzero again later in the run. Prediction-if-false: clip_fraction stays pinned at 0 and walk_freeprog_score stays flat through the full 5M -- budget is refuted as a lever (matches the STATUS.md diagnosis that this is a fixed-point/vanishing-advantage basin, not a slow-optimization one), closing the cheapest untested option and sharpening the case for rung-0/RND/BC-kickstart.

**gate**: Rung-1 gate (same as fwd1-fwd5), read at 2M (for apples-to-apples with the family) AND at 5M (own bar): env/walk_freeprog_score > -0.02 and rising, or clip_fraction nonzero again after the collapse = continue (budget helps). Still flat/zero at 5M = budget-alone hypothesis refuted; no same-recipe continuation.

**refused_reason**: discovery runs cap at 2000000 steps (asked 5000000): the question is 'did qualitatively correct behavior emerge?' - continue as --phase hardening with --evidence.


# cw-standwalk-stance-mesh2-holdheight-rung2-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T22:20:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1-hha1

**wandb_id**: nngk147g

**hypothesis**: Can the height-aware-anchor stance policy that already rides a +/-15mm commanded-height elevator on a clean six-foot stand learn the FULL [-40,20]mm range (STAND_HEIGHT rung 2) without re-discovering the leg-unload cheat? Warm-start from rung-1 seed 0's CLEAN checkpoint (0/12 min-load terms), bc_anchor_hold_height_aware=1 stays on as recipe default per the rung-1 joint PASS call; only the command range widens. Predict-if-true: DR-0 zero hold_min_load terms and duty ~1.0 all legs across the wider range. Predict-if-false: min-load dips reappear at the low end (-40mm near the lower envelope) -> registered S-gate/min-load-pricing fallback fires. Strongest alternative: the wider range is fine but cur_max grows past the honest band (deep-crouch cost, not a cheat).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as rung 1, judged jointly as a 2-seed pass-rate pair with -s1: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, no per-leg duty sacrifice (all legs >=0.9), cur_max within noise of rung-1 seed 0's 0.66-1.03A band.

**verdict**: CANARY PASS - MECHANISM with one letter-breach caveat (seed 0 of the rung-2 joint pair; final joint call lands with -s1). Plain English: with the height-aware anchor promoted to the full [-40,20]mm commanded-height range, the robot still rides the height elevator on a clean, level six-foot stand — the flag-leg cheat did not come back at the wider range. Evidence: DR-0 gate 6/6 det + 6/6 sto valid_plant (needed 5/6+4/6), ZERO hold_min_load terminations, height_err_end 0.2-3.7mm, det duty all legs >=0.98, det cur_max 0.75-1.06A (vs rung-1 seed-0 band 0.66-1.03A — band-adjacent, plausibly the honest cost of the 2.7x wider range), video level/planted/quiet (hold_det + worst sto episodes eyeballed). CAVEAT, honest letter-breach: 2/6 sto episodes lighten leg idx5 to duty 0.89/0.87 (<0.9 gate letter) with 27-28 micro-swings and sto cur_max up to 1.38A — the SAME leg-lightening residual family as rung-1 seed-1, now at trace magnitude with zero terminations (vs 6/12 terminated there). Own-DR 0.2: 11/12, one det hold_min_load term (duty collapse 0.29-0.59, 1.84A) — own-DR hardening stays open as at rung-1. Next: the -s1 cycle owns the registered joint call; my recommendation — if -s1 shows the same trace-level-or-better picture, PROCEED up the ladder from the cleaner checkpoint but note the pre-registered S-gate/min-load-pricing fallback condition ('EITHER seed reproduces min-load dips') is arguably met at trace level and should fire at the NEXT rung if the dips grow rather than shrink. Reward rose all run (15.5/40.4/106.1/165.1), no misalignment signature.


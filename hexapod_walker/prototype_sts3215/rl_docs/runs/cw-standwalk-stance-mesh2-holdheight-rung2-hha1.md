# cw-standwalk-stance-mesh2-holdheight-rung2-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T22:20:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1-hha1

**hypothesis**: Can the height-aware-anchor stance policy that already rides a +/-15mm commanded-height elevator on a clean six-foot stand learn the FULL [-40,20]mm range (STAND_HEIGHT rung 2) without re-discovering the leg-unload cheat? Warm-start from rung-1 seed 0's CLEAN checkpoint (0/12 min-load terms), bc_anchor_hold_height_aware=1 stays on as recipe default per the rung-1 joint PASS call; only the command range widens. Predict-if-true: DR-0 zero hold_min_load terms and duty ~1.0 all legs across the wider range. Predict-if-false: min-load dips reappear at the low end (-40mm near the lower envelope) -> registered S-gate/min-load-pricing fallback fires. Strongest alternative: the wider range is fine but cur_max grows past the honest band (deep-crouch cost, not a cheat).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same as rung 1, judged jointly as a 2-seed pass-rate pair with -s1: DR-0 det>=5/6 + sto>=4/6 valid_plant, ZERO hold_min_load terminations, no per-leg duty sacrifice (all legs >=0.9), cur_max within noise of rung-1 seed 0's 0.66-1.03A band.


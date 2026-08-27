# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T00:36:04+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal

**wandb_id**: qggsdmzc

**hypothesis**: Plain sentence: the shared-log_std anneal from anchor4-stdanneal fixed stochastic hold (6/6->0-2/6 term) but destroyed walk into an anchor1-class leg-sacrifice freeze on both seeds -- this arm tests whether a MILDER final noise target (-2.0 instead of -4.0, i.e. std 0.135 instead of 0.018, same anneal-frac 0.5) keeps enough exploration noise on walk's still-fragile core (weak crawl, prog 0.3-0.4, far from converged) to avoid the degenerate freeze, while still meaningfully cooling the stochastic-hold failure. Single lever vs anchor4-stdanneal: --log-std-final -4.0 -> -2.0, everything else (coef=3.0, isolate_update=1) unchanged. Prediction-if-true: walk stays gait_valid >=5/6 det (no anchor1/anchor4-class catastrophe) while hold/sto term meaningfully improves vs the anchor2/3 6/6 baseline (need not hit the full <=2/6 bar). Prediction-if-false: walk still collapses even at this milder dose -- confirms noise MAGNITUDE isn't the tunable knob and the real fix needs the harder per-core log_std split (DualGruActorCriticPolicy currently shares one log_std across both gated cores; already-flagged DIG-IN).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/anchor3/anchor4-stdanneal. WALK-SURVIVES if det gait_valid >=5/6 on BOTH seeds (no anchor1/anchor4-class 3+-leg-sacrifice freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 termination drops meaningfully from the anchor2/3 6/6 baseline (any improvement counts partial credit; <=2/6 is the full bar) on at least one seed. FULL PASS (promote this dose) = WALK-SURVIVES both seeds AND HOLD-HELPS at least partial on both. FAIL if walk still shows the anchor4-class catastrophe on either seed at this dose (confirms magnitude isn't the lever -> per-core log_std split is the only remaining lever) or if hold shows zero improvement (confirms anneal isn't doing anything at this dose).


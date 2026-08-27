# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild2-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T00:43:25+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal-s1

**wandb_id**: 4qaiutn8

**hypothesis**: Plain sentence: the shared-log_std anneal from anchor4-stdanneal fixed stochastic hold (6/6->0-2/6 term) but destroyed walk into an anchor1-class leg-sacrifice freeze on both seeds -- this arm tests whether a MILDER final noise target keeps enough exploration noise on walk's still-fragile core (weak crawl, prog 0.3-0.4, far from converged) to avoid the degenerate freeze, while still meaningfully cooling the stochastic-hold failure. Single lever vs anchor4-stdanneal: --log-std-final -4.0 -> -2.0, everything else (coef=3.0, isolate_update=1) unchanged. Prediction-if-true: walk stays gait_valid >=5/6 det (no anchor1/anchor4-class catastrophe) while hold/sto term meaningfully improves vs the anchor2/3 6/6 baseline (need not hit the full <=2/6 bar). Prediction-if-false: walk still collapses even at this milder dose -- confirms noise MAGNITUDE isn't the tunable knob and the real fix needs the harder per-core log_std split (DualGruActorCriticPolicy currently shares one log_std across both gated cores; already-flagged DIG-IN).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/anchor3/anchor4-stdanneal. WALK-SURVIVES if det gait_valid >=5/6 on BOTH seeds (no anchor1/anchor4-class 3+-leg-sacrifice freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 termination drops meaningfully from the anchor2/3 6/6 baseline (any improvement counts partial credit; <=2/6 is the full bar) on at least one seed. FULL PASS (promote this dose) = WALK-SURVIVES both seeds AND HOLD-HELPS at least partial on both. FAIL if walk still shows the anchor4-class catastrophe on either seed at this dose (confirms magnitude isn't the lever -> per-core log_std split is the only remaining lever) or if hold shows zero improvement (confirms anneal isn't doing anything at this dose).

**verdict**: CANARY FAIL - MECHANISM (hold clause; walk survives). The milder noise-anneal dose (-2.0) kept the robot walking but did nothing at all for the shaky-stand problem it was meant to fix, completing a clean three-point dose-response on this seed that says noise MAGNITUDE is not the knob. Evidence (DR-0 gate + own-DR 0.5, seed1, annealed std=0.135): walk/det 6/6 gait_valid, 0 terms, slip 4.21/m, v 0.066-0.072 vs ref 0.080 -- statistically identical to the un-annealed anchor2-s1 baseline (6/6, slip 4.07); own-DR walk 6/6 det + 6/6 sto gait_valid, 0 terms. WALK-SURVIVES clause: MET on this seed (no anchor4-class freeze; videos show all six legs cycling). HOLD-HELPS clause: ZERO improvement -- hold/sto 6/6 hold_min_load terms at BOTH DR-0 and own-DR, exactly the anchor2/3 baseline (video: body level, h_err +1mm, feet unload under action noise at t~3s; a noise-driven unload, not a fall). Dose-response on seed1 is now: std 0.225 walk-ok/hold-fail(6/6), std 0.135 walk-ok/hold-fail(6/6), std 0.018 hold-ok(0/6)/walk-destroyed -- no shared-log_std window exists on this seed. Per the pre-registered gate, FULL PASS requires HOLD-HELPS at least partial on BOTH seeds, so the -2.0 dose is dead for promotion regardless of the seed-0 twin (other cycle). Caution flag: walk/sto picked up 2/6 over_current terms with sacrificed legs [1,2,5]/[0,2] (baseline 0/6) while sto slip halved (23.9 -> 10.4) -- even -2.0 already nudges stochastic walk toward anchor4's leg-sacrifice direction. What's next: this strengthens the already-flagged per-core log_std split (STATUS Next -1.8: separate learnable log_std_b for the stance core, anneal only that) as the real fix; final joint call on the bracket belongs to the cycles holding stdmild2 seed-0 and the stdmild1 pair. Big picture: stance-hold wants near-zero action noise, walk still needs real exploration noise -- one shared scalar cannot serve both gated cores.


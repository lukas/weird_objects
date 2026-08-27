# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T00:47:38+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal

**wandb_id**: hzjoj8xj

**hypothesis**: Plain sentence: same question as anchor5-stdmild2 (does a milder anneal target preserve walk while still helping hold) but at an even softer dose, giving a real 2-point dose bracket instead of one guess. Single lever vs anchor4-stdanneal: --log-std-final -4.0 -> -1.0 (std 0.018 -> 0.368, barely below the leak-fixed anchor2/3 parents' own trained std), everything else (coef=3.0, isolate_update=1) unchanged. Prediction-if-true: walk fully protected (gait_valid >=5/6 both seeds, matching anchor2/3's healthy band) with at least a mild hold/sto improvement over the 6/6 baseline. Prediction-if-false: even this soft a dose still tips walk into the anchor4-class freeze -- would mean ANY nonzero anneal on the shared log_std is unsafe for this dual-core recipe, strongly pointing at the per-core log_std split (DIG-IN flagged) as the only viable fix rather than a dose tune.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/anchor3/anchor4-stdanneal/anchor5-stdmild2. WALK-SURVIVES if det gait_valid >=5/6 on BOTH seeds (no anchor1/anchor4-class 3+-leg-sacrifice freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 termination drops meaningfully from the anchor2/3 6/6 baseline (any improvement counts partial credit; <=2/6 is the full bar) on at least one seed. FULL PASS (promote this dose) = WALK-SURVIVES both seeds AND HOLD-HELPS at least partial on both. Read jointly against anchor5-stdmild2's own -2.0 result as a 2-point dose-response: if BOTH doses wreck walk, the magnitude lever is closed and the per-core log_std split (DIG-IN) is the only remaining lever; if only the softer -1.0 dose survives, that pins the safe operating dose; if both survive, prefer whichever gives more hold improvement.


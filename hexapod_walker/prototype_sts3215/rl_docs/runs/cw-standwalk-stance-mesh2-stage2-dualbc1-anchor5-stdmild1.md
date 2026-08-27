# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor5-stdmild1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T00:47:38+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal

**wandb_id**: hzjoj8xj

**hypothesis**: Plain sentence: same question as anchor5-stdmild2 (does a milder anneal target preserve walk while still helping hold) but at an even softer dose, giving a real 2-point dose bracket instead of one guess. Single lever vs anchor4-stdanneal: --log-std-final -4.0 -> -1.0 (std 0.018 -> 0.368, barely below the leak-fixed anchor2/3 parents' own trained std), everything else (coef=3.0, isolate_update=1) unchanged. Prediction-if-true: walk fully protected (gait_valid >=5/6 both seeds, matching anchor2/3's healthy band) with at least a mild hold/sto improvement over the 6/6 baseline. Prediction-if-false: even this soft a dose still tips walk into the anchor4-class freeze -- would mean ANY nonzero anneal on the shared log_std is unsafe for this dual-core recipe, strongly pointing at the per-core log_std split (DIG-IN flagged) as the only viable fix rather than a dose tune.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or require mature gait. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2/anchor3/anchor4-stdanneal/anchor5-stdmild2. WALK-SURVIVES if det gait_valid >=5/6 on BOTH seeds (no anchor1/anchor4-class 3+-leg-sacrifice freeze, prog_ratio >=~0.2). HOLD-HELPS if hold/sto DR-0 termination drops meaningfully from the anchor2/3 6/6 baseline (any improvement counts partial credit; <=2/6 is the full bar) on at least one seed. FULL PASS (promote this dose) = WALK-SURVIVES both seeds AND HOLD-HELPS at least partial on both. Read jointly against anchor5-stdmild2's own -2.0 result as a 2-point dose-response: if BOTH doses wreck walk, the magnitude lever is closed and the per-core log_std split (DIG-IN) is the only remaining lever; if only the softer -1.0 dose survives, that pins the safe operating dose; if both survive, prefer whichever gives more hold improvement.

**verdict**: CANARY FAIL - MECHANISM (walk-survives branch, own-scope, seed0). Even the mildest tested anneal (-1.0, std=0.368, barely below the un-annealed dual-core parents' own trained std) wrecks walk on this seed -- the gate's own walk-catastrophe FAIL branch fires. Evidence (own report.json, DR-0 + own-DR(0.5), det+sto, video-watched): walk/det gait_valid 0/6 in EVERY cell (DR-0 det/sto, own-DR det/sto, startjitter det/sto) -- a consistent 2-leg sacrifice ([4,5], the two rear legs) on essentially every episode, slip/m 11-31 (vs anchor2/3's healthy 3-5), prog_ratio only 0.00-0.10 (well under the ~0.2 bar). Contact sheet (walk_det_0) confirms visually: two rear legs held rigid/splayed the entire strip while the other four make small adjustments -- the same signature class as anchor1/anchor4's leg-sacrifice freeze, just a different leg subset. hold/sto stays 6/6 hold_min_load term at both DR (zero improvement, same as its own seed1 twin and stdmild2) -- so even where this dose doesn't wreck walk (see -s1) it still doesn't help hold. This is the DECISIVE read for the whole magnitude-bracket question: this seed satisfies the track's own pre-registered escalation condition verbatim (STATUS.md -1.8: 'if even -1.0 still wrecks walk, this per-core split becomes the only remaining lever and should be funded immediately'). Cross-seed context: stdmild1-s1 (seed1, same -1.0 dose) does NOT show this catastrophe (gait_valid 6/6, prog 0.29) but also shows zero hold improvement -- so across the whole grid tested so far (-1.0, -2.0 x 2 seeds; -4.0 x 2 seeds already closed), no dose both reliably protects walk AND helps hold. Next: per the pre-registered rule this closes the anneal-MAGNITUDE lever for a SHARED log_std; funding the per-core log_std split (DualGruActorCriticPolicy currently shares one log_std across both gated cores) is now warranted regardless of stdmild2-s1's still-pending read.


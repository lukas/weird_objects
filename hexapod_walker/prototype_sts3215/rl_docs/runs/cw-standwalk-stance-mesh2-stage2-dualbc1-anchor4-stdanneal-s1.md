# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-26T22:17:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: v9qsbd20

**hypothesis**: Seed-1 twin of anchor4-stdanneal (launched without one), mirroring the anchor1/2/3 seed-pair pattern so the STDANNEAL-WORKS/FAIL joint call doesn't wait a full extra cycle. Same single lever as anchor4-stdanneal vs anchor2-s1: add --log-std-anneal-frac 0.5 --log-std-final -4.0 (the schedule that fixed the isolated hold rung and the full-mix hold this cycle) on top of anchor2-s1's own leak-fixed coef=3.0 dual-core checkpoint. Prediction-if-true: hold/sto termination drops from anchor2-s1's own total 6/6 toward isolated (<=2/6) on seed1 too, without walk regressing (gait_valid stayed 6/6 clean on anchor2-s1, even cleaner than seed0). Prediction-if-false: hold/sto stays pinned 6/6 noise-free on this seed too -- confirms cross-seed that the dual-core stochastic-hold failure is a shared-trunk/value coupling issue, not exploration noise, matching whatever seed0 anchor4-stdanneal reads.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2-s1/anchor3-s1. LEAK-STAYS-FIXED PASS if walk shows no anchor1-class catastrophe (det gait_valid >=5/6, prog_ratio >=~0.2, no 5-leg sacrifice/negative-prog shuffle). STDANNEAL-WORKS if hold/sto termination drops from anchor2-s1/anchor3-s1's baseline (total 6/6 both DR) to <=2/6 on at least one DR level without walk regressing. FAIL if hold/sto stays ~6/6 term noise-free too, or walk regresses toward anchor1-class catastrophe. JOINT call with anchor4-stdanneal (seed0) decides: STDANNEAL-WORKS both seeds -> promote as the dual-core stance recipe; FAIL either/both -> next lever targets shared-trunk/value coupling directly (per-mode value heads), not another stance-side knob.

**verdict**: CANARY FAIL - MECHANISM + JOINT CALL CLOSED (both seeds agree): seed 1 replicates seed 0's exact signature -- std-anneal fixes hold (even more cleanly: DR-0 sto term 6/6->0/6, fully isolated) but wrecks walk into the same anchor1-class leg-sacrifice freeze cross-seed. Evidence: hold/det DR-0 term 2/6 (vs anchor2-s1/anchor3-s1's mixed baseline), hold/sto DR-0 term 0/6 (clean, best hold read of the whole dual-core lineage); own-DR hold/det 3/6, hold/sto 1/6 (both improved vs the 6/6 baseline). walk: DR-0 det gait_valid 0/6 (sac=[5] repeated, prog~0.00, slip 36+/m -- even worse slip than seed0), DR-0 sto 0/6, own-DR det+sto 0/6 both, jitter panel 0/6 (1/6 on one det cell) everywhere. Video (walk_det_0..5) shows the identical pathology: one rear leg held rigid in the air, zero translation across the 30s strip -- cross-seed confirmed, not a seed-specific fluke. Reward quarters [39.9, -3.9, -154.6, -138.6] -- same trough-then-incomplete-recovery shape, still deeply negative Q4. JOINT CALL: the pre-registered LEAK-STAYS-FIXED clause fails on BOTH seeds (walk shows the anchor1-class catastrophe the clause was written to rule out) even though STDANNEAL-WORKS's hold clause is independently met on both -- per the gate's own OR-structured FAIL branch, this is a clean cross-seed FAIL, not a split/ambiguous read. Root cause (see anchor4-stdanneal's own verdict, code-read this cycle): DualGruActorCriticPolicy shares ONE log_std across both gated cores (value heads are already separate, so the gate's guessed 'per-mode value heads' lever was moot); annealing it starves walk's still-fragile core of the exploration noise it needed mid-training, letting PPO settle into a degenerate freeze that outscores the marginal weak-crawl under the walk reward's own height/posture gates. Next: do not fund a third full-dose (-4.0) shared-log_std arm. This cycle's refill is a milder-dose bracket (log_std_final -2.0 and -1.0, 2 seeds each, no code change) off anchor2/anchor2-s1 (the leak-fixed, pre-anneal parents) to test whether the ANNEAL MAGNITUDE is the tunable knob before committing to the harder per-core-log_std code fix (flagged DIG-IN for a deep cycle).


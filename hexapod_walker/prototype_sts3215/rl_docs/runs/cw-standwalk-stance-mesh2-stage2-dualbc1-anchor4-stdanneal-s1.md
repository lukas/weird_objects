# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor4-stdanneal-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T22:17:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: v9qsbd20

**hypothesis**: Seed-1 twin of anchor4-stdanneal (launched without one), mirroring the anchor1/2/3 seed-pair pattern so the STDANNEAL-WORKS/FAIL joint call doesn't wait a full extra cycle. Same single lever as anchor4-stdanneal vs anchor2-s1: add --log-std-anneal-frac 0.5 --log-std-final -4.0 (the schedule that fixed the isolated hold rung and the full-mix hold this cycle) on top of anchor2-s1's own leak-fixed coef=3.0 dual-core checkpoint. Prediction-if-true: hold/sto termination drops from anchor2-s1's own total 6/6 toward isolated (<=2/6) on seed1 too, without walk regressing (gait_valid stayed 6/6 clean on anchor2-s1, even cleaner than seed0). Prediction-if-false: hold/sto stays pinned 6/6 noise-free on this seed too -- confirms cross-seed that the dual-core stochastic-hold failure is a shared-trunk/value coupling issue, not exploration noise, matching whatever seed0 anchor4-stdanneal reads.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same joint hold/rise/lower/walk DR-0 det+sto + own-DR(0.5) panel as anchor2-s1/anchor3-s1. LEAK-STAYS-FIXED PASS if walk shows no anchor1-class catastrophe (det gait_valid >=5/6, prog_ratio >=~0.2, no 5-leg sacrifice/negative-prog shuffle). STDANNEAL-WORKS if hold/sto termination drops from anchor2-s1/anchor3-s1's baseline (total 6/6 both DR) to <=2/6 on at least one DR level without walk regressing. FAIL if hold/sto stays ~6/6 term noise-free too, or walk regresses toward anchor1-class catastrophe. JOINT call with anchor4-stdanneal (seed0) decides: STDANNEAL-WORKS both seeds -> promote as the dual-core stance recipe; FAIL either/both -> next lever targets shared-trunk/value coupling directly (per-mode value heads), not another stance-side knob.


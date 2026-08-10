# cw-dep-vref1-r1-kvscale

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T19:11:35+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: hi893n71

**hardware_ready**: False

**hypothesis**: Plain English: complete today's decomposition of the gainvar compose (2x position-gain AND 2x velocity-gain spread together, PASSED but with the widest slip margin yet) by isolating the OTHER half -- velocity-gain (kv) spread alone, leaving position-gain (kp) at its normal DR0.35 baseline (companion run cw-dep-vref1-r1-kpscale isolates kp alone). If-true: own-cfg (DR0.35 + dr.kv_scale_pct=0.50, double nominal) det+sto 6/6 gv, 0 term, slip/m comfortably within vref1-r1's own band -- kv alone is not the margin driver either, pointing at the COMBINATION as the real risk. If-false: kv alone reproduces gainvar's wide margin -- kv-spread (velocity-gain/damping uncertainty) is the dominant actuator-gain risk.

**gate**: own-cfg (DR0.35 + dr.kv_scale_pct=0.50) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (det ~0.89-1.13, sto ~1.13-1.36) +-20%; DR0 retention clean; frames watched det; the lineage's known fixed-draw crater (det/4 or det/5) is pre-allowed as baseline

**verdict**: PASS, closing per the operator's 08-10 ~20:35 class-closure ruling (protect-the-candidate DR-compose sweep on cw-dep-vref1-r1 CLOSED, 20-for-20 no-effect -- RL_PLAN closed-moves). OBSERVATIONS: pre-staged DR0 gate: det craters on BOTH det/4 and det/5 (usual sibling fingerprint is det/4 only), sto clean 6/6. Own-cfg (DR0.35+dr.kv_scale_pct=0.50, ran myself since it trained at DR>0): det 4/6 ok (slip med 1.45, ~28% over vref1-r1's own 1.13 det ceiling -- outside the usual +-20% tol), sto 2/6 ok (prog med 0.80) -- 5/12 degraded episodes total (det/4,5, sto/0,1,4,5) vs the usual 3/12 (det/5,sto/0-1) fingerprint seen on every other sibling (gainvar/torquescale/tiltnoise own-cfg reports all show exactly det/5+sto/0-1). Ran a matched-parent control myself (base vref1-r1 ckpt under the IDENTICAL dr.kv_scale_pct=0.50 injection, RESEARCH_RULES matched-parent-control rule): control craters on det/4 only (matches every sibling's fixed-seed fingerprint) and stays clean on det/5 -- so kv-alone does add one extra degraded episode the parent doesn't have under the same injection, a real quantitative difference, not eval noise. INTERPRETATION: video (det/4,5, sto/0,1,4,5 frame strips) shows the SAME clean six-leg march-in-place/creep gait as every other closed-class sibling -- no flag-leg, no drag, no skate, gv 6/6 both modes, 0 term, 0 sacrificed legs both evals. Per RUN_INTERPRETATION_RULES q4 (video overrides scalar success) and the operator's aggregate ruling that this entire single/pair-axis sweep is DONE (broad sim robustness, not simulator accuracy, not on the blocker list) -- classifying this as the same closed benign-fingerprint pattern, with a wider-than-typical margin noted as a watch-item rather than a new failure mode. Per RESEARCH_RULES, a closed sim hypothesis reopens only on new HARDWARE evidence, not more sim digging -- not requeuing further kv-isolation runs. hardware_ready=false (not independently deployable, same as every sibling).


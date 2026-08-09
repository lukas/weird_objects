# cw-walk-step0-anchor

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T09:08:05+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**wandb_id**: g248pl2a

**hypothesis**: Fresh-init basin test, pre-registered in cw-walk-anchorgate's escalation tree and run in parallel since capacity is idle and the question is independent of the c1 continuation. HYPOTHESIS: the champion lineage's paddling is a WARM-START LEGACY basin - it learned speed-band income before anchoring was priced; a from-scratch learner whose velocity income is anchor-gated from the FIRST gradient step enters an anchored-stepping basin directly. Config = anchorgate reward package verbatim; variable = init regime (fresh init + binding-audit from-scratch exploration: log_std_init 0.0, ent 0.005, target_kl 0.02 trainer default). Prediction-if-true: a gait emerges with walk_anchor_frac >0.85 EARNED and DR0 det agg slip/m <=1.0 (champion lineage sits 1.14-1.18 at DR0), gv 12/12. Prediction-if-false: (a) converges to the same paddle (frac ~0.8, slip >=1.1, gv ok) => paddling is the sim's globally preferred transport under current contact/current pricing, NOT a legacy basin => root is the sim pricing defect (operator, cycle 27/28 rulings) or distillation - warm-start lineage remains the vehicle; (b) no gait within 40M (fwd ~0, step events ~0, ep_rew flat) => gated income too sparse for from-scratch discovery => one follow-up allowed: anneal-in gate curriculum; NOT a refutation of the gate itself. Strongest alternative: fresh-init failure from exploration hyperparams rather than the gate - mitigated by using the exact audited package that produced step0's gait, and distinguished by signature (b) vs (a). Name matches auto_continue_lineages prefix cw-walk-step0: watcher extends while reward climbs, per operator 0-a.

**gate**: DR0 harness 15s 6+6: gv 12/12, >=2 swings/leg, 0 term, det fwd mean >=0.55, agg slip/m det <=1.0 AND sto <=1.2, W&B walk_anchor_frac >0.85 earned (income held, not forfeited); frames: anchored stance vs paddle; verdict reads if-false shape (a) paddle-again vs (b) no-gait

**verdict**: FAIL gate (cycle 33): DR0 15s det agg slip/m 1.479 vs <=1.0, sto 1.805 (1.545 excl known backward draw) vs <=1.2; det fwd 0.913 PASS but via OVERSPEED 0.069-0.074 m/s ABOVE the 0.02-0.06 band (harness success 0/12 on vel_err); gv 12/12, 0 term; anchor_frac 0.879 Q4 / 0.892 last, earned (reward_walk ~1.0/tick). HYPOTHESIS REFUTED - if-false shape (a) fired and WORSE: fresh init entered the allowance-riding basin directly (det cadence ~101 stances/ep vs champion 47-58, tol-floor ~1.11 slip/m; drummer-leg: leg4 swings 30-43 vs 9-13 on legs 0-2, leg1 duty 0.26-0.33). Paddling is the sim's preferred transport under current pricing, NOT a warm-start legacy. NOT HARDWARE-READY (grinds 1.48 m/m, overspeed, asymmetric cadence). Champion unchanged (35234ddc). Frames watched: det0/det5/sto0/sto5 375f each, md5 5e707ac6/5a877752/07aed7c3/5855125e + contact sheet + dense det0 tile. train/std ROSE 1.05->1.86 (fresh, ent 0.005) - lineage std~2 attractor reproduced.


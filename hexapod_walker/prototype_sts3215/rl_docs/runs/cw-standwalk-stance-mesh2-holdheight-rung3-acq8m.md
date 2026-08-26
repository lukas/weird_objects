# cw-standwalk-stance-mesh2-holdheight-rung3-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T23:29:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: nuw6ttej

**hypothesis**: Does more training make the height-following stance robust under physics randomization? The rung-3 kind-mix canary passed clean at DR-0 (12/12, zero min-load terms, duty floor 0.98) but keeps exactly one det hold_min_load termination at its own training DR 0.2 — a residual persisting since rung 2 — and reward was still rising at the 2M cutoff (quarters 20.5/50.2/79.2/176.4), so per the 08-21 ruling this is a continuation, not a redesign. 8M acquisition, exact rung-3 recipe, warm from the rung-3 seed-0 ckpt. Prediction-if-true: own-DR 0.2 reaches 12/12 zero-term with DR-0 unchanged. Prediction-if-false: the own-DR min-load term is budget-invariant -> the registered S-gate/min-load-pricing fallback is the next lever, not budget. Strongest alternative: extended optimization re-buys the leg-unload cheat at DR-0 (tripwire fields watched). Judged jointly with -acq8m-s1 as a 2-seed pair.

**gate**: Joint 2-seed pair: own-DR 0.2 12/12 valid_plant with ZERO hold_min_load terms (the target residual); DR-0 det>=5/6 + sto>=4/6 valid_plant, zero min-load terms, per-leg duty >=0.85 (rung-3 tripwire carried), det cur_max within the rung-2/3 0.62-1.06A band.

**verdict**: FAIL (pre-registered fail branch; scoped to seed 0 of the joint pair — joint bookkeeping lands with -s1's cycle). Plain result: the 8M acquisition continuation made the rung-3 stance WORSE, not more robust — extended optimization re-buys the leg-unload cheat, the hypothesis's own named strongest alternative. Evidence: DR-0 gate det 6/6 valid_plant zero terms (duty floor 0.91) BUT sto 2/6 hold_min_load terms with per-leg duty floors 0.75/0.76 (legs idx3/idx5) — the rung-3 TRIPWIRE (any per-leg duty <0.85 OR any DR-0 min-load term, EITHER seed) fired on both clauses; det Imax 0.79-1.12A drifts above the 1.06A band top; own-DR 0.2 = 2/12 terms (det ep3 duty 0.90 + sto ep5 duty 0.60) vs the 2M canary's 1/12 — the target residual is budget-invariant and slightly worse at 4x budget. Reward rose all run (quarters 79.8/316.9/545.0/709.3) while evals regressed vs the 2M canary (duty floor 0.98, zero DR-0 terms, Imax<=0.82) => per the 08-21 ruling this is the MISALIGNMENT branch, not undertraining: reward still pays for partial unloading the S-gate does not price. Rung-3 champion remains the 2M canary ckpt pair (hha1/-s1); this 8M ckpt is NOT promoted. Next per the pre-registered FAIL branch: the registered S-gate/min-load-pricing fallback FIRES NOW (either-seed tripwire) — that is reward/env code + semantics-bank rows, so DIG-IN flagged for the deep cycle; no more budget on this recipe. Frame strips level/planted, unloading sub-visual — video and metrics agree.


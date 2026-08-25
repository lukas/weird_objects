# cw-standwalk-stance-mesh2-holdheight-rung3-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T23:29:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung3-hha1

**wandb_id**: nuw6ttej

**hypothesis**: Does more training make the height-following stance robust under physics randomization? The rung-3 kind-mix canary passed clean at DR-0 (12/12, zero min-load terms, duty floor 0.98) but keeps exactly one det hold_min_load termination at its own training DR 0.2 — a residual persisting since rung 2 — and reward was still rising at the 2M cutoff (quarters 20.5/50.2/79.2/176.4), so per the 08-21 ruling this is a continuation, not a redesign. 8M acquisition, exact rung-3 recipe, warm from the rung-3 seed-0 ckpt. Prediction-if-true: own-DR 0.2 reaches 12/12 zero-term with DR-0 unchanged. Prediction-if-false: the own-DR min-load term is budget-invariant -> the registered S-gate/min-load-pricing fallback is the next lever, not budget. Strongest alternative: extended optimization re-buys the leg-unload cheat at DR-0 (tripwire fields watched). Judged jointly with -acq8m-s1 as a 2-seed pair.

**gate**: Joint 2-seed pair: own-DR 0.2 12/12 valid_plant with ZERO hold_min_load terms (the target residual); DR-0 det>=5/6 + sto>=4/6 valid_plant, zero min-load terms, per-leg duty >=0.85 (rung-3 tripwire carried), det cur_max within the rung-2/3 0.62-1.06A band.


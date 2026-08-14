# cw-stand-postlower2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-14T22:55:12+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-stand-postlower1

**wandb_id**: hzgabwvb

**hypothesis**: Stand back up out of the exact sit-down pose without breaking the rest of standing up: test whether 0.35 was simply too strong a dose of the post-lower start-state bank (cw-stand-postlower1 regressed sto post-lower rise 0.801->0.717 plus a small cold-rise/det-session hit) by cutting exposure to 0.15 with every other line of the recipe unchanged. Prediction-if-true (dose): post-lower rise moves back toward/above 0.801 and det/cold-start retention returns to parent level -- frac is the lever, harden at a tuned value next. Prediction-if-false (mechanism): post-lower rise stays regressed or flat despite less exposure -- the fixed rise_ref_track reference (built for flat-topology starts) is mis-pricing this pose family regardless of dose, and the next lever is a bank-aware reference/pricing change, not another frac sweep.

**gate**: Discovery pilot, 2M steps, decision-only (no promotion claim). PASS (=dose confirmed, proceed to a tuned hardening run) iff the standard pod gate eval (own-cfg, per-mode=6) shows sto rise post_lower >=1/2 (vs this pilot's own small-n) AND no NEW visible pathology (flag-leg/tripod/park) in the det/sto rise videos AND det hold/lower stay 6/6. FAIL (=mechanism, not dose -- do not re-dose again) iff post_lower sto rise stays 0/2 (or video shows the robot still visibly stuck/over-current at the bank pose) despite the lower dose. Either way this is a DISCOVERY read only; any promotion claim needs a fresh bulk cohort (SESSION_BULK_GATE.md convention) pre-registered before that run.


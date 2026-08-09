# cw-walk-torquedroop

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:40:46+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: rg6gujl6

**hypothesis**: OPERATOR WISHLIST 13b (richer physics, one axis per run): torque droop / battery sag is a cheap-servo sim2real axis (STS3215 under load + voltage sag). ISOLATED axis via cycle-49 dr.<field> overrides: dr-scale 0.0 with ONLY dr.torque_scale=0.60,1.05 randomized (down to 60% torque, wider than the standard 0.80-1.05 envelope) - one variable off the no-DR champion. If-true: gait holds across the torque spread (own-cfg harness gv 12/12, 0 term, det median fwd >=1.2m @30s) and DR0 retention holds - torque robustness is trainable by exposure and joins the transfer recipe. If-false: low-torque draws collapse height/progress - torque sag needs explicit adaptation (estimator rung), not exposure. Strongest alternative: policy survives by slowing cadence/crouching - check cadence + height_err vs champion.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.torque_scale=0.60,1.05, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal retention det 6/6 gv, det slip/m <=1.24; frames watched det


# cw-standwalk-stance-mesh2-holdminload40-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T10:36:08+00:00

**pod**: hexapod-mjx-train-1

**steps**: 6000000

**parent**: cw-standwalk-stance-mesh2-holdterm40

**wandb_id**: cvacvokp

**hypothesis**: Plain English: seed replicate of holdminload40 -- with the min-load termination lever added, does a SECOND seed also converge to (or return to) the six-foot stance, or does it only work for one optimization path? Every rung in this campaign (holdload1min, holdprod, holdterm40) has found a DIFFERENT basin per seed, so a single-seed pass is weak evidence. Byte-identical to holdminload40 except seed=1. Prediction-if-true: same PASS shape as seed 0 (>=10/12 valid plant at 6M). Prediction-if-false: seed 1 lands in a different escape basin (or the same alternative-cheat basin) while seed 0 passes -- joint read decides whether rung-6 needs a dose adjustment or a DR/entropy pairing.

**gate**: Same as holdminload40: hold panel at 6M, pod_eval hold DR-0 det+sto n=6+6, >=10/12 survive with zero OC/tilt/hold_low_height/hold_min_load terms, six-foot stance (valid_plant or all-leg duty>=0.9), cur_p95<=1.0A; own-DR(0.2) alongside. Joint pass-rate read with seed 0 (n=2).


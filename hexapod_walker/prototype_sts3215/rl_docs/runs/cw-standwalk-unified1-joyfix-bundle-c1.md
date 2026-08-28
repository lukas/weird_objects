# cw-standwalk-unified1-joyfix-bundle-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-28T16:24:36+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-unified1-mix-long-s0

**wandb_id**: 31xrlzi8

**hypothesis**: Plain English: the combined fix for everything the 08-28 drivability audit found real — full-circle command headings (lateral commands finally drawn), the deployable leg-odometry velocity estimator in the obs (policy can finally see its own speed), and the direct normalized command objective — applied together to the unified 60s session recipe, warm from long-s0's 16M PASS checkpoint, as the bundle arm of the 4-arm canary grid (the three single-lever siblings attribute any effect). Coupled bundle justified: heading exposure without velocity observability may not be learnable, and cmd_track prices exactly the velocity error mode 3 makes observable. Predict-if-true: at 2M the lateral response ratio at least doubles from 0.033 AND fwd stays >=0.35 with terminations recovering to parent band. Predict-if-false: the joint perturbation destabilizes the warm policy worse than any single lever (compare siblings) — would say stage the levers sequentially in acquisition. Strongest alternative: bundle matches the best single lever (no synergy) — then acquisition funds only that lever.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. CANARY (mechanism health only): at 2M — training reward recovering (not collapsed vs siblings at matched steps); DR-0 det walk gait_valid >=5/6 no new sacrificed leg; session terminations <=6/90; probe: max(left,right) speed_ratio >=0.07 AND fwd >=0.35. PASS -> 2-seed 16M acquisition of the bundle (or the winning subset per siblings); FAIL with siblings healthy -> stage levers sequentially.


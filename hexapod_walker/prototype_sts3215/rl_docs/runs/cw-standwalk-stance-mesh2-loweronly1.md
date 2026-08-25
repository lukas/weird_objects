# cw-standwalk-stance-mesh2-loweronly1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T07:00:45+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-cur1

**wandb_id**: 1dg40jin

**hypothesis**: Plain English: cur1/cur1-seed1's lower mode is the one mode where the priced-hot-current charge is a KNOWN residual risk (STATUS's own named fallback fork: the 25-55mm crouch is intrinsically hot on the 3.5kg mesh body even for honest descent, honest -44 vs grind -27 at the chosen dose -- profit erased but honest not dominant). This arm isolates lower: goal-mix forced to lower=1.0, same pricing, 2M discovery. Prediction-if-true (lower is the specifically-hard mode, not a general failure): hold/rise's absence lets lower learn a real controlled descent, ending in posture-strict valid low plant on a majority of DR-0 episodes. Prediction-if-false: lower-only ALSO over_current-terminates the same way -- confirms the STATUS's own named fork is real and the fix is recalibrating goal.lower_height_mm for mesh (belly-rest supported, less knee torque) rather than more pricing or curriculum share.

**gate**: Discovery/canary read at 2M: pod_eval stance panel, lower mode only, n=6 det DR-0. PASS-qualitative: majority (>=4/6) reach a posture-strict low plant without an over_current termination. FAIL: same over_current-at-crouch signature as cur1 -- escalate to goal.lower_height_mm mesh recalibration, per the pre-registered fallback fork.

**verdict**: FAIL at the 2M discovery gate (0/6 det valid-plant, majority over_current; 1/6 sto clean success). Video (lower_det_0) shows the robot standing TALL and upright (not lowering to the crouch target, not the full-mix's rearing pathology) then tripping current holding that tall stance -- height_err 17-39mm off the lower target in every det episode. sto is noisier: several non-terminated near-target episodes (height_err <15mm, cur_p95 2.2-2.4A, no term) alongside 1 clean success, suggesting the policy is closer to a working descent than the det read shows. Reward still declining at 2M (7.1/-40.1/-147.5/-184.3), not flat. Launched cw-standwalk-stance-mesh2-loweronly1-acq1 (+8M, 10M total) testing whether budget lets it commit to the actual lower target instead of parking tall; pre-registered fallback if it still fails at 10M is goal.lower_height_mm mesh recalibration (named in STATUS).


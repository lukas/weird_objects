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

**verdict**: Diet isolation answer: lower does not emerge cleanly at 100% share either — DR-0 det 6/6 over_current with the body sagging into a hot splayed grind instead of a tucked descent (video); sto 1/6 lucky-passes, own-DR det 6/6 OC. Reward declined all 2M. Confirms the dig-in's named residual risk: the mesh lower is intrinsically hot under this recipe. Lower's rung comes AFTER a valid six-foot hold exists (warm start from it), with the pre-registered goal.lower_height_mm recalibration (belly-rest, servos unloaded) as the fallback fork if it still runs hot. Discovery arm; question answered, lineage closed.


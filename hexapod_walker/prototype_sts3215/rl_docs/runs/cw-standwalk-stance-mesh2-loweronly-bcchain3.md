# cw-standwalk-stance-mesh2-loweronly-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T13:29:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: kysdqcu1

**hypothesis**: Can the robot learn to SIT DOWN -- lower its body with all six feet staying planted -- on the heavier mesh model when every lower tick gets the honest IK-descent pose as an imitation target (bc_anchor_lower=1, FixedFootBodyIK at the next commanded height)? Rung-8 lower read, sibling of riseonly-bcchain3: the footlow2-PASS anchor bundle on the mesh bcanchor3 recipe (coef 3.0), lower-only diet. bc_anchor_lower chain is bank-green at 100 Hz as of this cycle (test_bc_anchor.py 56/56 incl. lower_anchor_chain_descends, tag exp/bcanchor-chain-tests-rate-fix). Prediction-if-true: det lower tracks the commanded descent with feet planted and no over_current. Prediction-if-false: aloft/outrigger residue or belly-crash lowering (the mesh bank's known thrash-collapse luck), or the dig-in's named residual risk bites -- the 25-55mm crouch is intrinsically hot on mesh (~2.2A one knee even honest; honest -44 vs grind -27 at chosen pricing) -- in which case the fork is goal.lower_height_mm mesh recalibration (belly-rest, servos unloaded), not more pricing. Strongest alternative: descends but with a flag leg (stratified+foot_z should prevent it; watch the strips).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, lower DR-0 det n=6: >=4/6 honest descents (>=60% of commanded drop, every foot within ~25mm of ground throughout, zero over_current terms) = PASS (fund acquisition); partial/slow descents trending = PARTIAL; 0/6 with aloft/outrigger or crash-collapse signature = anchor-lower refuted on mesh, fork to goal.lower_height_mm belly-rest recalibration per the 08-25 dig-in fallback.


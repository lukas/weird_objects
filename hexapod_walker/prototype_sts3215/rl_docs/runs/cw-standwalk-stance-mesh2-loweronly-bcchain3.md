# cw-standwalk-stance-mesh2-loweronly-bcchain3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T13:29:35+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3

**wandb_id**: kysdqcu1

**hypothesis**: Can the robot learn to SIT DOWN -- lower its body with all six feet staying planted -- on the heavier mesh model when every lower tick gets the honest IK-descent pose as an imitation target (bc_anchor_lower=1, FixedFootBodyIK at the next commanded height)? Rung-8 lower read, sibling of riseonly-bcchain3: the footlow2-PASS anchor bundle on the mesh bcanchor3 recipe (coef 3.0), lower-only diet. bc_anchor_lower chain is bank-green at 100 Hz as of this cycle (test_bc_anchor.py 56/56 incl. lower_anchor_chain_descends, tag exp/bcanchor-chain-tests-rate-fix). Prediction-if-true: det lower tracks the commanded descent with feet planted and no over_current. Prediction-if-false: aloft/outrigger residue or belly-crash lowering (the mesh bank's known thrash-collapse luck), or the dig-in's named residual risk bites -- the 25-55mm crouch is intrinsically hot on mesh (~2.2A one knee even honest; honest -44 vs grind -27 at chosen pricing) -- in which case the fork is goal.lower_height_mm mesh recalibration (belly-rest, servos unloaded), not more pricing. Strongest alternative: descends but with a flag leg (stratified+foot_z should prevent it; watch the strips).

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition or close a behavior/reward class at this checkpoint. 2M, lower DR-0 det n=6: >=4/6 honest descents (>=60% of commanded drop, every foot within ~25mm of ground throughout, zero over_current terms) = PASS (fund acquisition); partial/slow descents trending = PARTIAL; 0/6 with aloft/outrigger or crash-collapse signature = anchor-lower refuted on mesh, fork to goal.lower_height_mm belly-rest recalibration per the 08-25 dig-in fallback.

**verdict**: CANARY PASS — anchor-lower works on mesh: first honest sit-down. DR-0 det 6/6 honest descents: full commanded drop (height_err_end 0.1-3.7mm, >>60% bar), zero terminations, zero over_current, roll clean (peak <=1.0deg), det video = level six-foot descent from plant to crouch, feet grounded throughout, held to truncation. Rung-8 lower read confirms the footlow2 anchor bundle (bc_anchor_lower IK-descent chain, coef 3.0) transfers to the mesh model, sibling to the hold rung's bcanchor3 result. Honest caveats: (1) sto 0/6, all fell (tilt/over_current) — the exact un-annealed policy_std~1.0 signature the hold rung had, which stdanneal proved fixable (sto 0/6->6/6 by anneal alone); (2) the crouch is hot as the dig-in predicted: det cur_max 2.17-2.26A, cur_s_above_soft up to 10.2s — no over_current term in det but watch whether anneal cools it like it did hold (0.53->0.44A); belly-rest goal.lower_height_mm recalibration stays the fallback if acquisition can't. Own-DR(0.2) det 6/6 ok, sto 0/6 same signature. Next: pre-registered acquisition funded — 8M lower-only + log-std anneal to -4.0 warm-started from this checkpoint, mirroring the hold stdanneal recipe. NOTE: watcher SUSPECT (rc=2, 'W&B no longer running') was a budget-complete finalization false alarm — clean sync at 2,031,616/2M steps.


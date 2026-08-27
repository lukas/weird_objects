# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor13-walkretaincoef1-base

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T17:18:29+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor12-walkretain-base

**wandb_id**: ja1hfozq

**hypothesis**: Plain English: anchor12's own read this cycle showed the SAME anchor coefficient (3.0, the stance dose) pulling a healthy walk off its gait when applied to walk ticks too (DR-0 gate seed0: prog_ratio 0.38->0.13, slip/m 3.63->8.27, sto lower 5/6->1/6) on the walk-clean anchor2 recipe. This arm is anchor12-walkretain-base's exact recipe with ONE change: train.bc_anchor_walk_coef=1.0 (the walk teacher stotight45-seed13's own training dose) instead of implicitly sharing the stance modes' coef=3.0 -- same init-from anchor2's checkpoint, same bc_anchor_walk=1/phase_lock=1/knee_abs=1. Prediction-if-true: a lighter, decoupled walk dose keeps walk near anchor2's own band (prog_ratio >=0.30, slip/m <=~5) while still measurably anchoring (train/bc_anchor_loss_walk nonzero, less than anchor12's own walk-mode loss). Prediction-if-false: walk still regresses by a similar magnitude even at 1/3 the dose -> the anchor's mere PRESENCE (any nonzero coef) taxes walk on this recipe, not just its magnitude, and the reward/task-level route named in anchor9's REFUTED branch becomes the next audit.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY at 2M, joint call with -s1 twin; matched controls = anchor2{,-s1} own DR-0/own-DR AND anchor12-walkretain-base{,-s1}'s own just-recorded FAIL numbers (coef=3.0-for-walk-too). WIRING CHECK: train/bc_anchor_loss_walk nonzero, train/bc_anchor_fill_walk nonzero (proves the split branch actually fired, not a silent no-op). PASS if on BOTH seeds walk det DR-0 prog_ratio >= anchor2's 0.32-0.38 band (within eval noise) AND slip/m within ~15% of anchor2's AND hold/lower/rise stay in anchor2's band -- the decoupled dose removes the cost anchor12 measured. PARTIAL if walk improves markedly vs anchor12-walkretain-base's own numbers (narrows most of the gap to anchor2) but does not fully close it -- fund a still-lighter dose (0.3) next. FAIL if walk regresses from anchor2 by a SIMILAR magnitude to anchor12-walkretain-base despite the 3x-lighter dose -- the anchor's presence itself is the cost driver on this recipe, not its magnitude; escalate to the reward/task-level audit anchor9's REFUTED branch names, not another dose point. Read reward trend per 08-21 first.


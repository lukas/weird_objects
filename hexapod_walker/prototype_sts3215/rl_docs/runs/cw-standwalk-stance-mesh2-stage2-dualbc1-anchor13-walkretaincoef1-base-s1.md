# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor13-walkretaincoef1-base-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T17:22:44+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor12-walkretain-base-s1

**wandb_id**: q8e946s1

**hypothesis**: Seed1 twin of anchor13-walkretaincoef1-base: same single-lever change (train.bc_anchor_walk_coef=1.0, decoupled from stance's 3.0) on anchor2-s1's walk-clean checkpoint, joint call with the seed0 twin per that arm's own gate text.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as cw-standwalk-stance-mesh2-stage2-dualbc1-anchor13-walkretaincoef1-base (read that run's ledger text) -- this is the seed1 half of the joint 2-seed call, matched against anchor2-s1/anchor12-walkretain-base-s1's own numbers.

**verdict**: CANARY FAIL - MECHANISM (seed1 half of the joint 2-seed call; mechanism-health scope only). Plain English: on this seed the LIGHTER walk-anchor dose (1.0 instead of 3.0) made the walk dramatically WORSE, not better -- the robot ends up stepping nearly in place -- which kills the whole 'just lower the dose' idea. WIRING CHECK PASSED: train/bc_anchor_loss_walk (last 0.0029) and train/bc_anchor_fill_walk (~42k/rollout) nonzero -- the split branch fired. DR-0 gate walk/det vs matched controls (same seed, same init from anchor2-s1's checkpoint, only lever = walk anchor dose): anchor2-s1 (no walk anchor) prog 0.32 / slip 4.07; anchor12-s1 (coef=3.0) prog 0.16 / slip 6.47; THIS RUN (coef=1.0) prog 0.02 / slip 43.07, fwd 0.09m -- WORSE than the 3x-heavier control by a wide margin, an inverted dose-response. gait_valid 6/6 / sac [] are trivially satisfied by near-static leg cycling; video (walk_det strips) confirms upright six-leg stance stepping in place with heavy foot slip, no falls, no collapse. Stance modes stayed in band (rise/lower det clean, hold sto terms 6/6 same as controls); own-DR walk det 0.10/10.88. Reward trace matches the lineage-standard log-std-anneal trough -- no 08-21 undertrained read at canary scope with matched-budget controls healthy. JOINT CALL (with seed0's 0.18/5.49 partial-recovery-at-best): pre-registered FAIL branch confirmed -- no monotone dose-response across seeds, the anchor's PRESENCE on walk ticks (not its magnitude) taxes the walk, with strong seed instability. Next per the pre-registered FAIL text: reward/task-level audit named by anchor9's REFUTED branch; do NOT fund more dose points (0.3 is explicitly dead). Wave-level route decision joins the anchor14 rescue twins' read (watcher fan-out).


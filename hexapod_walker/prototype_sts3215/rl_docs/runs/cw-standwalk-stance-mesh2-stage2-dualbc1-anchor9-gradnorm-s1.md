# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor9-gradnorm-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T12:58:24+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor8-advstats-s1

**wandb_id**: dzhhyrhv

**hypothesis**: Seed1 half of the anchor9-gradnorm diagnostic pair (see seed0 for the full hypothesis text). Same read-only bc_anchor_debug_gradnorm=1 swap on top of the EXACT anchor6b-logstdsplit-fix-s1 recipe lineage (same seed, same init-from anchor2_s1) -- this is the seed whose walk previously went to TOTAL catastrophe (fix-s1: 0/6 gait_valid, 3-5 legs sacrificed) and whose anchor8-advstats read ALSO refuted advantage-normalization-scope (adv_loco_std up to ~6x adv_stance_std at the trough, same inverted direction as seed0) -- its grad-norm read is the more informative half: if shared-clip coupling is real, THIS seed (the one that fully collapsed) should show the sharpest a/b grad-norm divergence.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor9-gradnorm (seed0): DIAGNOSTIC-ONLY, no training-behavior change. WIRING CHECK FIRST (nonzero train/gradnorm_a_mean + train/gradnorm_b_mean present in the cached W&B history, paired by call-order not by _step). Joint read with seed0: SUPPORTED if gradnorm_a_mean is persistently several-x gradnorm_b_mean (or vice versa) on BOTH seeds especially during the reward trough -- design a per-core gradient clip next; REFUTED if the two stay comparable on either/both seeds -- escalate to a reward/task-level audit of the walk objective, every architecture-sharing candidate now tested.


# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor8-advstats-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-27T12:23:11+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: xqi6b5dc

**hypothesis**: Seed1 half of the anchor8-advstats diagnostic pair (see seed0 for the full hypothesis text). Same read-only bc_anchor_debug_adv_stats=1 addition on top of the EXACT anchor6b-logstdsplit-fix-s1 recipe (same seed, same init-from anchor2_s1, no detach_trunk) -- this is the seed whose walk previously went to TOTAL catastrophe (fix-s1: 0/6 gait_valid, 3-5 legs sacrificed, prog ~0.004-0.009), so its diagnostic read is the more informative half: if the advantage-normalization-scope hypothesis is right, this seed's train/adv_stance_std should be large relative to train/adv_loco_std for MOST/ALL of training (not just briefly), plausibly coincident with or preceding the walk collapse.,

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor8-advstats (seed0): DIAGNOSTIC-ONLY, no training-behavior change. WIRING CHECK FIRST (nonzero train/adv_loco_share + train/adv_stance_share present in the cached W&B history). Joint read with seed0: SUPPORTED if train/adv_stance_std is persistently several-x train/adv_loco_std on BOTH seeds (fund a per-mode-group advantage-normalization mechanism arm next); REFUTED if the two stds stay comparable on either/both seeds (escalate to the critic/value-function coupling candidate instead).


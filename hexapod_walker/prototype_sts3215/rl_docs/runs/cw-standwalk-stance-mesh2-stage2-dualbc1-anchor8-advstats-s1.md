# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor8-advstats-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM - JOINT CLOSE

**created**: 2026-08-27T12:23:11+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6b-logstdsplit-fix-s1

**wandb_id**: xqi6b5dc

**hypothesis**: Seed1 half of the anchor8-advstats diagnostic pair (see seed0 for the full hypothesis text). Same read-only bc_anchor_debug_adv_stats=1 addition on top of the EXACT anchor6b-logstdsplit-fix-s1 recipe (same seed, same init-from anchor2_s1, no detach_trunk) -- this is the seed whose walk previously went to TOTAL catastrophe (fix-s1: 0/6 gait_valid, 3-5 legs sacrificed, prog ~0.004-0.009), so its diagnostic read is the more informative half: if the advantage-normalization-scope hypothesis is right, this seed's train/adv_stance_std should be large relative to train/adv_loco_std for MOST/ALL of training (not just briefly), plausibly coincident with or preceding the walk collapse.,

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor8-advstats (seed0): DIAGNOSTIC-ONLY, no training-behavior change. WIRING CHECK FIRST (nonzero train/adv_loco_share + train/adv_stance_share present in the cached W&B history). Joint read with seed0: SUPPORTED if train/adv_stance_std is persistently several-x train/adv_loco_std on BOTH seeds (fund a per-mode-group advantage-normalization mechanism arm next); REFUTED if the two stds stay comparable on either/both seeds (escalate to the critic/value-function coupling candidate instead).

**verdict**: CANARY FAIL - MECHANISM - JOINT CLOSED (confirms seed0): diagnostic-only, seed1 own read. WIRING CHECK FIRST confirmed (train/adv_loco_share ~0.72 stance / rest loco every rollout, train/adv_{loco,stance}_std present all 30 train() calls, paired by call-order per the anchor8 seed0 pitfall note, not by wandb _step). Read: adv_stance_std is NEVER several-x adv_loco_std -- the OPPOSITE holds for most of training, especially the reward-trough window (rollout idx 6-13, this runs own reward quarters [40.0,-7.6,-466.6,-112.6], closely matching seed0s [38.4,21.0,-287.6,-49.9] shape): adv_loco_std spikes to 37-51 while adv_stance_std stays 4.5-7.7 (ratio as low as 0.13-0.17x, i.e. loco dominates stance by 6-8x at the trough, not the reverse). Only rollout 0-1 show comparable magnitude. This replicates seed0s already-recorded REFUTED read almost exactly (seed0: adv_loco_std up to 30-42 vs adv_stance_std 5-9, ratio up to ~4-5x the wrong way at the same trough window). Per this arms own pre-registered decision rule (REFUTED if the two stay comparable on either/both seeds), seed1 independently refutes on top of seed0 -- sb3-contribs shared global advantage normalizer is NOT the walk-starvation mechanism on either seed, joint-closed. No new action from this run: the chain already escalated past this (anchor9-gradnorm -> anchor10-percoreclip -> anchor11/12/13-walkretain -> anchor14-walkretaincoef1-rescue, the last of which just JOINT-PASSED its own 8M acquisition read this same cycle). This is a housekeeping close of a diagnostic left unverdicted from an earlier cycle, not a new finding or a blocker to anything currently funded.


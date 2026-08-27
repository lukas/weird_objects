# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor9-gradnorm-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T12:58:24+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor8-advstats-s1

**wandb_id**: dzhhyrhv

**hypothesis**: Seed1 half of the anchor9-gradnorm diagnostic pair (see seed0 for the full hypothesis text). Same read-only bc_anchor_debug_gradnorm=1 swap on top of the EXACT anchor6b-logstdsplit-fix-s1 recipe lineage (same seed, same init-from anchor2_s1) -- this is the seed whose walk previously went to TOTAL catastrophe (fix-s1: 0/6 gait_valid, 3-5 legs sacrificed) and whose anchor8-advstats read ALSO refuted advantage-normalization-scope (adv_loco_std up to ~6x adv_stance_std at the trough, same inverted direction as seed0) -- its grad-norm read is the more informative half: if shared-clip coupling is real, THIS seed (the one that fully collapsed) should show the sharpest a/b grad-norm divergence.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate text as anchor9-gradnorm (seed0): DIAGNOSTIC-ONLY, no training-behavior change. WIRING CHECK FIRST (nonzero train/gradnorm_a_mean + train/gradnorm_b_mean present in the cached W&B history, paired by call-order not by _step). Joint read with seed0: SUPPORTED if gradnorm_a_mean is persistently several-x gradnorm_b_mean (or vice versa) on BOTH seeds especially during the reward trough -- design a per-core gradient clip next; REFUTED if the two stay comparable on either/both seeds -- escalate to a reward/task-level audit of the walk objective, every architecture-sharing candidate now tested.

**verdict**: CANARY FAIL - MECHANISM - JOINT DIVERGENCE: seed1's own gradnorm read does NOT replicate seed0's 'persistently several-x' pattern that the concurrent cycle already used to launch anchor10-percoreclip -- under this run's own pre-registered decision rule ('REFUTED if the two stay comparable on either/both seeds') seed1 alone already fails the SUPPORT bar. Evidence: WIRING CHECK FIRST passed (train/gradnorm_{a,b}_mean present at all 30 train() calls, nonzero, paired by call-order not _step, matching the anchor8 read-pitfall note). Full 30-rollout b/a (stance/walk) ratio: median 1.95x, mean 2.83x (skewed by two single-rollout outliers at idx0=6.25x and idx6=12.4x where core-a's raw norm collapses to a near-zero 3.3); only 10/30 rollouts clear the >=3x bar, 20/30 sit inside a comparable +/-3x band. Critically, AT THE EXACT REWARD-TROUGH WINDOW (rollout idx 6-13 of 30, this run's own reward quarters [40.0,-8.7,-460.3,-112.0]) the pattern is comparable-to-INVERTED: b/a = [12.4(outlier), 0.73, 0.73, 1.62, 0.88, 0.71, 0.56, 0.63], median 0.73 -- core A (walk) is usually the LARGER raw gradient there, opposite of seed0's reported 3.5x stance-dominant trough. Why this matters: seed0 (median 4.2x, 23/30 rollouts >=3x, persistent through the trough) was already treated by the concurrent cycle as sufficient SUPPORT to build+launch train.bc_anchor_percore_clip and start cw-standwalk-stance-mesh2-stage2-dualbc1-anchor10-percoreclip, without waiting for this seed's read -- reasonable given the diagnostic is cheap and anchor10 is itself a direct behavioral test, but the strict joint rule this arm pre-registered is NOT met (seed1 is comparable/inverted, not several-x). What's next: do not treat anchor10-percoreclip's premise as doubly-confirmed by gradnorm evidence -- it rests on seed0 alone. If anchor10-percoreclip's own behavioral read shows walk still fails, the correct next step per this run's own REFUTED branch is to escalate straight to a reward/task-level audit of the walk objective (every architecture-sharing/optimizer-coupling candidate -- log_std, trunk, advantage-norm, grad-clip -- will then have been tested with at least one seed contradicting each), not another per-core-isolation variant. Evidence: logs/experiments/cw-standwalk-stance-mesh2-stage2-dualbc1-anchor9-gradnorm-s1/wandb_history.csv (train/gradnorm_{a,b}_mean, paired by call order).


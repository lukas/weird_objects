# cw-standwalk-stance-mesh2-holdheight-rung1-hha1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T21:45:40+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1-s1

**wandb_id**: gfksq1nx

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-holdheight-rung1-hha1 (same recipe, seed 1): does the height-aware BC anchor fix (bc_anchor_coef restored 0.0->3.0 + train.bc_anchor_hold_height_aware=1, FixedFootBodyIK target at the next commanded height instead of the constant q_nom) replicate cross-seed, not just for one init? Same fix, same holdheight-rung1-s1 FAIL evidence.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as the seed-0 twin, judged jointly as a pass-rate pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load terminations, cur_max within noise of the champion's 0.67-0.71A band.


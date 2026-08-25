# cw-standwalk-stance-mesh2-holdheight-rung1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T20:52:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal

**wandb_id**: roplbk1z

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-holdheight-rung1 (same recipe, seed 1): does the already-solved quiet-stand hold champion track a slowly moving commanded body height (rung-1 elevator, +/-15mm at 8mm/s) cross-seed, not just for one init? Same design/preflight-bank evidence as the seed-0 twin; see rl_docs/tracks/standwalk/STAND_HEIGHT.md.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as the seed-0 twin, judged jointly as a pass-rate pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load/walk_low_height terminations, bounded height tracking on video.


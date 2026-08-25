# cw-standwalk-stance-mesh2-holdheight-rung1-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-25T20:52:41+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal

**wandb_id**: roplbk1z

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-holdheight-rung1 (same recipe, seed 1): does the already-solved quiet-stand hold champion track a slowly moving commanded body height (rung-1 elevator, +/-15mm at 8mm/s) cross-seed, not just for one init? Same design/preflight-bank evidence as the seed-0 twin; see rl_docs/tracks/standwalk/STAND_HEIGHT.md.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as the seed-0 twin, judged jointly as a pass-rate pair: DR-0 det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load/walk_low_height terminations, bounded height tracking on video.

**verdict**: CANARY FAIL - MECHANISM: dropping the hold pose anchor entirely (train.bc_anchor_coef forced 0.0, to avoid the height-blind q_nom target fighting a moving command) broke the champion's solid quiet-stand skill, not just its height-blindness. Evidence: DR-0 gate (hold_height_cmd_frac=1.0 rides into the eval per pod_eval's own-cfg-set carry-forward) hold det 1/6 valid_plant + sto 1/6 (5/6 det and 5/6 sto episodes trip the hold_min_load safety termination ~3-8s into the 15s episode); own-DR(0.2) det 2/6 + sto 1/6. cur_max climbs to 2.0-2.63A (near the 2.64A torque-saturation ceiling) vs the deployed champion's clean 0.67-0.71A on the identical static hold, and drag 75-191mm vs ~0mm. Video contact sheets show NO visible fall/tip (roll_peak 0.6-2.8deg, all six feet visibly grounded every frame) -- the failure is a genuine but visually-subtle foot-underload trip, not a collapse (gate-vs-video read: real, just not dramatic). Training reward DECLINED across the run (optimization/reward_per_tick 0.42->0.24), so per the 08-21 ruling this is a genuine fail, not misalignment-continue. Cross-seed: the seed-0 twin (cw-standwalk-stance-mesh2-holdheight-rung1, read independently this cycle from its own synced report.json, verdict owned by the concurrent cycle) shows the byte-similar signature (cur_max 2.59A, 5/6 det + 3/6 sto hold_min_load terminations) -- same mechanism, not seed noise. Why: STAND_HEIGHT.md's own design doc named this exact fallback ('if [dropping the anchor] proves insufficient, the correct fix is a height-AWARE anchor ... not the height-blind one') -- the height-blind anchor was doing real pose-regularization work beyond its height-blindness, and removing it let PPO drift into a noisier, higher-current stance. What's next: built + tested the fix this cycle (train.bc_anchor_hold_height_aware, default 0=bit-exact, rl_move/sim/sim_env.py) -- re-targets the hold BC-anchor at the FixedFootBodyIK pose that reaches the NEXT commanded height (same one-tick-ahead convention bc_anchor_lower already uses) instead of the constant height-blind q_nom, so the anchor keeps supervising pose quality while tracking a moving target. 5 new tests in test_bc_anchor.py (default-off bit-exact, zero-height no-op, targets-the-commanded-height, track-mode-excluded, feet-planted-descent chain), all green; full bc_anchor + hold_height_cmd suites green (84/84); STAND_HEIGHT.md's own preflight bank unaffected (4/4 green). Launching the fix canary pair (bc_anchor_coef restored to 3.0 + bc_anchor_hold_height_aware=1.0, same hold_height_cmd_range/rate/kinds) this cycle.


# cw-standwalk-stance-mesh2-holdheight-rung1-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-25T21:41:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1

**wandb_id**: isxy1d5b

**hypothesis**: holdheight-rung1/-s1 (both seeds) CANARY FAIL-MECHANISM: dropping the hold BC-anchor entirely (to dodge its height-blind q_nom target fighting a moving height command) broke the champion's quiet-stand skill on the STATIC gate too (cur_max 2.0-2.63A vs champion 0.67A, 5/6 det hold_min_load trips) -- the anchor was doing real pose regularization beyond its height-blindness. Does restoring the anchor (bc_anchor_coef 0.0->3.0) with a NEW height-AWARE target (train.bc_anchor_hold_height_aware=1, FixedFootBodyIK solved at the next commanded height instead of the constant q_nom -- rl_move/sim/sim_env.py, 5 tests green in test_bc_anchor.py) recover the clean quiet-stand current/load profile while still tracking the moving command? Same hold_height_cmd_range_mm/rate/kinds (rung-1 elevator, +/-15mm at 8mm/s, hold+ramp only) and every other cfg unchanged from holdheight-rung1.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. DR-0 gate (hold_height_cmd_frac rides into the eval) det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load terminations, cur_max within noise of the champion's 0.67-0.71A band (not the 2.0-2.63A failure signature) -- judged jointly with the -s1 seed twin as a pass-rate pair against holdheight-rung1/-s1's own numbers.

**verdict**: CANARY PASS - MECHANISM (seed 0 of the joint pair; final joint pass-rate call lands with -s1). The height-aware BC anchor fixed the flag-leg cheat: the robot now holds a clean six-foot stand while riding the +/-15mm commanded-height elevator. DR-0 gate 6/6 det + 6/6 sto valid_plant (needed 5/6+4/6), ZERO hold_min_load terminations (parent recipe: 8/12), height_err_end 0.0-2.6mm, all six legs duty 0.91-1.0 with no per-leg sacrifice (parent flagged one leg at duty 0.23-0.86), det Imax 0.66-1.03A vs the parent's 2.0-2.63A failure signature -- modestly above the champion's static 0.67-0.71A band, plausibly the honest cost of the moving height command. Video: level, planted, quiet. Reward rose all run (quarters 12.9/28.6/74.2/105.8). CAVEAT outside the registered gate: at own-DR 0.2, 2/6 det episodes still trip hold_min_load with transient duty dips (0.4-0.66) and 2.5A spikes -- DR hardening is real remaining work for the next rung, not a canary failure. Next: -s1 cycle makes the joint pass-rate call; on joint PASS, proceed up the STAND_HEIGHT rung ladder with bc_anchor_hold_height_aware=1 as the recipe default. Watcher SUSPECT at 21:52 was a false alarm (fired during final flush; run finished cleanly at 2.03M).


# cw-standwalk-stance-mesh2-holdheight-rung1-hha1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T21:41:20+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdheight-rung1

**wandb_id**: isxy1d5b

**hypothesis**: holdheight-rung1/-s1 (both seeds) CANARY FAIL-MECHANISM: dropping the hold BC-anchor entirely (to dodge its height-blind q_nom target fighting a moving height command) broke the champion's quiet-stand skill on the STATIC gate too (cur_max 2.0-2.63A vs champion 0.67A, 5/6 det hold_min_load trips) -- the anchor was doing real pose regularization beyond its height-blindness. Does restoring the anchor (bc_anchor_coef 0.0->3.0) with a NEW height-AWARE target (train.bc_anchor_hold_height_aware=1, FixedFootBodyIK solved at the next commanded height instead of the constant q_nom -- rl_move/sim/sim_env.py, 5 tests green in test_bc_anchor.py) recover the clean quiet-stand current/load profile while still tracking the moving command? Same hold_height_cmd_range_mm/rate/kinds (rung-1 elevator, +/-15mm at 8mm/s, hold+ramp only) and every other cfg unchanged from holdheight-rung1.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. DR-0 gate (hold_height_cmd_frac rides into the eval) det>=5/6 + sto>=4/6 valid_plant, zero hold_min_load terminations, cur_max within noise of the champion's 0.67-0.71A band (not the 2.0-2.63A failure signature) -- judged jointly with the -s1 seed twin as a pass-rate pair against holdheight-rung1/-s1's own numbers.


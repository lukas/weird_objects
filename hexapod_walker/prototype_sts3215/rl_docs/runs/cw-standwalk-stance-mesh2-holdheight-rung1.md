# cw-standwalk-stance-mesh2-holdheight-rung1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T20:48:17+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-holdminload40-bcanchor3-stdanneal

**wandb_id**: tp53jcv8

**hypothesis**: Can the already-solved quiet-stand hold champion track a SLOWLY MOVING commanded body height (a rung-1 joystick up/down elevator, +/-15mm at 8mm/s, ramp/hold segments only) instead of just holding at a fixed height, while its existing solid-stand gate (hold_still_gate/hold_feet_load_min/hold_flag_fade) keeps pricing it? Operator MCP request fb_20260825T195117_3dce6e asked for a commandable standing-height controller; this codebase already has every piece proven in isolation (the S-gate on hold, the height-tracking kernel, the loaded-foot slip charge) except a height_ref that MOVES -- new cfg goal.hold_height_cmd_frac (default off, bit-exact) adds exactly that to the hold goal kind (rl_move/sim/goal_task.py _hold_height_schedule), reusing the existing reward stack unchanged. Warm-started from the deployed champion (ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_stdanneal) WITHOUT train.bc_anchor_coef (that anchor targets a height-BLIND fixed pose and would fight a moving command -- bc_anchor_coef forced to 0.0 here, overriding the parent's 3.0). Preflight bank (test_task_semantics.py, HOLD bank COMMANDABLE HEIGHT variant) is GREEN on the exact pinned schedule: honest six-foot IK tracking beats refusing the command by a wide margin (2551 vs -67 at -30mm, 1269 vs 126 at +20mm), a flag leg/hovering foot that nominally reaches the same height earns under half of honest tracking, and slip-during-adjustment loses to a quiet planted adjustment under k_drag_trans. Full design: rl_docs/tracks/standwalk/STAND_HEIGHT.md, REWARD.md sec 4d.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY: does the champion's static-hold skill survive a moving height target at all, without the fixed-pose BC anchor? PASS if DR-0 gate det>=5/6 + sto>=4/6 valid_plant (no regression vs the champion's own 24/24 static band beyond canary noise), zero hold_min_load/walk_low_height-style terminations, and height tracking error stays visibly bounded on video/contact-sheet during the commanded ramps (not a freeze-at-old-height or a flag-leg cheat). PARTIAL if valid_plant holds but tracking is sluggish/overshoots (extend budget or lower the rate before widening range). FAIL if valid_plant regresses below the champion's own band or a flag/hover-style cheat appears on video -- then either the bc_anchor omission was wrong (needs a height-aware anchor, see STAND_HEIGHT.md) or the S-gate needs strengthening for a moving target, not more budget.


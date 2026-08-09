# cw-walk-speedband2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:09:42+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip

**wandb_id**: tcip7dk0

**hypothesis**: RETRY 1 of cw-walk-speedband2 (died at env reset on train-4: /dev/shm full of leaked segments — 0 steps trained; fresh relaunch). OPERATOR WISHLIST 8b (operator-tunable speed), reissue of the c43-killed cw-walk-speedband. One policy tracking the whole 0.02-0.12 m/s band so speed is a runtime knob, not a retrain. One variable off wander30 (PASS: 30s driving, resample/5s, 15% stops): speed band 0.05-0.06 -> 0.02-0.12 m/s. Plain: the operator's joystick should set the pace, slow creep to double speed, without falls or parking. Prediction-if-true: own-cfg gv 12/12, 0 term, prog_ratio med 0.8-1.2, fast-command eps (>=0.09 m/s) keep prog >=0.6. Prediction-if-false: band edges break - fast commands trip the paddle gait or slow commands park = band needs splitting or a curriculum. Strongest alternative: policy averages the band (walks ~0.06 regardless of command) - check per-episode speed vs command correlation. Parent: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip.

**gate**: own-cfg DR0 30s 6+6: gv 12/12, 0 term, prog_ratio med 0.8-1.2, no non-stall ep prog<0.5, per-ep speed tracks command across the band (fast eps prog >=0.6); frames watched det+sto


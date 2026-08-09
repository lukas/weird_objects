# cw-walk-speedband2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T17:03:20+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip

**hypothesis**: RETRY 1 (first launch died at reset: train-4 /dev/shm full of leaked segments; relaunched on train-10). OPERATOR WISHLIST 8b (operator-tunable speed), reissue of the c43-killed cw-walk-speedband. One policy tracking the whole 0.02-0.12 m/s band so speed is a runtime knob, not a retrain. One variable off wander30 (PASS: 30s driving, resample/5s, 15% stops): speed band 0.05-0.06 -> 0.02-0.12 m/s. Plain: the operator's joystick should set the pace, slow creep to double speed, without falls or parking. Prediction-if-true: own-cfg gv 12/12, 0 term, prog_ratio med 0.8-1.2, fast-command eps (>=0.09 m/s) keep prog >=0.6. Prediction-if-false: band edges break - fast commands trip the paddle gait or slow commands park = band needs splitting or a curriculum. Strongest alternative: policy averages the band (walks ~0.06 regardless of command) - check per-episode speed vs command correlation. Parent: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip.

**gate**: own-cfg DR0 30s 6+6: gv 12/12, 0 term, prog_ratio med 0.8-1.2, no non-stall ep prog<0.5, per-ep speed tracks command across the band (fast eps prog >=0.6); frames watched det+sto

**verdict**: INFRA FAILURE, no training happened (0 steps): same train-4 /dev/shm leak as cw-walk-lowgait-dr05's first attempt. Retried as cw-walk-speedband2-r1 (VERIFIED RUNNING on train-2). Not a hypothesis result.

**refused_reason**: W&B already has a run named cw-walk-speedband2 (names are append-only; pick a new one)


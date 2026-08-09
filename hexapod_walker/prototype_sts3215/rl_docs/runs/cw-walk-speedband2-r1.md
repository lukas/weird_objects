# cw-walk-speedband2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:09:42+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip

**wandb_id**: tcip7dk0

**hardware_ready**: no

**hypothesis**: RETRY 1 of cw-walk-speedband2 (died at env reset on train-4: /dev/shm full of leaked segments — 0 steps trained; fresh relaunch). OPERATOR WISHLIST 8b (operator-tunable speed), reissue of the c43-killed cw-walk-speedband. One policy tracking the whole 0.02-0.12 m/s band so speed is a runtime knob, not a retrain. One variable off wander30 (PASS: 30s driving, resample/5s, 15% stops): speed band 0.05-0.06 -> 0.02-0.12 m/s. Plain: the operator's joystick should set the pace, slow creep to double speed, without falls or parking. Prediction-if-true: own-cfg gv 12/12, 0 term, prog_ratio med 0.8-1.2, fast-command eps (>=0.09 m/s) keep prog >=0.6. Prediction-if-false: band edges break - fast commands trip the paddle gait or slow commands park = band needs splitting or a curriculum. Strongest alternative: policy averages the band (walks ~0.06 regardless of command) - check per-episode speed vs command correlation. Parent: rl_move/sim/policies/ppo_goal_cw_walk_wander30.zip.

**gate**: own-cfg DR0 30s 6+6: gv 12/12, 0 term, prog_ratio med 0.8-1.2, no non-stall ep prog<0.5, per-ep speed tracks command across the band (fast eps prog >=0.6); frames watched det+sto

**verdict**: FAIL — and the question was already closed. Speed pinned at 0.05-0.06 m/s across the whole 0.02-0.12 command band: overshoots slow commands (sto prog 1.39 at 0.023 m/s cmd), undershoots fast ones (prog 0.38-0.61 at 0.08-0.09 cmd), det prog med 0.65 (<0.8 gate). gv 12/12, 0 falls, det+sto frames watched (upright, legs cycling) — the gait survives, tracking doesn't. Confirms cw-walk-fast's gait ceiling (~0.065 m/s) that killed the original speedband (c43) and speedband-r1's CLOSED-no-requeue on the achievable band; this spec was a stale c54 reissue of the refuted 0.02-0.12 band (name-dedupe missed the class again, cf. slowband). 8b speed-knob line stays CLOSED pending operator contact-pricing calibration.


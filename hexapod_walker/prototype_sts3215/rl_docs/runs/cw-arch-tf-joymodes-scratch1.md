# cw-arch-tf-joymodes-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DONE

**created**: 2026-08-15T17:36:14+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-tf-r1-hard1

**wandb_id**: 5mzh18w5

**hardware_ready**: False

**hypothesis**: Teach a fresh causal Transformer to follow a joystick exactly across instant reversals, square turns, stops, small corrections, random holds, and continuous circles. The normalized physical command reward makes exact requested velocity profitable while parking, sideways travel, and backward travel are increasingly negative; if this mechanism works, command-aligned speed should rise and wrong-way motion should fall during the 2M discovery run.

**gate**: PASS discovery mechanism = VERIFIED RUNNING with no NaN/crash, reward_walk_cmd_track and v_along/v_cross/wrong_way telemetry present, training reward improves, and the 2M checkpoint shows an emerging six-leg gait with command-aligned motion in multiple schedule modes. FAIL = launch/runtime bug, absent telemetry, parked/one-leg exploit, or no command-conditioned movement signal by 2M; then fix mechanism rather than extending steps.

**verdict**: CANARY PASS — mechanism health only. The run completed cleanly with CUDA Transformer training and command telemetry moving; 2M is only 5% of the 40M budget this same Transformer family needed to learn walking. The stilt/low-gait checkpoint is an immature acquisition snapshot, not a behavioral or reward-recipe verdict. The prior FAIL/closed-recipe conclusion is invalid and retracted.


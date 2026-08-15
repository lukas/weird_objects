# cw-arch-tf-joymodes-scratch1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-15T17:34:28+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-arch-tf-r1-hard1

**hypothesis**: Teach a fresh causal Transformer to follow a joystick exactly across instant reversals, square turns, stops, small corrections, random holds, and continuous circles. The normalized physical command reward makes exact requested velocity profitable while parking, sideways travel, and backward travel are increasingly negative; if this mechanism works, command-aligned speed should rise and wrong-way motion should fall during the 2M discovery run.

**gate**: PASS discovery mechanism = VERIFIED RUNNING with no NaN/crash, reward_walk_cmd_track and v_along/v_cross/wrong_way telemetry present, training reward improves, and the 2M checkpoint shows an emerging six-leg gait with command-aligned motion in multiple schedule modes. FAIL = launch/runtime bug, absent telemetry, parked/one-leg exploit, or no command-conditioned movement signal by 2M; then fix mechanism rather than extending steps.

**refused_reason**: hexapod-mjx-train-1 code marker 4c723840ae847a63c4b579da7cf7055c70aefab6 != local HEAD ea25552a3d29516e37b96fda7715298424183ecb. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).


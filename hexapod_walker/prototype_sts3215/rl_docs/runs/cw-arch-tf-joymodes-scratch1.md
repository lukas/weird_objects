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

**verdict**: FAIL -- KNOWN EXPLOIT (leg-sacrifice/stilt), one-line STOP per RUN_INTERPRETATION_RULES check 1, no forensics. DR0 gate (the gated mode) det gait_valid 0/6 -- EVERY episode sacrifices 2 legs (idx [0,2] or [0,3]), prog_ratio med 0.28 (bar 0.85), slip/m 5.16, roll never settles (peak ~10deg, settled 0/6); own-cfg DR0.5 same signature (sacrificed [0,2], gv 4/6, prog 0.50, slip 5.94-7.13, roll settled 0/6). Video (both DR passes, all 12 det+12 sto episodes) shows two legs held rigid off the ground the whole clip while the body visibly tilts onto a 4-leg stilt stance -- a hardware-familiar cheat, not a new architecture finding. Telemetry confirms the deeper problem: train/v_along_ratio_active_cumulative 0.078 (barely any real command-aligned motion) and wrong_way_frac ~0.43-0.44 pinned, matching cw-joystick-translate1/cw-joystick-translate-scratch1's already-CLOSED signature almost exactly (same near-zero real movement, same ~0.43 wrong-way, same reward-quarters declining -398->-1258 as the policy discovers the parking/leg-sacrifice cheat instead of moving). CROSS-TRACK INSIGHT: this arm re-tests the SAME joystick command-tracking reward recipe multitask already closed (warm AND from-scratch both failed identically there), this time on the transformer trunk under the arch track banner -- it reproduces the identical failure mode a THIRD independent time (translate1 warm, translate-scratch1 from-scratch, this from-scratch-transformer), which is further confirmation the problem is the reward/command recipe itself, not init or architecture. Per the two-miss/closed-class rule this closes the question for a third lineage without a resweep; the fix belongs to whoever owns the reward recipe (multitask), not a per-architecture retry.


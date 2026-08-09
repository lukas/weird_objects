# cw-walk-stopgo35-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:38:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 12000000

**parent**: cw-walk-stopgo35

**wandb_id**: pzvca3a7

**hardware_ready**: no

**hypothesis**: REBALANCE continuation (not a new variable): cw-walk-stopgo35 was killed at ~8.5M/20M because node g142d86 was host-starved by a foreign tenant (load 110-146/128, fps 4.4k). Same hypothesis as parent: wander30 handles 15% stop segments; one variable off it, stop_frac 0.15->0.35 for dense park->restart cycling, park pricing unchanged. If-true: restarts stay reliable at high stop density (stop transitions solved, no shaping needed). If-false: policy rides parked segments or fails to restart cleanly (prog drop / stall after stops) - stop transitions need their own shaping arm.

**gate**: own-cfg DR0 30s det+sto 6/6: gait_valid 12/12, zero terminations, prog_ratio median 0.85-1.15, no ep prog<0.5; frames watched det for post-stop restarts

**verdict**: PASS (if-true): own-cfg DR0 gate 12/12 gv, 0 term, prog med 0.97 (min ep 0.92), det+sto frames clean — level body, six-leg cycling, quiet parks and prompt restarts across 35% stop density; stop transitions need no extra shaping. Det slip/m 1.43 (lineage transport slip, not gated) -> NOT hardware-ready (global skating defect). Checkpoint ppo_goal_cw_walk_stopgo35_c1.zip.


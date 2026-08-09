# cw-walk-stopgo35-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T16:38:16+00:00

**pod**: hexapod-mjx-train-2

**steps**: 12000000

**parent**: cw-walk-stopgo35

**wandb_id**: pzvca3a7

**hypothesis**: REBALANCE continuation (not a new variable): cw-walk-stopgo35 was killed at ~8.5M/20M because node g142d86 was host-starved by a foreign tenant (load 110-146/128, fps 4.4k). Same hypothesis as parent: wander30 handles 15% stop segments; one variable off it, stop_frac 0.15->0.35 for dense park->restart cycling, park pricing unchanged. If-true: restarts stay reliable at high stop density (stop transitions solved, no shaping needed). If-false: policy rides parked segments or fails to restart cleanly (prog drop / stall after stops) - stop transitions need their own shaping arm.

**gate**: own-cfg DR0 30s det+sto 6/6: gait_valid 12/12, zero terminations, prog_ratio median 0.85-1.15, no ep prog<0.5; frames watched det for post-stop restarts


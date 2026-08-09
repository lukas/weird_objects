# cw-walk-stopgo35

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T16:15:07+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-wander30

**wandb_id**: utc2kygs

**hypothesis**: Stop-and-go hardening (wishlist 7, joystick operability: the stick gets released a LOT). wander30 handles 15% stop segments; one variable off it: stop_frac 0.15→0.35 — dense park→restart cycling. Park pricing unchanged, so parking still earns less than stepping by construction. If-true: restarts stay reliable at high stop density (stop transitions are solved, no shaping needed). If-false: policy rides parked segments or fails to restart cleanly (prog drop / stall after stops) — stop transitions need their own shaping arm.

**gate**: own-cfg DR0 30s det+sto 6/6: gait_valid 12/12, zero terminations, prog_ratio median 0.85-1.15, no ep prog<0.5 (prog is vs commanded displacement so stops don't dilute it); frames watched det for post-stop restarts

**verdict**: REBALANCE, not a result: killed at ~8.5M/20M steps because node g142d86 is host-starved by a foreign tenant (host load 110-146/128 with only ~70 of ours; fps 4.4k at checkup vs 15.1k for a comparable run on healthy g129004; GPU util 0% sampled). No verdict on the stop_frac 0.35 hypothesis — continued unchanged as cw-walk-stopgo35-c1 from checkpoint md5 1fc1f2d4 on train-2 (g131eec).


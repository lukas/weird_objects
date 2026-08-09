# cw-walk-stopgo35

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T16:15:07+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-wander30

**hypothesis**: Stop-and-go hardening (wishlist 7, joystick operability: the stick gets released a LOT). wander30 handles 15% stop segments; one variable off it: stop_frac 0.15→0.35 — dense park→restart cycling. Park pricing unchanged, so parking still earns less than stepping by construction. If-true: restarts stay reliable at high stop density (stop transitions are solved, no shaping needed). If-false: policy rides parked segments or fails to restart cleanly (prog drop / stall after stops) — stop transitions need their own shaping arm.

**gate**: own-cfg DR0 30s det+sto 6/6: gait_valid 12/12, zero terminations, prog_ratio median 0.85-1.15, no ep prog<0.5 (prog is vs commanded displacement so stops don't dilute it); frames watched det for post-stop restarts


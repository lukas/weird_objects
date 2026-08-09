# cw-walk-endur60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T13:00:09+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: ml3y8z72

**hypothesis**: OPERATOR WISHLIST: long-distance endurance. 60 s episodes (4x champion horizon), narrow band. If-true: det median fwd distance >=2.4 m @ 60 s with gait_valid and no late-episode degradation; if-false: gait decays over time (fatigue-like drift in policy state or accumulating pose error).

**gate**: DR0 det+sto 6/6: median fwd distance >=2.4 m @ 60 s, zero terminations, gait_valid


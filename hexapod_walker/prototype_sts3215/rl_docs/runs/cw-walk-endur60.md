# cw-walk-endur60

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T12:39:26+00:00

**pod**: hexapod-mjx-train-10

**steps**: 20000000

**parent**: cw-walk-anchorgate

**hypothesis**: OPERATOR WISHLIST: long-distance endurance. 60 s episodes (4x champion horizon), narrow band. If-true: det median fwd distance >=2.4 m @ 60 s with gait_valid and no late-episode degradation; if-false: gait decays over time (fatigue-like drift in policy state or accumulating pose error).

**gate**: DR0 det+sto 6/6: median fwd distance >=2.4 m @ 60 s, zero terminations, gait_valid

**refused_reason**: hexapod-mjx-train-10 code marker c52dbc252e65add21de1cdf6c6c47e98be18e9d0 != local HEAD bdd50f9a20bf240eae68e879f073f6d5df6fa61e. Sync first: snapshot.sh --sync hexapod-mjx-train-10 (and snapshot/commit before that if the tree is dirty).


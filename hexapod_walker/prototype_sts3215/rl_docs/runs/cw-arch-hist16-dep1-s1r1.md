# cw-arch-hist16-dep1-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T20:15:45+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1

**hypothesis**: Plain English: seed-twin confirmation of the temporal-arch line's first hardware-contract rung (16-frame memory trained directly on the deployment obs contract) -- only tested at seed 0 so far, still mid-training, and this is the plan's named next temporal-arch rung so it earns a 2nd pod per WISHLIST -0.5. Same recipe, seed 0->1, nothing else changed. If-true: seed 1 reaches a similar walk quality/gate-clearing trajectory as seed 0 -- the hist16-dep contract rung is a real recipe, not scratch-training luck. If-false: seed 1 fails to bootstrap a gait or lands well below seed 0's trajectory -- flag the from-scratch hist16-dep recipe as seed-fragile before judging the 16-vs-8-frame question on a single seed.

**gate**: own-cfg DR0.5 det+sto 12/12 gait_valid, 0 term (or clearly matching seed 0's own trajectory at the same step count); dep-eval @DR0.35 det prog med tracked against seed 0's own number, not an absolute cap yet (still training); frames watched det


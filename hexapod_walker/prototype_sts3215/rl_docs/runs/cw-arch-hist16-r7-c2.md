# cw-arch-hist16-r7-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T16:59:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-arch-hist16-r7-c1

**hypothesis**: Plain English: keep the temporal-architecture line (obs.history_frames=16) occupied per the operator's standing 1-2-pod reservation -- the line just freed a pod (r7-c1 finished at 40M more steps on top of r7, reward still climbing 934.7->994.1 across its last 4 quarters, no plateau/crash) while the sibling hist24-r1 rung is still the only other arch pod running. Continues r7-c1's own checkpoint with an IDENTICAL config for another 20M steps -- same question as the c1 continuation itself: does more training close the slip/economy gap vs the deployment-contract champion (was 1.3-1.6 vs 0.89-1.13), or plateau. Not independently verdicted by me (c1 is unclaimed/awaiting its own triage); this is a mechanical line-occupancy continuation per WISHLIST -0.5, not a claim that c1 passed anything beyond its own visible reward curve.

**gate**: Same gate as r7/r7-c1: own-cfg DR0.5 gv >=5/6, JOYSTICK GATE 0 falls, progress >=0.85; report slip/m trend vs the 1.3-1.6 band -- closing toward champion (0.89-1.13) or flat


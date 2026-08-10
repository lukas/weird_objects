# cw-arch-hist16-dep1-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T20:15:45+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1

**wandb_id**: z1ic4485

**hardware_ready**: False

**hypothesis**: Plain English: seed-twin confirmation of the temporal-arch line's first hardware-contract rung (16-frame memory trained directly on the deployment obs contract) -- only tested at seed 0 so far, still mid-training, and this is the plan's named next temporal-arch rung so it earns a 2nd pod per WISHLIST -0.5. Same recipe, seed 0->1, nothing else changed. If-true: seed 1 reaches a similar walk quality/gate-clearing trajectory as seed 0 -- the hist16-dep contract rung is a real recipe, not scratch-training luck. If-false: seed 1 fails to bootstrap a gait or lands well below seed 0's trajectory -- flag the from-scratch hist16-dep recipe as seed-fragile before judging the 16-vs-8-frame question on a single seed.

**gate**: own-cfg DR0.5 det+sto 12/12 gait_valid, 0 term (or clearly matching seed 0's own trajectory at the same step count); dep-eval @DR0.35 det prog med tracked against seed 0's own number, not an absolute cap yet (still training); frames watched det

**verdict**: PASS (seed-twin confirms the recipe) -- seed 1 also bootstraps a clean, valid six-leg walking gait directly on the deployment contract from scratch: 6/6 gait_valid both DR passes, 0 terminations, joystick gate 0 falls, video shows the same ordinary six-leg swing/stance cycling as seed 0 -- no seed fragility, no flag leg. Slip/m is actually slightly better than seed 0 (det 1.28-1.35 vs seed 0's 1.41-1.48), but velocity-tracking overshoot is larger (vel_err_mean 0.032-0.047 vs seed 0's 0.028-0.036; the robot walks a bit faster than commanded), so the tight per-episode success threshold (<=0.03 m/s) is 0/12 here vs seed 0's ~8/12 -- read as a speed-calibration difference between seeds, not a broken gait. Confirms if-true: the hist16-dep bootstrap recipe is real across seeds, not scratch-training luck restricted to seed 0. Both seeds sit in the same 'needs continuation to close the gap to vref1-r1's band' place as the r7 architecture line did before its continuations -- same next step applies (see cw-arch-hist16-dep1's verdict).


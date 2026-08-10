# cw-arch-hist16-r7-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T16:59:42+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-arch-hist16-r7-c1

**wandb_id**: ht7oxdmw

**hardware_ready**: False

**hypothesis**: Plain English: keep the temporal-architecture line (obs.history_frames=16) occupied per the operator's standing 1-2-pod reservation -- the line just freed a pod (r7-c1 finished at 40M more steps on top of r7, reward still climbing 934.7->994.1 across its last 4 quarters, no plateau/crash) while the sibling hist24-r1 rung is still the only other arch pod running. Continues r7-c1's own checkpoint with an IDENTICAL config for another 20M steps -- same question as the c1 continuation itself: does more training close the slip/economy gap vs the deployment-contract champion (was 1.3-1.6 vs 0.89-1.13), or plateau. Not independently verdicted by me (c1 is unclaimed/awaiting its own triage); this is a mechanical line-occupancy continuation per WISHLIST -0.5, not a claim that c1 passed anything beyond its own visible reward curve.

**gate**: Same gate as r7/r7-c1: own-cfg DR0.5 gv >=5/6, JOYSTICK GATE 0 falls, progress >=0.85; report slip/m trend vs the 1.3-1.6 band -- closing toward champion (0.89-1.13) or flat

**verdict**: PASS -- confirms if-true (mechanical continuation, identical config +20M steps on r7-c1's checkpoint). DR0 gate: det+sto 6/6 gv, 0 term, prog med 1.21/1.05, slip/m med 1.09/1.27 -- flat-to-marginally-better vs r7-c1's own DR0 (prog 1.21/1.03, slip 1.14/1.33). Own-cfg DR0.5: det+sto 6/6 gv, 0 term, prog med 1.16/1.12, slip/m med 1.14/1.28 -- flat-to-marginally-better vs r7-c1's own-DR0.5 (prog 1.13/1.08, slip 1.16/1.38), no regression. JOYSTICK GATE (eval_drive DR0.2, heading45/speed0.06): 0 falls across the full direction panel + flip stress. Video (DR0 det/sto + own-DR0.5 det/sto, all 12+12 episodes): six legs visibly cycling swing/stance every episode, no flag leg, no dragging -- same unremarkable-but-real gait as r7/r7-c1. More steps continue nudging slip/economy toward the deployment-contract champion band (0.89-1.13) without regression, consistent with a slow asymptote not a plateau/crash; not yet inside the band. Architecture-line result only (deployment-contract obs, not the dep-line's deployment-exact contract) -- not a hardware candidate. Line stays open per operator directive.


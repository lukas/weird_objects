# cw-arch-hist16-dep1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T19:55:18+00:00

**pod**: hexapod-mjx-train-9

**steps**: 40000000

**parent**: cw-arch-hist16-r7

**wandb_id**: rfytzx9e

**hypothesis**: Does a longer memory (16 past sensor frames instead of 8) walk as well when the robot can only sense what the REAL robot senses? This arm trains the 16-frame architecture directly on the hardware deployment contract (velocity obs = commanded reference, 25-degree tilt envelope) for a head-to-head against the 8-frame hardware candidate cw-dep-vref1-r1 — the plan's named next temporal-arch rung. From-scratch by necessity (obs-width change forbids warm start); recipe = the proven hist16-r7 boot recipe (3072 envs, fits the 64M /dev/shm); the only deltas are the three deployment-contract cfg pairs. If-true (matches/beats vref1-r1's own band on the dep contract): the temporal line earns a hardware-candidate rung and longer memory helps under deployment-blind sensing. If-false (can't reach the warm 8-frame baseline): keep the 8-frame contract checkpoint for hardware, hist ladder stays exploratory. Strongest alternative: any gap is warm-vs-scratch, not 8-vs-16 — judged against hist16-r7's own from-scratch trajectory, not just vref1-r1.

**gate**: own-cfg DR0.5 det+sto 12/12 gait_valid, 0 term; dep-eval @DR0.35 det prog med >=0.85 with slip/m near vref1-r1's own band (0.89-1.36); JOYSTICK GATE (eval_drive DR0.2) zero in-envelope falls; if zero gait emerges, verdict = bootstrap failure (hist16-dep untested), NOT a hist16 FAIL; frames watched det


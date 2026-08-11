# cw-stand-riserock1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-11T23:08:07+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: b0rruyke

**hardware_ready**: False

**hypothesis**: HARDWARE-DRIVEN, CODE/SPEC FIRST (operator bench 08-11 22:42, bench_blast_20260811_184229 + RL_LOG): the stand specialist port's FIRST hardware run FAILED tilt_roll at 10.2deg rel roll ~9s in, DURING THE BELLY-CURL (before the height ramp; currents low 0.27A, runner limped clean). Sim probe on the same checkpoint (6 det seeds DR0): |roll| <= 1.7deg through the whole rise — hardware rocks over the tucked legs, sim never does. Trip threshold is CORRECT (do not bump it). Suspects: loaded-knee actuator lag breaking curl symmetry + belly contact geometry. Arm: add a rise-tick ROCKING DR axis (random roll perturbation during curl/ramp ticks — tilted-belly starts and/or body-roll impulses, capped well below the 25deg envelope, tilt reference stays LEVEL so leveling is paid), default OFF, guarded rng, bank test green BEFORE launch (MDP_PREFLIGHT binding). Warm from holdbc1_hard1 with the BC-anchor stack UNCHANGED. Optional second axis: bus.servo_params=loaded on rise ticks.

**gate**: Rise under +-10-15deg rocking injections det >= 5/6 valid_plant with zero falls; nominal rise/hold retention matches hard1's own probe (12/12 valid_plant RSI-off incl. flat 4/4, hold 11/12); matched-parent control under identical injection; frames watched.

**verdict**: INVALID LAUNCH, no science: the run trained a DEFAULT joint_goal config warm from hard1 — no goal-mix, no BC-anchor stack, servo_params=air, and the rocking-DR axis it exists to test WAS NEVER WRITTEN (zero rise_rock knobs in domain_rand.py/codebase; W&B resolved config confirms defaults). The operator's backlog entry was CODE-FIRST but the drain launched the stub args as-is. Hypothesis (hardware belly-curl rocking gap, 5/5 deterministic tilt_roll trips) remains UNTESTED and still queued conceptually: next action is SPECIFICATION — implement rise-tick rocking DR (guarded rng, bank green) then relaunch. Checkpoint quarantined, do not deploy or warm-start from it.


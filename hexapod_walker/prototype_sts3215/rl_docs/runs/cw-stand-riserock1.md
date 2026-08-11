# cw-stand-riserock1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T23:08:07+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: b0rruyke

**hypothesis**: HARDWARE-DRIVEN, CODE/SPEC FIRST (operator bench 08-11 22:42, bench_blast_20260811_184229 + RL_LOG): the stand specialist port's FIRST hardware run FAILED tilt_roll at 10.2deg rel roll ~9s in, DURING THE BELLY-CURL (before the height ramp; currents low 0.27A, runner limped clean). Sim probe on the same checkpoint (6 det seeds DR0): |roll| <= 1.7deg through the whole rise — hardware rocks over the tucked legs, sim never does. Trip threshold is CORRECT (do not bump it). Suspects: loaded-knee actuator lag breaking curl symmetry + belly contact geometry. Arm: add a rise-tick ROCKING DR axis (random roll perturbation during curl/ramp ticks — tilted-belly starts and/or body-roll impulses, capped well below the 25deg envelope, tilt reference stays LEVEL so leveling is paid), default OFF, guarded rng, bank test green BEFORE launch (MDP_PREFLIGHT binding). Warm from holdbc1_hard1 with the BC-anchor stack UNCHANGED. Optional second axis: bus.servo_params=loaded on rise ticks.

**gate**: Rise under +-10-15deg rocking injections det >= 5/6 valid_plant with zero falls; nominal rise/hold retention matches hard1's own probe (12/12 valid_plant RSI-off incl. flat 4/4, hold 11/12); matched-parent control under identical injection; frames watched.


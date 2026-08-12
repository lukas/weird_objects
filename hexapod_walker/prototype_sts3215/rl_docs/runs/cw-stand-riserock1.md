# cw-stand-riserock1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DEFECTIVE

**created**: 2026-08-11T23:08:07+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-stand-holdbc1-hard1

**wandb_id**: b0rruyke

**hardware_ready**: False

**hypothesis**: HARDWARE-DRIVEN, CODE/SPEC FIRST (operator bench 08-11 22:42, bench_blast_20260811_184229 + RL_LOG): the stand specialist port's FIRST hardware run FAILED tilt_roll at 10.2deg rel roll ~9s in, DURING THE BELLY-CURL (before the height ramp; currents low 0.27A, runner limped clean). Sim probe on the same checkpoint (6 det seeds DR0): |roll| <= 1.7deg through the whole rise — hardware rocks over the tucked legs, sim never does. Trip threshold is CORRECT (do not bump it). Suspects: loaded-knee actuator lag breaking curl symmetry + belly contact geometry. Arm: add a rise-tick ROCKING DR axis (random roll perturbation during curl/ramp ticks — tilted-belly starts and/or body-roll impulses, capped well below the 25deg envelope, tilt reference stays LEVEL so leveling is paid), default OFF, guarded rng, bank test green BEFORE launch (MDP_PREFLIGHT binding). Warm from holdbc1_hard1 with the BC-anchor stack UNCHANGED. Optional second axis: bus.servo_params=loaded on rise ticks.

**gate**: Rise under +-10-15deg rocking injections det >= 5/6 valid_plant with zero falls; nominal rise/hold retention matches hard1's own probe (12/12 valid_plant RSI-off incl. flat 4/4, hold 11/12); matched-parent control under identical injection; frames watched.

**verdict**: DEFECTIVE LAUNCH, not a test of the hypothesis. The backlog item was CODE-FIRST (the rise-rock DR axis had to be implemented before launch) but the drain placed it with a bare warm-start arg set: no dr.rise_rock_* config existed anywhere in the code, AND the cloned extra_args carried none of hard1's stance recipe (no BC anchor, no rise shaping stack, no goal-mix) — so the run trained 2M plain-default steps warm from hard1 and its gate eval predictably collapsed (rise det 0/6 with over_current plant fails, hold 6/6 retained). The rocking hypothesis was never exercised; no science verdict possible. Axis has since been implemented + bank-tested (commit c794de0: dr.rise_rock_prob/deg, physical-command one-side fold bias on rise episodes, mechanism probe reproduces the bench 10deg trip signature on hard1, P-feedback closability pinned) and relaunched properly as cw-stand-riserock2. Process lesson: drain must refuse CODE-FIRST backlog items whose cfg keys don't resolve — the unknown-DR-override guard would have caught this at env construction had the cfg keys been passed at all.


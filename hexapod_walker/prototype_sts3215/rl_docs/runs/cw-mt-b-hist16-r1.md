# cw-mt-b-hist16-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T23:26:42+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-mt-b-hist16-1

**wandb_id**: wghl8srm

**hypothesis**: Wave-2 REPRESENTATION lever for the multitask track (retry of cw-mt-b-hist16-1, which crashed at 0 steps on a documented /dev/shm limit -- see COMMANDS.md #13c: obs.history_frames=16 + model-DR at n-envs=4096 needs ~78MB against the pod's 64MB cap). Same question, same recipe, fixed launch (--n-envs=3072, ~50MB, fits): does giving the network the last 16 proprioceptive frames (obs.history_frames 1->16) change DISCOVERY at all at the identical 2M budget where the single-frame narrow generalist (cw-mt-b1) completely failed to walk (det prog med 0.16, gait_valid 0/6, low-crouch splay)? Fresh init, b1's exact command distribution. obs.history_frames is the only scientific variable vs b1 (n-envs is a launch-mechanics fix, not a hypothesis variable, mirroring the identical fix already applied in the arch-track hist16 lineage). Prediction-if-true: hist16 shows meaningfully more forward progress and/or partial gait_valid at 2M than b1's baseline -- worth a matched 20M arm next. Prediction-if-false: same low-crouch nothing-walks-yet creep regardless of temporal window (matching arch256's result already-failed capacity probe) -- representation window is not the 2M discovery bottleneck either, and the cheap-probe menu for this track is exhausted; next call is either a direct-to-20M hist16 arm citing the b2/widen2 budget precedent, or an operator call on a command-width curriculum.

**gate**: At 2M: PASS(discovery signal) = det prog med >= 0.32 (2x cw-mt-b1's 0.16) OR gait_valid >=1/6 det -> queue a matched 20M hist16 arm next (budget-matched to cw-mt-b2). FAIL = prog med and gait_valid both at/below cw-mt-b1's baseline (0.16, 0/6) -> temporal window is not the discovery lever either; no further capacity/representation retry at 2M on this recipe. Report slip_per_m/roll_tail regardless.


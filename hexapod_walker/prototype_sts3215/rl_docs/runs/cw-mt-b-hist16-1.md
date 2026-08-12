# cw-mt-b-hist16-1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: DEFECTIVE

**created**: 2026-08-12T23:19:22+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-mt-b1

**hardware_ready**: False

**hypothesis**: Wave-2 REPRESENTATION lever for the multitask track, now that the budget confound is closed: cw-mt-widen2 just showed that even with the FULL 20M budget that let cw-mt-b2 partially acquire commands, the staged-widened policy stays command-invariant (stop-hold speed_med 0.0417 m/s barely below fwd-hold's 0.0688, nowhere near the 0.02 bar; tip-left/tip-right yaw differential 0.0032, essentially zero) -- so budget is dead as an explanation and the pre-registered next lever is representation. This arm tests the cheapest version of that lever first: does giving the network the last 16 proprioceptive frames (obs.history_frames 1->16, the SAME hist16 rung already validated elsewhere in the campaign) change DISCOVERY at all, at the identical 2M budget where the single-frame narrow generalist (cw-mt-b1) completely failed to walk (det prog med 0.16, gait_valid 0/6, low-crouch splay)? Fresh init, b1's exact command distribution (vx 0-0.06, +-0.15 rad/s yaw on 20% of segments, 40% stop segments) -- obs.history_frames is the ONLY variable vs b1, mirroring the net-width capacity probe already run and failed (cw-mt-b-arch256-1, 256x256, FAILED at 2M: prog 0.11, gait_valid 0/6). Prediction-if-true: hist16 shows meaningfully more forward progress and/or partial gait_valid at 2M than b1's baseline -- a positive discovery signal worth a matched 20M arm to test actual command acquisition next. Prediction-if-false: the same low-crouch nothing-walks-yet creep regardless of temporal window -- representation window is not the 2M discovery bottleneck either (matching arch256's result), and the cheap-probe menu for this track is exhausted; the next call is either a direct-to-20M hist16 arm (citing the b2/widen2 budget precedent) or an operator decision on a command-width curriculum.

**gate**: At 2M: PASS(discovery signal) = det prog med >= 0.32 (2x cw-mt-b1's 0.16) OR gait_valid >=1/6 det -> queue a matched 20M hist16 arm next (the real command-acquisition test, budget-matched to cw-mt-b2). FAIL = prog med and gait_valid both at/below cw-mt-b1's baseline (0.16, 0/6) -> temporal window is not the discovery lever either; no further capacity/representation retry at 2M on this recipe -- next call is a straight-to-20M hist16 arm citing the established budget precedent (b2, widen1->widen2) or an operator call on a command-width curriculum. Report slip_per_m/roll_tail regardless.

**verdict**: DEFECTIVE LAUNCH, not a science result: 0 steps trained. All 24 workers SIGBUS (exitcode -7) at the very first env reset. Root cause: obs.history_frames=16 (this arms only intended variable) at the default --n-envs=4096, combined with model-field DR (on by default whenever --no-dr is absent, which it was here) blows the shm layout to ~78MB against the pods 64MB /dev/shm cap (measured: hist1+DR=60MB, hist16+DR=78MB). This is the EXACT documented gotcha from cw-arch-hist16 8-attempt death chain (COMMANDS.md #13c / rl_docs/runs/cw-arch-hist16-r6-rr1.md): fix is --n-envs=3072 (~50M) or recreate the pod with dshm-4Gi. The spec simply omitted the known fix. No verdict on the representation-lever hypothesis is possible from this attempt; retry queued as cw-mt-b-hist16-r1 with --n-envs=3072, same hypothesis/gate.


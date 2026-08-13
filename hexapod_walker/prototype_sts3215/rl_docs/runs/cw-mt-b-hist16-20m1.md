# cw-mt-b-hist16-20m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-12T23:54:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-mt-b-hist16-r1

**wandb_id**: 2u0ao0y3

**hardware_ready**: False

**hypothesis**: Test whether giving the robot a 16-frame memory of its own body lets one brain learn to walk AND actually obey stop/turn commands when trained long enough — the same memory recipe that (expectedly) failed at the 2M starvation budget, now at the full 20M budget where the single-frame twin (cw-mt-b2) learned a real gait but stayed slow and couldn't steer. Prediction-if-true: gait emerges AND speed clears the 0.5x-of-control bar b2 missed AND yaw responds sign-correctly both ways with stop-segments actually stopping. Prediction-if-false: numbers match b2 (prog ~0.5, unreliable yaw) — frame-stacking adds nothing and the command-acquisition gap is not a short-memory problem; strongest alternative: recurrence/phase-feature (arch track's territory) or a command-width curriculum (operator call), not more history frames.

**gate**: At 20M vs cw-mt-b2's matched numbers: PASS = gate(DR0) det gait_valid >=5/6 AND det prog med >=0.615 (the 0.5x-of-a2 bar b2 missed at 0.51) AND probe_signed_yaw tip-L/R wz differential >=0.10 sign-correct AND eval_yaw turn |wz_err| med <=0.10 with <9 falls (b2: 0.137, 9 falls). FAIL(match-b2) = prog/yaw within noise of b2 -> history lever closed at this recipe; next is escalation (arch recurrence / operator curriculum call), no further hist-frames variants. FAIL(worse/no-gait) = hist16 actively hurts from-scratch multitask learning. Report slip_per_m + roll_tail vs b2 regardless.

**verdict**: FAIL(worse/no-gait) per pre-registered gate: gait_valid collapses to 2/6 det, 2/6 sto (gate DR0) and 4/6 det, 3/6 sto (own-DR0.2) vs b2s clean 6/6 in all four passes, driven by a chronically near-zero-duty front leg (duty 0.01-0.17 across every one of 24 episodes, video-confirmed rigid/splayed leg) — worse than b2s already-marginal leg-3 (0.11-0.35). Progress-ratio looks flat-to-better (med 0.60-0.64 vs b2 0.51-0.53) and slip_per_m better (2.5-2.8 vs 3.4-3.9), but this is the known drag-exploit-inflates-progress pattern, not genuine improvement; roll_tail is flat-to-worse (4.2-4.85 vs 2.9-4.45). History (16-frame) actively hurts this recipe at the exact budget where the plain recipe (b2) got a clean six-leg gait. Compound gate already fails on clause 1 (gait_valid) so yaw probes not run. Closes the representation lever definitively: FAIL at 2M (r1) and now FAIL(worse) at the 20M budget-match. No further hist-frames variants on this recipe.


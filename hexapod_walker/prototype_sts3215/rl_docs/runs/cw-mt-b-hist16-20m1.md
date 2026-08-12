# cw-mt-b-hist16-20m1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T23:54:02+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-mt-b-hist16-r1

**wandb_id**: 2u0ao0y3

**hypothesis**: Test whether giving the robot a 16-frame memory of its own body lets one brain learn to walk AND actually obey stop/turn commands when trained long enough — the same memory recipe that (expectedly) failed at the 2M starvation budget, now at the full 20M budget where the single-frame twin (cw-mt-b2) learned a real gait but stayed slow and couldn't steer. Prediction-if-true: gait emerges AND speed clears the 0.5x-of-control bar b2 missed AND yaw responds sign-correctly both ways with stop-segments actually stopping. Prediction-if-false: numbers match b2 (prog ~0.5, unreliable yaw) — frame-stacking adds nothing and the command-acquisition gap is not a short-memory problem; strongest alternative: recurrence/phase-feature (arch track's territory) or a command-width curriculum (operator call), not more history frames.

**gate**: At 20M vs cw-mt-b2's matched numbers: PASS = gate(DR0) det gait_valid >=5/6 AND det prog med >=0.615 (the 0.5x-of-a2 bar b2 missed at 0.51) AND probe_signed_yaw tip-L/R wz differential >=0.10 sign-correct AND eval_yaw turn |wz_err| med <=0.10 with <9 falls (b2: 0.137, 9 falls). FAIL(match-b2) = prog/yaw within noise of b2 -> history lever closed at this recipe; next is escalation (arch recurrence / operator curriculum call), no further hist-frames variants. FAIL(worse/no-gait) = hist16 actively hurts from-scratch multitask learning. Report slip_per_m + roll_tail vs b2 regardless.


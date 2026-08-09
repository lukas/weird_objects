# cw-walk-joystick45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:20:17+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-wander30

**wandb_id**: 7xqzgrtc

**hypothesis**: OPERATOR BINDING TARGET: joystick operability - sudden command changes must never fall the robot. wander30 trained on gentle 5s resamples; a joystick means IRREGULAR instant flips. This run hardens transitions within +-45deg off wander30 with RANDOMIZED resampling: interval 1.5s +-60% jitter, blend 0.1-1.0s (flick to smooth), 20% stops, speed band 0.02-0.08. If-true: eval_drive JOYSTICK GATE passes with zero in-envelope falls (transition hardening works; combine with heading ladder); if-false: fast random resampling degrades base gait (needs explicit shaping/curriculum, not exposure).

**gate**: JOYSTICK GATE: python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale 0.2 with own cfg-set - ZERO in-envelope falls across direction panel + flip-stress; plus own-cfg harness walk 12/12 gait_valid


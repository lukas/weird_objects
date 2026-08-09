# cw-walk-joyhead90

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T16:56:49+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-joyjit-dr05-c1

**hypothesis**: JOYSTICK envelope rung: joyjit-dr05-c1 PASSed flip hardening (abrupt 1.5s+-60% resamples, blend 0.1-1.0s, 20% stops) composed with DR0.5, but only within +-45deg headings. The operator wants to steer the whole front hemisphere. One variable off the joyjit-dr05-c1 checkpoint: goal.walk_heading_max_rad 0.7854 -> 1.5708 (+-90deg), keeping the abrupt-resample package and DR0.5. Distinct from in-flight head90-dr05 (gentle 5s resamples). If-true: JOYSTICK GATE (eval_drive DR0.2, +-90 envelope incl. strafe legs) zero in-envelope falls AND own-cfg DR0.5 harness gv 12/12, 0 term, prog med >=0.80 - full front-hemisphere joystick driving under DR lands. If-false: abrupt flips to lateral headings fall the robot or crater progress - envelope widening needs a heading curriculum, not exposure. Strongest alternative: policy passes by parking through hard lateral flips - check per-scenario dist_m in eval_drive (left/right >=0.15m).

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg (+-90 envelope) - ZERO in-envelope falls, left/right scenario dist >=0.15m (no park-through); own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**verdict**: LAUNCH FAILURE, not a science result: lost a placement race with cw-walk-lowgait-dr05-r1 on train-4 (both trainers started, worker EOFError at init, W&B run 6ttogl28 crashed at 0 steps). No training happened; no verdict on the hypothesis. Retried immediately as cw-walk-joyhead90-r1 on train-5 (VERIFIED RUNNING).

**failed_reason**: run never appeared as 'running' in W&B within 240s


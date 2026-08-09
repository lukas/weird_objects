# cw-walk-joyhead90-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:08:32+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-joyjit-dr05-c1

**wandb_id**: 23i0yqkz

**hardware_ready**: no

**hypothesis**: RETRY of cw-walk-joyhead90 (launch race with lowgait-dr05-r1 on train-4, worker EOF at init; not a science failure). JOYSTICK envelope rung: joyjit-dr05-c1 PASSed flip hardening (abrupt 1.5s+-60% resamples, blend 0.1-1.0s, 20% stops) composed with DR0.5, but only within +-45deg headings. One variable off the joyjit-dr05-c1 checkpoint: goal.walk_heading_max_rad 0.7854 -> 1.5708 (+-90deg), keeping the abrupt-resample package and DR0.5. Distinct from head90-dr05 (gentle 5s resamples). If-true: JOYSTICK GATE (eval_drive DR0.2, +-90 envelope) zero in-envelope falls AND own-cfg DR0.5 harness gv 12/12, 0 term, prog med >=0.80. If-false: abrupt flips to lateral headings fall the robot or crater progress - envelope widening needs a heading curriculum. Strongest alternative: policy passes by parking through hard lateral flips - check left/right scenario dist in eval_drive (>=0.15m).

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg (+-90 envelope) - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**verdict**: PASS — ±90° heading envelope composes with the abrupt-flip DR0.5 driving package: JOYSTICK GATE at ±90 (eval_drive DR0.2 --heading-max-deg 90) 0 in-envelope falls, left 0.25m/right 0.30m (gate ≥0.15); own-cfg DR0.5 harness gv 12/12, 0 term, prog med 0.87 det/0.90 sto (gate ≥0.80), slip/m med 1.73/1.81 = wander-dr05 band. Frames det: level six-leg cycling through flips, no flag leg. Widest hardened driving envelope (joyjit ±45 → ±90). Reverse still no transport (back 0.04m, non-gating per ruling 4). Paddle slip root — not hardware-ready. NOTE: gating drive artifact is cw_walk_joyhead90_r1_drive90.json (first drive.json ran default ±45 envelope).


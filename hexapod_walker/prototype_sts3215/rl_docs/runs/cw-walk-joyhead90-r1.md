# cw-walk-joyhead90-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-09T17:07:51+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-joyjit-dr05-c1

**hypothesis**: RETRY of cw-walk-joyhead90 (launch race with lowgait-dr05-r1 on train-4, worker EOF at init; not a science failure). JOYSTICK envelope rung: joyjit-dr05-c1 PASSed flip hardening (abrupt 1.5s+-60% resamples, blend 0.1-1.0s, 20% stops) composed with DR0.5, but only within +-45deg headings. One variable off the joyjit-dr05-c1 checkpoint: goal.walk_heading_max_rad 0.7854 -> 1.5708 (+-90deg), keeping the abrupt-resample package and DR0.5. Distinct from head90-dr05 (gentle 5s resamples). If-true: JOYSTICK GATE (eval_drive DR0.2, +-90 envelope) zero in-envelope falls AND own-cfg DR0.5 harness gv 12/12, 0 term, prog med >=0.80. If-false: abrupt flips to lateral headings fall the robot or crater progress - envelope widening needs a heading curriculum. Strongest alternative: policy passes by parking through hard lateral flips - check left/right scenario dist in eval_drive (>=0.15m).

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg (+-90 envelope) - ZERO in-envelope falls, left/right dist >=0.15m; own-cfg DR0.5 harness 6+6: gv 12/12, 0 term, prog_ratio med >=0.80; frames watched det

**refused_reason**: hexapod-mjx-train-5 code marker c47f0d5c04ddd22a41ab9d31d1319e32236445ef != local HEAD ce89b5e672375000980c0fc1ab970d05987fed80. Sync first: snapshot.sh --sync hexapod-mjx-train-5 (and snapshot/commit before that if the tree is dirty).


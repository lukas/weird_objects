# cw-walk-head135

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-09T16:16:17+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-head90

**hypothesis**: Heading LADDER rung 2: rung 1 (head90, ±90°) PASSED with gv 12/12 + joystick gate zero falls, confirming gradual widening off a command-trained parent works where the abrupt ±180 jump (backforth) failed. This run widens ±90→±135 warm-started FROM head90, bringing rear-diagonal commands into the envelope. If-true: gait_valid tracking at ±135 with joystick gate pass (final rung ±180 next); if-false: rear-hemisphere commands break the warm-started gait even one rung at a time — rear coverage needs mirror-symmetry or from-scratch, and the ladder freezes at ±90.

**gate**: own-cfg DR0 det+sto 6/6: gait_valid 12/12, zero terminations, no sacrificed leg, rear-diag tracking err <= 2x forward; plus JOYSTICK GATE eval_drive --dr-scale 0.2 --heading-max-deg 135: zero in-envelope falls


# cw-walk-head135

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T16:16:17+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-head90

**wandb_id**: e6zrfhrm

**hardware_ready**: no

**hypothesis**: Heading LADDER rung 2: rung 1 (head90, ±90°) PASSED with gv 12/12 + joystick gate zero falls, confirming gradual widening off a command-trained parent works where the abrupt ±180 jump (backforth) failed. This run widens ±90→±135 warm-started FROM head90, bringing rear-diagonal commands into the envelope. If-true: gait_valid tracking at ±135 with joystick gate pass (final rung ±180 next); if-false: rear-hemisphere commands break the warm-started gait even one rung at a time — rear coverage needs mirror-symmetry or from-scratch, and the ladder freezes at ±90.

**gate**: own-cfg DR0 det+sto 6/6: gait_valid 12/12, zero terminations, no sacrificed leg, rear-diag tracking err <= 2x forward; plus JOYSTICK GATE eval_drive --dr-scale 0.2 --heading-max-deg 135: zero in-envelope falls

**verdict**: FAIL. Heading ladder rung 2 (±135°): own-cfg DR0 det ep0 TERMINATED tilt_pitch (gate required zero terminations); prog med collapsed to 0.53 det / 0.61 sto (parent head90: 0.84) with slip/m med 3.19/2.83 (lineage band ~1.6); train reward declined Q2→Q4 (1417→1277). gv 12/12 and JOYSTICK GATE @DR0.2 ±135 did pass (0 falls; left 0.11 m vs right 0.26 m asym persists) — the policy survives commands it can no longer track. Pre-registered if-false confirmed: rear-diagonal commands degrade the warm-started gait even one rung at a time. Heading ladder FROZEN at ±90 (no ±180 rung); rear coverage needs mirror-symmetry (3rd independent motivation: head90 L/R asym, strafe-dr10 flag legs, this) or from-scratch.


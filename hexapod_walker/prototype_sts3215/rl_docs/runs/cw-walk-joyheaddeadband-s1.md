# cw-walk-joyheaddeadband-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:36:07+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyheaddeadband

**hypothesis**: Ruling-7 seed twin of cw-walk-joyheaddeadband (PASS this cycle: servo-deadband 1.0-3.0x composes cleanly onto the widest +-90deg driving package, JOYSTICK GATE 0 falls). One variable: seed 0->1. If-true: seed1 reproduces gv 6/6, JOYSTICK GATE 0 falls, prog_ratio med>=0.80, DR0 retention gv 6/6 -- deadband-at-widest-envelope recipe is seed-robust. If-false: seed1 shows falls or a gait_valid miss the seed0 draw didn't -- the pass was seed-lucky at this wider envelope specifically.

**gate**: JOYSTICK GATE @90deg 0 in-envelope falls; own-cfg (DR0.5+lat+deadband1-3x) det+sto 6/6 @15s: gait_valid 6/6, 0 term, prog_ratio med>=0.80; DR0 retention gv 6/6; frames watched det.

**verdict**: Died at init (EOFError in mjx_sharded_vec_env reset_finalize), 0 steps -- fleet launch-collision gotcha 13b, not a science result. Requeued as -r1 (also died -r1->r2 in the same collision storm per the concurrent cycle handling that retry chain).

**failed_reason**: run never appeared as 'running' in W&B within 240s


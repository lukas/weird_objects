# cw-walk-joydeadband-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T00:25:12+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-joydeadband-r3

**hypothesis**: 4th retry of cw-walk-joydeadband (base+r1+r2+r3 all lost launch-collision races, gotcha 13b, 0 steps each). Same spec unchanged: driving-deadband compose off joylat25.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det


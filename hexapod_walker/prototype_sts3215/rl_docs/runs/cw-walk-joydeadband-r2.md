# cw-walk-joydeadband-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:08:30+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joylat25

**hypothesis**: 2nd retry of cw-walk-joydeadband (2 consecutive launch-collision losses, gotcha 13b). Same spec unchanged: driving-deadband compose off joylat25.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


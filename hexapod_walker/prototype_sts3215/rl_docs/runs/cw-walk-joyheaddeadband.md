# cw-walk-joyheaddeadband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T23:42:05+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-joyhead90-lat25

**wandb_id**: 4mv6c8mi

**hypothesis**: Compose rung off joyhead90-lat25 (widest +-90deg envelope, driving package): completes the deadband-onto-driving gap (see cw-walk-joydeadband, same axis on the narrower +-45deg package) at the wider heading envelope. One variable off joyhead90-lat25: add dr.deadband_scale=1.0,3.0. If-true: JOYSTICK GATE @90 0 falls AND own-cfg (DR0.5+latency+deadband) gv 12/12, 0 term, prog med >=0.80 -- widest envelope also absorbs deadband. If-false: deadband breaks flip recovery specifically at the wider steering angle (in-envelope falls or prog crater) even though the narrower package (joydeadband) held -- envelope width and deadband tolerance interact. Strongest alternative: passes but slip/prog shades vs joyhead90-lat25 baseline.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det


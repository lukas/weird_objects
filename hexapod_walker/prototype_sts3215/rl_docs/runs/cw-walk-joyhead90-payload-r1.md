# cw-walk-joyhead90-payload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T00:21:37+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-joyhead90-payload

**wandb_id**: e7t9hn7k

**hypothesis**: Retry of cw-walk-joyhead90-payload (lost the fleet launch-collision race, gotcha 13b, 0 steps trained, no science result). Same spec unchanged: chassis-payload exposure (dr.mass_scale=1.0,1.4x) composed onto the widest driving package (joyhead90-lat25: +-90deg abrupt flips + DR0.5 + latency jitter). Tests whether payload composability generalizes to the wide envelope, distinct from the narrower joyfric-payload base.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 own cfg - ZERO in-envelope falls; own-cfg DR0.5+latency+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; frames watched det


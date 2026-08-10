# cw-walk-joyheadfric-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:09:08+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**hypothesis**: Seed twin of the cw-walk-joyheadfric PASS (this cycle): identical widest-envelope (+-90deg) + DR0.5 + latency + friction 0.4-1.6x compose, seed 1 instead of 0. Ruling-7 practice before leaning on this driving recipe. If-true: seed 1 matches -- JOYSTICK GATE @90 0 falls, own-cfg gv 12/12, prog med >=0.80, DR0 retention clean. If-false: seed 1 collapses on a mid-range draw or an in-envelope fall appears -- recipe is seed-fragile.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 90 -- ZERO in-envelope falls; own-cfg (DR0.5+latency+friction) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


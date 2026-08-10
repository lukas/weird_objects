# cw-walk-groundtilt8-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:26:32+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r1

**hardware_ready**: False

**hypothesis**: 2nd retry of cw-walk-groundtilt8 (1st retry -r1 also lost the launch-collision race, gotcha 13b, 0 steps). Same spec unchanged: 8deg tilt ladder rung off groundtilt5's own checkpoint.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to groundtilt5 at triage; frames watched det

**verdict**: Launch failure (gotcha 13b EOFError launch-collision at reset, severe fleet-wide storm this window, loadavg 275+/128) -- 0 steps trained, no science result. 2nd consecutive collision loss for this spec (r1+r2).


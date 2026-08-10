# cw-walk-groundtilt8-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T00:41:52+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r2

**hypothesis**: 3rd retry of cw-walk-groundtilt8 (base+r1+r2 all lost launch-collision races, gotcha 13b, 0 steps each -- severe fleet-wide storm this window, loadavg 275+/128). Same spec unchanged: 8deg tilt ladder rung off groundtilt5's own checkpoint. Queued to backlog for the passive/self-repairing drain to place once contention eases, not forced immediately into the live storm.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=8.0, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.1m, 0 falls/terminations even on steepest draws; DR0 flat retention det 6/6 gv, slip/m <=1.24; compare episode pattern to groundtilt5 at triage; frames watched det


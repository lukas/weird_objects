# cw-walk-groundtilt5-payload-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T02:11:20+00:00

**pod**: hexapod-mjx-train-2

**steps**: 18000000

**parent**: cw-walk-groundtilt5-payload-r4

**hypothesis**: 6th launch attempt (r1-r4 all died 0-step to the launch-collision EOFError, gotcha 13b, during a heavy multi-cycle drain-storm window ~01:00-02:00; also hit + fixed a self-repair bug this cycle where a same-name backlog requeue after a started-then-crashed attempt gets silently dropped by the dedup guard instead of retried/parked -- launch_run.py rr-rename fix landed, snapshot 178fc8a). Same hypothesis unchanged: slope(5deg)+payload(1.0-1.5x) compose off cw-walk-groundtilt5.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal (no tilt, no payload) retention det 6/6 gv, det slip/m <=1.24; frames watched det


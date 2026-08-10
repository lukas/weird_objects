# cw-walk-groundtilt5-payload-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T02:11:20+00:00

**pod**: hexapod-mjx-train-2

**steps**: 18000000

**parent**: cw-walk-groundtilt5-payload-r4

**hardware_ready**: no

**hypothesis**: 6th launch attempt (r1-r4 all died 0-step to the launch-collision EOFError, gotcha 13b, during a heavy multi-cycle drain-storm window ~01:00-02:00; also hit + fixed a self-repair bug this cycle where a same-name backlog requeue after a started-then-crashed attempt gets silently dropped by the dedup guard instead of retried/parked -- launch_run.py rr-rename fix landed, snapshot 178fc8a). Same hypothesis unchanged: slope(5deg)+payload(1.0-1.5x) compose off cw-walk-groundtilt5.

**gate**: Own-cfg harness at --dr-scale 0.0 + dr.ground_tilt_deg=5.0 + dr.mass_scale=1.0,1.5, det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m; plus DR0 nominal (no tilt, no payload) retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: PASS (6th launch attempt, first to actually train): floor-slope (5deg) x chassis payload (1.0-1.5x) compose holds. Own-cfg det+sto gv 12/12, 0 term, det med fwd 1.38m (>=1.2 gate), same 2/6 steepest-tilt shuffle tail as plain groundtilt5 (prog 0.41-0.55, no falls/flag leg); DR0 nominal (no tilt, no payload) retention CLEAN (slip 0.99<=1.24, prog 1.01>=0.90, fwd 1.56m = champion band, no erosion at all).


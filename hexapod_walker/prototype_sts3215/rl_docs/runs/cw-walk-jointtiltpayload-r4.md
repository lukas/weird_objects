# cw-walk-jointtiltpayload-r4

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T01:33:29+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-jointtiltpayload-r3

**hypothesis**: 4th launch attempt (r1-r3 all died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: payload (mass_scale) x joytilt3's slope+latency driving package compose.

**gate**: Own-cfg harness DR0.5+latency+ground_tilt3deg+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 heading<=45deg 0 in-envelope falls; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


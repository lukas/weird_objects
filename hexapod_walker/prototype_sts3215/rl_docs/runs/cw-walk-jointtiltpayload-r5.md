# cw-walk-jointtiltpayload-r5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:43:42+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-jointtiltpayload-r4

**wandb_id**: voh4hem0

**hardware_ready**: no

**hypothesis**: 5th launch attempt (r1-r4 all died 0-step to fleet collision storm, gotcha 13b). Same hypothesis unchanged: payload (mass_scale) x joytilt3's slope+latency driving package compose.

**gate**: Own-cfg harness DR0.5+latency+ground_tilt3deg+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 heading<=45deg 0 in-envelope falls; frames watched det

**verdict**: PASS (5th launch attempt, first to actually train): payload (1.0-1.4x) composes cleanly onto the joytilt3 driving package (+-45deg heading, DR0.5+latency+3deg floor slope). Own-cfg det+sto gv 12/12, 0 term, det/sto prog med 0.92/1.03 (>=0.75 gate); JOYSTICK GATE @45deg 0 in-envelope falls (trk_err 0.030-0.032); DR0 nominal retention clean gv 12/12, det prog med 0.96 (>=0.9 gate), slip 1.18<=1.24. One fixed-draw sto near-stall outlier (prog -0.07, known non-gating canary-class stall, no fall).


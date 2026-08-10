# cw-walk-jointtiltpayload-r3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:45:11+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-jointtiltpayload-r2

**hypothesis**: 3rd retry of cw-walk-jointtiltpayload (2 consecutive launch-collision losses, gotcha 13b, 0 steps each). Same spec unchanged: chassis-payload (dr.mass_scale=1.0,1.4) composed onto the slope-hardened driving package joytilt3. Leaving for the self-repairing drain once fleet contention clears -- not fighting the storm further this cycle.

**gate**: Own-cfg harness DR0.5+latency+ground_tilt3deg+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 heading<=45deg 0 in-envelope falls; frames watched det

**verdict**: Launch failure (gotcha 13b EOFError collision storm), 0 steps, no science result.

**failed_reason**: run never appeared as 'running' in W&B within 240s


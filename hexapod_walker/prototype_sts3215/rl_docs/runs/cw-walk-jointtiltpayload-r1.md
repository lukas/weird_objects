# cw-walk-jointtiltpayload-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:36:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-jointtiltpayload

**hypothesis**: Retry of cw-walk-jointtiltpayload (lost the fleet launch-collision race, gotcha 13b, 0 steps trained -- no science result). Same spec unchanged: chassis-payload (dr.mass_scale=1.0,1.4) composed onto the slope-hardened driving package joytilt3.

**gate**: Own-cfg harness DR0.5+latency+ground_tilt3deg+dr.mass_scale=1.0,1.4 det+sto 6/6 @15s: gait_valid 12/12, 0 term, det prog median >=0.75; DR0 nominal retention det 6/6 gv, slip/m<=1.24, prog>=0.9; JOYSTICK GATE @DR0.2 heading<=45deg 0 in-envelope falls; frames watched det

**verdict**: INFRA-ONLY: drain collision — placed onto train-0 while cw-uni-mix40 was mid-launch-verify there (gotcha-13b class), 0 steps, no science; requeued as -r2.

**failed_reason**: never appeared running in W&B within window (fleet launch-collision storm, gotcha 13b) - 0 steps, no science result


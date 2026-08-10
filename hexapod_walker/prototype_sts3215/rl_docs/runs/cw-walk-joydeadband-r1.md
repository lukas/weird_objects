# cw-walk-joydeadband-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:46:23+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-joylat25

**hardware_ready**: no

**hypothesis**: Retry of cw-walk-joydeadband (lost a launch-collision race, gotcha 13b, 0 steps). Same spec unchanged: driving-deadband compose off joylat25.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 --heading-max-deg 45 -- ZERO in-envelope falls; own-cfg (DR0.5+latency0.5-2.5x+deadband1-3x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.80; DR0 retention det 6/6 gv; frames watched det

**verdict**: INFRA CRASH, no science result (2nd consecutive attempt): worker EOFError at env reset before any training step (0 steps) -- launch-collision under sustained concurrent-cycle drain storm (gotcha 13b, host load 300-400/128 this window). Retrying as joydeadband-r2.

**failed_reason**: run never appeared as 'running' in W&B within 240s


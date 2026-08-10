# cw-walk-joytilt3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T23:19:47+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joytilt3

**hardware_ready**: False

**hypothesis**: Seed twin of this cycle's cw-walk-joytilt3 PASS (ruling-7: seed-confirm before banking a driving-envelope axis). Identical config: joylat25 driving package + 3deg floor-slope, seed 1 instead of 0. If-true: seed 1 matches -- JOYSTICK GATE @DR0.2 0 falls, own-cfg gv 12/12, 0 term, prog med >=0.85, DR0 flat retention clean. If-false: seed-fragile, needs a caveat before folding into the driving-envelope panel.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.ground_tilt_deg=3.0) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 flat retention det 6/6 gv; frames watched det

**verdict**: LAUNCH FAILURE, not a science result: worker EOFError at env reset (gotcha 13b, launch-collision amid an unusually severe concurrent-cycle drain storm, host load1 hit 189/128 this cycle), 0 steps, W&B run wipw8egx crashed at init. Requeuing as -r1 for the self-repairing drain once contention clears.


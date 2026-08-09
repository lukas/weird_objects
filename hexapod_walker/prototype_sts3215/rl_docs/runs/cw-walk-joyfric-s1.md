# cw-walk-joyfric-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-09T22:56:18+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-joyfric

**hypothesis**: Ruling-7 seed twin of today's joyfric PASS (friction 0.4-1.6x composes cleanly onto the joylat25 driving package: JOYSTICK GATE 0 falls, own-cfg gv 12/12, DR0 retention clean). One variable off joyfric: --seed 0 -> 1, identical config+parent joylat25. If-true: seed 1 reproduces the pass - friction-grip composability is recipe-robust, not a seed fluke. If-false: seed 1 falls on slick flips or craters progress - the pass was seed luck.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


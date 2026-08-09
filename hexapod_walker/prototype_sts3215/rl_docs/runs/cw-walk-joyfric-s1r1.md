# cw-walk-joyfric-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T23:04:51+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-joyfric

**wandb_id**: ypzg59xt

**hypothesis**: RETRY of cw-walk-joyfric-s1 (lost a launch-collision race amid heavy concurrent-drain load -- worker EOFError at env reset, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: ruling-7 seed twin of today's joyfric PASS (friction 0.4-1.6x composes cleanly onto the joylat25 driving package). If-true: seed 1 reproduces the pass. If-false: seed 1 falls on slick flips or craters progress.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det


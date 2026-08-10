# cw-walk-joyfric-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T23:04:51+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-joyfric

**wandb_id**: ypzg59xt

**hardware_ready**: no

**hypothesis**: RETRY of cw-walk-joyfric-s1 (lost a launch-collision race amid heavy concurrent-drain load -- worker EOFError at env reset, 0 steps, no science result; COMMANDS.md gotcha 13b). Same spec unchanged: ruling-7 seed twin of today's joyfric PASS (friction 0.4-1.6x composes cleanly onto the joylat25 driving package). If-true: seed 1 reproduces the pass. If-false: seed 1 falls on slick flips or craters progress.

**gate**: JOYSTICK GATE: eval_drive --dr-scale 0.2 own cfg - ZERO in-envelope falls; own-cfg (DR0.5 + dr.latency_scale=0.5,2.5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.85; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS — seed-1 twin reproduces cw-walk-joyfric (seed0) closely: JOYSTICK GATE @DR0.2 0 in-envelope falls (fwd/back/L/R/diag/stop-go/flip-stress). Own-cfg (DR0.5+lat0.5-2.5x+fric0.4-1.6x) det+sto 6/6 gv 12/12, 0 term, prog med 1.00/0.95 (gate>=0.85), slip med 1.37/1.66. DR0 retention gv 6/6, prog med 1.00/1.01, slip med 1.50/1.45 — no erosion. Frames: level six-leg cycling, no flag leg, paddle slide persists (known contact-pricing root). Floor-grip+latency driving recipe is seed-robust.


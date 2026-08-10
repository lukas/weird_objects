# cw-walk-lowgait-dr035-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T02:39:51+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035

**wandb_id**: 3o0ip9k4

**hypothesis**: Friction-grip variation composed onto the -50mm crouch + DR0.35 package (lowgait-dr035 PASS: own-cfg DR0.35 gv 12/12, DR0 retention slip 0.98/height 4.0mm). ONE variable off lowgait-dr035: add dr.friction_scale=0.4,1.6 (same range that composed cleanly onto joyfric/joyheadfric/joylat60-fric 15-60s driving packages). If-true: own-cfg (DR0.35+friction0.4-1.6x) det+sto 6/6: gait_valid 12/12, 0 term, height err<=10mm, slip/m<=1.6; DR0 nominal retention det 6/6 gv, height err<=8mm, slip/m<=1.15 -- friction composes onto the crouch rung same as it did onto driving packages. If-false: grip variation interacts badly with the low-stance contact geometry (crouch changes foot/ground contact angle vs nominal height) -- crouch and grip-robustness don't compose for free.

**gate**: Own-cfg (DR0.35+friction0.4-1.6x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 nominal retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det for lurching/height overshoot


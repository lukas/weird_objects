# cw-walk-lowgait-dr035-groundtilt5-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:26:28+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-groundtilt5

**wandb_id**: sjs3sj70

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-groundtilt5 PASS (crouch -50mm DR0.35 x 5deg floor-slope compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run for lowgait-dr035, groundtilt5-fric, groundtilt8. If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 flat retention gv 6/6 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+tilt5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 flat no-tilt retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det


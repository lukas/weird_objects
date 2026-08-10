# cw-walk-lowgait-dr035-fric-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:29:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-fric

**wandb_id**: xiwy1w1t

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-fric PASS (crouch -50mm DR0.35 x floor-friction 0.4-1.6x compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run for lowgait-dr035, groundtilt5-fric, groundtilt8, and joyheaddeadband. If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 crouch retention gv 6/6 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+friction0.4-1.6x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 nominal crouch retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det


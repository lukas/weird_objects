# cw-walk-lowgait-dr035-groundtilt5-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T06:26:28+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-groundtilt5

**wandb_id**: sjs3sj70

**hardware_ready**: False

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-groundtilt5 PASS (crouch -50mm DR0.35 x 5deg floor-slope compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run for lowgait-dr035, groundtilt5-fric, groundtilt8. If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 flat retention gv 6/6 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+tilt5) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 flat no-tilt retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det

**verdict**: PASS: seed-1 twin confirms crouch(-50mm,DR0.35) x floor-slope(5deg) compose is a recipe, not a seed-0 fluke. Own-cfg det+sto gv 12/12, 0 term, mean end-height err 3.8mm det/4.5mm sto (<=10mm gate), slip/m med 1.13 det/1.25 sto (<=1.6 gate). DR0 flat retention det+sto gv 12/12, mean height err 5.4mm det/5.9mm sto (<=8mm gate), slip/m med 1.06 det/1.06 sto (<=1.15 gate). Same inherited fixed-draw march-in-place stall as seed-0 (1 det crater in DR0 mode, 1 det+2 sto craters in own-cfg mode) -- frames confirm level body cycling on the slope, no fall/flag-leg. Not hardware-ready (paddle-lineage foot slide, contact-pricing root).


# cw-walk-lowgait-dr035-fric-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T06:29:13+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-fric

**wandb_id**: xiwy1w1t

**hardware_ready**: False

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-fric PASS (crouch -50mm DR0.35 x floor-friction 0.4-1.6x compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run for lowgait-dr035, groundtilt5-fric, groundtilt8, and joyheaddeadband. If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 crouch retention gv 6/6 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+friction0.4-1.6x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 nominal crouch retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det

**verdict**: PASS: seed-1 twin confirms crouch(-50mm,DR0.35) x floor-friction(0.4-1.6x) compose is a recipe, not a seed-0 fluke. Own-cfg det+sto gv 12/12, 0 term, mean end-height err 3.2mm det/3.7mm sto (<=10mm gate), slip/m med 1.04 det/1.44 sto (<=1.6 gate). DR0 crouch retention det+sto gv 12/12, mean height err 6.0mm det/1.6mm sto (<=8mm gate), slip/m med 1.11 det/1.09 sto (<=1.15 gate). Same inherited fixed-draw march-in-place stall as seed-0 (1 det crater in DR0 mode, 1 det+2 sto craters in own-cfg mode) -- frames confirm level body, all six legs cycling, no fall/flag-leg. Not hardware-ready (paddle-lineage foot slide, contact-pricing root).


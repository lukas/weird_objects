# cw-walk-lowgait-dr035-deadband-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T06:32:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-deadband

**wandb_id**: tj4wqti5

**hardware_ready**: False

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-deadband PASS (crouch -50mm DR0.35 x servo deadband 1.0-3.0x compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run across the crouch lineage (lowgait-dr035, groundtilt5-fric, groundtilt8, fric just this cycle). If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 crouch retention gv 12/12 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+deadband1.0-3.0x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 no-deadband crouch retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det

**verdict**: PASS: seed-1 twin confirms crouch(-50mm,DR0.35) x servo-deadband(1.0-3.0x) compose is a recipe, not a seed-0 fluke. Own-cfg det+sto gv 12/12, 0 term, mean end-height err 4.5mm det/3.4mm sto (<=10mm gate), slip/m med 1.08 det/1.31 sto (<=1.6 gate). DR0 crouch retention det+sto gv 12/12, mean height err 3.3mm det/1.0mm sto (<=8mm gate), slip/m med 1.01 det/0.94 sto (<=1.15 gate). Same inherited fixed-draw march-in-place stall as seed-0 (1 det crater in DR0 mode, 1 det+2 sto craters in own-cfg mode) -- frames confirm level body, all six legs cycling, no fall/flag-leg, matches parent's own crater episodes exactly. Not hardware-ready (paddle-lineage foot slide, contact-pricing root).


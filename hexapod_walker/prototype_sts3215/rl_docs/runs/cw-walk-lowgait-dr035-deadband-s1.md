# cw-walk-lowgait-dr035-deadband-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T06:32:53+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-lowgait-dr035-deadband

**wandb_id**: tj4wqti5

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-lowgait-dr035-deadband PASS (crouch -50mm DR0.35 x servo deadband 1.0-3.0x compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run across the crouch lineage (lowgait-dr035, groundtilt5-fric, groundtilt8, fric just this cycle). If-true: own-cfg det+sto gv 12/12, slip/m med<=1.6, DR0 crouch retention gv 12/12 slip<=1.15 -- same band as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (DR0.35+deadband1.0-3.0x) det+sto 6/6 @15s: gait_valid 12/12, 0 term, mean end-height err<=10mm, slip/m med<=1.6; DR0 no-deadband crouch retention det 6/6 gv, height err<=8mm, slip/m<=1.15; frames watched det


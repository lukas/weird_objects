# cw-walk-terrain10-comshift-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T06:36:53+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-terrain10-comshift

**wandb_id**: 9annu1qy

**hardware_ready**: False

**hypothesis**: Seed twin (ruling-7 completeness) of cw-walk-terrain10-comshift PASS (rough-terrain amp1.0 x off-center CoM payload compose): confirm the result is not a seed-0 fluke, matching the seed-robustness pattern already run across other composed lines. If-true: own-cfg det+sto gv 12/12, prog matching seed-0 band, DR0 flat retention clean -- same as seed-0. If-false: seed-0's clean compose was a lucky draw.

**gate**: Own-cfg (terrain amp1.0 + comshift) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med matching seed-0's band; DR0 flat-no-terrain-no-offset retention clean; frames watched det

**verdict**: PASS: seed-1 twin of cw-walk-terrain10-comshift (parent PASS) confirms rough-terrain(amp1.0,36mm) x off-center CoM(0.03m) compose is a recipe not seed luck -- own-cfg det gv 6/6 (0 term, prog med 1.00 matching parent's 1.01 band), sto gv 6/6 (prog med 1.00), det/4 craters to prog -0.01/slip 22.7 -- the SAME idx4 fixed-draw march-in-place stall the parent checkpoint shows episode-for-episode (level body, all 6 legs cycling, no flag-leg), reproducing across the eval's fixed seed regardless of training seed; DR0 flat-no-terrain-no-offset retention det gv 6/6, prog 1.08, slip/m 0.91 -- clean, no erosion.


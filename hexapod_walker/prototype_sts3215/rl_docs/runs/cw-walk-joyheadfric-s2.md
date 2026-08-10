# cw-walk-joyheadfric-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:05:19+00:00

**pod**: hexapod-mjx-train-9

**steps**: 20000000

**parent**: cw-walk-joyheadfric

**wandb_id**: vgkr75ou

**hardware_ready**: False

**hypothesis**: 3rd-seed panel completion for the widest +-90deg envelope + floor-grip driving package (joyheadfric): seed0 PASSED, seed1 (joyheadfric-s1r1) PASSED and matched seed0's band closely this cycle. This seed asks whether the recipe holds a 2/2 -> 3/3 clean sweep (ruling-7 completeness), matching the 3-seed panels already banked for joylat25/joyjit-dr05.

**gate**: Own-cfg (DR0.5+friction) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog med >=0.80 (matching seed0/seed1 band 0.87-0.95); DR0 nominal retention det 6/6 gv, prog med >=0.85; JOYSTICK GATE @90deg 0 in-envelope falls; frames watched det

**verdict**: PASS -- 3rd-seed panel completion: joyheadfric (widest +-90deg envelope + friction 0.4-1.6x) now confirmed 3/3 across seeds. JOYSTICK GATE @90deg 0 in-envelope falls (flip-stress trk_err 0.030-0.035). Own-cfg (DR0.5+friction) det+sto gv 6/6 each, 0 term, prog med 0.89/0.83 (>=0.80 gate; sto sits at the low end of the seed0/1 band 0.87-0.95 but still clears). DR0 nominal retention gv 6/6, 0 term, prog med 0.86/0.89 (>=0.85 gate, det just barely), slip med 1.75/1.76 -- same band as seed0/1, no erosion. Frames (det): level six-leg cycling, no flag leg -- same paddle gait as parent/seed1. Ruling-7 promotion panel (3 seeds) satisfied for this recipe. Not hardware-ready (foot-slide).


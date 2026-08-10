# cw-uni-mix40-r2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T00:55:08+00:00

**pod**: hexapod-mjx-train-8

**steps**: 18000000

**parent**: cw-uni-mix40-r1

**wandb_id**: 709l9vrk

**hardware_ready**: no

**hypothesis**: 3rd mechanical attempt (2 straight launch-collision EOFErrors, gotcha 13b, 0 steps each -- still not a science result). Same inverse mix-ladder spec unchanged (walk=.4/rise=.25/lower=.25 off cw-uni-blend1-r2).

**gate**: rise/lower success >=5/6 det each AND JOYSTICK GATE (eval_drive DR0.2, own cfg) zero in-envelope falls AND quiet hold AND VIDEO: no leg-through-floor in rise/lower; frames watched det

**verdict**: FAIL — inverse-mix-ladder rung (walk=.4/rise=.25/lower=.25) still 0/6 rise + 0/6 lower det (height_err_end 20-60mm, never reaches target in 15s; wandb eval/rise_*_frac + lower_success_frac flat ZERO across all 18M steps). Walk (survived 1, prog 0.88-1.06) and hold (h_err 6mm, track 1.1deg) retained clean; frames confirm rise strip stuck mid-height churning legs, lower strip stays at full standing height never descending — same crouch-parking/no-descend pathology already root-caused in cw-uni-blend1-r2 (10/10 mix), now reproduced at a much higher 25/25 share. Not hardware-ready.


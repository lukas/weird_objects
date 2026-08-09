# cw-walk-terrain05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T13:45:14+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: ahfhey55

**hypothesis**: OPERATOR arm (08-09): rough-terrain ROBUSTNESS, explicitly NOT a slip arm. New cfg hook env.terrain_amp populates the warp-verified hfield (18mm smooth indoor bumps, flat 0.32m spawn patch, fade-in). Operator local preview: champion paddles across even amp 1.0 deterministically unimpeded (0.74m vs 0.76m flat), so do NOT expect slip movement. Tests whether training ON amp 0.5 terrain yields a gait that stays valid/upright on bumps without regressing flat performance — hardware floors are not billiard tables. Caveat: spawn patch is flat, bumps arrive ~6s into an episode. If-false: terrain training destabilizes the gait (tilt/falls) -> needs an amp ramp curriculum.

**gate**: non-promotion exploratory: flat DR0 det 6/6 retention (gv, 0 term, slip/m <= 1.35) AND own-cfg terrain det 6/6 gv 0 term; frames watched on terrain


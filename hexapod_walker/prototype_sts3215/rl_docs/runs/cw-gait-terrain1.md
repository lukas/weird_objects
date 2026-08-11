# cw-gait-terrain1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INVALID

**created**: 2026-08-11T17:29:50+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: xablk3ns

**hypothesis**: GAIT CLEANUP P3 lever 1, terrain-as-teacher (operator 08-11, rl_docs/GAIT.md): let PHYSICS price the dragging instead of the reward. FROM SCRATCH on rough ground at env.terrain_amp=2.0 (~72mm bumps, 2x the saturated 36mm rung that never perturbed the paddle -- terrain10 TRAINED on 36mm and still paddles at slip/m~1.0, so 36mm is refuted; 72mm is chassis-height-scale where a dragging foot must physically catch). If paddling cannot travel on this ground, plain progress income selects lift-and-place stepping with NO reward surgery, and no paddle habit exists to break because there is no warm start. Same contract stack as cw-dep-vref1-r1 otherwise (meas:=ref, 25deg envelope, drag/park charges at stock values).

**gate**: By 2M: policy travels on its own terrain (fwd distance in the parent joystick band, zero falls beyond tilt-envelope noise) AND slip/m on FLAT DR0 retention eval < 0.6 (vs champion band 1.1-1.5) -- the stepping gait must transfer to flat. Kill signatures (pre-registered): (a) no travel at all by 2M -- amp 2.0 too hard from scratch, retry ONE rung down (amp 1.5) informed by the parallel champion-on-terrain probe, not a coef sweep; (b) travels on bumps but flat retention shows paddle slip >1.0 -- lever refuted: stepping on bumps does not transfer, next lever is drag-charge-annealed-up (GAIT.md P3.2).

**verdict**: INVALID ARM (operator 08-11 ~13:5x): env.terrain_amp=2.0 was silently clamped to 1.0 = 18mm peak bumps by servo_model.py (bug fixed 434a6e0) — this trained FROM SCRATCH on the gentle saturated rung, not 72mm. Do not triage as the terrain-as-teacher arm; supersede with cw-gait-terrain2 at a probe-informed true amplitude. Checkpoint still a usable scratch-on-18mm reference if ever needed.


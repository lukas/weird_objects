# cw-walk-wander-dr05-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T17:49:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-wander

**wandb_id**: 6q1tkv9v

**hardware_ready**: no

**hypothesis**: Seed panel completion for the steering-DR recipe (operator ruling 7: promotion needs multi-seed panels). wander-dr05 (seed 0) PASSed and is the driving line's robustness ceiling; s1 (seed 1) is in flight; this is seed 2 of the same config - identical to wander-dr05 except --seed 2. head90-dr05 PASSed today on the same widen-then-harden recipe, so the recipe's seed-robustness now anchors two skills. If-true: s2 matches wander-dr05's panel (own-DR0.5 gv 12/12, 0 term, prog med >=0.85, slip/m med <=2.4) - 3-seed concordance, recipe robust. If-false: recipe is seed-lucky and the steering-DR skill row needs a caveat + the driving line re-plans.

**gate**: Own-cfg DR0.5 det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.85, slip/m med <=2.4; DR0 retention det 6/6 gv; frames watched det

**verdict**: PASS — 3-seed panel for the steering-DR recipe complete: own-cfg DR0.5 det+sto gv 12/12, 0 term, prog med 0.97/0.93 (gate >=0.85), slip/m med 1.46/1.88 (gate <=2.4) vs seed-0 wander-dr05 1.39/1.70 and s1 1.59/1.75 — all within the eval noise band; DR0 det retention gv 6/6, prog med 1.01, slip med 1.44 (partners 1.52/1.56). Frames det watched at DR0.5 and DR0: level six-leg cycling through command changes and stops, no flag leg, no stalls; weakest DR0.5 det draw (prog 0.73) transports slower with more slip but stays level. Steering-DR rung is seed-robust across seeds 0/1/2 (ruling 7 satisfied). hardware-ready: no (contact-pricing slip root).


# cw-walk-wander-dr05-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T17:49:37+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-wander

**wandb_id**: 6q1tkv9v

**hypothesis**: Seed panel completion for the steering-DR recipe (operator ruling 7: promotion needs multi-seed panels). wander-dr05 (seed 0) PASSed and is the driving line's robustness ceiling; s1 (seed 1) is in flight; this is seed 2 of the same config - identical to wander-dr05 except --seed 2. head90-dr05 PASSed today on the same widen-then-harden recipe, so the recipe's seed-robustness now anchors two skills. If-true: s2 matches wander-dr05's panel (own-DR0.5 gv 12/12, 0 term, prog med >=0.85, slip/m med <=2.4) - 3-seed concordance, recipe robust. If-false: recipe is seed-lucky and the steering-DR skill row needs a caveat + the driving line re-plans.

**gate**: Own-cfg DR0.5 det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog_ratio med >=0.85, slip/m med <=2.4; DR0 retention det 6/6 gv; frames watched det


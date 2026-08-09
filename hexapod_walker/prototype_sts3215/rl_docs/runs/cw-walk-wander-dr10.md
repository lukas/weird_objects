# cw-walk-wander-dr10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:41:09+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-wander-dr05

**wandb_id**: urbzb061

**hypothesis**: DR ladder rung for the steering line (wishlist 9): wander-dr05 held everything at DR 0.5; one variable — train at model-DR 1.0 off the wander-dr05 checkpoint. Full-strength physics randomization is what the champion lineage trains at (longdist-dr10 in flight), and the driving demo needs the steering skill at the same robustness level. If-true: own-eval DR1.0 keeps gv 12/12, 0 term, prog med >=0.85, slip/m med <=2.4 — steering matches the champion's DR envelope. If-false: transitions break at full DR (stalls/parks/falls at command changes) — DR 0.5 is the steering ceiling until the gait itself changes.

**gate**: own-cfg DR1.0 6+6 resampled commands: gv 12/12, 0 term, prog_ratio med >=0.85, slip/m med <=2.4; DR0 det retention gv 6/6; frames watched det


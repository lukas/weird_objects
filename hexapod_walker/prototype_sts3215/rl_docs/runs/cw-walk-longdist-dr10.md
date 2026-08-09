# cw-walk-longdist-dr10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T15:00:32+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 6okjse8d

**hypothesis**: OPERATOR (08-09): reliability rung for the NEW champion — its known flaw is stochastic stalls (DR1.0 sto 3/6 both seeds). Train the champion UNDER full randomization; per promotion ruling 8, reliability is the promotion axis. If-true: DR1.0 sto gv >=5/6 with det slip held <=1.10; if-false: stalls persist under DR training = draw-level pathology, needs a start-state fix not DR.

**gate**: DR1.0 det 6/6 slip/m <=1.10 AND sto gv >=5/6, 0 term; DR0 retention det 6/6


# cw-walk-longdist-dr10

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T15:00:32+00:00

**pod**: hexapod-mjx-train-6

**steps**: 20000000

**parent**: cw-walk-longdist-r2

**wandb_id**: 6okjse8d

**hardware_ready**: no

**hypothesis**: OPERATOR (08-09): reliability rung for the NEW champion — its known flaw is stochastic stalls (DR1.0 sto 3/6 both seeds). Train the champion UNDER full randomization; per promotion ruling 8, reliability is the promotion axis. If-true: DR1.0 sto gv >=5/6 with det slip held <=1.10; if-false: stalls persist under DR training = draw-level pathology, needs a start-state fix not DR.

**gate**: DR1.0 det 6/6 slip/m <=1.10 AND sto gv >=5/6, 0 term; DR0 retention det 6/6

**verdict**: FAIL (pre-registered if-false + nominal erosion). Gate 'DR1.0 det slip/m <=1.10': MISS - med 1.28 (parent r2 on the SAME DR1.0 panel: 1.06; s1 0.98). DR1.0 training bought NO reliability: the bad fixed draws are unchanged (DR1.0 sto worst prog 0.535 vs r2 0.52; DR0 stall draw 0.258 vs r2 0.24) while nominal quality eroded on every axis (DR1.0 det prog 0.824 vs 0.885, sto prog med 0.726 vs ~0.83; DR0 det slip 1.19 vs 0.96, prog 0.91 vs 0.98). gv 12/12 at both DR levels, 0 term; frames level six-leg gait - the gait survives, just sloppier. Full-DR retraining of the champion is REFUTED as a reliability lever (consistent with c52 stall-class closure); champion stays longdist-r2.


# cw-walkcurr-pf-fwd6-rscale50-sde-s2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T03:16:00+00:00

**pod**: hexapod-mjx-train-6

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50-sde

**wandb_id**: oqp7upaw

**hypothesis**: Plain English: seed replicate of the sde breakthrough before building a lineage on it -- the parent produced the track's first-ever forward-progress states (sto prog 0.32-0.47) and the first belly-sit escape WITHOUT the action-bias fix (height 10-18mm all run), and both claims currently rest on a single seed. Exact parent recipe, seed 2, fresh 2M. Prediction-if-true (mechanism, not luck): level stance + fall-dominated forward excursions reproduce (height <30mm all run, tilt-term-dominated training, nonzero sto prog). Prediction-if-false (belly-sit or frozen splay recurs): the parent's signature was seed-contingent and the sde-actbias combos must be read with that caveat -- any success there would credit actbias, not sde.

**gate**: Same rung-1 gate as every fwd6 arm: C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Replication read: env/height_err_mm band (<30mm = replicated escape), sto prog > 0.2, tilt-term-dominated training.


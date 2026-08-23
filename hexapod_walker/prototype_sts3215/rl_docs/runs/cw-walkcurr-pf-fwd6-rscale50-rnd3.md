# cw-walkcurr-pf-fwd6-rscale50-rnd3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-23T22:34:37+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-walkcurr-pf-fwd6-rscale50

**wandb_id**: 277om3m4

**hypothesis**: Plain English: dose sibling of rscale50-rnd1 (3x the intrinsic weight) applied directly to the rung-1 travel diet -- same logic as the rung0-swing3-rnd dose pair, testing whether 0.02 is too weak against the rung-1 landscape's static-crouch optimum. Single lever vs rscale50: --rnd-coef 0.0 -> 0.06, fresh 2M init. Prediction-if-true/false mirrors rscale50-rnd1; read the 2x2 grid (rung0 vs rung1-direct x low vs mid dose) jointly -- whichever base+dose first shows freeprog/gait_valid movement decides the next warm-start lineage.

**gate**: Rung-1 gate (same as rscale50-rnd1): C-env det fixed-forward panel -- zero tilt terms, cmd_prog_frac >= 0.35, direction_err <= 30 deg, slip/m <= 3.0, six legs cycling on >=4/6 episodes. Mechanism health: clip_fraction > 0.02, freeprog trend vs rscale50's own no-RND 6M baseline.


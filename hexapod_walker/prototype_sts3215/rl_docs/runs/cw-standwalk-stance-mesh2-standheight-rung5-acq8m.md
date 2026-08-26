# cw-standwalk-stance-mesh2-standheight-rung5-acq8m

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T06:01:59+00:00

**pod**: hexapod-mjx-train-6

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5

**wandb_id**: i78gd6k3

**hypothesis**: Does more budget (8M vs the 2M canary) close the lower-phase softening (seed0's high-herr det drift, 18-20.5mm on 4/6 episodes) without a new bc_anchor-coef mechanism, given both seeds' reward was still rising at the 2M cutoff (Q4 recovering hard out of the shared mid-training valley)? Same recipe, seed 0, warm-started from its own 2M canary checkpoint.

**gate**: PASS: lower/det in the mode_seq_stance+hold_height_cmd seqprobe reaches >=5/6 success (herr<=15mm) with no majority over_current, matching the isolated lower champion's own established band, while hold/rise stay at-or-above the 2M canary's levels (no regression). FAIL at the same signature (4-5/6 det >15mm herr, no term): budget is refuted for this residual; next lever is a height-cmd-segment-specific bc_anchor_coef loosening (new default-off cfg + bank rows + unit tests).


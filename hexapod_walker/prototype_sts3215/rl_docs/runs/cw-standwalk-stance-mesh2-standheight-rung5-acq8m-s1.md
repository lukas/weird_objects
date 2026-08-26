# cw-standwalk-stance-mesh2-standheight-rung5-acq8m-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-26T06:06:10+00:00

**pod**: hexapod-mjx-train-0

**steps**: 8000000

**parent**: cw-standwalk-stance-mesh2-standheight-rung5-s1

**wandb_id**: auf0f70c

**hypothesis**: Seed-1 twin of the rung5-acq8m continuation: does more budget close seed1's lower-phase residual (lower/det trips over_current on 6/6 seqprobe episodes despite tight herr tracking) without a new bc_anchor-coef mechanism? Same recipe, seed 1, warm-started from its own 2M canary checkpoint.

**gate**: PASS: lower/det in the mode_seq_stance+hold_height_cmd seqprobe reaches >=5/6 success with no majority over_current, while hold/rise stay at-or-above the 2M canary's levels. PARTIAL/FAIL judged jointly with the seed-0 acq8m twin per the same disagreement rule as the 2M pair (both must clear for a clean PASS; disagreement -> next lever is the height-cmd-segment bc_anchor-coef loosening, not a 3rd seed).


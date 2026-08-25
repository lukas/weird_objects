# cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T06:07:31+00:00

**pod**: hexapod-mjx-train-11

**steps**: 38000000

**parent**: cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1

**wandb_id**: y03eby83

**hypothesis**: Plain English: architecture-replication continuation -- the transformer sibling (tf64-mesh-movecur1) shows the SAME matched-step-indistinguishable-from-uncharged-reference pattern at 2M as the MLP arm (over_current/height/reward tracking its own uncharged reference tf64-mesh-acq1 closely through 2M), so this canary also can't yet tell whether k_walk_move_current=2.0 transfers across architecture. Continuing to the same 40M budget tf64-mesh-acq1 got (FAIL at 38M via the identical over_current tripod-lock as the MLP), charge held on, to test transfer once cmd_prog actually moves.

**gate**: PASS: by ~38-40M, over_current termination rate + Imax stay well under tf64-mesh-acq1's fail signature (38/48 held-out falls, Imax 2.64-2.70A) AND frontier/promotions move past b0, matching or beating the MLP sibling's own matched-step read (movecur1-acq1). PARTIAL: over_current measurably lower than tf64-mesh-acq1 at matched steps but frontier stuck at b0. FAIL: converges to the SAME locked-leg tripod signature as tf64-mesh-acq1 -- the fix is architecture-independent-insufficient at this dose, matching whatever movecur1-acq1 (MLP sibling) finds; no more architecture-replication arms needed if so, since the MLP answer would already generalize.


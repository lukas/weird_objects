# cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T06:07:31+00:00

**pod**: hexapod-mjx-train-11

**steps**: 38000000

**parent**: cw-arch-tf64-mesh-joyfullcurr13-v7-hz100-movecur1

**wandb_id**: y03eby83

**hypothesis**: Plain English: architecture-replication continuation -- the transformer sibling (tf64-mesh-movecur1) shows the SAME matched-step-indistinguishable-from-uncharged-reference pattern at 2M as the MLP arm (over_current/height/reward tracking its own uncharged reference tf64-mesh-acq1 closely through 2M), so this canary also can't yet tell whether k_walk_move_current=2.0 transfers across architecture. Continuing to the same 40M budget tf64-mesh-acq1 got (FAIL at 38M via the identical over_current tripod-lock as the MLP), charge held on, to test transfer once cmd_prog actually moves.

**gate**: PASS: by ~38-40M, over_current termination rate + Imax stay well under tf64-mesh-acq1's fail signature (38/48 held-out falls, Imax 2.64-2.70A) AND frontier/promotions move past b0, matching or beating the MLP sibling's own matched-step read (movecur1-acq1). PARTIAL: over_current measurably lower than tf64-mesh-acq1 at matched steps but frontier stuck at b0. FAIL: converges to the SAME locked-leg tripod signature as tf64-mesh-acq1 -- the fix is architecture-independent-insufficient at this dose, matching whatever movecur1-acq1 (MLP sibling) finds; no more architecture-replication arms needed if so, since the MLP answer would already generalize.

**verdict**: Result: FAIL -- the transformer (tf64) architecture replicate of k_walk_move_current=2.0 (charge alone) converges to the IDENTICAL architecture-independent locked-leg-tripod signature as its own uncharged tf64-mesh-acq1 reference, closing the architecture axis: the MLP finding generalizes, this is not an MLP-specific mechanism gap. Evidence: held-out 60s joygate 38/48 falls (24/24 dr0, 14/24 dr0p5) -- exactly matching tf64-mesh-acq1's own 38/48 -- with gait_valid_frac 0.0 (worse than acq1's 0.5), dir_err actually improves (29.25deg, clears the 40deg allow) but slip/m 3.10 still over cap and zero_falls/gait_valid both fail; per-leg sacrificed_frac shows 3 legs ([0,2,5]) sacrificed 77-100% of episodes, a hard multi-leg lock, not an intermittent flag-leg. wandb summary at the 38M read: env/max_current_a 2.09A, terminations/over_current 126/window, walkcurr/frontier=0, walkcurr/promotions=0 -- no forward-progress unlock, consistent with the MLP sibling's own frontier-stuck-at-0 result. This is the pre-registered FAIL branch verbatim ('converges to the SAME locked-leg tripod signature ... no more architecture-replication arms needed'). What's next: with all 3 movecur1 arms now read (MLP-alone FAIL, MLP+gait_gate-combo FAIL per the sibling cycle's own ledger note, tf64-transformer-alone FAIL here), k_walk_move_current=2.0 is CLOSED as a fix for the mesh-family over_current/leg-sacrifice exploit across every tested combination (architecture, +gate) -- do not fund further same-dose continuations or architecture replicates on this lever. The joint next-lever call (higher current-charge dose vs a structural per-leg current-clamp curriculum vs deferring entirely to the standwalk track's from-scratch mesh stance-retrain + teacher distillation, which is already addressing the analogous primitive-family walk-champion crouch/posture problem via a teacher rather than a reward-shaping proxy) is DIG-IN territory, not a same-cycle call.


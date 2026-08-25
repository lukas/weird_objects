# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-25T10:10:17+00:00

**pod**: hexapod-mjx-train-4

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x

**wandb_id**: ndzkxjrb

**hypothesis**: Plain English: the 2M mechanism-health canary just PASSED -- 5x the current-dwell charge (k_walk_move_current=10.0) knocked cur_max_a off the 2.64A pinned safety-trip edge (det gate 0/6 over_current, was 6/6 at k=2.0), but the robot at 2M is just holding a static all-legs-sacrificed crouch, not walking. This is the full acquisition-budget continuation the canary's own PASS branch calls for: same checkpoint, same single lever (k=10.0), no other change, extended to the SAME 38-40M budget the k=2.0 trio got, to see whether real six-leg walking emerges once cmd_prog actually starts moving, or whether the over_current exploit re-locks in once the policy commits to a gait (the exact way the k=2.0 lineage's acq1r2 continuation re-converged to the pinned signature by 20-38M despite looking clean early).

**gate**: PASS: by ~38-40M, held-out 60s joygate falls stay well under the acq1/tf64-mesh-acq1 fail signature (was 38-40/48) AND walkcurr frontier/promotions move past b0 (genuine forward progress, not a frozen/parked plateau) AND det gate cur_max_a stays measurably below 2.64A (mechanism fix holds under full training pressure, not just at 2M). PARTIAL: over_current measurably better than the k=2.0 trio's matched-step trajectory but frontier still stuck at b0 -- real but incomplete, dig into the progress/current tradeoff. FAIL: over_current/cur_max_a re-converge to the SAME pinned-2.64A/locked-leg-tripod signature as the k=2.0 trio by matched steps -- even 5x dose is insufficient once the policy fully commits, and the current-dwell-charge mechanism class is closed regardless of magnitude tested so far; the standwalk teacher-distillation route becomes the next lever for this exploit.


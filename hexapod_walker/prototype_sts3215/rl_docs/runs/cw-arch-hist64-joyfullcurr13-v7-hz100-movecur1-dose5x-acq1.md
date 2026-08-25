# cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T10:10:17+00:00

**pod**: hexapod-mjx-train-4

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-movecur1-dose5x

**wandb_id**: ndzkxjrb

**hypothesis**: Plain English: the 2M mechanism-health canary just PASSED -- 5x the current-dwell charge (k_walk_move_current=10.0) knocked cur_max_a off the 2.64A pinned safety-trip edge (det gate 0/6 over_current, was 6/6 at k=2.0), but the robot at 2M is just holding a static all-legs-sacrificed crouch, not walking. This is the full acquisition-budget continuation the canary's own PASS branch calls for: same checkpoint, same single lever (k=10.0), no other change, extended to the SAME 38-40M budget the k=2.0 trio got, to see whether real six-leg walking emerges once cmd_prog actually starts moving, or whether the over_current exploit re-locks in once the policy commits to a gait (the exact way the k=2.0 lineage's acq1r2 continuation re-converged to the pinned signature by 20-38M despite looking clean early).

**gate**: PASS: by ~38-40M, held-out 60s joygate falls stay well under the acq1/tf64-mesh-acq1 fail signature (was 38-40/48) AND walkcurr frontier/promotions move past b0 (genuine forward progress, not a frozen/parked plateau) AND det gate cur_max_a stays measurably below 2.64A (mechanism fix holds under full training pressure, not just at 2M). PARTIAL: over_current measurably better than the k=2.0 trio's matched-step trajectory but frontier still stuck at b0 -- real but incomplete, dig into the progress/current tradeoff. FAIL: over_current/cur_max_a re-converge to the SAME pinned-2.64A/locked-leg-tripod signature as the k=2.0 trio by matched steps -- even 5x dose is insufficient once the policy fully commits, and the current-dwell-charge mechanism class is closed regardless of magnitude tested so far; the standwalk teacher-distillation route becomes the next lever for this exploit.

**verdict**: 40M acquisition confirms the pre-registered FAIL branch: the 5x current-dwell-charge dose (k_walk_move_current=10.0) that looked clean at the 2M canary re-converges to the SAME pinned-2.64A/locked-leg-tripod exploit once the policy fully commits. Evidence: DR-0 gate 24/24 over_current terminations (walk+startjitter, det+sto), held-out 60s joygate 38/48 falls (dr0 24/24, dr0.5 14/24), gait_valid_frac 0.0, slip/m med 5.32 (cap 2.9), dir_err med 52.6deg (cap 40); per-leg duty_median [1.0,0.03,0.88,0.245,0.71,0.34] and sacrificed_frac up to 1.0 on leg1 -- the identical rigid rearing-tripod video signature as acq1/acq1r2/tf64-mesh-acq1/gaitgate-acq1r3. walkcurr/frontier stayed 0 across all 80 cert rounds to 40M -- never left b0. Training reward DID rise every quarter (-1338.8/-1086.2/-793.7/-570.6, the run's own pre-registered 'reward still rising' scenario) but this is the textbook 08-21 misalignment shape the run's own gate text pre-committed to reading as FAIL: charges get paid down as episodes get incrementally longer/current-avoidant without the underlying leg-sacrifice topology ever changing -- not undertraining. CLOSES k_walk_move_current across every tested dose (1x/2x/5x) and every tested combination (MLP-alone/MLP+gait-gate/transformer): the current-dwell-charge mechanism CLASS is refuted for this exploit at any magnitude tried. Per the pre-registered next-step: defer this specific over_current/leg-sacrifice exploit to the standwalk track's mesh stance-retrain + teacher-distillation route (already active, rung-7 bc-anchor arms just launched) rather than funding a 6th movecur dose or a structural per-leg current-clamp mechanism on the joystick track directly.


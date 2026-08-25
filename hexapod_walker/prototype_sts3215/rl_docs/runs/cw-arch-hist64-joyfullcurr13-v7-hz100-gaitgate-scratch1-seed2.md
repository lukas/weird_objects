# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-25T04:16:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**wandb_id**: 23i9tpjm

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe, reward.walk_gait_gate=1.0 from step 0, only --seed changes (0->2). Second leg of the seed-robustness grid alongside seed1 (now running on train-5); re-launched directly after 3 backlog-drain REFUSED attempts during a moving-HEAD window (multiple concurrent cycles snapshotting at once) -- code marker now clean (c8726f9f).

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed1: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 PASS = seed-dependent or non-working.

**verdict**: 3rd/3rd seed of the walk_gait_gate-from-scratch trio: closes the lever for good. Held-out 60s joygate falls 39/48 (dr0 alone 24/24 = every episode falls), gait_valid_frac 0.667, slip/m med 3.439 (cap 2.9), dir_err med 46.87deg (allow 40) -- worse than the ungated pre-gate ancestor (12/48 falls). Video (walk_det_0/2, walk_sto_0) shows the same signature as its siblings: starts six-leg cycling then rears back nose-up onto stiff planted hind legs and topples, never sustains real walking. Per-leg sacrificed_frac spreads 0.04-0.27 across all six legs (no single flag leg) -- the min-across-legs gate stops one leg from being fully zeroed but does nothing for the underlying over_current/rearing basin. Reward rose the whole 40M run (quarters -411/157/483/568) while joygate got no better than its two siblings -- textbook 08-21 MISALIGNED-and-refuted, not undertrained: scratch1 (39/48), seed1 (48/48), seed2 (39/48) = 0/3 PASS, joint reading settles it. reward.walk_gait_gate applied from scratch is CLOSED as a fix for the mesh-family leg-sacrifice/over_current/rearing basin on this recipe; the current-dwell-charge trio (movecur1/-gaitgate/tf64-movecur1, in flight on other pods) is the untouched next lever and needs no further seed spend here.


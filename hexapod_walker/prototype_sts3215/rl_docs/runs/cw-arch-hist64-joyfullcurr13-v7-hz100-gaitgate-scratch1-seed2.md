# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-25T04:16:10+00:00

**pod**: hexapod-mjx-train-8

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe, reward.walk_gait_gate=1.0 from step 0, only --seed changes (0->2). Second leg of the seed-robustness grid alongside seed1 (now running on train-5); re-launched directly after 3 backlog-drain REFUSED attempts during a moving-HEAD window (multiple concurrent cycles snapshotting at once) -- code marker now clean (c8726f9f).

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed1: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 PASS = seed-dependent or non-working.


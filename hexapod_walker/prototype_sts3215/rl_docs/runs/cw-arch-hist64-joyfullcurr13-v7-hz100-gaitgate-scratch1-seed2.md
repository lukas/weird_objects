# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-25T04:19:38+00:00

**pod**: hexapod-mjx-train-7

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1, still training) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe with reward.walk_gait_gate=1.0 baked in from step 0, only --seed changes (0->2). Second leg of the pre-registered seed-robustness grid (operator 08-22 batching guidance). New context (hist64-mesh-acq1 dig-in): on the mesh-default family the over_current trip is only avoidable by a properly CYCLING gait (teacher dwell 0.32s < 0.8s trip), so this grid also reads on the mesh over_current failure mode.

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0 -- matches scratch1 if scratch1 itself passes. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed1: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 = seed-dependent or non-working.

**refused_reason**: a process for cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed2 already exists on hexapod-mjx-train-8


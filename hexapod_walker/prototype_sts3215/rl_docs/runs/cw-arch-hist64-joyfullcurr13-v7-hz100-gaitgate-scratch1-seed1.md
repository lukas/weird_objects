# cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1-seed1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

<<<<<<< Updated upstream
**created**: 2026-08-25T03:54:00+00:00

**pod**: hexapod-mjx-train-1
=======
<<<<<<< Updated upstream
**created**: 2026-08-25T03:58:50+00:00

**pod**: hexapod-mjx-train-0
=======
**created**: 2026-08-25T03:57:39+00:00

**pod**: hexapod-mjx-train-2
>>>>>>> Stashed changes
>>>>>>> Stashed changes

**steps**: 40000000

**parent**: cw-arch-hist64-joyfullcurr13-v7-hz100-gaitgate-scratch1

**hypothesis**: Plain English: is the from-scratch walk_gait_gate leg-sacrifice-prevention result (gaitgate-scratch1, still training) a repeatable fix or one-seed luck? Byte-identical V7/100Hz/hist64 recipe with reward.walk_gait_gate=1.0 baked in from step 0, only --seed changes (0->1). Re-queued: the code-marker mismatch that REFUSED the earlier attempts is resolved (fresh snapshot a688b351, pod synced clean).

**gate**: PASS: DR-0 det gait_valid >=4/6, no leg pinned near-zero duty, frontier promotes past b0 -- matches scratch1 if scratch1 itself passes. FAIL: {0,2,5}-style (or any 3+ leg) sacrifice signature reproduces on this seed too. Read jointly with scratch1 and seed2: 2-3/3 PASS = robust mechanism; 1/3 or 0/3 PASS = seed-dependent or non-working, respectively.

<<<<<<< Updated upstream
**refused_reason**: hexapod-mjx-train-1 code marker dd798dc7fc6606d1e6181770d529ee6980e54e18-dirty != local HEAD dd798dc7fc6606d1e6181770d529ee6980e54e18 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).

=======
<<<<<<< Updated upstream
=======
**refused_reason**: hexapod-mjx-train-2 code marker bd571510e2c947e1365f1290433d1c85072c40ce-dirty != local HEAD cbadc81a99d2cc033a2badb06c87b3424ad2158f and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-2 (and snapshot/commit before that if the tree is dirty).

>>>>>>> Stashed changes
>>>>>>> Stashed changes

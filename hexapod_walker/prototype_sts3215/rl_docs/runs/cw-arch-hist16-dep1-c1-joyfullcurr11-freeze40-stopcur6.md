# cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T09:16:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur6

**wandb_id**: z7kv7bsw

**hypothesis**: Plain English: this cycle's stopfreeze-probe-stopcur6 proved the structural stop-hold (goal.walk_stop_freeze_s=0.4) also clears the V6 b1 stop cert on the stopcur6 checkpoint (k=6.0 current charge, the dose that carries a known leg-3 rigid-lock trade under own-DR), matching stopcur2's precert read exactly (0.0133). This is the real-training twin of freeze40 (which warm-starts from stopcur2): same freeze cfg, same 40M budget, but from the stopcur6 base, to test under real training + the full randomized joygate mix (1) whether walkcurr/frontier promotes past b1 into b2-b9 the same way, and (2) whether the freeze's forced hold at stop ALSO reduces or removes stopcur6's leg-3 sacrifice pathology (the freeze prevents the isometric fight that plausibly drives the lock) -- a question the eval-only precert probe cannot answer, only a full training + own-DR eval run can.

**gate**: PASS needs: held-out 60s joygate falls stay <=1/48 (matching stopcur6's own current-charge win, not regressed by the freeze), own-DR det gait_valid recovers toward 6/6 (i.e. the leg-3 lock is reduced/gone, not just unchanged), and walkcurr/frontier promotes past b1 (b2-b9 actually get practiced.) FAIL/PARTIAL: any regression in joygate falls vs stopcur6's 1/48, or leg-3 lock persists unchanged (freeze doesn't help that pathology, it's an orthogonal fix), or frontier still stuck at b1.


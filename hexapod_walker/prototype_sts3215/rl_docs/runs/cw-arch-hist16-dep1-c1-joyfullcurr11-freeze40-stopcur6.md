# cw-arch-hist16-dep1-c1-joyfullcurr11-freeze40-stopcur6

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAIL

**created**: 2026-08-24T09:16:40+00:00

**pod**: hexapod-mjx-train-0

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr9-stopcur6

**wandb_id**: z7kv7bsw

**hypothesis**: Plain English: this cycle's stopfreeze-probe-stopcur6 proved the structural stop-hold (goal.walk_stop_freeze_s=0.4) also clears the V6 b1 stop cert on the stopcur6 checkpoint (k=6.0 current charge, the dose that carries a known leg-3 rigid-lock trade under own-DR), matching stopcur2's precert read exactly (0.0133). This is the real-training twin of freeze40 (which warm-starts from stopcur2): same freeze cfg, same 40M budget, but from the stopcur6 base, to test under real training + the full randomized joygate mix (1) whether walkcurr/frontier promotes past b1 into b2-b9 the same way, and (2) whether the freeze's forced hold at stop ALSO reduces or removes stopcur6's leg-3 sacrifice pathology (the freeze prevents the isometric fight that plausibly drives the lock) -- a question the eval-only precert probe cannot answer, only a full training + own-DR eval run can.

**gate**: PASS needs: held-out 60s joygate falls stay <=1/48 (matching stopcur6's own current-charge win, not regressed by the freeze), own-DR det gait_valid recovers toward 6/6 (i.e. the leg-3 lock is reduced/gone, not just unchanged), and walkcurr/frontier promotes past b1 (b2-b9 actually get practiced.) FAIL/PARTIAL: any regression in joygate falls vs stopcur6's 1/48, or leg-3 lock persists unchanged (freeze doesn't help that pathology, it's an orthogonal fix), or frontier still stuck at b1.

**verdict**: FAIL: both pre-registered FAIL branches fired -- held-out joygate falls regress 1/48 (stopcur6 parent) -> 6/48 (5/6 over_current, 1 tilt_roll), and the leg-3 lock is numerically UNCHANGED (own-DR det gait_valid 2/6, sac=[3], video-confirmed) -- while the freeze's curriculum win reproduces (frontier b1->b5, 4 promotions/0 rollbacks, b1 cert held 72/79). DIG-IN FINDINGS: (1) CFG DRIFT -- the respec inherited freeze40's reward cfg, so this 'k=6.0 twin' actually TRAINED at k_walk_stop_current=2.0 (W&B config confirms); the prior triage's 'dose-invariant across k=2.0/k=6.0' claim is WRONG -- the correct claim is INIT-invariant: the training-time-freeze regression reproduces from a second, differently-trained warm-start (stopcur6 weights), both runs at k=2.0. The class-stop on training-time freeze stands on freeze40's controlled 2x2 + this second-init reproduction. (2) NEW B-CONTROL (this dig-in, same held-out seeds 90000, artifacts logs/ckpt_eval/cw_arch_hist16_dep1_c1_joyfullcurr11_freeze40_stopcur6_joygate_freezeoff/): twin ckpt + freeze-OFF eval = 3/48 (0 falls at DR-0, dir_err dr0 36.65deg == parent 36.02; 3 over_current at dr0p5) vs freeze-ON 6/48 vs parent 1/48. Unlike freeze40 (whose DR-0 det falls persisted freeze-off), THIS checkpoint's DR-0 regression is entirely eval-mechanism-caused; the weights damage expresses under own-DR only (1->3 over_current with the mechanism off) -- milder but same training-data-corruption root cause, and the certfreeze repair (cert-only freeze, training freeze off) covers both expressions. (3) BONUS: 40M steps at the REDUCED k=2.0 dose did NOT undo the leg-3 lock (identical 2/6) -- the lock is baked into the stopcur6 weights, not sustained by ongoing k=6.0 reward pressure; dose-reduction alone is refuted as a lock repair, it needs its own lever. NEXT: certfreeze (running, train-1) is the repair arm; training-time freeze (--cfg-set goal.walk_stop_freeze_s>0) stays CLOSED as a class; leg-lock repair is a separate future arm off the stopcur2 lineage.


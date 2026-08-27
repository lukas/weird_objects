# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - INFRASTRUCTURE

**created**: 2026-08-27T04:07:37+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit

**wandb_id**: oek1m6qy

**hypothesis**: Seed-1 twin of anchor6-logstdsplit (see that entry for the full hypothesis): per-core log_std split (separate learnable log_std_b for the stance core, mixed by the same mode gate as mean/value) + anneal targeting ONLY the stance core toward -4.0, warm-started from anchor2-s1's own leak-fixed checkpoint. Tests whether the FULL PASS (hold/sto <=2/6 term AND walk gait_valid>=5/6 both seeds) replicates cross-seed, per this campaign's established 2-seed-per-arm convention.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. JOINT call with seed-0 (anchor6-logstdsplit) -- same clauses: FULL PASS = WALK-SURVIVES (det gait_valid >=5/6, prog_ratio >=~0.2, no leg-sacrifice freeze) AND HOLD-HELPS-FULL (hold/sto DR-0 term <=2/6) on BOTH seeds. PARTIAL if hold improves less than anchor4-stdanneal's own result but beats the anchor2/3 6/6 baseline while walk survives. FAIL if walk still shows the catastrophe or hold shows zero improvement on this seed.

**verdict**: CANARY FAIL - INFRASTRUCTURE (CORRECTED by dig-in; seed1 twin — supersedes the earlier 'cross-seed replication closes the exploration-noise theory' read): INVALID as a test of the per-core log_std split for the same wiring bug as seed0. This checkpoint's saved policy_kwargs are {'lstm_hidden_size': 256} (no log_std_split), no log_std_b tensor exists, and the single shared log_std sits at exactly -4.0 — the plain --init-from warm start rebuilt the policy from the parent's own policy_kwargs (dropping --gru-dual-log-std-split) and the 'stance-only' anneal silently cooled the one SHARED log_std both cores sample from. What both seeds actually replicated is anchor4-stdanneal (shared -4.0 anneal: hold improves, walk collapses); the split itself was never tested on either seed and the exploration-noise-starvation theory remains OPEN, not refuted. Full forensics + code fixes (commit 4fe10154: enable_log_std_split retrofit on warm starts, fail-closed --log-std-anneal-core) in the seed0 verdict. Next: fixed-code relaunch pair anchor6b-logstdsplit-fix{,-s1} answers the original question.


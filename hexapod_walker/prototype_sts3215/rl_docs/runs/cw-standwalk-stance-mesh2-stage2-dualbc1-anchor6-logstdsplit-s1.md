# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY FAIL - MECHANISM

**created**: 2026-08-27T04:07:37+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor6-logstdsplit

**wandb_id**: oek1m6qy

**hypothesis**: Seed-1 twin of anchor6-logstdsplit (see that entry for the full hypothesis): per-core log_std split (separate learnable log_std_b for the stance core, mixed by the same mode gate as mean/value) + anneal targeting ONLY the stance core toward -4.0, warm-started from anchor2-s1's own leak-fixed checkpoint. Tests whether the FULL PASS (hold/sto <=2/6 term AND walk gait_valid>=5/6 both seeds) replicates cross-seed, per this campaign's established 2-seed-per-arm convention.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. JOINT call with seed-0 (anchor6-logstdsplit) -- same clauses: FULL PASS = WALK-SURVIVES (det gait_valid >=5/6, prog_ratio >=~0.2, no leg-sacrifice freeze) AND HOLD-HELPS-FULL (hold/sto DR-0 term <=2/6) on BOTH seeds. PARTIAL if hold improves less than anchor4-stdanneal's own result but beats the anchor2/3 6/6 baseline while walk survives. FAIL if walk still shows the catastrophe or hold shows zero improvement on this seed.

**verdict**: CANARY FAIL - MECHANISM (seed1 twin, CROSS-SEED REPLICATION of anchor6-logstdsplit's seed0 read below). Result: WALK-SURVIVES clause fails just as hard as seed0, in fact worse -- walk/det gait_valid 0/6 at DR-0 with 4-5-LEG sacrifice every episode ([0,2,3,5] or [0,2,3,4,5], prog_ratio pinned at -0.02, i.e. zero net motion), walk/sto also 0/6 gait_valid with the same sacrifice pattern and 2/6 raw terminations. Same class of failure as seed0 (total leg-sacrifice freeze under the per-core log_std split), just a different specific leg subset -- this is the cross-seed replication the gate's JOINT call needed. Why/what's next: see anchor6-logstdsplit's own verdict (same cycle) for the full mechanism reasoning -- both seeds confirm the per-core split does NOT rescue walk from the anchor4-class catastrophe, closing the exploration-noise-starvation theory as walk's failure cause on BOTH seeds. JOINT CALL: CLOSED, FAIL -- do not fund a third log_std configuration (shared or split) on this coef=3.0 dual-core recipe; DIG-IN owns the next step (verify log_std_b wiring, then investigate shared critic/trunk).


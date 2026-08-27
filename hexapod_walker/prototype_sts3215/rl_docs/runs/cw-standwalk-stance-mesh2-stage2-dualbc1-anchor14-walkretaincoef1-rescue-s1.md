# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS - MECHANISM (RESCUE) - JOINT CLOSE

**created**: 2026-08-27T17:31:26+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor11-walkretain-s1

**wandb_id**: b9y6ry0s

**hypothesis**: Seed1 twin of anchor14-walkretaincoef1-rescue: same single-lever change (train.bc_anchor_walk_coef=1.0) on anchor6b-logstdsplit-fix-s1's checkpoint -- THIS is the seed that actually carries the anchor4-class catastrophe (gait_valid 0/6, sacrificed legs [0,1,2,4,5]) the whole rescue thread targets, so this run's own read is the most decision-relevant half of the joint call.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. Same gate as cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue (read that run's ledger text) -- this is the seed1 half (the catastrophe seed itself) of the joint 2-seed RESCUE-PASS/PARTIAL/FAIL call.

**verdict**: CANARY PASS - MECHANISM (RESCUE), JOINT CLOSE with already-verdicted anchor14-walkretaincoef1-rescue (seed0, own-scope CANARY PASS). Plain English: the lighter per-mode walk-anchor dose (coef=1.0) genuinely rescues the real seed1 catastrophe (the anchor6b-logstdsplit-fix-s1 lineage that froze with legs sacrificed) into a clean six-leg walking gait, while seed0 stays in its own recovered band -- the dose window this wave was hunting for exists. Evidence (own harness, this run): DR-0 gate det walk gait_valid 6/6, sacrificed_legs=[] on ALL 6 episodes (clears the gate's >=5/6-zero-sac bar cleanly), progress_ratio 0.10 (med), slip/m 10.13 (med). Own-DR(0.5) det walk: gait_valid 6/6, sac=[], progress_ratio 0.16 (med), slip/m 6.81 (med) -- matches/slightly beats anchor11-walkretain-s1's own already-closed rescue shape (0/6->6/6 gv, prog 0.16) at a LIGHTER, decoupled dose. Stochastic passes stay gait_valid (6/6, sac=[]) but low-progress/high-slip (prog 0.02-0.03, slip ~28-30) -- expected at a 2M mechanism-health canary, not part of the gate bar. Video (contact sheets + walk_det/walk_startjitter frame strips, both DR-0 and own-DR) confirms a genuine alternating six-leg gait, upright, no drag or persistent leg lift -- matches the numbers, no unwatched-success risk. Combined with the already-recorded seed0 half (own-scope CANARY PASS, prog 0.178-0.181 clearing its own >=0.18 clause), BOTH halves of this run's pre-registered gate clear: RESCUE-PASS. Why: the walk teacher's own training-time dose (1.0), decoupled per-mode from stance's 3.0, is enough to both prevent (seed0) and reverse (seed1) the anchor4-class leg-sacrifice catastrophe without paying anchor11/12's full quality tax -- refutes 'the anchor's mere presence is the cost' as the whole story; dose DOES matter once decoupled per mode. Cross-recipe flag (carried from seed0's own verdict, not contradicted here): anchor13's control-recipe pair under the same coef=1.0 landed at a similar absolute prog_ratio despite a healthier starting ceiling -- worth an audit, but this IS the catastrophe-prone lineage that control pair wasn't, so the rescue read stands on its own gate. What's next: per this run's own pre-registered gate text, RESCUE-PASS funds a 2x-seed acquisition wave -- continuing both cw-standwalk-stance-mesh2-stage2-dualbc1-anchor14-walkretaincoef1-rescue{,-s1} checkpoints (warm-start, coef=1.0 recipe unchanged) to an 8M acquisition budget to see whether progress_ratio/slip continue improving past this 2M snapshot (both seeds' reward quarters still show the anchor4-class trough-then-partial-Q4-recovery shape, consistent with continuing per the 08-21 ruling, not a plateau).


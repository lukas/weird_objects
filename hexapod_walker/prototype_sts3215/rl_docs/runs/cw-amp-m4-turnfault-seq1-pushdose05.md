# cw-amp-m4-turnfault-seq1-pushdose05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-23T04:41:16+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-amp-m4-turnfault-seq1-pushcont1

**hypothesis**: Plain English: same push-dose-sensitivity test as pushdose025, at the middle of the range. pushcont1 (dose=1.0, fault-first order) softened but did not fix push-driven turn erosion (0.27/0.30 vs fault-only parent's 0.18/0.17). Single lever vs pushcont1: dr.ext_push_prob 1.0 -> 0.5. Re-inits from the SAME pre-cheat turnfault-seq1 checkpoint, per the init-basin rule.

**gate**: Same gate as pushdose025: own-cfg DR-0 floor gait_valid>=9/12; eval_yaw tip-left/right err PASS-clean<=0.20-0.25, PARTIAL between that and 0.27/0.30, FLAT ~unchanged. eval_amp_m5 push section at eval-time ext_push_prob=1.0 confirms push-recovery survives the lower training dose.

**refused_reason**: hexapod-mjx-train-1 code marker b126ceb3191b86ee7a267146a88a3d84b8358c40 != local HEAD 871d6db40f82d3aa09878905fa508bea890f4944 and the delta is not benign-orchestrator-only. Sync first: snapshot.sh --sync hexapod-mjx-train-1 (and snapshot/commit before that if the tree is dirty).


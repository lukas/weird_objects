# cw-standwalk-stance-mesh2-stage2-dualbc1-anchor3-s1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: CANARY PASS

**created**: 2026-08-26T19:20:43+00:00

**pod**: hexapod-mjx-train-1

**steps**: 2000000

**parent**: cw-standwalk-stance-mesh2-stage2-dualbc1-anchor2-s1

**wandb_id**: 95em5s9b

**hypothesis**: Seed twin of cw-standwalk-stance-mesh2-stage2-dualbc1-anchor3 (identical recipe, seed 1) -- cross-seed replication for the joint dose-response call. Plain English: tests whether doubling the stance-only BC anchor coef (3.0->6.0), now that the shared-Adam momentum-leak fix (isolate_update=1) is confirmed on both seeds, finally protects hold/lower (stuck at a clean total hold_min_load term 6/6 both DR in anchor2/-s1) without re-wrecking walk.

**gate**: MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY: do not judge skill acquisition, close a behavior/reward class, or require mature gait at this checkpoint. MECHANISM-HEALTH CANARY ONLY. Same as anchor3; joint call reads both seeds together (LEAK-STAYS-FIXED PASS / DOSE-WORKS / FAIL branches as registered there).

**verdict**: CANARY PASS (LEAK-STAYS-FIXED at 2x dose, joint w/ anchor3) with a flagged caution -- DOSE-WORKS branch FAILS, same as seed0. walk/det stays clean (gait_valid 6/6, prog 0.39-0.40, zero sacrificed legs) -- no anchor1-class det catastrophe, mechanism intact. CAUTION (new this seed): walk/sto degraded from anchor2-s1's clean 6/6 gait_valid to 2/6 here, with 3/6 episodes now hitting over_current and 1-2 sacrificed legs each (sac[2,4,5], sac[5], sac[2,5]) -- worse than seed0's clean walk/sto (6/6 valid, no sac) at the same dose. This is NOT the full anchor1 catastrophe (det is unaffected, no freeze/shuffle signature) but is a real seed-specific fragility increase on the stochastic axis at 2x dose -- treat as a soft warning against pushing bc_anchor_coef higher without re-checking this seed. hold/sto unchanged total 6/6 hold_min_load term (identical to seed0 and to anchor2) -- coef confirmed not the lever for stance. Joint call with anchor3: overall CANARY PASS on mechanism, FAIL on the dose-works question; do not raise this coef further, pivot to std-anneal for the stance axis.


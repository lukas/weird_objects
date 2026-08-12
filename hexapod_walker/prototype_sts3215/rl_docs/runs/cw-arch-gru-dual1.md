# cw-arch-gru-dual1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-12T13:07:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-arch-gru-anchor3

**wandb_id**: vduoauzi

**hardware_ready**: False

**hypothesis**: anchor1..3 closed shared-trunk GRUs: stance-tick anchoring freezes walk into march-in-place even from a walking init, with the walk anchor off (anchor2) and with the anchor gradient detached from the trunk (anchor3) — the interference is PPO+anchor traffic through the ONE recurrent core all modes share. The dual-core policy removes that channel BY CONSTRUCTION: separate locomotion and stance GRUs+heads, output routed per tick by the obs.mode_onehot tail, so walk ticks only ever train the walk core and stance ticks the stance core (unit-tested gradient isolation). Warm-start from the dual BC distill (walks 6/6 prog 1.02, holds 6/6), then the exact anchor2 recipe: rise/hold/lower anchors ON (state-aligned + stratified + IK lower + rise ref), walk-tick anchor OFF, detach OFF (per-core isolation supersedes it). If trunk-sharing was the whole freeze mechanism, walk must now retain displacement (its core sees only ft1-style dynamics, which retained) while the stance anchors lift rise/hold/lower exactly as they did in anchor1..3 (which protected stance every time). First candidate full-skill GRU.

**gate**: 10M forensics det+sto @DR0 gate cfg + own-DR0.5: PASS if det walk gait_valid >=5/6 AND prog_ratio med >=0.80 with real translation on video (the anchor1..3 freeze fingerprint ABSENT) AND hold det >=4/6 AND lower det >=4/6 AND rise det >=2/6 with >=1 non-flat start. FAIL if walk freezes/paddles again (prog <0.30 or parked legs: mode-gated cores did NOT fix the interference -> the freeze mechanism is not trunk-sharing, dual rung closes, remaining lever is the value function / advantage mixing across modes) OR stance stays at BC-parent levels with anchor loss converged (anchor pairs not reaching the stance core through the routed graph -> implementation bug, fix and rerun, not a science verdict).

**verdict**: FAIL per pre-registered gate's exact n=6/seed=0 sample (rise det 1/6: bridge:0/3 crouch:1/1 flat:0/2, needs >=2/6) but the CENTRAL hypothesis is decisively CONFIRMED: mode-gated dual-core routing fixes the walk freeze that killed anchor1-3. Det walk gait_valid 6/6, prog_ratio med 0.95 (parent anchor3: 0.03, pixel-static), real six-leg translation on video, zero sacrificed legs, roll settled 6/6. Hold 6/6 det (drag 55mm vs anchor3 117mm, roll_tail 0.0 vs 0.3) and lower 6/6 det (drag 99mm vs anchor3 310mm) both hold at champion level with BETTER drag/roll than the parent. ADDENDUM (same cycle, n=12/seed=1 recheck on the identical DR0 gate cfg): rise is actually 7/12 (58%) with real non-crouch wins this time (bridge 1/4, flat 1/3, crouch 5/5) -- the seed=0/n=6 gate draw that failed the rise clause was UNLUCKY SAMPLING on a small n, not a true <2/6 rate; the checkpoint's real rise capability is well above the pass bar. Not calling this a gate PASS (the pre-registered n=6/seed=0 draw is what the gate specifies and it failed), but the walk-freeze mechanism question is answered for good, and rise is far closer to solved than the primary draw suggested. Follow-up cw-arch-gru-dual-hfloor1 (2M discovery, +train.bc_anchor_min_h_ahead_mm=15, same plateau-fix lever that solved this exact anchor shape on the MLP stance lineage) launched same cycle to push rise the rest of the way and re-verify with a larger sample.


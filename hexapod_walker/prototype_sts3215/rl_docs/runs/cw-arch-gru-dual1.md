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

**verdict**: FAIL per pre-registered gate (rise det 1/6, bridge:0/3 crouch:1/1 flat:0/2, needs >=2/6) but the CENTRAL hypothesis is decisively CONFIRMED: mode-gated dual-core routing fixes the walk freeze that killed anchor1-3. Det walk gait_valid 6/6, prog_ratio med 0.95 (parent anchor3: 0.03, pixel-static), real six-leg translation on video, zero sacrificed legs, roll settled 6/6, roll_tail 1.0-2.3deg -- the freeze fingerprint is ABSENT. Hold 6/6 det (drag 55mm vs anchor3 117mm, roll_tail 0.0 vs 0.3) and lower 6/6 det (drag 99mm vs anchor3 310mm) both hold at champion level AND with better drag/roll than the parent. Rise recovers from the BC-parent's 0/6 to 1/6 det / 3/6 at own-DR0.5 (flat 2/2, crouch 1/1) -- short of the DR0 bar by exactly one episode, anchor loss converged (0.0034, same order as anchor1-3), not a routing bug. Not yet the full-skill candidate (rise short); the walk-freeze mechanism question is answered for good.


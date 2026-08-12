# cw-arch-gru-dual1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-12T13:07:25+00:00

**pod**: hexapod-mjx-train-1

**steps**: 10000000

**parent**: cw-arch-gru-anchor3

**wandb_id**: vduoauzi

**hypothesis**: anchor1..3 closed shared-trunk GRUs: stance-tick anchoring freezes walk into march-in-place even from a walking init, with the walk anchor off (anchor2) and with the anchor gradient detached from the trunk (anchor3) — the interference is PPO+anchor traffic through the ONE recurrent core all modes share. The dual-core policy removes that channel BY CONSTRUCTION: separate locomotion and stance GRUs+heads, output routed per tick by the obs.mode_onehot tail, so walk ticks only ever train the walk core and stance ticks the stance core (unit-tested gradient isolation). Warm-start from the dual BC distill (walks 6/6 prog 1.02, holds 6/6), then the exact anchor2 recipe: rise/hold/lower anchors ON (state-aligned + stratified + IK lower + rise ref), walk-tick anchor OFF, detach OFF (per-core isolation supersedes it). If trunk-sharing was the whole freeze mechanism, walk must now retain displacement (its core sees only ft1-style dynamics, which retained) while the stance anchors lift rise/hold/lower exactly as they did in anchor1..3 (which protected stance every time). First candidate full-skill GRU.

**gate**: 10M forensics det+sto @DR0 gate cfg + own-DR0.5: PASS if det walk gait_valid >=5/6 AND prog_ratio med >=0.80 with real translation on video (the anchor1..3 freeze fingerprint ABSENT) AND hold det >=4/6 AND lower det >=4/6 AND rise det >=2/6 with >=1 non-flat start. FAIL if walk freezes/paddles again (prog <0.30 or parked legs: mode-gated cores did NOT fix the interference -> the freeze mechanism is not trunk-sharing, dual rung closes, remaining lever is the value function / advantage mixing across modes) OR stance stays at BC-parent levels with anchor loss converged (anchor pairs not reaching the stance core through the routed graph -> implementation bug, fix and rerun, not a science verdict).


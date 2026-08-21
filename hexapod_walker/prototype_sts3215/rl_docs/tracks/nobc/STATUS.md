# nobc - learn without BC anchors

Last compacted: 2026-08-20 UTC. Excess-capacity research track. Not part of
the current SIM SPRINT download answer.

## Goal

Learn honest standing and gait discovery without imitation losses. The main
hardware line may still use BC anchors; this track tests whether the crutch
can eventually be retired.

## Current Sprint Rule

No new nobc launches during SIM SPRINT unless they directly serve the
rise+walk download answer or the operator explicitly orders them.

## Now

- ANTI-SLIP from-scratch walking is REOPENED by operator order 2026-08-21
  (MCP operator lane 20260821T133626Z, asked by Lukas), overriding the
  08-13 closure for this sub-line only. Goal: penalize loaded foot slip
  hard, do NOT require a target speed, move in the commanded direction,
  one fixed direction first, stage to direction changes only if the
  first rung shows real low-slip movement.
- Mechanisms built + preflighted this cycle (default-off, bit-exact when
  off): `reward.k_walk_freeprog` / `walk_freeprog_cap_m_s` (direction-
  first income, NO speed target) and `reward.k_walk_idle_charge` /
  `walk_idle_speed_m_s` / `walk_idle_tau_s` (anti-park travel floor),
  paired with the structural `k_loadslip_excess` (episode-accumulated
  loaded slip per metre) and `walk_gait_gate`. SLIPWALK bank in
  `test_task_semantics.py` pins the ordering: fast(0.44 m) +851 >
  gait(0.22 m) +417 > creep(0.16 m) +108 > stall -143 > park -244 >
  skate -1195.
- RESULT (08-21): `cw-nobc-slipwalk1-r1` FAILED its pre-registered gate —
  the freeze again. DR-0 det: travel 0.001 m/episode (bar 0.15), slip/m
  6.75 (bar 3.0) on 0.34 m of absolute scuffing, gait_valid 0/6 with four
  legs sacrificed, zero falls only because nothing moved. Sto: 0.04 m,
  slip/m 14.5. Training return fell monotonically (-70 -> -1241).
- The discriminating fact: the SPEC was clean this time. The SLIPWALK
  preflight bank (green pre-launch) prices real travel 300-2000 points
  above stall/park/skate under this exact stack, so the failure is
  EXPLORATION from a blank init, not reward specification. Six
  from-scratch gait arms now end in the same freeze.
- Sub-line STOPPED per the operator's instruction ("if it freezes/marches/
  skates like prior nobc arms, verdict it honestly and stop"). No re-run
  with more steps. DOWNLOAD_ANSWER unchanged.

## Next

- Nothing agent-doable on this sub-line. If the operator reopens it, the
  honest lever is a START THAT ALREADY MOVES (short scripted-gait
  kickstart, mid-stride RSI starts, or a physics-easing ramp), not
  another reward term — reward pricing is now preflight-proven correct.
- The new mechanisms stay in the repo, default-off and tested:
  `reward.k_walk_freeprog` / `walk_freeprog_cap_m_s` (direction-first
  income, no speed target) and `reward.k_walk_idle_charge` /
  `walk_idle_speed_m_s` / `walk_idle_tau_s` (anti-park travel floor).

## Older State

- Gait-from-scratch (generic, speed-tracking form) was closed by operator ruling on 2026-08-13; the 08-21 order reopens only the anti-slip/no-speed-target form above.
- Closed gait levers include structural drag charge, RSI mid-stride spawns, slow-first speed, warm-start anneal, true in-run coefficient scheduling, and physics easing. All collapsed to freeze, march-in-place, or unresolved skating rather than honest stepping.
- `sched.*` and `ease.*` mechanisms remain in repo, default-off, for other tracks if needed.
- Stand-from-scratch remains the live charter, but it is not the current sprint blocker.

## If Reopened

For stand-from-scratch, start from a fresh spec/preflight pass. Do not infer
that the BC-anchored stance line's success transfers to nobc without evidence.
For gait-from-scratch, require new hardware evidence before any run.

## Operator Gates

- Whether stand-from-scratch deserves capacity during SIM SPRINT.
- What new evidence would justify reopening gait-from-scratch.

Keep under 120 lines. Replace stale bullets; do not append history.

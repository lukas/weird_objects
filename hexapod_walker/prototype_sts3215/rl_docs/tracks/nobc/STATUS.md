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
- Live canary: `cw-nobc-slipwalk1` (2M, discovery, from scratch, pure
  forward command, dr 0.3). Gate is plain behavior — zero falls, median
  along-command travel >= 0.15 m per 15 s episode, direction error
  <= 30 deg, gait_valid >= 4/6, slip/m <= 3.0 with slip_m_total <= 1.0 m.
  Harness walk "success" (in-band speed) is NOT part of this gate.

## Next

- Triage `cw-nobc-slipwalk1` on video + travel/slip/direction medians.
  PASS -> stage rung 2 (two directions, then commanded changes).
  FAIL (freeze / march / skate fingerprint) -> verdict honestly and stop
  the sub-line; do not re-run with more steps.

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

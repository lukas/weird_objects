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

## Current State

- Gait-from-scratch is closed by operator ruling on 2026-08-13. Reopen only on new hardware evidence.
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

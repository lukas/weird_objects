# quad - quadruped with two hands

Last compacted: 2026-08-20 UTC. Excess-capacity track. This is not part of
the current SIM SPRINT download answer.

## Goal

Make the robot stand and walk on four support legs with the front pair lifted
as hands/arms.

## Current Sprint Rule

No new quad launches during SIM SPRINT unless the operator explicitly orders
one or it directly improves rise+walk download readiness.

## Current State

- Quad hold is real and useful as a specialist skill.
- Mixing quad behavior into general walking erodes normal walk; use specialists/composition, not naive mixed training.
- Open-loop/static four-leg crawl was measured geometrically infeasible; a feedback/RL policy is required for actual four-leg walking.
- Quadwalk1-3 closed the fronts-down cheat family: pricing/lift rewards were paid or ignored, not obeyed.
- Quadwalk4-7 closed the mid-leg-sacrifice family: spawn changes, structural `walk_gait_gate`, and 20x entropy all converged to the same bad topology.
- `cw-quadwalk7` is the current terminal read: exploration-only is closed, no more entropy/coefficient scans.

## If Reopened

The next valid direction is design-level, not another reprice:

1. BC/reference from a feedback stepping controller, if one exists.
2. A staged single-leg-swing curriculum.
3. A temporal policy explicitly gated as architecture/curriculum research.

Any launch must preserve quad-hold retention and use `QUADWALK_REF_GATE.md`.

## Operator Gates

- Pick curriculum/reference/architecture direction, or park quadwalk until after the mainline robot works.

Keep under 120 lines. Replace stale bullets; do not append history.

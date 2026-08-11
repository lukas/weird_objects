# Track: quad — Quadruped with two hands

W&B filter: tag `track:quad` in l2k2/hexapod-balance.

## Goal (operator, 08-11)

Learn walking on four legs with the two front legs lifted as
hands/arms: stand on four, walk on four, front pair free for tricks.
Runs on excess capacity.

## Current state (update when a verdict changes the story)

- 08-10: Four-leg HOLD mechanism is solid (cw-quad-hold1-r2:
  survived_frac 1.0, level body, fronts lifted, no tipping) but the
  50/40/10 quad/walk/hold mix eroded walk retention below the
  standard gate; the dose-dependence rung (quad 0.5->0.3) was queued
  as cw-quad-hold2. Quad is also a MAINLINE joystick command
  (drive_policy key `4`) — deploy integration of a PASSING quad
  checkpoint belongs to hw.
- Open: walk-retention vs quad-mix dose; four-leg WALKING (never
  attempted beyond hold); front-pair posture control while walking.

## What belongs here

Quad-mode goal-mix arms, four-leg gait discovery, front-leg posture
work, quad-specific eval modes. Detail: ledger cw-quad-* lineage.

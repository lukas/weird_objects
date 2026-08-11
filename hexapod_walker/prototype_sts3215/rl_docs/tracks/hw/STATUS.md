# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- Sim side largely solved: rise + quiet hold (stand_holdbc1_hard1),
  walk champion `cw-dep-vref1-r1`, rot-60 full-circle wrapper, full
  cycle rise→walk→stop→sit composes with zero falls.
- Critical path is BENCH-ONLY: hardware attempt #2 (fresh set_zero,
  matching tilt trip, re-push the profile-carrying stand export).

## Next

- Hardware attempt #2 (operator-supervised) → tape-measure walk
  distance, wz sign audit.
- Gait cleanup for hardware (anti-scrape): GAIT.md P1/P2.
- Crouch-start rise: crouchrise2 reproduced the rise win but the
  flag-leg hold cheat returned — root cause open before deploy.

Detail: RL_PLAN.md queue · rl_docs/HARDWARE.md · RISE.md · GAIT.md.

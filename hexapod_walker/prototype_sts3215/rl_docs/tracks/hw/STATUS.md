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
- Gait cleanup (anti-scrape): P0 diagnostic DONE 08-11 late (tilt
  penalties exonerated; paddle is a sim-effectiveness optimum —
  GAIT.md bottom). Structural per-stance charge, FROM SCRATCH
  (`cw-gait-dragstance1`, audit-derived k=8000) FAILED 08-11: parked
  motionless instead of stepping, paying the charge the whole
  episode rather than resolving it (its own pre-registered false
  branch, verbatim) — GAIT.md. CROSS-TRACK: this is also nobc's
  drag-charge-audit item, same conclusion both tracks. Warm-start
  companion `cw-walk-dragstance1` (audit-derived k, on the actual
  champion) still training — a policy that already travels may
  answer differently since freezing forfeits real walk income it
  would otherwise keep collecting.
- Crouch-start rise: the fix works (crouchrise1/2/3 all rise from
  crouch) but EVERY dose (0.60, 0.60+mix-restore, 0.45 — crouchrise3,
  08-11) reproduces the identical legs-1+4 flag-leg hold cheat; the
  dose/mix axes are closed. Next lever is CODE, spec first:
  state-aligned BC anchor (clock-indexed anchor mis-teaches
  plant-adjacent states on crouch starts). hard1 stays deployed.

Detail: RL_PLAN.md queue · rl_docs/HARDWARE.md · RISE.md · GAIT.md.

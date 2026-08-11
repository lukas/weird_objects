# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **A learned policy HAS driven the robot (08-10 night, HARDWARE.md):**
  `dep-tip1` 3 clean level walks / 1 runaway; parent `vref1-r1` 0/2
  with runaway roll (pinned-leg feedback, sim-to-real contact gap —
  sim recovery exploits skating feet, rubber on grip can't). Obs
  pipeline verified bit-exact. Do NOT describe RL walking as
  "pending bench time" — the open question is the intermittent
  runaway, not whether it walks.
- Sim side solved: rise + quiet hold (stand_holdbc1_hard1), rot-60
  full-circle wrapper, full cycle rise→walk→stop→sit composes with
  zero falls. rot60 + stand specialist are PORTED but not yet
  hardware-run (deploy re-push required; stale on-robot stand copy
  lacks its goal profile — never press STAND on it).

## Next

- Next bench session (operator-supervised): vref1-r1 vs tip1 same-
  floor A/B (roll-ramp RATE, runaways per N — the 08-10 A/B never
  actually switched policies), RL-walk tape reading, first runs of
  rot60 + stand specialist after the re-push, wz sign audit. If tip1
  also runs away: sim contact/pinning model (no-skate feet), not DR.
- Gait cleanup (anti-scrape): P0 diagnostic DONE 08-11 late (tilt
  penalties exonerated; paddle is a sim-effectiveness optimum —
  GAIT.md bottom). Structural per-stance charge, FROM SCRATCH
  (`cw-gait-dragstance1`, audit-derived k=8000) FAILED 08-11: parked
  motionless instead of stepping, paying the charge the whole
  episode rather than resolving it (its own pre-registered false
  branch, verbatim) — GAIT.md. CROSS-TRACK: this is also nobc's
  drag-charge-audit item, same conclusion both tracks. Warm-start
  companion `cw-walk-dragstance1` (same k, on the actual champion)
  also FAILED, the other way: it neither parked nor stepped — kept
  full travel and simply absorbed −7/tick for 2M (slip only 1.1–1.3 →
  0.95–1.15). Static fine at either init is closed; the from-scratch
  40M `cw-gait-dragstance1-r1` (running) and anneal-up curriculum
  carry the lever.
- Crouch-start rise: the fix works (crouchrise1/2/3 all rise from
  crouch) but EVERY dose (0.60, 0.60+mix-restore, 0.45 — crouchrise3,
  08-11) reproduces the identical legs-1+4 flag-leg hold cheat; the
  dose/mix axes are closed. **08-11 later: the reward-pricing lever
  is closed too** (`cw-stand-holdload1` — measured-foot-load income
  correctly taxes the hover per its own bank, but the identical
  legs-1+4 park reproduces anyway, det duty 0.03–0.04, `valid_plant`
  blind to it mid-episode). **State-aligned BC anchor tested
  (`cw-stand-anchorstate1`, 08-11): PARTIAL confirmation** — leg 4
  recovers (duty 0.01→0.93) but leg 1 still parks, and the fix
  stalls flat-start rise + adds lower falls. Follow-up
  `cw-stand-anchorstate2` (lookahead dose, running) tests whether
  that trade-off resolves. hard1 stays deployed. RISE.md.

Detail: RL_PLAN.md queue · rl_docs/HARDWARE.md · RISE.md · GAIT.md.

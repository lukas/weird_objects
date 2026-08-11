# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **08-11 eve: fully-unattended camera bench IS the workflow now**
  (`bench_blast --go --auto --camera 0`: iMac camera records the whole
  session, exact unix sync, video_review cuts the sheets; fall-detect →
  safe_zero → stand recovery loop; terminal results recorded, never
  kickoff responses). Three unattended sessions run 08-11 eve
  (hardware_traces/bench_blast_20260811_18*).
- **Walking on hardware (08-11 eve, on camera):** both policies show a
  large TAKEOFF roll transient. vref1-r1: one clean-start fall (its 3rd
  runaway) and one full-6s walk that rode a 23–24° early transient and
  recovered to dead level — the "runaway" flag conflates recoverable
  transients with tips; judge by fell/tail. tip1 fwd tripped tilt_roll
  2/3 in the 21:4x attended A/B (robot's own log; the old "3 clean
  walks" summary was kickoff-response fiction). **First off-wedge rot60
  run (tip1 BACKWARD): FELL** (peak 27°) — rot60 port itself works
  (k engaged, terminal result logs it).
- **Stand specialist port: first honest hardware run FAILED with a
  REAL gap** (08-11 22:42): tilt_roll trip at 10.2° during the
  belly-curl. Sim probe: the same rise keeps roll ≤1.7° across 6 det
  seeds — hardware rocks over the tucked legs, sim doesn't. Trip
  threshold is correct; fix is training-side (rocking/tilt DR on rise
  ticks, loaded-knee actuator), NOT a threshold bump.
- 08-11 22:29 incident (resolved): unattended session 1 had no upright
  gate between steps → post-fall walks/turns ground the sprawled legs →
  board brownout; operator power-cycled, 18/18 healthy. The recovery
  loop + SessionAbort added in response and validated live in session 3.

## Next

- Sim-side (launchable): (1) rise-tick rocking robustness — train the
  stand specialist with roll/tilt perturbation DR during the curl
  (loaded-knee actuator params as a second axis); gate = rise under
  ±10° rocking injections. (2) Takeoff-transient hardening for walk —
  episodes starting at the plant with the measured 20–25° takeoff roll
  injected; both policies must recover like vref1-r1-184741 did.
  (3) rot60 backward: one fall is one data point — needs reps, but
  after (2).
- Bench (cheap, unattended OK now): more fwd A/B reps with the
  recovery loop to get honest fell-rates per policy; wz turn-sign
  audit (still open — both attempts died to the brownout/abort);
  learned-lower retry ONLY after the over_load trip is understood.
- Runaway metric fix in bench_blast: split "recovered transient"
  (peak high, tail level) from "fell" (terminal result / tail high).
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
- **TALL LADDER (walk from a taller stance, same problem as
  anti-scrape): the wall is HABIT not kinematics** (`probe_tall_wall.py`,
  08-11 — GAIT.md/RL_PLAN queue -0.5). Ref-tracking alone is tradeable
  for speed (T1); a reachable income gate (T3, `cw-dep-tall-gate1`)
  buys 15mm at 2M but the trade WINS BACK under a 6M hardening budget
  (`cw-dep-tall-gate1-h1`, confirmed 08-11 late): steady-state walking
  height -72.6mm, statistically unchanged from the ungated -75mm wall,
  legs still pinned at the 35° yaw-splay limit (lateral-stability
  purchase). Gate-income alone CLOSED at this dose. **08-11 late:
  PRICING FAMILY CLOSED FOR POSTURE** — kh3 (-74.5mm), kh10 (-72.7mm,
  a 10x height charge that pays MORE than walk income rather than
  stand up), slow1 (-73.8mm, didn't even adopt its eased 0.03-0.04
  speed band, still walking 0.048-0.051) all flat at -72..-75mm, leg
  yaw pinned at the 35° limit in all six pricing arms tried (ref
  ladder, income gate, gate+budget, height 3x/10x, speed relief). The
  optimizer cannot FIND the taller basin at any dose — it isn't
  underpaying for it. Next lever is RSI-for-walk, not more pricing:
  `cw-dep-tall-rsi1` (running) spawns episodes mid-stride in the
  scripted gait's tall pose instead of asking the policy to discover
  it from a standing start. If flat even with direct state injection,
  the wall is dynamic stability itself (physics easing / taller
  scripted reference), not reward work.
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
  stalls flat-start rise + adds lower falls. **`cw-stand-anchorstate2`
  (lookahead 0.25→0.5s) fixes the flat-rise stall and the lower falls
  exactly as hypothesized, but leg 1 still parks (duty 0.03) — sixth
  run in a row, lookahead axis now EXHAUSTED for the park.**
  Follow-up `cw-stand-loweranchor1` (BC-anchor the LOWER ticks toward
  the lower bank's own honest IK descent — the last undocumented
  incentive gap) launched, running. hard1 stays deployed. RISE.md.

Detail: RL_PLAN.md queue · rl_docs/HARDWARE.md · RISE.md · GAIT.md.

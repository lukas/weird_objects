# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **08-11 eve session 2 (19:07–19:19, four camera sessions,
  bench_blast_20260811_19*): learned rise is DETERMINISTIC-FAIL on
  hardware** — 5/5 tilt_roll trips (incl. 22:42's), every one at tick
  ~227 (~9 s, mid-curl) with roll 10.1–10.6° and currents ≤0.27 A.
  From verified clean zero (max pose delta 0.5°), so start pose is
  exonerated; sim keeps the same rise ≤1.7° roll. This is THE stand
  blocker; the queued `cw-stand-riserock1` drained as a STUB (the
  rocking-DR code was never written — run VOID, no science); the
  rise-rock DR axis is still unbuilt CODE work. Scripted `POST
  /api/zero pose=stand` is the working stand-up meanwhile.
- **A/B fell-rates finally honest (on camera): vref1 fell 3/3, tip1
  clean 1/1 — and that one was BACKWARD**, tip1's first clean
  off-wedge rot60 walk (rode a 16.7° takeoff transient, tail 1.5°,
  ends standing in frame). vref1's falls all follow the same 20–25°
  takeoff-transient-then-capsize shape. The queued takeoff arm
  drained as a STUB (default DR — VOID); the proper relaunch
  `cw-dep-tip1-takeoff25-r1` FAILED with a decisive read: under the
  identical 20–25° injection vs matched tip1 baseline, child==parent
  (0/12 valid both, zero falls both) — sim ALREADY recovers static
  tipped starts at the hardware regime, so the dose lever is CLOSED
  (2nd no-separation arm). The takeoff fix must be a DYNAMIC
  roll-rate perturbation during gait start (CODE) or contact/pinning
  work. tip1 is the deploy champion on tonight's evidence.
- **Thermal budget is real:** the second recovery stand-glide of the
  19:16 session limped on **L2 hip at 72 °C** (shutoff 65). Falls +
  recoveries stack heat fast; back to 40 °C within minutes limped.
  Session ended there; robot parked limped/belly-down, 18/18 healthy.
- Recovery loop hardened from tonight's failures: recovery safe_zero
  now `force=true` (a fall always trips the tilt gate), scripted-stand
  fallback when the learned rise trips, demo-aware waits (`/api/zero`
  and `safe_zero` run as demos that `wait_idle` never saw — one abort
  came from reading a mid-glide pose), auto-safe_zero when the opening
  pose isn't belly zero (an earlier stalled safe_zero left L4 knee 78°
  off and quietly hold-hunting — the "twitching leg").
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

- Sim-side (08-11 late): the two queued bench-answer arms drained as
  STUBS without their variables — both VOID (no science; verdicts in
  ledger). Proper relaunch `cw-dep-tip1-takeoff25-r1` then FAILED
  decisively (see Now bullet): tipped-start DOSE closed, sim
  saturates the static-tilt axis. BOTH remaining sim answers to the
  bench falls are now the same CODE family: dynamic roll-rate
  perturbation during gait-start ticks (walk) and rocking DR on rise
  ticks (stand) — spec + bank BEFORE launch (MDP_PREFLIGHT); neither
  is written. rot60 backward: one fall AND one clean walk — more
  reps when a takeoff-hardened checkpoint exists.
- Bench (blocked until operator resets): L2 hip hit 72 °C, so motion
  stopped for the night per safety rules. When resumed: wz turn-sign
  audit (STILL open — three sessions in a row died before reaching
  it), more A/B reps (vref1 3/3 fallen — consider dropping it from
  the rotation), learned-lower retry ONLY after the over_load trip is
  understood.
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
  40M `cw-gait-dragstance1-r1` was KILLED pre-verdict (known-exploit
  rule: identical recipe to the refuted 2M arm — RL_LOG 08-11 20:03);
  the anneal-up curriculum (CODE) carries the lever.
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
  underpaying for it. RSI-for-walk (`cw-dep-tall-rsi1`, T6)
  was the last lever and it is FLAT TOO (-77.4mm mid-gait; the
  policy learned to recover from tall mid-stride spawns DOWN into
  the crouch — verdict 08-11 22:33, ledger recorded): neither
  pricing (6 arms) nor state injection moves posture. Remaining, in
  order: BC-INIT from the scripted tall gait, physics-easing ladder,
  or accept the pareto (tall15-h1 = fastest dep walker, 0.051 m/s).
  Not a joystick blocker.
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
  incentive gap) **SOLVED lower (det+sto 6/6, zero falls, from 2/6)
  but REGRESSED hold to a two-leg park + re-stalled flat rise 96mm —
  root cause found: the three per-mode BC anchors share one ring
  buffer/uniform sampling, so lower's pair volume diluted rise/hold
  supervision (ANCHOR DILUTION, a new testable mechanism, not the
  shared-habit theory).** `cw-stand-anchormix1-r1` (stratified per-mode
  minibatch sampling, equal quotas — the direct dilution fix)
  **FAILED per its pre-registered FAIL branch (08-11 late): lower
  6/6 and crouch rise 4/4 RETAINED, zero falls, but det flat rise
  still stalls 105mm short and det hold still parks one foot (duty
  0.02, hold_feet_factor ~0.14 all run) — and the park MOVED to a
  different leg (old parkers now 0.90+), so the parked-leg identity
  is anchor-dependent, not a fixed habit. Dilution theory
  incomplete. Mandatory pre-registered next step before ANY further
  stand arm: log per-mode train/bc_anchor_loss (CODE — aggregate
  only today).** hard1 stays deployed. RISE.md.

Detail: RL_PLAN.md queue · rl_docs/HARDWARE.md · RISE.md · GAIT.md.

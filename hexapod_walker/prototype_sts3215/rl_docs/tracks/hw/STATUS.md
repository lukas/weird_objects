# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **08-11 late MODEL TOUR (all 27 deployable ckpts through the
  interactive play.py session; rl_docs/MODEL_TOUR_2026-08-11.md):
  two NEW deployed-pair defects.** (1) `holdbc1_hard1` sit from the
  142 mm walk plant frame tips tilt_pitch at ~2.5 s,
  DETERMINISTIC (10/10 + clean-stand probe) — do not command
  sit-after-walk on hardware; (2) its belly rise stalls at 55 mm
  forever under the interactive goal ramp (training-profile rise
  passes — profile overfit, a separate axis from the hardware
  rise-rock). Landed in response: `rl_move.sim.eval_session` (the
  session gate, exit-code enforced — run on every stand/sit
  candidate) + `goal.rise_ramp_jitter`/`goal.lower_ramp_jitter`
  (default-off training axis, bank green). Family-wide walk notes
  (height collapse to ~70 mm, CCW veer ~3°/s unpriced in the dep
  lineage, reverse ~20 % of command) map onto the existing tall-wall
  / yaw-lineage / heading-exposure lines — no new axes there.
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
- **FULL-NIGHT A/B (18 walks, bench_report): the takeoff transient is
  UNIVERSAL and there is NO policy winner.** Every walk crosses 5°
  roll within 0.6–1.5 s and peaks 13–27°; falls are ~a coin flip for
  BOTH policies (vref1 6/10 fell, tip1 4/7) with no predictor in peak
  size or direction. The early-evening "tip1 clean, vref1 3/3 fell"
  read did not survive the sample — and the A/B has a design confound
  (round 1 is always vref1-fwd/tip1-back, so tip1 never walked
  forward tonight). Verdict: the problem is surviving the takeoff
  transient, not policy choice. Sim-side: the queued takeoff arm
  drained as a STUB (default DR — VOID); the proper relaunch
  `cw-dep-tip1-takeoff25-r1` FAILED with a decisive read — under the
  identical 20–25° injection vs matched tip1 baseline, child==parent
  (0/12 valid both, zero falls both), sim ALREADY recovers static
  tipped starts at the hardware regime, dose lever CLOSED (2nd
  no-separation arm). The takeoff fix must be a DYNAMIC roll-rate
  perturbation during gait start (CODE) or contact/pinning work;
  gate on fell/tail, not peak.
- **Tonight's "thermal wall" was mostly PHANTOM BUS READS.** The
  "L4 hip 150 °C" abort read a steady 33 °C seconds later; the
  debounced watchdog never tripped all night. Single-read temp checks
  in safe_zero/pinned_tip were killing sessions on corrupted bytes —
  now debounced (two consecutive hot reads), and the always-on
  `servo_watch` gained a THERMAL PANIC that kills ALL motion (not one
  servo) on a real overtemp; busy cadence 10→5 s. All deployed. The
  19:18 "L2 hip 72 °C" stays unconfirmed-possible, not proven.
- Turn signs: **+0.3 = CCW from above (matches z-up convention)** off
  the 19:33 camera frames — single reading. **−0.3 still unmeasured**
  (first try silently refused on a pending measure record — fixed;
  rerun coincided with the camera being removed). First item next
  session.
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
  saturates the static-tilt axis. **08-12: the dynamic follow-up
  landed AND ran (`dr.walk_kick_*` code, commit 7d34fc6;
  `cw-dep-tip1-kick1` trained) — SAME NULL: matched-parent probe at
  the gate's own dose (prob 1, 14–22°, n=24 seeds/side,
  `probe_walk_kick.py`) gives ZERO falls for BOTH child and frozen
  tip1, tail roll well under the bar for both. The WALK
  command-pulse family is now CLOSED (2nd axis, 3rd arm, to saturate
  with no separation) — do not schedule another dose. Remaining
  lever for the takeoff transient is contact/pinning modeling, not
  more command-side DR.** **08-12: rise-rock (same command-bias
  family, belly-curl mode) also FINISHED (`cw-stand-riserock2-r1`)
  — a null too, but in the OPPOSITE direction.** Matched-parent gate
  at the exact bench trip threshold (dr.rise_rock_prob=1.0,
  deg=10,10 fixed, det, baseline hard1): child 0/6 valid_plant (1/6
  tilt fall), hard1 ALSO 0/6 (2/6 tilt falls) — zero separation, both
  sides fail this specific guaranteed dose (own-mix retention at
  prob 0.5/deg 6-12 stays clean, 6/6, no regression). Two roll-
  injection axes now show zero learned separation from a frozen
  parent, in opposite directions (walk-kick: both pass; rise-rock:
  both fail) — mounting evidence this whole "randomize a temporary
  body-roll bias" family isn't teaching resilience either way.
  **08-12: the gentler dose retry ran (`cw-stand-riserock3`, deg
  6-10) — CLOSES the family, but on a new failure mode**: own-mix
  det LOWER collapsed from riserock2-r1's clean 6/6 to 1/6 (worst
  foot clearance up to 126mm vs the 60mm bar), video-confirmed a
  fresh three-leg flag-leg/outrigger cheat (legs 1/3/5 plant hard,
  legs 0/2/4 stay splayed 10-126mm off the ground) — a KNOWN LOWER
  exploit class, one-line STOP verdict, no forensics. Breaks the
  gate's own "no retention regression" clause outright, so it fails
  regardless of the rise-rock injection result in isolation (which
  looked fine: 5/6, no falls). **RISE-ROCK DR FAMILY NOW CLOSED**
  (2 doses, 2 misses: zero separation then a new cheat) — do not
  schedule a third dose. `hard1` stays deployed. Both command-bias
  roll-injection axes (walk-kick, rise-rock) are now closed; the
  remaining lever for takeoff/rocking transients on hardware is
  contact/pinning modeling (belly/foot contact geometry), not more
  DR dose. rot60 backward: one fall AND one clean walk — more reps
  when a takeoff-hardened checkpoint exists (none is coming from
  this lever; look to contact/pinning work instead).
- **08-12: the contact/pinning follow-up ran — open-loop trace
  replay (`rl_move/sim/replay_trace.py`) DIAGNOSED both transients**
  (full findings: rl_docs/SIM.md known-gaps §4). Ten stand-failure +
  nine walk tapes replayed action-for-action in the free-base sim:
  joints track at ~1° RMSE (actuator model exonerated); walk takeoff
  excursions reproduce open-loop (sim 8.7–29.5° vs hw 6–25°) — the
  policy never VISITS them in training; the stand failure is a
  support-geometry knife-edge (hw pivots on L4, left pads unload;
  sim keeps them planted — CoM/μ sweeps don't move it). Two
  calibrated MECHANISM-CORRECTED axes shipped: `dr.rise_rock_*` now
  RAMP-GATED (flat curl → last-1.2 s ramp, matching every tape —
  both riserock nulls tested the WRONG shape, a curl-long rock no
  tape shows; this is the replay-derived shape fix the closure's own
  "remaining lever" analysis called for, NOT a third dose of the
  closed persistent-bias axis) and NEW `dr.walk_push_*` (2.0–3.0 N·m
  half-sine chassis roll torque via xfrc, 0.8–1.5 s; reproduces the
  hardware coin-flip regime policy-in-the-loop where the command-side
  kick saturated at 5–10° — a TORQUE axis, not command-pulse family).
  Push works on both stacks (xfrc plumbed through the MJX batched
  stepper + both vec envs 08-12; warp parity test in
  test_mjx_parity.py). Bank tests green (`test_task_semantics.py`
  WALK-PUSH + rise-rock banks). OPERATOR-ORDERED (08-12): retrain
  tip1 with walk_push and a rise specialist with ramp-gated
  rise_rock against the measured disturbances; gates = matched-parent
  probe at the calibrated dose PLUS the riserock3 lesson pinned
  (det LOWER/flag-leg retention is part of both gates).
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
  pricing (6 arms) nor state injection moves posture.
  **08-12: BC-INIT BREAKS THE WALL (`cw-dep-bcgait1`)** — pure action
  pretraining on the scripted tall gait (`bc_init_gait.py`), then a 2M
  RL fine-tune: `probe_tall_wall` steady height -10..+6mm (every
  pricing/RSI arm above: -72..-75mm), leg-yaw margin now POSITIVE
  +17..+18deg (every prior arm: pinned negative at the 35° limit) —
  the crouch+splay habit is GONE, existence-proof-grade. Harness
  confirms real travel (prog_ratio 0.77, gait_valid 6/6, zero falls,
  roll settles clean). Not yet polished: secondary slip bar missed
  (det 2.12 vs the run's own <=1.8 bar, sto sacrifices a leg 1/6) —
  not hardware-ready, next is a hardening continuation. Detail:
  GAIT.md bottom.
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
  shared-habit theory).** `cw-stand-anchormix1-r1` (stratified
  per-mode minibatch sampling, equal quotas) **RAN 08-11 23:4x: FAIL
  per gate, LINE CLOSED — but the park MIGRATED.** Stratification
  fixed the seesaw as predicted (lower kept 6/6 det+sto, crouch rise
  4/4, hold det valid_plant 6/6) and the six-run foot-idx1 park
  finally recovered 0.03→0.90 — but foot idx4 parked at 0.02 in its
  place, and det flat rise still stalls 106mm. The persistent habit is
  SHED EXACTLY ONE FOOT; every lever so far only moves which foot.
  Per pre-registration: hard1 stays deployed, stand-specialist handoff
  stands, no further blind axes. Reopen lever (unqueued): price the
  min-over-feet load + land per-mode bc_anchor_loss logging FIRST
  (CODE — aggregate only today) before ANY further stand arm. RISE.md.
- **New sub-line: unified get-up-and-walk (one policy, no scripted
  handoff).** `cw-getup1` (fresh init) and `cw-getup2-r1` (warm-started
  from the rise+hold specialist) both FAIL the same way: getup_S
  never nears the 0.3 gate target, and cw-getup2-r1 shows the
  specialist's inherited stand skill actively DECAYING (0.09→0.06
  over 2M steps) back into cw-getup1's exact static collapse — a
  warm-start prior alone doesn't survive this task. CODE landed +
  banked (`train.bc_anchor_getup`, default off, state-aligned pull
  toward the rise reference demo, 7 tests green): `cw-getup3` queued
  to test whether an explicit anchor (not just a head start) stops
  the decay. Not a joystick blocker (the working handoff already
  composes rise→walk cleanly); this is about replacing that two-piece
  handoff with one policy.
  **08-12: `cw-getup3` PASSES the pre-registered gate** — the explicit
  anchor stops the decay: `env/getup_S` climbed 0.09→0.17 (target
  >0.15) instead of falling, and video shows a genuine floor-to-stand
  rise (2mm→110mm over ~3s, level six-foot hold after, zero flag-leg)
  from one sampled floor-adjacent start. Not yet reliable — a second
  sampled start stayed stuck low the whole episode. Still not a
  joystick blocker; low-priority sub-line, no further budget queued
  this cycle while named stand/walk blockers are unattacked.

Detail: **rl_docs/BENCH_REPORT_2026-08-11.md** (tonight's consolidated
bench read + RL implications; regenerate tables with
`python -m rl_move.scripts.bench_report`) · RL_PLAN.md queue ·
rl_docs/HARDWARE.md · RISE.md · GAIT.md.

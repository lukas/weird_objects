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
  WALK-PUSH + rise-rock banks). OPERATOR-ORDERED retrains LAUNCHED
  08-12 (this cycle): `cw-dep-tip1-push1` (train-3, warm from tip1,
  dr.walk_push_prob=0.5 at the calibrated 2.0-3.0 N·m/0.8-1.5 s dose)
  and `cw-stand-riserock4` (train-4, warm from holdbc1-hard1,
  dr.rise_rock_prob=0.5, deg=8,18 — the ramp-gated calibrated
  default). **08-12 verdict: `cw-dep-tip1-push1` is PARTIAL/
  INFORMATIVE — the FIRST real (if sub-threshold) separation in
  this whole family.** New `probe_walk_push.py` (matched-parent,
  forced 2.6 N·m/1.5 s, n=12 seeds/side): child falls 5/12 vs frozen
  tip1 9/12 (1.8x lower, short of the pre-registered >=2x bar), but
  paired by seed all 4 disagreements favor the child and ZERO favor
  the parent — a real, directionally consistent effect, unlike
  walk-kick/rise-rock's exact-zero nulls. Nominal DR0 retention clean
  (gait_valid/slip/prog match tip1's own band, zero new falls).
  Per the campaign's own "more steps cleans up the rough edges"
  pattern (just re-confirmed on `cw-dep-bcgait1-hard1`), queued+ran
  `cw-dep-tip1-push1-hard1` (train-3, 10M, identical recipe) rather
  than closing the torque-DR family on a near-miss. **08-12 verdict:
  FAILS bit-for-bit** — the matched-parent `probe_walk_push.py`
  (n=12/side, forced 2.6N·m/1.5s) gives hard1 the IDENTICAL fall
  count as the 2M discovery arm (5/12 vs frozen tip1 9/12, same 1.8x
  gap, same 4 discordant seeds), tail-roll among survivors slightly
  worse. 10M more steps bought nothing. **TORQUE-DR (walk_push)
  FAMILY NOW CLOSED FOR GOOD** — all three perturb-during-training
  axes for the takeoff-roll transient (walk-kick, rise-rock,
  walk-push) are closed. **08-12 ~08:30: `cw-stand-riserock4` (the
  ramp-gated shape-corrected rise-rock, the family's last variant)
  FAILED the same way riserock3 did** — nominal det LOWER fell to
  4/6 with the video-confirmed outrigger/flag-leg park, rise sto
  2/6 tilt falls; disqualified by its own retention clause. And the
  contact/pinning hypothesis itself is now FALSIFIED: the
  `env.leg_chassis_collision` axis was built (default-off, tests
  green) and tape-replay shows the recorded curls NEVER touch the
  chassis — instead the support-polygon trace found the real
  mechanism: the deployed policy ends its rise on THREE feet
  (L0/L1/L4) with the CoM margin flickering **±25 mm every tick** —
  a knife edge sim survives by a hair-trigger catch and hardware
  doesn't (SIM.md gap 4). New arm `cw-stand-margin1` (2M discovery,
  warm from holdbc1-hard1) prices exactly that via the never-used
  `reward.k_support_margin` term; gate = det-rise plant_margin_mm
  up vs matched parent + full retention + no outrigger cheat.
  **08-12: `cw-stand-margin1` FAILS both pre-registered branches** —
  the margin stat itself never moved (det-rise plant_margin_mm 157 vs
  matched frozen parent 154, inside noise: the BC anchor pins rise's
  trajectory too hard for a new income term to shift it) AND a known
  exploit reappeared in retention: det hold parks foot idx1 (duty
  0.05 vs parent 0.90, visible outrigger in frame) even with
  hold_still_gate+hold_flag_fade already on. **Same conclusion from a
  totally different reward term:** `cw-stand-transdrag1`
  (`reward.k_drag_trans`, charging loaded-foot scraping during
  stand/sit — queued 08-11 night off the new drag-meter finding)
  also FAILS — drag dropped only 10-20% (hold 0.196->0.156m vs a
  <=0.05 bar, lower 0.736->0.658m vs <=0.20) and the SAME idx1 park
  reappeared (duty 0.03), because a foot that's mostly airborne is
  almost never "on" for two consecutive ticks and so can't accrue a
  per-tick drag charge — parking is a free escape valve from this
  charge too. **Between minfeet1 (hold pricing), margin1 (rise
  pricing), and transdrag1 (drag pricing), THREE independent
  reward-side levers now confirm the same closed door: any new
  pricing term on an anchored stand mode gets evaded by parking one
  foot, or doesn't move the anchored quantity at all.** Do not queue
  a fourth. Meanwhile the fleet also got the one CLEARLY ready,
  non-blocked lever on the board: the tall-walking champion
  `cw-dep-bcgait1-hard1`'s own DR/tipped-start retention panel (see
  "Next" below) — its first two axes (friction, ground-tilt) queued.
  **08-12 ~09:5x: the anchor-side spec/verify pass RAN (same cycle
  as the two verdicts) and settled the six-run mystery.** Audit on
  train-0 against the live hard1 cfg: (a) the `_q_nom` theory is
  FALSIFIED — 48/48 hold resets settle with all six feet loaded
  3.2–3.6 N, none under 0.5 N; the anchor reference is a genuine
  six-foot stance. (Suggestive detail: feet 1/4 are the two LIGHTEST
  at settle, 3.19 vs 3.57 N — exactly the only two feet any park has
  ever chosen.) (b) "PPO defies a working anchor" is falsified too:
  rolling the parked margin1 policy through a det hold and scoring
  per-leg action-MSE against the anchor target gives the PARKED leg
  0.0032 vs the clean parent's 0.0031 on the same leg — not even the
  worst leg of the six. **The park is geometrically INVISIBLE to
  joint-space supervision: a mm-scale hover is fractions of a degree
  of hip lift, 3 dims of 18, ~1e-4 of MSE.** That is why six anchor
  variants "converged" while the park persisted, and why every
  pricing term found parking as the escape valve (a hovering foot
  pays no per-foot charge). CODE landed same cycle:
  `train.bc_anchor_foot_z` (+ `_mm` scale) — an additional anchor
  term supervising commanded FK foot HEIGHTS (torch twin of
  `body_ik.fk_all_feet`, z = −F·sin(hip) − T·sin(hip+knee)); a 10 mm
  hover costs ~1.0 at default scale (bank test pins ≥50x the joint
  MSE ratio; default-off bit-exact; 41 anchor tests + 78-test
  semantics bank green). First arm: `cw-stand-footz1-r1` (2M discovery,
  warm from holdbc1-hard1, ONE variable, gate = all-six-feet det
  hold duty ≥0.5 + rise/lower retention vs the matched parent probe
  + no outrigger).
  **RESULT (08-12): PASS (partial) — the fix works.** Det hold: ALL
  SIX feet duty 0.92–0.98 across all 6 episodes (frozen parent
  `margin1` scores 0.05 on leg idx1 in the identical test), valid_plant
  6/6, video-confirmed level quiet stand with zero flag-leg — the
  first clean six-foot hold after 6+ straight pricing-arm failures.
  Two clauses miss narrowly, both matching the parent's own noise
  rate, neither park-related: sto hold valid_plant 4/6 (2/6 trip a
  >2.0A tail-current spec check, not a duty/park issue — no leg drops
  below 0.26 duty); det rise 5/6 vs parent's 6/6 (one flat-start
  height-only miss, zero falls, video reads as an honest crouch-to-
  stand). Det lower stays at 4/6, matching the parent's own baseline
  exactly with the IDENTICAL pre-existing 3-leg-proud pattern
  (confirmed against margin1's own report — inherited, not introduced
  by this run). Hold drag 188mm vs parent's 159mm (+18%, the
  plausible cost of a foot that now actually bears load instead of
  hovering free). `train/bc_anchor_footz_loss` fell 5.1→1.3-1.5 and
  plateaued (didn't converge near 0 — residual hover likely sits in
  rise/lower ticks, not hold). 10M hardening `cw-stand-footz1-hard1`
  queued to consolidate the rise miss and confirm durability; `hard1`
  stays the deployed stance checkpoint until a footz-lineage arm
  passes clean. **08-12: `cw-stand-footz1-hard1` FAILED its own
  gate** — hold survives hardening (det+sto all-six duty 0.92-0.99)
  but LOWER regressed to 0/12 (the known 3-leg outrigger, worse
  under budget, clearances to 170mm); this lineage never had the
  lower-mode anchor its sibling `anchormix1-r1` used to solve lower.
  **08-12 midday: the combination arm `cw-stand-footlow1`
  (footz1-hard1's hold fix + anchormix1-r1's lower-anchor bundle,
  one merge, 2M discovery) FAILED its own gate but is the most
  informative stance arm yet: HOLD stays clean (det duty ≥0.94
  every foot, 6/6) AND LOWER fully recovers (12/12 det+sto, feet
  flush sub-mm vs parent's 0/12 at up to 126mm — video-clean honest
  descent) — the first policy ever with both. The pre-registered
  dilution branch (hold park reopening) did NOT fire; instead RISE
  paid: det 3/6 / sto 2/6, stalling belly-down ~100mm short of
  target — the anchormix lineage's known det flat-rise stall
  (loweranchor1 96mm, anchormix1-r1 106mm), carried into the merge
  by the state-aligned/lookahead bundle. **08-12 (same day): the
  alignment audit RAN (`probe_anchor_align.py`, live stalled policy,
  the run's own cfg incl. loaded servos) and RESOLVED the mechanism —
  a PLATEAU FIXED POINT, correcting the "anchor-BLIND" read: the
  matched ref index PINS at j≈128–137 (0 ticks advance over the last
  3 s) inside the demo's 5+ s 0→25 mm prep crawl, so the +0.5 s
  pursuit target commands only 1–5 mm of height gain (ref_h 6.4–8.4
  mm vs chassis at 4–7 mm), loaded-servo sag (~0.3 s settle) cancels
  it, and the policy OBEYS — mse(act,target) 0.004–0.006 during the
  stall, its episode MINIMUM. The converged `bc_anchor_loss_rise`
  was the anchor actively supervising the stall. Fix landed
  (`train.bc_anchor_min_h_ahead_mm`: height-floor pursuit, target
  tick must command ≥Δmm above current chassis height; default off,
  bit-exact, 3 bank tests + 44-test anchor suite + 78-test semantics
  bank green): one-variable retry `cw-stand-footlow2-r1` (footlow1
  recipe + floor=15, 2M discovery) ran. Side finding,
  noted not attacked: off-path bridge starts (33° RMS from any ref
  tick) match the path END and get supervised straight to plant,
  ignoring the ramp. `holdbc1_hard1` stays deployed;
  rise-from-flat is still the last broken stance mode.**
  **08-12 midday+ RESULT: `cw-stand-footlow2-r1` FAIL per own gate,
  mechanism CONFIRMED, two new residuals (DEEP DIG-IN flagged).**
  The floor works where it aimed: det flat stall moved ~100 mm →
  15–16 mm short, sto rise 6/6 incl 4/4 flat (footlow1: 2/6),
  lower retained 12/12 flush. But (a) det flat still misses the
  height bar by ~15 mm on the eval's seeds while a seed-0 probe
  reaches 3 mm err with the anchor correctly targeting the demo's
  final plant frame (j=313, mse 0.003) — the residual is
  seed/start-dependent endgame, NOT the old plateau; and (b) the
  hold idx1 park REOPENED (duty 0.03 all 6 det eps, valid_plant
  still 6/6) despite `bc_anchor_foot_z=1` — the footlow1
  pre-registered rise/hold seesaw fired one arm late. Next arm
  waits on the seeded audit (which target/mse at the 15 mm-short
  states; why foot-z lost to the rise floor), not another dose.
  **08-12 afternoon DIG-IN RESULT: both residuals OVERTURNED —
  rise-from-flat is SOLVED in this checkpoint.** (a) The 15 mm-short
  "flat" episodes were RSI MID-PATH SPAWNS mislabeled by the eval
  (`rise_rsi_frac=0.5` rides into the gate; `_start_kind` couldn't
  see RSI): floored probe = 12/12 cold flat rises within ±3 mm
  across seeds 0–5 (anchor at path end, mse 0.0028, 6/6 contacts);
  RSI-off gate rerun = det rise 6/6 valid_plant, roll_tail ≤0.3°.
  eval_checkpoint now emits `start_kind="rsi"` (snapshot da367c9);
  judge cold-start clauses on the label. (b) The "park" is a
  +0.9 mm COMMANDED hover (FK probe vs q_nom; footlow1's same foot
  commands +0.4 mm at duty 0.97) — sub-resolution for the 10 mm
  foot_z scale, not the historical 10 mm weight-shed park. First
  policy with rise+hold+lower simultaneously clean to mm scale.
  Queued: `cw-stand-footlow2-hard1` (10M consolidation, PASS =
  deployment candidate incl. eval_session hard gates) +
  `cw-stand-footzsharp1` (foot_z_mm 10→3, one variable, closes or
  refutes the last-mm hover). Detail: rl_docs/RISE.md; artifacts
  logs/experiments/cw-stand-footlow2-r1/digin/.
  **08-12 midday+: `cw-stand-rampjit1` (model-tour ramp-jitter
  axis, holdbc1-hard1 + rise/lower_ramp_jitter=0.3) FAIL — axis
  CLOSED per its own gate.** Session hard gate still misses
  (interactive rise z_end 59.5 vs 60 mm @9.5 s; parent 55) AND det
  lower retention broke (2/6, sto 0/6, outrigger class, clearances
  to 147 mm). Honest positive: the parent's deterministic
  sit-from-142mm-plant tip did NOT occur (no_falls + sit_descends
  PASS, tilt peak 9.1°). Per the pre-registered gate: no dose-down
  retry (not retention-only); next lever is START-STATE exposure —
  and the stance candidate that should face eval_session next is
  the footlow lineage once its gates pass.
  **08-12 afternoon: `cw-stand-footlow2-hard1` (the 10M
  consolidation) PASSES — all four pre-registered clauses, first
  time.** Cold det rises all valid_plant ≤5mm (bridge 2/2, crouch
  1/1 from the gate draw; flat 12/12 via a targeted probe since the
  6-episode draw sampled none, h_err 0.5–3.4mm, roll_tail 0.0°).
  Det hold ZERO real park: all six feet duty 0.95–0.99 at ~0.1–0.2mm
  commanded hover, tighter than r1's own 0.9mm residual. Lower
  12/12 det+sto, feet flush (end_clear ≤0.3mm det/≤6.4mm sto). AND
  it clears `eval_session` HARD gates outright (no_falls/rise/
  sit_descends) — rise reaches full 148mm by t=9.5s under the
  interactive ramp, where the currently-DEPLOYED `holdbc1_hard1`
  stalls at 55mm on the identical protocol. Visual-quality stats
  (drag/roll_tail) flat-to-improved vs the r1 parent on every mode.
  Video-confirmed clean six-foot stance throughout, no flag-leg/
  park/stilt. `ppo_goal_cw_stand_footlow2_hard1` is a genuine stance
  DEPLOYMENT CANDIDATE (sim-only, not yet bench-tested) — the
  promotion-over-`holdbc1_hard1` call is next. `cw-stand-footzsharp1`
  (the paired last-mm-hover probe) still to triage.
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
  **Same cycle, the hardening continuation RAN: `cw-dep-bcgait1-hard1`
  (10M) PASSES decisively** — height stays in-band (-8.5..-9.8mm),
  yaw margin stays positive, and BOTH secondary misses are fixed
  (det slip/m 1.43, sto 1.51 with the sacrificed-leg episode gone,
  gait_valid 6/6 both passes, prog_ratio 1.05/0.91, zero falls). Now
  the strongest tall-walking candidate in the campaign; next is the
  standard dep-line DR/tipped-start retention panel, NOT yet run,
  before any Gate 0 consideration. **08-12: panel STARTED** —
  bcgait1-hard1 already trains with dr.tipped_start_prob=0.30 baked
  in (its own gate/own-DR evals already exercise that), so the panel
  gap is the per-axis stress arms the vref1-r1/tip1 lineage went
  through (friction, ground-tilt, latency, encoder noise, ...) that
  this NEW checkpoint has never seen. Queued to backlog (hardening
  phase, warm from bcgait1-hard1's own checkpoint): `-fric`
  (dr.friction_scale 0.4-1.6x) and `-groundtilt5` (dr.ground_tilt_deg
  5.0), both k_current=0 per the standing hardware-arm rule. Two
  axes only this cycle — the historical panel ran dozens one at a
  time over many cycles; treat this as started, not complete.
  **08-12: tipped-start-dose isolation and combined-axis DR-compose
  DROPPED from this panel** — both are already closed generic classes
  (would just reconfirm, not inform); CURRENT_TRUTHS corrected. The
  one real open question — does this lineage share the dep-line's
  walk-takeoff roll vulnerability? — is ANSWERED instead:
  `probe_walk_push.py` generalized to the bcgait1 lineage (no new
  reward stack needed, its physically-relevant cfg already matches
  VREF1_STACK exactly) and run as a pure diagnostic (no training):
  forced 2.6N·m/1.5s injection, n=12 seeds, `cw-dep-bcgait1-hard1`
  falls 6/12 vs frozen `cw-dep-tip1` 9/12 — LOWER, not worse, and
  already close to the push-trained lineage's 5/12 with zero push
  exposure. No push-training respec warranted (family closed anyway).
  Gate 0 for this lineage now needs hardware bench evidence, not
  another sim DR axis. Detail: GAIT.md.
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
  stands, no further blind axes.
  **08-12: the reopened min-over-feet-load lever (`cw-stand-minfeet1`,
  with per-mode `bc_anchor_loss` logging landed) FAILS the same way —
  `env/hold_feet_factor` 0.105, deep in the same 0.1–0.35 failing
  plateau, while `train/bc_anchor_loss_hold` is LOW and converged
  (0.0107) — a working anchor, teaching the park. PRICING FAMILY NOW
  TERMINALLY CLOSED for the parked-foot habit** (min-over-feet was the
  last untried pricing axis). Rise/lower retention clean, hard1 stays
  deployed. Only remaining lever: anchor-side (find + patch the exact
  reference tick that shows a lifted-leg pose at a plant-adjacent
  state) — unqueued, needs a spec pass first. RISE.md.
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
  **08-12: the 10M hardening `cw-getup3-c2` (identical recipe, "give
  it the steps it was still climbing at") FAILS — the extra budget
  entrenches a cheat instead of closing the gap.** `env/getup_S`
  plateaued 0.17–0.21 for the full 2M–10M range (never approached the
  >0.30 gate), `reward_getup_hold` stayed ~0.009 (needed >0.05), and
  video confirms the pre-registered "strongest alternative": height/
  footprint keep climbing (0.33→0.73 / 0.37→0.72) while `feet_loaded`
  sits stuck at ~2.7–2.9/6 the whole run — a partial (~4-leg,
  quadruped-like) stand, not a real six-foot one. "More steps" is
  refuted for this lineage (one-line known-exploit stop, no
  forensics); next lever is a pricing/anchor fix, same family as the
  now-closed stand-hold pricing line. Still a low-priority research
  sub-line (not a joystick blocker) — no further budget queued.

Detail: **rl_docs/BENCH_REPORT_2026-08-11.md** (tonight's consolidated
bench read + RL implications; regenerate tables with
`python -m rl_move.scripts.bench_report`) · RL_PLAN.md queue ·
rl_docs/HARDWARE.md · RISE.md · GAIT.md.

# CURRENT TRUTHS — read this BEFORE any history file

Accepted current facts + rulings only, never campaign narrative. If
RL_LOG/archive prose disagrees with a line here, this file wins.
Update ONLY when a ruling is accepted or hardware produces new
evidence; keep 50–80 lines. Reading order: RL_GOALS.md → this file →
RL_PLAN.md → RESEARCH_RULES.md → rl_docs/SIM.md.
Last regenerated: 2026-08-12 midday+ (`cw-stand-footlow1` FAIL per
own gate but the most informative stance arm yet: merging
footz1-hard1's hold fix with anchormix1-r1's lower fix gets BOTH
clean at once for the first time — det hold all six feet duty
≥0.94, det+sto lower 12/12 sub-mm clearances — but det rise falls to
3/6 (sto 2/6), the anchormix lineage's known flat-rise stall carried
in by the merge. **DIG-IN RESOLVED same day (`probe_anchor_align`):
the stall is a PLATEAU FIXED POINT, not anchor blindness** — the
state-aligned match pins at ref ticks ~128–137 where the recorded
demo crawls 0→25 mm over 5+ s, so the +0.5 s pursuit target commands
only 1–5 mm of height gain, loaded-servo sag cancels it, and the
policy OBEYS (mse(act,tgt) 0.004–0.006 at the stall, its episode
minimum: the converged `bc_anchor_loss_rise` was the anchor actively
supervising the stall). Fix LANDED: `train.bc_anchor_min_h_ahead_mm`
(height-floor pursuit — the target tick must command ≥Δmm above the
current chassis height; default off, bit-exact, 3 bank tests; loaded-
params chained traversal 82.5 mm by t=50 vs ~0 legacy). First arm
`cw-stand-footlow2` (footlow1 recipe + floor=15, 2M discovery).
Rise-from-flat remains the LAST broken stance mode.
`holdbc1_hard1` stays deployed. Earlier
same day: `cw-stand-footz1-hard1` FAILED its own 10M hardening (hold
held, lower regressed to 0/12 — the lineage never had a lower
anchor, which is exactly what footlow1 then added). Earlier, 08-11
late: omni translation RESOLVED IN SIM via the rot-60 wrapper; rise/
hold solved via BC-anchor, both handoffs compose).

## Real robot facts (these outrank any sim result)

- The scripted tripod gait walks the REAL robot forward, crabs
  laterally, and turns both directions, from a clean zero.
- Measured scripted-gait ground travel is ~50–51% of commanded
  (~15 mm/s actual from 30 mm/s command; ~25 from 50). Loaded
  stance-foot sliding is real and contributes to locomotion —
  visible slip is NOT automatically failure.
- A successful gait rocks ±10–20° in roll/pitch. A 10° walk
  termination was too restrictive; the envelope is 25° plus
  directional angular-rate safety logic (never bare gyro magnitude).
- Hardware effort (servo current registers, bus total): walking
  0.33–0.45 A; SCRIPTED-stand hold 0.54–0.59 A (servos fighting the
  sag from an ideal commanded pose); walk-synced plant hold only
  ~0.11 A (cmd == settled stance → gear friction carries the load
  nearly free). Register current tracks the cmd-vs-settled FIGHT,
  not pose height (08-11 re-read of hw_session2 per-servo traces).
  k_current=0 on hardware arms until register-scale pricing is
  calibrated.
- Contact calibration DONE (08-10, `calibrate_slip.py`): sim replay
  of the exact hardware gait travels 0.35–0.41 of commanded (real
  0.50–0.51), speed-invariant, walking current in-band — sim does
  NOT price sliding as free (it is slightly conservative); friction
  saturates ≥1.5 so μ is not the knob. The "sim hold 0.11 A vs real
  0.59 A" effort-inversion is RETRACTED (08-11,
  `probe_hold_current.py`, logs/probe_hold_current/): it compared
  sim MEAN-PER-SERVO to hw BUS-TOTAL (×18) at mismatched poses.
  Pose/unit-matched, sim's |torque|-proxy OVERPRICES every condition
  3–25x (conservative, air AND loaded params) and reproduces the
  real walk>plant-hold ordering. Register-accurate current needs a
  cmd-fight/deadzone model — required only before a k_current>0
  hardware-pricing arm; NOT a joystick blocker.
- Stand-up (08-10, scripted `/api/standup` bench): pulling loaded
  feet inward CANNOT reach the plant — blend stalled short of full
  height at only 0.57 A peak (servos give up quietly under the 70%
  torque limit; the pinned-feet geometric lock, joints read tens of
  degrees short and snap to target when unloaded). Air-tuck
  strategies stand clean: tuck peak 2.48 A, tripod re-plant (step)
  2.97 A. Hardware behaves like the sim friction×torque sweep's
  tq≈0.35–0.5 rows. Rule of thumb for any rise: move feet UNLOADED
  (in air / belly carrying), then push vertically.
- Loaded actuator response is much slower than the air-only fit
  (2° loaded steps settle in 250–325 ms vs ~9 ms air; loaded peak
  velocity 48–67°/s). Loaded fit LANDED, opt-in via
  `bus.servo_params=loaded`; treat with uncertainty/DR, not as exact
  truth; hip/yaw numbers are an ASSUMPTION.
- Fresh `set_zero` / plant consistency matters: a stale, slumped
  logical stance caused scripted-gait falls even though the gait was
  sound. Always start a session with a fresh `set_zero` at a known
  visual pose. (All 18 servos are healthy — operator 08-10: no open
  servo issues, do not resurface old ones.)
- TFT redraws STALL the servo bus (08-10): the ST7789 job-panel
  repaint (`DJ`) holds the shared MCU serial link ~1.5 s, blocking
  ALL pose writes/reads behind the bus lock — measured as a mid-air
  freeze during a 10× stand-up. Any motion loop must set the
  `bus_hot` flag (standup worker pattern, bench_api) so
  StatusDisplay skips painting, or it will hitch whenever its
  progress text changes.
- Control is 25 Hz; reaction delay ~100–200+ ms; hidden contact
  state; no direct body-velocity measurement; possible zero drift.
- 08-06 incident rules (AGENTS.md) are absolute: no motion without
  an explicit operator ask; stop after tip/brownout/hot motor.

## Policy / deployment facts

- Action space: 18 raw joint-position targets through the
  SafetyLayer. The 1.5°/tick STATEFUL slew limiter exists in both
  training and hardware — it is not a train/deploy mismatch.
- Hardware feeds vx/vy_meas := vx/vy_ref; `cw-dep-vref1-r1` showed
  zero erosion under that exact contract (ACCEPTED). It is the
  hardware-attempt base checkpoint (md5 f9a466cf).
- Previous-action semantics audited PASS both sides (raw pre-safety
  proposal); don't re-audit.
- hist16 walks from scratch across seeds (joystick gate clean) and
  is the preferred temporal base for new unified-policy work —
  provided it does not delay hardware milestones. Ladder frozen at
  16 until the flagship experiment answers (RL_PLAN Architecture).
- MODEL TOUR 08-11 (all 27 deployable ckpts through the interactive
  play.py protocol; rl_docs/MODEL_TOUR_2026-08-11.md): the deployed
  stance `holdbc1_hard1` deterministically TIPS (tilt_pitch, ~2.5 s)
  when commanded to sit from the 142 mm walk plant frame — from a
  clean stand, no disturbance, 10/10 + isolated probe — and its
  belly rise STALLS at 55 mm forever under the interactive ramp
  (other stance ckpts reach 69–83 mm on the identical protocol).
  Same targets, different ramp shape = goal-profile overfit. DO NOT
  command sit-after-walk on hardware until a candidate passes the
  session gate. Gate: `rl_move.sim.eval_session` (hard: falls/rise/
  sit; soft: heading drift, per-axis tracking, drive height, hold
  duty). Training lever: `goal.rise_ramp_jitter` /
  `goal.lower_ramp_jitter` (landed, default off, bank green).

## Campaign rulings in force

- KPI = unresolved blockers between today's robot and the next
  useful joystick hardware test. NOT GPU occupancy; idle pods fine.
- OPERATOR DIRECTIVE 08-11 night (narrows the bullet above): sim
  stand/walk blockers are THE focus — keep the fleet firing arms
  that directly attack them, each spec'd as a deployable candidate;
  when the next lever is code, cycles WRITE it and check it in
  (cfg-gated, default-off, bit-exact when off, tests green,
  snapshotted) instead of parking the line. Peripheral runs stay
  banned. Full text: RL_PLAN.md "Operator directive (08-11 night)".
- Phases binding (RESEARCH_RULES.md; launcher-enforced):
  SPECIFICATION (never trains) / DISCOVERY (0.5–2M) / HARDENING
  (10–40M, needs evidence) / COMPOSITION / TRANSFER.
- MDP_PREFLIGHT binding: no reward/task-mechanism launch until
  `rl_move/tests/test_task_semantics.py` passes for that mode. A
  reward/eval bug found after training is a preflight failure, never
  a reason for a longer run.
- Matched-parent controls binding: injected-axis evals compare child
  vs frozen parent under the IDENTICAL injection
  (`eval_checkpoint.py --baseline`).
- Evals and reward are explicitly documented (08-10): headline scores
  are `SCORE/*` (pinned top of W&B; definitions rl_docs/EVALS.md);
  every run auto-records its resolved reward config in W&B notes +
  `config.reward_cfg` (term meanings rl_docs/REWARD.md). New reward
  terms must add their REWARD.md row in the same change.
- A closed sim hypothesis reopens only on new HARDWARE evidence.
- Sit NEVER refuses on pose delta (operator ruling 08-10, repeated
  request): a big delta makes `go_zero("sit")` glide SLOWER (6–10 s,
  abortable), it does not refuse. The delta refusal remains for STAND
  only (wrong zeros + stand glide = the stilts incident). Do not
  re-add a sit gate. UI errors must stay copyable (persistent error
  bar with Copy button, webui).
- Unified rise is UNSOLVED: every arm lost to the height-only cheat
  (flag-leg/tripod). Torso height alone never defines a stand — the
  geometric valid-plant spec is LANDED (08-10): PLANT_SPEC /
  `valid_plant()` in sim_env.py (height, attitude, feet down, no
  flags, CoM-in-polygon, walkable footprint, current), shared by
  reward gate (`reward.rise_plant_polygon_gate`), eval harness
  (reported always; `--valid-plant-gate` opt-in until champions
  baselined), and the rise bank (separates replay from all cheats).
  Working fallback: stance champion rises → scripted 1.5 s blend →
  walk champion drives (sim-proven, key `7`).
  **08-11 LATE: `cw-stand-bc1` (BC-anchor, lever (a)) PASSES —
  the first honest rise in 7 stand-arms.** Same rsi3 reward/goal-mix
  stack + one change (`train.bc_anchor_coef=1.0`, actions supervised
  toward the recorded stand-up path during rise ticks, outside the
  reward). Harness: gate 3/6 det valid_plant (bridge/crouch honest
  six-foot plants, video-confirmed, no flag-leg); an RSI-off/30-ep
  probe on the same checkpoint gets bridge 7/12, crouch 6/8 valid,
  and flat-belly cold start (hardest case) reaches a real six-foot
  stand 10/10 times but misses only the footprint-precision bar
  (0/10, not a height/posture cheat). Zero flag-leg cheat in 42
  video-checked episodes; the identical-minus-anchor parent
  (`cw-stand-rsi3`) still cheats 0/12 on the same stack — clean
  one-variable attribution. Cost: training's own diagnostic (n=2,
  weak) suggests some raise/tipped-recovery/hold-track drag,
  unverified by harness. Follow-ups RESOLVED (08-11): coef-dose
  `cw-stand-bc1-coef03` FAILED (0/16, keep coef>=1.0); 10M hardening
  `cw-stand-bc1-hard1` consolidates rise decisively — RSI-off probe
  12/12 valid_plant incl. flat 4/4 (footprint miss resolved), feet
  factor stable all 10M, **`ppo_goal_cw_stand_bc1_hard1` is the RISE
  SPECIALIST champion**. Matched-parent control same probe: parent
  already 0/12 on hold/track/raise/lower (166mm lower flag-leg at
  2M) — hardening lost nothing, but hold splays worse with steps
  (51→162mm, 2.6A): a pre-existing hold/track stillness-pricing gap.
  Lineage CLOSED for further hardening; next is a HOLD-mode
  stillness SPECIFICATION + the rise-specialist→walk-champion
  handoff composition test. Detail: rl_docs/RISE.md.
  **08-11: HOLD SOLVED — `cw-stand-holdbc1` (BC-anchor extended to
  hold/track ticks, third lever after two pricing-only levers
  failed) PASSES: harness hold 12/12 valid_plant det+sto, worst-foot
  2–13mm, video-confirmed level motionless six-foot stand both
  modes — first genuine quiet hold in the campaign.**
  `env/hold_feet_factor` cleared the 0.1–0.35 plateau to ~1.0 within
  the first 500k steps. Rise retention mostly clean (bridge 2/2 det,
  sto 6/6); det crouch showed 2/6 tilt_roll falls, matched against
  the identical pre-existing fingerprint already in the immediate
  parent (`cw-stand-holdstill2`, 1/6) — a known crouch fragility, not
  a new regression. Checkpoint `ppo_goal_cw_stand_holdbc1`.
  **08-11: 10M hardening `cw-stand-holdbc1-hard1` PASSES** — hold
  11/12 valid_plant (matches discovery, no regression),
  `env/hold_feet_factor` held 0.99–1.0 all 10M, crouch-start rise
  improved 33%→50%, zero cheat in 24 videos. `ppo_goal_
  cw_stand_holdbc1_hard1` is the hardened HOLD+RISE checkpoint;
  lineage CLOSED for hardening.
  **08-11: rise+hold→walk-champion HANDOFF composition PASSES**
  (`eval_handoff.py`): specialist rises belly→settled hold, walk
  champion switched in on the exact final state — 12/12 successful
  rises hand off with zero falls, drive metrics in the clean-plant
  baseline band, the scripted 1.5 s blend adds nothing; holds on air
  AND loaded servo physics. Crouch-start rises still tip PRE-handoff
  (0/6 RSI-off, known fragility; flat+bridge rises 12/12).
  **08-11 later: the crouch fragility is FIXED by start-mix bias**
  (`cw-stand-crouchrise1`, 60% crouch starts vs legacy 25%; RSI-off
  all-crouch probe 16/16 stands, det 8/8 valid_plant, ZERO falls vs
  hard1's 0/8 with 8/8 tilt falls, matched control) — but NOT
  promoted: det-hold PARKS TWO FEET (contact duty 0.07/0.01 on legs
  1+4, all other legs 0.90+) while `valid_plant` still reads True —
  a flag-leg cheat the geometric check misses, caught only by the
  explicit per-foot duty telemetry (corrected 08-11 from an earlier,
  less specific "current-tail flags" read). hard1 STAYS the deployed
  stance policy; `ppo_goal_cw_stand_crouchrise1` (md5 3877e16c) is
  banked. **`cw-stand-crouchrise2`** (same fix, restoring hard1's
  exact goal-mix to rule out the mix-skew as the hold cheat's cause)
  REPRODUCES the crouch-rise win cleanly (det rise 6/6 valid_plant
  incl. crouch 4/4) but the IDENTICAL flag-leg fingerprint returns on
  the SAME two legs (duty 0.03/0.03) — restoring the goal-mix did
  NOT fix hold, so the mix skew was not the (sole) cause; the real
  mechanism is still unknown. Lower also measurably worsens vs a
  matched hard1 control under the same eval (hard1 3/6 succeed/1/6
  falls; crouchrise2 0/6 succeed/2/6 falls) — hard1's own lower was
  never gated before, so this is a real, quantified degradation, not
  a clean inherited gap. **`cw-stand-crouchrise3` (08-11 late,
  operator-directed dose probe 0.60→0.45, one axis) FAILS the same
  way: crouch rise clean 4/4 but the identical legs-1+4 park (duty
  0.04/0.01) and det lower 2/6 — the dose axis is dead too.** Stand
  lineage stays CLOSED for hardening; the hold-cheat root cause (not
  goal-mix, not step count, not dose) is the open question; prime
  suspect is the CLOCK-indexed BC anchor showing lifted-leg reference
  poses in plant-adjacent states on crouch starts — next lever is a
  state/height-aligned anchor (CODE, spec first).
  **08-11: reward-side lever also REFUTED by direct measurement
  (`cw-stand-holdload1`, mechanism test).** New `reward.hold_feet_load`
  prices hold/track income on MEASURED foot touch-force (not
  clearance), confirmed correctly-calibrated by its own FEET-LOAD
  bank (hover taxed to 0.25x quiet-stand income) — yet the identical
  legs-1+4 park reproduces exactly: det-hold `duty_cycle` 0.03–0.04 on
  those two legs across all 6 det episodes (vs 0.73–0.99 on the
  other four), unchanged from crouchrise1/2/3, despite paying the new
  tax. `valid_plant` still reads True throughout (blind to
  mid-episode duty — both feet drift back to the floor exactly at
  episode end); crouch rise stays clean (det 6/6 incl. crouch 4/4,
  zero falls) and det lower regresses to 2/6 (dangling leg-2, no
  falls, matches crouchrise3). Three distinct lever families now
  refuted (start-mix, dose, reward-pricing) — the state/height-aligned
  BC anchor is the SOLE remaining suspect, by measurement not
  inference. hard1 remains deployed.
  **08-11: the state-aligned anchor (`cw-stand-anchorstate1`) gets a
  PARTIAL confirmation — first fingerprint movement in five runs.**
  Re-indexing the BC anchor to the nearest reference pose to the
  robot's current joints (not a fixed clock) RECOVERS leg 4
  (det-hold duty 0.01→0.93) but leg 1 still parks (0.04) — anchor-
  bleed is confirmed as *a* mechanism, not the whole story. Net FAIL
  on gate: det flat rise stalled 62mm short (under-drive, not a
  cheat — a state anchor only pulls 0.25s ahead of wherever the
  policy is, so a stalled policy gets weak supervision) and det
  lower picked up 3 tilt_pitch falls (2/6). **08-11:
  `cw-stand-anchorstate2` (one axis, lookahead 0.25→0.5s) FIXES both
  anchorstate1 regressions exactly as hypothesized** (det flat rise
  restored 1/1, det lower falls 3→0, leg 4 stays recovered at 0.95)
  but leg 1 still parks (det-hold duty 0.03) — the SIXTH consecutive
  run with that exact fingerprint, unmoved by four pricing changes
  and two anchor lookaheads; det lower is now a shortfall not a
  fall (2/6, zero falls). The lookahead axis is EXHAUSTED for the
  park (pre-registered branch). hard1 stays deployed; anchorstate2 is
  otherwise the strongest unified-stand checkpoint yet (crouch+flat+
  bridge rise, six-foot-minus-one hold, zero falls anywhere det).
  Follow-up `cw-stand-loweranchor1` (new lever: BC-anchor the LOWER
  ticks toward the lower bank's own honest IK descent — the one
  documented incentive gap not yet attacked, since leg-1's hold-park
  shares the dangling-leg class with det-lower's shortfall) **RAN:
  LOWER SOLVED (det+sto 6/6, zero falls, from 2/6) but hold park
  REGRESSED to a two-leg version and flat rise re-stalled 96mm —
  outside every pre-registered branch.** Root cause identified: rise/
  hold/lower anchors share ONE ring buffer + uniform sampling, so
  adding thousands of lower pairs diluted the rise/hold gradient
  (ANCHOR DILUTION, not a shared taught habit). Follow-up
  `cw-stand-anchormix1-r1` (`train.bc_anchor_stratified=1.0`, equal
  per-mode minibatch quotas; first launch crashed on a warm-start
  attribute bug, fixed) **RAN: FAIL per gate and the blind-axis LINE
  IS CLOSED — but the park MIGRATED.** Stratification fixed the
  dilution seesaw as predicted (lower kept 6/6 det+sto, crouch rise
  4/4, hold det valid_plant 6/6) and foot idx1 — parked six straight
  runs — recovered 0.03→0.90; foot idx4 parked at 0.02 in its place.
  The habit is SHED EXACTLY ONE FOOT (five-foot stance is sufficient
  and cheaper); supervision only moves WHICH foot, never WHETHER. Det
  flat rise still stalls (106mm under-drive). Per pre-registration:
  hard1 stays deployed, stand-specialist handoff stands; reopen lever
  (unqueued) = price the min-over-feet load, not the product, and log
  per-mode bc_anchor_loss FIRST (CODE — only the aggregate is logged
  today) before ANY further stand arm.
  **08-12: that reopened lever (`cw-stand-minfeet1`, min-over-feet
  hold income + per-mode bc_anchor_loss logging, both landed) FAILS
  identically: `env/hold_feet_factor` 0.105, same 0.1–0.35 plateau,
  while `train/bc_anchor_loss_hold` is LOW/converged (0.0107) — the
  anchor is teaching the park, not losing to it. PRICING FAMILY NOW
  TERMINALLY CLOSED for the parked foot; only remaining lever is
  anchor-side (find/patch the reference tick showing a lifted leg at
  a plant-adjacent state), unqueued, spec first.** Detail: RISE.md.
  **08-12: two MORE independent pricing arms reconfirm the closure
  from different angles — `cw-stand-margin1` (reward.k_support_margin,
  attacking rise's 3-foot knife-edge) and `cw-stand-transdrag1`
  (reward.k_drag_trans, attacking the stand/sit foot-scrape) BOTH
  FAIL, and BOTH independently reproduce the identical hold one-foot
  park (idx1, duty 0.03-0.05 vs the frozen parent's 0.90) even with
  hold_still_gate+hold_flag_fade already on.** margin1's own target
  stat didn't even move (det-rise plant_margin_mm 157 vs parent 154,
  inside noise — the BC anchor pins the rise trajectory so hard a new
  income term can't shift it either). transdrag1 confirms the
  mechanism precisely: a foot with near-zero duty is almost never
  "on" for two consecutive ticks, so it can't accrue the drag charge
  — parking is the escape valve for ANY per-foot pricing term, not
  just the ones aimed at it. Three independent reward-side arms
  (minfeet1, margin1, transdrag1) have now spent 6M steps confirming
  the same closed door. Raising any of these coefficients is
  provably counterproductive (parking becomes MORE attractive, not
  less). Do not queue another hold/rise/lower pricing variant on this
  mechanism. The anchor-side fix is now the sole lever and is
  overdue: the working theory (untested) is that `_q_nom` — the
  frozen hold/track BC target, captured post-settle at
  `sim_env.py:1160` — may itself carry a small per-foot asymmetry
  from whatever pose the settle/warm-start happens to land on, which
  the anchor then faithfully (and increasingly, under any added
  pressure) teaches forward. Spec/verify this before writing code.
  **08-12 ~09:5x: verified — and BOTH standing theories are wrong.**
  (1) `_q_nom` exonerated: 48/48 hold resets settle all-six-loaded
  (3.2–3.6 N per foot; feet 1/4 lightest at 3.19 N — the only two
  feet any park ever chose). (2) PPO does not "defy" the anchor:
  the parked leg's anchor MSE (0.0032, measured on the live parked
  margin1 policy in a det hold) matches the clean parent's same leg
  (0.0031). **ROOT CAUSE: the one-foot park is INVISIBLE to
  joint-space action supervision — a mm-scale contact break is
  fractions of a degree of hip lift across 3 of 18 dims (~1e-4
  MSE).** Every "converged" anchor and every per-foot price was
  blind to it by construction. Landed fix (default off, bit-exact,
  tests green): `train.bc_anchor_foot_z` supervises commanded FK
  foot heights (10 mm hover ≈ 1.0 loss at default scale, ≥50x the
  joint-MSE ratio, pinned by bank test). First arm cw-stand-footz1-r1.
  **08-12: `cw-stand-footz1-r1` RESULT — PASS (partial), the fix
  WORKS on its primary target.** Det hold: ALL SIX feet duty 0.92–0.98
  in every one of 6 episodes (frozen parent `margin1` scores 0.05 on
  leg idx1 in the identical test) — the first clean six-foot det hold
  after 6+ straight pricing-arm failures, valid_plant 6/6,
  video-confirmed level quiet stand. Two minor clauses miss: sto hold
  valid_plant 4/6 (2/6 trip a >2.0A tail-current spec check, the SAME
  rate the frozen parent itself trips at — not park-related, no leg
  <0.1 duty); det rise 5/6 valid_plant (parent was 6/6) — one
  flat-start height-only miss, zero falls, video reads as an honest
  crouch-to-stand. Det lower 4/6 matches the parent's own baseline
  exactly, with the identical pre-existing 3-leg-proud pattern
  independently confirmed in margin1's own report (inherited, not a
  new cheat introduced by this run). Hold drag 188mm vs parent's
  159mm (+18%, plausible cost of a foot that now actually loads
  instead of hovering above the ground rent-free).
  `train/bc_anchor_footz_loss` fell 5.1→1.3–1.5 and plateaued
  (did not converge to ~0 — residual commanded hover likely
  concentrated in rise/lower ticks, not hold). 10M hardening
  continuation `cw-stand-footz1-hard1` queued to consolidate the rise
  miss and confirm durability before any champion-replacement call;
  hard1 remains deployed meanwhile.
  **08-12: `cw-stand-footz1-hard1` (10M) FAILED its own gate** —
  hold survives hardening (det+sto all-six duty 0.92–0.99) but
  LOWER regressed to 0/12 (known 3-leg outrigger, clearances to
  170mm, worse under budget; this lineage never had the lower-mode
  anchor). **08-12 midday: the combination arm `cw-stand-footlow1`
  (footz hold fix + anchormix1-r1's lower-anchor bundle) FAILED its
  own gate but proved the two fixes ADDITIVE on hold+lower: first
  policy ever with clean six-foot hold (det duty ≥0.94 all feet,
  6/6) AND 12/12 det+sto lower with sub-mm end clearances. RISE
  paid instead: det 3/6 / sto 2/6, belly-down stall ~100mm short —
  the anchormix lineage's known det flat-rise stall, carried in by
  the state-aligned/lookahead bundle, with `bc_anchor_loss_rise`
  LOW/converged (0.011) during the stall: joint-space supervision
  is anchor-BLIND to global rise progress (same lesson class as the
  mm-hover park). Rise-from-flat is now the LAST broken stance mode
  in an otherwise-complete policy. Next lever is an alignment-audit
  spec pass (which reference tick does the state-aligned anchor
  select at the stalled belly state?), NOT another blind anchor
  variant (anchormix closure pre-registration binds). hard1 remains
  deployed.**
  **08-11: REVERSE handoff (walk→stop→sit) also PASSES**
  (`eval_handoff_reverse.py`): specialist lowering on the walker's
  exact stopped state matches its own clean band (4/6 posture-strict
  both physics, zero falls; only miss a cosmetic 62–99mm dangling
  foot, NOT the old weight-bearing flag-leg), and the scripted
  go_zero-sit glide is 6/6 both physics — the sit side of the
  deliverable is COVERED by the scripted glide; the full sim joystick
  motion cycle (rise→drive→stop→sit) now composes with zero falls.
  Optional unqueued polish: BC anchor on lower ticks. rl_docs/RISE.md.
  **08-11 late: rise+hold specialist DEPLOY PORT LANDED** — the robot
  runner's stance slot (stand/lower buttons) now runs
  `ppo_goal_cw_stand_holdbc1_hard1`; the trained goal-ramp profile
  (hold 5 s / ramp 6 s / +111 mm / switch 12.5 s) ships INSIDE the
  weights meta (export_policy_np --extra-meta) so runner constants can
  never drift from a checkpoint's training config; legacy files keep
  the old constants (stance_dr10 rollback = one picker call).
  Contract-locked by `rl_move/tests/test_stand_runner.py` (live-file
  identity, ramp==GoalGenerator, obs layout, SB3 parity); closed-loop
  sim smoke with the DEPLOYED numpy artifacts: flat-belly rise +111 mm,
  zero falls. deploy_adb.sh now also ships rl_walk_weights.json +
  policies/. (The profile-less-copy trap was resolved by the 08-11
  deploy_ssh re-push; bench_blast's info step verifies the profile.)
  **FIRST hardware run HAPPENED 08-11 22:42 (unattended camera bench):
  FAILED tilt_roll trip at 10.2° rel roll ~9 s in, during the
  BELLY-CURL phase, currents low (0.27 A), runner limped clean. Sim
  probe (6 det seeds, DR0): the same rise keeps |roll| ≤1.7° — the
  hardware body rocks over the tucked legs, sim's doesn't. GENUINE
  sim-to-real rocking gap; the 10° trip is CORRECT (raising it invites
  a tip). Fix is training-side (rocking/tilt DR on rise ticks,
  loaded-knee actuator axis) — do NOT bump the trip threshold.
  08-11 eve round 2 UPGRADED this from "one data point" to
  DETERMINISTIC: 5/5 tilt_roll trips, every one at tick ~226–228
  (mid-curl) with roll 10.1–10.6°, two of them from VERIFIED clean
  zero (pose delta 0.5°, preflight green) — start pose exonerated.
  `cw-stand-riserock1` drained as a STUB and is VOID (08-11 late:
  the rocking-DR code was never written — the run trained a default
  config, no science; the rise-rock DR axis + bank is still unbuilt
  CODE work); scripted `/api/zero pose=stand` is the working
  hardware stand-up until a real riserock arm lands. Do NOT retry
  learned STAND on hardware before a riserock-gated checkpoint
  exists.**
  **08-11: the pool-restore bug (commit 65edba7) briefly CONFOUNDED
  the score1/scoreref1/rsi1 "CLOSED" verdicts (episode-recycle pool
  was silently dropping the score-stack + RSI per-episode attrs, so
  those arms weren't paid the reward they were designed to get).
  `cw-stand-rsi2` is the clean re-run on the fixed pool and it
  REPORTS: `env/rise_rsi` held 0.48–0.58 the entire 2M steps (fix
  confirmed working, no more corruption) yet `env/reward_rise_ref`
  and `env/rise_score` still flatlined exactly as before and the
  harness shows the identical tripod cheat (0/6 valid_plant det+sto,
  worst-foot clearance 146–161mm). Ruling: the pool bug is
  EXONERATED as the cause; income-shaping, reference-tracking-as-
  crutch, AND RSI-as-a-fix are all RE-CLOSED, now on clean evidence.
  `cw-stand-rsi3` (one more change: strip the old k_height PENALTY
  that might have funded the cheat) collapsed identically again.
  Decisive read across all six reward-different arms: the
  feet-factor collapse (0.87→~0.17 by the 25% mark) has the same
  shape/timescale regardless of the reward mechanism — behavior that
  doesn't respond to reward changes isn't reward-driven. Diagnosis:
  **warm-start out-of-distribution drift** (the 108–114mm command
  band is ~2.2x the stance champion's trained range; the tight
  tracking kernel only pays a policy already nearly perfect there,
  so early update noise drifts it into the tripod with nothing to
  anchor it back — widening the kernel is bank-blocked). Two CODE
  levers queued, both need a SPECIFICATION pass first: (a) a BC
  anchor loss in the trainer pulling actions toward the reference at
  RSI-spawned states (operator's preferred first spec), (b) the
  structural height↔foot-contact coupling (RL_PLAN queue item 2b).
  Do not queue another reward/income/RSI coefficient variant.
  Detail: rl_docs/RISE.md.
- Yaw: price escalation on a command-invariant drift is CLOSED. Turn
  was DE-SCOPED from the joystick deliverable (operator 08-11
  morning: no camera = no front) and RE-OPENED as a compute
  EXPERIMENT line the same afternoon — still NOT an attempt-#2
  blocker. **08-11 later, RL_PLAN queue 0.2 steps 1–2 DONE
  (rl_docs/TURN.md bottom):** (a) the latent yaw-stack defect is
  FIXED + BANKED — `reward.walk_yaw_hold_prog_gate` (heading-hold
  yaw income gated on achieved linear progress; pre-fix a FROZEN
  body collected +375/ep, the largest channel in the turn stack)
  and `reward.yaw_still_avg_s` (drift charge on the wz EMA; pre-fix
  it taxed the honest gait −110/ep and degenerates ~0). Post-fix
  income is monotone in honesty incl. a calibrated drift-rider;
  stillness-subsidy bank (4 tests, legacy reproduction pinned)
  green; TURN_OVERRIDES trains both fixes ON; the yawcmd1/yawgate2/
  turnfix1 ckpts are gone from all pods (ckpt audit UNTESTABLE).
  (b) the REFLECTION wrapper PASSES on cw-dep-vref1-r1
  (`mirror.MirrorPolicy` + `probe_mirror_turn.py`): mirrored policy
  drifts RIGHT at the naked policy's rate with identical travel
  (DR 0 and 0.35, zero falls); bang-bang chirality selection holds
  heading to 2–4 deg vs 34–50 deg naked runaway — arc-left/
  arc-right/straight with ZERO training, at the drift rate
  (~2 deg/s; commanded-rate tracking still open). (c) mirror-
  symmetry TRAINING is now licensed and queued
  (`cw-walk-mirturn1`, discovery 2M on the fixed pricing) — the
  hypothesis has still never had a clean test; if it fails healthy,
  the mirror line closes and MirrorPolicy selection is the shipped
  turning story. BC-anchor on turn ticks stays in reserve. Sign
  audit still OPEN at the hardware boundary (sim +CCW vs measured
  +omega=CW) and gates any bench turn. Plan: rl_docs/TURN.md.
- Omni translation (walk in ANY direction — the "walk where pointed"
  blocker; no learned policy has ever walked backward): three arms
  collapsed into three different degenerate gaits. **08-11 income
  re-probe (`probe_walk_income.py`) exonerates the pricing on the
  deliverable stack**: honest gait out-earns every degenerate 2-4x
  uniformly across directions at DR 0 AND 0.5, and the collapsed
  checkpoints earn BELOW a freeze under their own reward —
  optimization failure, not a paid basin; reward surgery CLOSED.
  Latent defect in the TURN stack only (ungated yaw kernel paid a
  motionless body full income on linear ticks — FIXED + banked 08-11,
  see the Yaw bullet). Next lever, BC anchor on walk ticks toward the
  command-conditioned scripted TripodGait (third application of the
  twice-proven lever), **FAILED (`cw-omni-transbc1`, 08-11)**:
  anchor loss converged cleanly (0.14→0.0097, better than the
  rise/hold precedent) and policy std stayed flat, yet the identical
  march-in-place/paddle fingerprint reappeared (median forward
  travel 0.01 m/episode, slip/m 6–19 vs champion ~1.2–1.5, zero net
  floor travel in all 12 video-checked det+sto episodes) — the
  pre-registered prediction-if-false. Per-tick imitation does not
  teach the different global stepping pattern each direction needs;
  BC-anchor/reward tuning on this stack is CLOSED (4th
  distinct-or-near collapse).
  **08-11: RESOLVED IN SIM — rot-60 exact equivariance
  (`rl_move/sim/rot60.py`), zero training.** The robot is a regular
  hexagon (six identical leg templates at exact 60° spacing,
  axisymmetric chassis inertia): rotate-60°+relabel-legs is an EXACT
  symmetry of the compiled model (proved mechanically,
  test_rot60.py). The wrapper canonicalizes any commanded heading
  into the ±30° wedge at eval time; a wedge-trained policy covers
  the full circle by construction. `cw-dep-vref1-r1` (THE hardware
  checkpoint) wrapped: every direction 0.024–0.036 trk_err at DR0 +
  own DR0.35, zero falls incl. full-circle flip stress, harness
  20/24 success, slip/m 1.1–1.3 (own band), video-clean six-leg gait
  — naked it is frozen backward (0.027 m of 0.30). hist16-dep1 naked
  DEGENERATES AT EVAL TIME into the leg-sacrifice on off-wedge
  commands (slip 7–11/m, gait_valid 3–5/6); wrapped: gait_valid
  24/24, slip 1.3–1.6. The four training collapses were PPO failing
  to DISCOVER rotated gaits — structural fix, no omni arm needed.
  **Deploy-side port LANDED 08-11 (later cycle): the robot runner's
  walk mode wraps rot60.Rot60Policy itself** (no ported copy —
  `linux_control/rl_policy.py make_walk_canonicalizer`, rot60.py
  shipped by deploy_adb.sh, numpy-only verified). Default ON, k=0 is
  a BIT-EXACT no-op for forward-wedge commands (proven contract
  untouched); off-wedge commands are refused if the wrapper is
  missing/disabled. Replay-parity locked by
  `rl_move/tests/test_rot60_runner.py` (obs-layout, full-circle +
  hysteresis parity, real deployed weights);   per-tick `rot60_k`
  logged in the episode CSV for on-hardware replay checks. **FIRST
  off-wedge hardware run HAPPENED 08-11 22:48 (tip1, BACKWARD 6 s):
  the port WORKS (rot60:true, k engaged, terminal result logged) but
  the walk FELL — peak 27° roll, tilt trip. **08-11 eve round 2: tip1
  BACKWARD walked CLEAN (rode a 16.7° takeoff transient to tail 1.5°,
  full 6 s, ends standing on camera) — rot60 off-wedge is 1 clean /
  1 fall.** More reps after the takeoff-transient arm lands.
- Tipped-start DR is default-ON everywhere (operator ruling 08-10,
  "ideally all runs would learn this capability", after the deployed
  walk's hardware runaway roll): `dr.tipped_start_prob=0.30` (scaled
  by dr_scale; dose 6–18° is NOT scaled) in `domain_rand.py`, applied
  at plant/park starts with a LEVEL tilt reference so the policy is
  paid to level out, capped at 70% of the run's tilt envelope. Every
  run's recovery shows as `SCORE/tipped_recovery_success`
  (rl_docs/EVALS.md — read the gate-fix caveat there; baseline is
  vref1-r1 7/8 at 12°, NOT the retracted 0.25). Discovery arm
  `cw-dep-tip1` TRAINED 08-10: no sim separation vs parent (static-
  lean recovery was already present — the hardware runaway is a
  sim-to-real pinning gap, HARDWARE.md). **tip1 then RAN ON HARDWARE
  4x (08-10 night): 1 runaway / 3 CLEAN level walks — a learned
  policy has driven the robot; obs pipeline verified bit-exact
  offline.** It is the ACTIVE walk slot. **08-11 eve correction
  (robot's own event log + camera): in the 21:4x attended A/B, tip1
  fwd tripped tilt_roll 2/3 — the bench summary's "done" entries were
  kickoff responses, since fixed (terminal results now recorded).
  Dominant on-camera signature for BOTH policies: a 20–25° TAKEOFF
  roll transient right after gait start — sometimes fully recovered
  (vref1 full-6s walk ending dead level, tail 0.9°), sometimes a fall
  (vref1's 3rd runaway; tip1 backward). Judge walks by fell/tail, not
  the peak-based "runaway" flag. **08-11 FULL-NIGHT VERDICT (18 walks,
  9 camera sessions, `bench_report`): the takeoff transient is
  UNIVERSAL — every walk crosses 5° roll by 0.6–1.5 s, peaks 13–27° —
  and falls are ~a coin flip for BOTH policies (vref1 6/10 fell, tip1
  4/7, no predictor in peak/direction). NO A/B winner; the
  early-evening "tip1 champion" read was a small-sample artifact, and
  the A/B has a direction confound (tip1 never walked forward — fix
  the alternation before re-judging). vref1's "zero-erosion ACCEPTED"
  stays a SIM contract fact only.** Sim-side status: the two operator
  backlog stubs drained WITHOUT their DR knobs and are VOID (trained
  defaults, no science — do not build on those checkpoints). The
  proper relaunch `cw-dep-tip1-takeoff25-r1` (dr.tipped_start
  0.5/12–25, warm tip1) then FAILED its gate with a decisive
  mechanism read: under the identical 20–25° injection with matched
  tip1 baseline, child==parent (0/12 valid both, ZERO falls both) —
  sim already recovers static 20–25° tipped starts, the axis is
  SATURATED. **Tipped-start DOSE is CLOSED as the takeoff-fall fix
  (2nd no-separation arm); the hardware transient is DYNAMIC — a
  roll-rate injection during gait start.** **08-12: that CODE landed
  (`dr.walk_kick_*`, commit 7d34fc6) and TRAINED (`cw-dep-tip1-kick1`)
  — SAME NULL RESULT (3rd no-separation arm).** Matched-parent probe
  at the gate's own dose (forced prob=1.0, 14–22°, n=24 seeds each,
  `rl_move/sim/probe_walk_kick.py`): ZERO tilt falls for BOTH
  cw-dep-tip1-kick1 and frozen tip1 (0/24 vs 0/24), tail \|roll\|
  medians 0.68°/0.93° (both far under the 4° bar) — nominal DR0
  retention holds (gait_valid 6/6, prog 1.01, slip 0.97). **The
  command-pulse/dynamic-DR-injection family for the takeoff transient
  is now CLOSED** (static dose + dynamic dose both saturate with zero
  separation) — do not schedule another dose/coefficient variant on
  this mechanism. **08-12: the rise-rock axis (same command-side
  family, for the belly-curl rocking fall) TRAINED and gives the
  OPPOSITE-DIRECTION null** (`cw-stand-riserock2-r1`, matched-parent
  gate: dr.rise_rock_prob=1.0/deg=10,10 fixed — the exact bench-
  measured trip threshold — det, `--baseline hard1`): child rise
  0/6 valid_plant (1/6 tilt_roll fall, 5/6 stall short of plant),
  frozen hard1 under the IDENTICAL injection ALSO 0/6 valid_plant
  (2/6 falls) — zero separation, but here BOTH sides fail hard
  instead of both sides passing clean. Own-mix retention (prob 0.5,
  deg 6–12, as trained) stays clean: det rise/hold/lower 6/6, no
  regression — the failure is specific to the guaranteed near-
  threshold dose. Two command-bias roll-injection axes (walk-kick,
  rise-rock) now both show zero measurable learned separation from a
  frozen parent, in opposite directions — building evidence this
  whole family (randomize a temporary body-roll bias) doesn't teach
  resilience either way. **08-12: the gentler dose retry
  (`cw-stand-riserock3`, deg 6–10) CLOSES the axis, via a new failure
  mode, not a repeat null** — own-mix det LOWER collapsed 6/6→1/6
  (worst clearance 126mm vs the 60mm bar), video-confirmed a fresh
  three-leg flag-leg/outrigger cheat (known LOWER exploit class, no
  forensics needed) that breaks the gate's own retention clause
  outright, regardless of the rise-rock injection looking fine in
  isolation (5/6, no falls). **RISE-ROCK DR FAMILY CLOSED** (2 doses,
  2 misses). Both command-bias roll-injection axes (walk-kick,
  rise-rock) are now closed. **08-12: open-loop trace replay
  (`replay_trace.py`, full findings rl_docs/SIM.md §4) diagnosed
  BOTH transients from real hardware tapes** — the walk takeoff
  excursion reproduces open-loop in sim (the policy never visits
  these states in training, not an actuator-model gap) and the stand
  failure is a support-geometry knife-edge (hw pivots on L4 and
  unloads the left pads; sim keeps them planted regardless of CoM/μ).
  Two MECHANISM-CORRECTED (not command-bias) axes shipped from this
  diagnosis: `dr.walk_push_*` (an xfrc chassis-roll TORQUE pulse, not
  a command pulse) and ramp-gated `dr.rise_rock_*`. **`cw-dep-tip1-
  push1` (walk_push, 2M discovery) is the FIRST axis in this whole
  family to show real separation**: matched-parent
  `probe_walk_push.py` (forced 2.6N·m/1.5s, n=12/side) — child falls
  5/12 vs frozen tip1 9/12 (1.8x lower, short of the pre-registered
  2x bar), but paired-by-seed every disagreement favors the child and
  none favor the parent. Nominal DR0 retention clean. PARTIAL/
  INFORMATIVE, not a clean PASS. **08-12: hardening continuation
  `cw-dep-tip1-push1-hard1` (10M, identical recipe) FAILS the same
  matched-parent probe bit-for-bit** — fall count is IDENTICAL to the
  2M discovery arm (5/12 vs frozen tip1 9/12, same 1.8x, same 4
  discordant seeds), tail-roll among survivors marginally worse
  (1.63 vs 1.26° median); 5x more budget bought nothing. DR0
  retention still clean (gait_valid matches parent's own band, zero
  new falls). **TORQUE-DR (walk_push) FAMILY NOW CLOSED FOR GOOD** —
  all three perturb-during-training axes (walk-kick, rise-rock,
  walk-push) are closed; the takeoff-roll transient's remaining lever
  is contact/pinning geometry modeling (belly/tucked-leg collision),
  not any further DR variant. Turn signs:
  CLOSED (operator 08-11 night): the robot turns the way the drawn
  signs say for BOTH command signs — the convention is correct end
  to end, no flip needed in the deploy bridge. Rotation RATE remains
  unmeasured (degrees are hard to eyeball; the earlier single-camera
  +0.3=CCW reading stands, the 08-09 "+omega=CW" scripted-gait note
  is superseded). Bench turn sessions are no longer sign-gated. The evening's
  "thermal wall" was mostly PHANTOM single-read bus temps (a "150 °C"
  hip read 33 °C seconds later); safe_zero/pinned_tip temp trips now
  debounced, servo_watch gained a kill-all-motion thermal panic, all
  deployed. Consolidated read + RL implications:
  rl_docs/BENCH_REPORT_2026-08-11.md.
- Quad-hold is solid but mixing erodes walk (four dose points) —
  deploy-time specialist, never a mixed diet. FOUR-LEG WALKING is a
  sanctioned NEW experiment line (operator 08-11 afternoon): weight
  shifted BACK onto the four rear legs, front pair raised as
  "hands". SPEC FIRST — the current reward provably punishes it
  (park-duty/flag-leg/gait_valid machinery defines it as the
  leg-sacrifice cheat; k_pitch + level-referenced tilt termination
  charge the rear-shift posture); reward must be lift-command- and
  posture-conditioned, and banked, before any launch (RL_PLAN
  queue 0.3).
- GAIT CLEANUP / tall walking (RL_PLAN queue -0.5): before any new
  arm, the P0 reward-accuracy probe is binding (operator 08-11):
  replay the tape-proven scripted gait AT PLANT HEIGHT through the
  full champion reward stack (income + penalties + termination
  risk) vs the trained crouch-paddle's actual earnings. dep-hgt1/2
  showed height INCOME is not the lever; the suspect is the penalty
  side (the real gait rocks ±10–20° — gyro/tilt pricing may be
  anti-incentivizing honest tall stepping). Seven-plus pricing/RSI
  arms all converged on the identical -72..-75mm crouch with leg-yaw
  pinned negative at the 35° splay limit — CLOSED as a lever.
  **08-12: BC-INIT (pure action pretraining on the scripted tall
  gait, then RL fine-tune) BREAKS the wall — `cw-dep-bcgait1`:
  probe_tall_wall steady height -10..+6mm, leg-yaw margin POSITIVE
  +17..+18deg, harness confirms real travel (prog_ratio 0.77,
  gait_valid 6/6, zero falls).** Not yet polished (secondary slip bar
  missed, one stochastic leg-sacrifice episode) — informative PASS on
  the existence question, not a deployable candidate; next is a
  hardening continuation. **Same cycle: `cw-dep-bcgait1-hard1` (10M)
  PASSES decisively** — height stays in-band, yaw margin stays
  positive, slip drops under the 1.8 bar both passes (1.43/1.51), the
  sacrificed-leg episode is gone, zero falls. Strongest tall-walking
  candidate yet; DR/tipped-start retention panel is the next step,
  NOT yet run — not hardware-ready. **08-12: panel items 1+2 (friction
  0.4-1.6x, floor tilt 5deg) PASS, both free** (`cw-dep-bcgait1-hard1-fric`,
  `-groundtilt5`: gait_valid 6/6 all slices, zero falls, slip in-band,
  probe_tall_wall height matches/beats the parent, clean video) — watch
  item: leg-yaw limit margin narrows further under DR (down to 0.16deg
  on one seed, still positive), continuing the lineage-wide narrowing
  trend; not gate-breaking. **CORRECTION: tipped-start dose isolation
  and combined-axis DR-compose are NOT needed here — both are already
  closed generic classes (zero-separation / proven-free).** The real
  open question (does this lineage share the dep-line's walk-takeoff
  roll vulnerability, SIM.md gap 4) is ANSWERED 08-12 (`probe_walk_push`,
  generalized, no training): forced-injection fall rate
  `cw-dep-bcgait1-hard1` 6/12 vs frozen `cw-dep-tip1` 9/12 — lower, not
  worse, matching the push-trained lineage's 5/12 for free. No
  push-training respec needed (that family stays closed anyway); Gate 0
  now rests on hardware bench evidence, not another sim DR axis. Detail:
  GAIT.md, hw/STATUS.md.
- MoE only after clean multitask training (explicit mode ID, correct
  rewards, enough plain-MLP capacity) shows real skill interference.

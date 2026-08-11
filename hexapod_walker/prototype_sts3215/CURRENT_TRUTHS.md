# CURRENT TRUTHS — read this BEFORE any history file

Accepted current facts + rulings only, never campaign narrative. If
RL_LOG/archive prose disagrees with a line here, this file wins.
Update ONLY when a ruling is accepted or hardware produces new
evidence; keep 50–80 lines. Reading order: RL_GOALS.md → this file →
RL_PLAN.md → RESEARCH_RULES.md → rl_docs/SIM.md.
Last regenerated: 2026-08-11 late (omni translation RESOLVED IN SIM:
rot-60 exact-equivariance wrapper, zero training — the hardware
checkpoint now walks the full circle; remaining omni work is the
deploy-side port. Earlier same day: rise/hold solved via BC-anchor,
both handoffs compose).

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
- Walking is CHEAPER than standing on hardware (0.33–0.45 A mean
  total vs 0.59 A). Old sim effort/current assumptions are NOT
  trusted; k_current=0 on hardware arms until calibrated.
- Contact calibration DONE (08-10, `calibrate_slip.py`): sim replay
  of the exact hardware gait travels 0.35–0.41 of commanded (real
  0.50–0.51), speed-invariant, walking current in-band — sim does
  NOT price sliding as free (it is slightly conservative); friction
  saturates ≥1.5 so μ is not the knob. Sim hold current 0.11 A vs
  real 0.59 A is the remaining (effort) gap — needs a holding-
  current model fit, not a scalar.
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

## Campaign rulings in force

- KPI = unresolved blockers between today's robot and the next
  useful joystick hardware test. NOT GPU occupancy; idle pods fine.
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
  promoted: it missed the pre-registered hold no-regression bar
  (hold sto valid_plant current-tail flags 5/6 vs hard1 1/6; posture/
  stillness identical). hard1 STAYS the deployed stance policy;
  `ppo_goal_cw_stand_crouchrise1` (md5 3877e16c) is banked if bench
  shows crouch starts matter. Stand lineage now fully CLOSED.
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
  policies/. CAUTION: the operator independently activated a
  PROFILE-LESS copy on the robot that morning (md5 6620705c) — it
  would get the legacy +50mm/4s ramp, out-of-distribution for this
  policy; re-push/re-select before STAND (HARDWARE.md bench-state
  note). Awaiting bench validation (attempt #2).
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
- Yaw: price escalation on a command-invariant drift is CLOSED. The
  new mechanism set is landed and its TURN bank PASSES (08-10):
  signed rotation income (k_yaw_prog), heading-hold drift charge
  (k_yaw_still), turn-in-place curriculum (walk_turn_in_place_frac).
  Sign audit still OPEN at the hardware boundary (sim +CCW vs
  measured +omega=CW). Turn is DE-SCOPED from the joystick
  deliverable (operator 08-11: no camera = no front). Plan:
  rl_docs/TURN.md.
- Omni translation (walk in ANY direction — the "walk where pointed"
  blocker; no learned policy has ever walked backward): three arms
  collapsed into three different degenerate gaits. **08-11 income
  re-probe (`probe_walk_income.py`) exonerates the pricing on the
  deliverable stack**: honest gait out-earns every degenerate 2-4x
  uniformly across directions at DR 0 AND 0.5, and the collapsed
  checkpoints earn BELOW a freeze under their own reward —
  optimization failure, not a paid basin; reward surgery CLOSED.
  Latent defect in the de-scoped TURN stack only (ungated yaw kernel
  pays a motionless body full income on linear ticks; fix before any
  turn re-scope). Next lever, BC anchor on walk ticks toward the
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
  hysteresis parity, real deployed weights); per-tick `rot60_k`
  logged in the episode CSV for on-hardware replay checks. Awaiting
  bench validation only (attempt #2).
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
  sim-to-real pinning gap, HARDWARE.md); hardware A/B pending,
  `dep_tip1.json` staged in the robot's walk picker.
- Quad-hold is solid but mixing erodes walk — deploy-time
  specialist; quad comes after the core joystick set is coherent.
- MoE only after clean multitask training (explicit mode ID, correct
  rewards, enough plain-MLP capacity) shows real skill interference.

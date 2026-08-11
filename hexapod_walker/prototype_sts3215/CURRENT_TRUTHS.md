# CURRENT TRUTHS — read this BEFORE any history file

Accepted current facts + rulings only, never campaign narrative. If
RL_LOG/archive prose disagrees with a line here, this file wins.
Update ONLY when a ruling is accepted or hardware produces new
evidence; keep 50–80 lines. Reading order: RL_GOALS.md → this file →
RL_PLAN.md → RESEARCH_RULES.md → rl_docs/SIM.md.
Last regenerated: 2026-08-11 (rise: clean re-run `cw-stand-rsi2`
reports — pool-restore bug exonerated, income-shaping/RSI/reference-
tracking RE-CLOSED on clean data; only remaining lever is the
structural height<->contact coupling, CODE).

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
  Detail: rl_docs/RISE.md. The only remaining lever is a structural
  coupling between the height goal and measured foot contact (CODE,
  not yet built) — RL_PLAN queue item 2b.
- Yaw: price escalation on a command-invariant drift is CLOSED. The
  new mechanism set is landed and its TURN bank PASSES (08-10):
  signed rotation income (k_yaw_prog), heading-hold drift charge
  (k_yaw_still), turn-in-place curriculum (walk_turn_in_place_frac).
  Sign audit still OPEN at the hardware boundary (sim +CCW vs
  measured +omega=CW). Plan: rl_docs/TURN.md.
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

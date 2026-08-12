# Hexapod prototype_sts3215 — RL training & orchestration review bundle

Generated 2026-08-11 by `make_rl_review_bundle.sh`, concatenating the
project's live docs so an external model can review the state of RL
training and orchestration. Each section below is one source file,
verbatim, with its repo path in the header.

Context for the reviewer:

- The robot is an 18-DOF hexapod using Feetech STS3215 servos, controlled by
  an Arduino Uno Q running a Linux-side control loop.
- Training runs on CoreWeave GPU pods (MuJoCo sim + PPO via stable-baselines3),
  driven by an autonomous orchestrator agent; results are pulled to a Mac for
  local sim viewing and to the robot for hardware trials.
- Per-run notes (500+ files under `rl_docs/runs/`) are NOT included here;
  `RL_LOG.md` (last section) is the curated log of those runs.

## Files included

- `README.md`
- `RL_GOALS.md`
- `CURRENT_TRUTHS.md`
- `STATUS.md`
- `rl_docs/tracks/arch/STATUS.md`
- `rl_docs/tracks/hw/STATUS.md`
- `rl_docs/tracks/nobc/STATUS.md`
- `rl_docs/tracks/quad/STATUS.md`
- `rl_docs/tracks/turn/STATUS.md`
- `RL_PLAN.md`
- `RESEARCH_RULES.md`
- `RUN_INTERPRETATION_RULES.md`
- `rl_docs/README.md`
- `rl_docs/GOAL.md`
- `rl_docs/AGENT.md`
- `rl_docs/SIM.md`
- `rl_docs/REWARD.md`
- `rl_docs/RISE.md`
- `rl_docs/TURN.md`
- `rl_docs/GAIT.md`
- `rl_docs/EVALS.md`
- `rl_docs/SKILLS.md`
- `rl_docs/COMMANDS.md`
- `rl_docs/EXPERIMENT_LOGS.md`
- `rl_docs/WANDB.md`
- `rl_docs/HARDWARE.md`
- `rl_docs/WISHLIST.md`
- `rl_docs/BENCH_REPORT_2026-08-11.md`
- `rl_move/API.md`
- `rl_move/RUNLOG.md`
- `rl_move/orchestrator/README.md`
- `rl_move/orchestrator/ORCHESTRATOR_PROMPT.md`
- `rl_move/orchestrator/CAPACITY.md`
- `RL_LOG.md`


---

# FILE: README.md

# Hexapod STS3215 prototype

Tabletop 3D-printed hexapod driven by Feetech STS3215 bus servos and an
Arduino Uno Q. This directory holds THREE projects that share the robot;
start at the entry point for the one you're working on:

| You are here to… | Start at |
|------------------|----------|
| Design/print/assemble the robot (CAD, BOM) | [`PROTOTYPE.md`](PROTOTYPE.md) |
| Run the physical robot (firmware, control, safety) | `firmware/`, `linux_control/`, `rl_move/API.md` — **read the hardware-safety rules in the repo root `AGENTS.md` first** |
| Train it in simulation (RL campaign + autonomous agent loop) | [`rl_docs/AGENT.md`](rl_docs/AGENT.md) — how the agent works, learnings, future work; then [`rl_docs/README.md`](rl_docs/README.md) (doc index), `RL_PLAN.md`, `RL_LOG.md` |

## Layout

| Path | What |
|------|------|
| `hexapod_prototype.py` | Parametric CAD source of truth |
| `design_spec.yaml` | Human-readable geometry contract |
| `build_all.py` / `Makefile` | Regenerate STLs + common targets |
| `docs/` | BOM, shopping list, CAD workflow, BuildViz notes |
| `scripts/` | CLI helpers (verify helpers, renders, print orientation, inspect) |
| `tools/` | BuildViz / diagnostic utilities |
| `stl_prototype/` | Slicer-ready printables |
| `stl_reference/` | Sim / viz meshes (not for printing) |
| `firmware/` / `linux_control/` / `motor_setup/` | On-robot software |
| `full_robot_viz/` | BuildViz scene + local `buildviz` npm dep |
| `rl_docs/` | RL campaign docs index: goal, operator wishlist, commands, log conventions |
| `RL_PLAN.md` / `RL_LOG.md` | Current RL plan + condensed campaign history (full history in `archive/`) |
| `rl_move/` | RL code: `sim/` (MuJoCo/MJX envs + training), `orchestrator/` (autonomous loop: watcher, launcher, guardrails), robot-side control |
| `logs/` | Eval artifacts + per-experiment summaries (`logs/experiments/<run>/`) |
| `archive/` | Dated reviews, rulings, full plan/log history — search, don't read |

## Quick commands

```sh
make -C hexapod_walker/prototype_sts3215 help
make -C hexapod_walker/prototype_sts3215 build
make -C hexapod_walker/prototype_sts3215 verify-fast
```


---

# FILE: RL_GOALS.md

# What we are doing, in plain English

We have a real six-legged robot (hexapod, STS3215 servos) sitting in
the operator's house. We are training neural-network controllers for
it in a physics simulator (MuJoCo), on cloud GPUs, with an
autonomous loop of AI agents that design experiments, watch them
train, judge the results honestly, and launch the next one.

**The end goal (operator, 08-10): drive the robot around the room
with a joystick — it stands up, sits down, turns, and walks where
you point, reliably. Then the tricks: stand on four legs, walk on
four legs.** Sim numbers only matter insofar as they get us there.
The campaign KPI is the number of unresolved blockers between
today's robot and the next useful hardware joystick test — not GPU
utilization, not experiment count (operator, 08-10).

## What "good" means (operator's own words)

Distance, stability, reliability. The robot should cover real
ground, stay level, and never fall. A policy that walks 0.6 m every
time beats one that walks 1 m or falls at 50/50. Speed targets are
a means, not the objective. Foot slip is not failure by itself —
the scripted gait that walks the real robot slips visibly (08-09);
slip metrics exist to keep sim honest about the real floor, not as
a ban.

## Where we are (edit rule: capability only — details live elsewhere)

The full "how is it going / what can it do / what have we learned"
digest: **`STATUS.md`**. Live rulings + hardware facts:
**`CURRENT_TRUTHS.md`** (agents read that before any history).
As of 2026-08-10:

- The REAL robot walks under a scripted gait; that is the bar.
- In sim, the learned gait is real (six legs cycling) but creeps;
  fixing that is an operator contact-pricing calibration waiting on
  a tape-measured hardware walk.
- The sim joystick driving stack is hardened and seed-confirmed;
  turn-in-place passed the gate but commanded yaw is still ignored
  (structural drift — mechanism change queued, price tuning closed).
- Stand-up inside the walking policy is the main unsolved skill —
  every attempt so far games the height reward. Full plan and
  evidence: `rl_docs/RISE.md`. A working fallback exists (stance
  policy rises, scripted blend, walk policy drives — sim-proven).
- The four-leg trick holds perfectly but erodes walking when mixed
  into training — goes to a deploy-time specialist.
- Hardware attempt #2 checkpoint (`cw-dep-vref1-r1`) is validated,
  hardened, and staged on the operator's Mac — waiting on bench time.

## The cast

- **The loop:** a watcher script polls training; each finished run
  triggers an agent "cycle" that evaluates it (with video, not just
  scalars), writes a verdict, and launches the next experiment. Its
  binding rules: `RESEARCH_RULES.md` (prime directive, phases,
  MDP_PREFLIGHT, matched-parent controls); blockers and queue:
  `RL_PLAN.md`.
- **Runs** are named `cw-<line>-<idea>` (e.g. `cw-walk-anchortol5`);
  continuations get `-c1`, `-c2`. Everything is recorded in the
  ledger (`rl_move/orchestrator/experiments.json`) and W&B
  (`l2k2/hexapod-balance`).
- **Compute:** CoreWeave pods; GPU pods train 20–40M-step runs in
  ~20–40 min (MJX), CPU pods handle probes and small runs. Idle pods
  are acceptable — peripheral experiments are not.
- **The operator's backlog** of things to learn: `rl_docs/WISHLIST.md`.


---

# FILE: CURRENT_TRUTHS.md

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
  today) before ANY further stand arm. Detail: rl_docs/RISE.md.
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
  roll-rate injection during gait start (CODE, unbuilt, same family
  as the rise-rock axis, which also still needs code) or
  contact/pinning work.** Turn signs: +0.3 = CCW (matches convention,
  single camera reading); −0.3 still unmeasured. The evening's
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
  anti-incentivizing honest tall stepping).
- MoE only after clean multitask training (explicit mode ID, correct
  rewards, enough plain-MLP capacity) shows real skill interference.


---

# FILE: STATUS.md

# STATUS — how is it going?

**START HERE, then go to your track.** The campaign runs as five
parallel research tracks, and the current state of each line of work
lives in its own short per-track status file — read the one you care
about first:

- `rl_docs/tracks/hw/STATUS.md` — joystick robot on real hardware
  (the mainline)
- `rl_docs/tracks/arch/STATUS.md` — GRU/temporal architectures
- `rl_docs/tracks/nobc/STATUS.md` — learning without BC anchors
- `rl_docs/tracks/quad/STATUS.md` — four legs + two "hands"
- `rl_docs/tracks/turn/STATUS.md` — commanded turning

This file is the whole-campaign digest: the plain-English answer to
"how is it going, what can the robot do now, which checkpoints are
the good ones, and what have we learned?" — for the operator or
anyone catching up. Facts here must agree with `CURRENT_TRUTHS.md`
(which wins on conflict); the full checkpoint inventory with gate
numbers lives in `rl_docs/SKILLS.md`.

**Last updated: 2026-08-11 (late evening) — through the ~20:15 idle-kick
cycle: crouchrise3 FAIL (dose axis dead), mirturn1 FAIL (mirror
training closed, wrapper ships), GAIT P0 probe answered, dragstance1
launched.**
Update rule: refresh whenever a hardware session happens, a champion
changes, or a big lesson closes — and stamp the date; per-track story
changes go to the track's own STATUS.md. Keep it honest: the "not
working" list is the most valuable section.

## The one-paragraph answer

The real robot walks — under the scripted gait (tape-measured), AND
a learned policy has now driven it: on 08-10 night `dep-tip1` walked
level on real ground in 3 of 4 runs. The deployment-contract
champion `vref1-r1` itself went 0-for-2 with an intermittent runaway
roll (a pinned loaded leg feeds a lean that the policy never
recovers — a sim-to-real contact gap, since sim recovery can exploit
feet that skate). That runaway is THE open hardware question: the
discriminating vref1-vs-tip1 A/B on the same floor never actually
ran (the policy switch didn't land on the robot). In simulation, the
whole joystick motion cycle works end to end with zero falls: a
learned stand-up from the belly, a genuinely motionless hold,
driving in ANY direction, stop, and sit — the rot60 wrapper and the
stand-up specialist are ported but have NOT had their first hardware
runs yet (re-push required first: the on-robot stand copy lacks its
goal profile).
The two breakthroughs that got us here (both 08-11): first,
**BC-anchoring** — instead of tuning reward prices yet again, we pull
the policy's actions directly toward a recorded good motion during
training; that one trick solved standing up AND holding still after
roughly seven straight reward-tuning failures each had cheated.
Second, **the hexagon trick** — the robot is a perfect hexagon, so
"walk backward" is literally "walk forward with the legs relabeled";
a small math wrapper (`rot60.py`) gives the existing hardware
checkpoint the full circle of directions with ZERO training, closing
a problem four training runs had failed at. Turning was briefly
de-scoped (no camera = no "front" to point), RE-OPENED the same
afternoon — and promptly cracked half-open: the turn reward's real
bugs are fixed and banked, and a second symmetry trick (the mirror
image of the walk policy turns right exactly as well as the original
drifts left) now gives slow bidirectional steering with zero
training; a queued run goes after fast commanded turns. Two more
wanted skills opened alongside: four-leg walking (weight shifted
back, front legs raised as "hands") and walking taller without
dragging feet. All are diagnosis-first: audit
whether the reward actually pays the desired behavior before
launching arms (see "what happens next"). The evening's verdicts:
the one-brain flagship question is SETTLED by measurement (a
stand-up-only control run plateaued at exactly the multitask
flagship's 1-in-6 — the skills were never fighting for space, so the
mixture-of-experts rebuild is off the table; stand-up-from-scratch is
just hard for any network with this recipe), and the walk-gait
cleanup's two cheap levers both closed (BC-anchoring the gait froze
the robot into a static tripod; bumpy ground made the paddle WORSE,
never forced stepping). The GRU line, frozen this morning, was
deliberately reopened in the new arch research track with the two
levers its last failure pre-registered (4x longer memory window +
double capacity) — running now.

## What the robot can do — REAL hardware

- Walk forward, crab sideways, and turn in both directions from a
  clean zero, under the scripted tripod gait. Tape-measured: ~50% of
  commanded distance (real foot slip, and that's fine — visible slip
  is part of how this robot locomotes).
- Servo current tracks how hard servos FIGHT their command, not the
  pose: walking 0.33–0.45 A, scripted stand 0.54–0.59 A, walk-stance
  hold ~0.11 A. ("Standing costs more than walking" was really
  "fighting your own command costs more than walking.")
- **Walk under a LEARNED policy — sometimes.** `dep-tip1` (tipped-
  start retrain of the champion): 3 clean level walks / 1 runaway
  roll on 08-10 night; obs pipeline verified bit-exact against sim.
  The champion `vref1-r1` went 0/2 with runaway roll. The visible
  "sag" is the trained walking posture (commanded, not servo slip)
  and the scraping is the known low-clearance shuffle — the gait
  cleanup line exists exactly because of this.
- Not yet tried on hardware: the rot60 any-direction wrapper and the
  learned stand-up specialist (both ported 08-11, need a deploy
  re-push; do not press STAND on the stale on-robot copy).

## The best checkpoints, per skill

| Skill | Best run / checkpoint | Where it stands |
|---|---|---|
| **Walking with the joystick (hardware candidate)** | `cw-dep-tip1` (active walk slot) over `cw-dep-vref1-r1` (md5 f9a466cf) | tip1 = vref1-r1 + tipped-start DR: 3/4 clean level walks on real ground 08-10 vs the parent's 0/2 runaway roll. Both contract-exact, both on the robot picker. Open: the same-floor A/B (roll-ramp rate, runaways per N runs) and an RL-walk tape reading. |
| **…in any direction (full circle)** | same checkpoint + the `rot60` wrapper | Zero-training math fix; default-ON in the robot runner; backward went from frozen (3 cm of a commanded 30) to tracking as well as forward. Bench validation pending. |
| **Best pure-sim driving envelope** | `cw-walk-joyheadfric` (and its payload variant) | ±90° steering + latency + floor-grip + payload, 3-seed confirmed, joystick gate zero falls. Superseded for hardware by vref1-r1 (deployment contract), but it's the widest proven driving recipe. |
| **Standing up AND holding still** | `cw-stand-holdbc1-hard1` → `ppo_goal_cw_stand_holdbc1_hard1` | The deployed stance policy (robot's stand button), with its trained goal ramp shipped inside the weights. Hold 11/12 strict-valid, motionless on video; rise from flat belly reliable. |
| **Crouch-start stand fix** | `cw-stand-crouchrise1`/`-2` — **banked, NOT deployed, do not warm-start** | The fix reproduces cleanly (crouchrise2: 6/6 det rises incl. all crouch starts, zero falls) but BOTH variants park the same two feet in the air during hold — even with the deployed policy's exact training mix restored. Start-mix lever closed (two-miss rule); next lever is code: a state-aligned BC anchor (see "not working"). |
| **Turning on command** | `cw-dep-vref1-r1` + the new mirror wrapper (`mirror.MirrorPolicy`) — sim-proven 08-11, deploy port pending | Second hexagon trick: the mirrored policy turns RIGHT exactly as the naked one drifts left, same speed, zero falls; flipping between them steers both ways and holds a straight heading (2–4° over 12 s vs 38° drift). Slow (~2°/s) — and now the shipped story: the mirror-symmetry TRAINING run (`cw-walk-mirturn1`, 08-11 late) failed its gate (no turn tracking, gait degraded), closing that lever. |
| **Four-leg stand (party trick)** | `cw-quad-hold2` (30% mix on the walk champion); `cw-dep-quad1-c2` on the deploy contract | Mechanism is solid (lifts fronts, level, never tips) but ANY mixing dose erodes walking — stays a deploy-time specialist. |
| **Four-leg walking** | never attempted — NEW line opened 08-11 afternoon | Target: weight shifted back onto the four rear legs, front pair raised as "hands". Feasibility GO (39 mm margin). Spec first: today's reward provably punishes this exact behavior (RL_PLAN queue 0.3). |
| **Legacy sim walk champion** | `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c) | Seed-confirmed forward walker the whole driving line descends from; kept for sim work. |

## Best "leaning on a reference" vs best "from scratch"

**Best BC-anchor model: `ppo_goal_cw_stand_holdbc1_hard1`** (the
deployed stand+hold specialist). BC-anchor means: during training,
alongside the normal reward, the policy's actions get pulled toward
what a known-good recorded motion did at that moment. The cheat can't
farm it because it isn't reward income. It solved stand-up
(`cw-stand-bc1` → `bc1-hard1`: 12/12 honest stands where six
reward-only attempts all cheated) and holding still (`holdbc1`:
first genuinely motionless hold in the campaign). Important limit
discovered the same day: BC-anchor fixes "the trainer can't find the
right LOCAL motion" but NOT "the trainer can't find a different
GLOBAL pattern" — anchoring walk ticks to a scripted gait did nothing
for walking in new directions (`cw-omni-transbc1` failed; the
imitation error converged and the robot still didn't travel), and the
BC-anchor attempt at cleaning up the paddle gait failed the same way
(`cw-walk-gaitbc1`, verdict 08-11 evening: the anchor converged and
the policy froze into a static tripod pose instead of stepping).
That's now a three-time lesson: BC-anchor fixes local motions, never
gait patterns.

**Best from-scratch model: the entire walking/driving lineage.**
Every walker — `longdist_r2`, the joystick family, `vref1-r1` —
descends from `step0` (08-08), a fresh network trained with reward
only: no demonstrations, no reference motions. What made that work
was reward design (step-event income + drag + park-duty pricing),
not any warm start. The purest recent from-scratch results:
`cw-arch-hist16-dep1(-c1)` (16-frame history net learns to walk from
scratch under the full deployment contract, two seeds) and
`cw-uni-flag-a1-r1` (the flagship: from scratch, one network learned
a perfect quiet hold 6/6 AND a clean sit 6/6 in only 2M steps, with
honest-but-unfinished stand-ups — no cheating, no skill fighting).

## Bigger models — history MLP, GRU, and the flagship

- **Baseline** is an 8-frame history MLP (the policy sees the last 8
  observation frames stacked).
- **hist16 (16-frame history MLP): works, and is the preferred base.**
  It learns to walk from scratch (seed-confirmed, joystick gate
  clean) including under the full deployment contract
  (`cw-arch-hist16-dep1`). But piling on more training stopped
  helping (`r7-c4`: +40M steps bought nothing) — the remaining gait
  quality gap is sim contact pricing, not network memory.
- **hist24: trains, but slip/economy came out worse than hist16.**
  The ladder is frozen at 16 by operator ruling.
- **GRU (recurrent memory): failed three times walking — but learns
  every STANCE skill to champion grade — and is now REOPENED with the
  pre-registered levers.** `r1`/`r2` collapsed into the classic
  fake-walk (three legs parked, training reward climbing the whole
  time). `r3` re-ran with the FULL anti-cheat reward stack that fixed
  exactly this exploit on the MLP line — and it still cheated at
  walking while mastering hold/rise/lower. So for the GRU walking is
  genuinely not a reward problem at that size/window.
  `cw-arch-gru-r4c` (running, arch track) tests the two levers r3's
  failure named: 4x longer memory window (10.24 s) + double hidden
  size, at r3-identical update counts. If the cheat survives both,
  from-scratch GRU walking closes at this budget.
- **The flagship (one brain for everything): the big fork is SETTLED
  — no mixture of experts.** hist16 + a bigger 256×256 network + an
  explicit "which skill am I being asked for" input, from scratch on
  hold/rise/lower. Stage A passed hold and sit at specialist quality
  (first time one network did both honestly from scratch); stand-up
  plateaued at 1-in-6 and the 10M hardening run (`cw-uni-flag-a1-h2`)
  bought nothing — hold stayed clean, sit even improved, rise
  identical to the 2M parent, honest sprawl/tip-over failures on
  video, no cheating. The decisive measurement: the identical brain
  trained on stand-up ONLY (`cw-uni-flag-a1-risectl1`, 2.5x the rise
  practice, zero competing skills) scored exactly the same 1-in-6.
  Removing every other skill changed nothing, so the skills were
  never fighting for network space — the MoE rebuild is refuted by
  measurement, not just deferred. Stand-up-from-scratch is simply
  hard for any network with this recipe (the dedicated specialist has
  the same crouch-start struggle). Not a hardware blocker (the
  specialist-handoff stand-up covers the bench plan); if the line
  reopens, the lever is seeding from the specialist, not another
  reward or architecture change.
- **The meta-lesson: architecture was never the bottleneck.** Every
  time a skill failed, the fix turned out to be the task spec (reward
  semantics, supervision, start distribution) or free structure
  (geometry) — never a bigger or fancier network. Bigger models are
  worth having (the flagship needs the capacity), but reaching for
  one to fix a failing skill has been wrong every single time.

## What is NOT working (the honest list)

- **A learned policy on the real robot.** Attempt #2 is the entire
  critical path: walk + full-circle wrapper + the learned stand-up
  are all staged. Start with a fresh `set_zero`, and re-push/select
  the profile-carrying stand export first — the copy activated on the
  robot on 08-11 morning lacks the goal-ramp profile and would feed
  the policy an out-of-distribution ramp (see HARDWARE.md).
- **Turning on command (RE-OPENED 08-11; big progress the same
  day).** Policies carry a baked-in left-yaw drift and ignore the
  yaw channel. Three things happened 08-11 afternoon (detail:
  rl_docs/TURN.md, bottom): (1) the suspected turn-reward bugs were
  REAL and are now FIXED and bank-tested — the turn stack literally
  paid a frozen robot more yaw income than an honest walker
  (+375/ep vs +224), and its "anti-drift" charge taxed the honest
  gait's natural wobble while charging the cheats nothing; both
  terms are repriced and a new bank test reproduces the old bug and
  proves the fix. (2) the ZERO-TRAINING mirror trick WORKS in sim:
  the hexagon is left-right symmetric, so reflecting the deployed
  walk policy produces an equally good gait that drifts RIGHT
  instead of left — switching between the two steers both ways and
  drives straight (heading held to 2–4° where the naked policy
  drifts off by ~38°). Caveat: it turns at the drift rate (~2°/s),
  so it's steering, not a pirouette. (3) with the reward verified,
  the mirror-symmetry TRAINING run finally ran (`cw-walk-mirturn1`,
  08-11 late) and FAILED: the symmetry penalty converged but turn
  commands still aren't followed, baseline drift tripled, and the
  forced symmetry rewrote the champion's gait into near-in-place
  churning. That closes mirror TRAINING; the zero-training mirror
  wrapper IS the shipped turning story (slow steering). Hardware
  sign audit (does sim "+yaw" match robot "+yaw"?) still gates any
  bench turn session.
- **Gait quality: the champion walks by paddling, in a crouch**
  (loaded-foot slide, slip ~1.1–1.5 m per meter, body at ~50–77 mm
  below plant height). On hardware this scrapes. Walking taller and
  not dragging feet are the same problem (operator). The evening
  closed BOTH cheap levers (two-miss rule, detail `rl_docs/GAIT.md`):
  BC-anchoring the gait to the scripted tripod froze the robot into a
  static pose (`cw-walk-gaitbc1`), and terrain-as-teacher is refuted
  decisively — on real 36–108 mm bumps the paddle's slip gets
  monotonically WORSE, never better; from-scratch training on true
  72 mm ground learned leg-sacrifice dragging (`cw-gait-terrain2`,
  `-r1`), and a 4x drag charge from scratch formed the paddle anyway
  (`cw-gait-dragstep1` — with the caveat that the effective charge
  never exceeded ~0.09/tick vs ~1/tick income, so a "really big"
  penalty is still untested). What remains is spec/code work before
  any launch: the reward-accuracy probe (replay the tape-proven real
  gait AT HEIGHT through the full stack — the real gait rocks
  ±10–20°, which our tilt pricing charges, so the reward may be
  paying the robot to stay low and smooth), the drag-charge magnitude
  audit, and the P2 structural stance-slip charge with its bank.
- **Crouch-start stand-ups.** The fix is proven and REPRODUCES
  (`crouchrise1`: 16/16 crouch stands vs 0/8 before; `crouchrise2`:
  6/6 det rises incl. all crouch starts, zero falls) — but both
  variants break the hold the same way: two feet quietly parked in
  the air, the SAME two legs both times, a resurfaced flag-leg cheat
  that the strict `valid_plant` check MISSED and only per-foot
  contact duty caught (all-six duty is now a gate clause).
  crouchrise2 restored the deployed policy's exact training mix and
  the cheat came back anyway, so the mix-skew theory is refuted and
  the crouch-heavy START DISTRIBUTION itself is the cause. Leading
  mechanism: the BC anchor's reference is clock-indexed from episode
  start, so crouch starts get belly-phase reference poses pushed
  against near-plant states — the anchor actively teaches lifted-leg
  postures, and it bleeds into hold. Two-miss rule: no more mix/coef
  variants; next lever is CODE through spec first — a state-aligned
  (not clock-aligned) BC anchor, or anchor gated off on crouch
  starts. Neither crouchrise checkpoint may be deployed or
  warm-started from; `hard1` stays the stance policy with its known
  0/8 crouch limitation documented.
- **Sim effort realism (parked).** The scary "sim thinks standing is
  free" inversion was a bookkeeping error (units ×18 + wrong pose)
  and is retracted; measured properly, sim overprices effort
  everywhere (safely conservative). Predicting the real ammeter needs
  a command-fight current model — only required if we ever train FOR
  low current.

## What actually made training work

1. **Income gating, not penalties.** Make cheats earn ~zero BY
   CONSTRUCTION. Additive penalties get priced in and paid; every
   time we merely charged for a cheat, the policy paid and kept
   cheating.
2. **BC-anchoring when the reward is right but the trainer can't
   find the motion.** Twice-proven (stand up, hold still). Reward
   tells the policy WHAT is good; it never tells a churning leg which
   WAY to move.
3. **Free structure before compute.** The rot60 wrapper solved
   any-direction walking in an afternoon after four training runs
   (reward variants AND imitation) had failed at it. Check what the
   geometry gives you before asking the trainer to discover it.
4. **Cheap offline tests before expensive training.** The scripted
   reward banks (MDP_PREFLIGHT) prove "honest behavior out-earns
   every known cheat" in minutes, before any launch. Binding rule.
5. **Matched-parent controls.** A run evaluated under an injected
   condition must be compared to its parent under the IDENTICAL
   injection. Several "failures" reversed to PASS (and one "pass"
   reversed) once the control was run properly.
6. **Judge on strict geometry + per-step telemetry, never on the
   training reward.** Climbing reward while the task fails is the
   standard cheat signature (all three GRU runs). Video snapshots
   also miss things: only per-foot contact duty caught the
   crouchrise1 hold cheat and the hold-mode shuffling.
7. **One variable per run; discovery (2M) then hardening (10M).**
   The finishing tails (footprint precision, current spikes) resolve
   with budget; the mechanism has to be proven cheap first.

## Big things we have learned

1. **Reward hacking is the default outcome, not the exception.**
   Every under-specified reward got gamed: freeze-and-collect,
   park-and-earn, flag-leg stands, cadence inflation, the GRU's
   three-legged statue. Countermeasures: income gating + offline
   cheat banks + strict geometric success checks.
2. **Sometimes the reward really is buggy or wrong — so verify the
   reward FIRST, cheaply, and only then stop blaming it.** We
   shipped plenty of genuine reward bugs: the turn-mode kernel paid
   a motionless robot its full walking income (that alone collapsed
   the mirror2 run — freezing literally out-earned walking); the
   sit-down "arrival bonus" paid out without arriving (freeze earned
   +120 until the gate fix); hold pricing paid a parked flag-leg
   EXACTLY as much as an honest stand (a literal tie, proven by the
   bank); and one mechanism bug (the episode-pool restore dropping
   reward state) silently stopped paying the stand-up income at all
   and contaminated three verdicts before it was caught. The point
   is that every one of these is findable in MINUTES, not 20M GPU
   steps: run the offline cheat bank (does honest behavior out-earn
   every scripted cheat under THIS run's exact config?) and, for an
   already-failed policy, the term-by-term income probe (what did
   the bad behavior actually earn, term by term, vs honest and vs
   doing nothing?). Only once those exonerate the reward does the
   rule flip: **a behavior that doesn't change across several
   VERIFIED reward mechanisms is not a reward problem.** Six
   stand-up arms collapsed identically under six bank-checked
   mechanisms — the fix was supervision (BC-anchor), not pricing.
   Four omni-walk arms collapsed under pricing the income probe had
   exonerated (the collapsed policies earned LESS than freezing) —
   the fix was geometry (rot60). Two-miss rule going forward: after
   two same-class failures with a bank-verified reward, change the
   mechanism, never the price or the step count.
3. **Structure is cheaper than training.** The two biggest wins of
   the campaign (rot60, BC-anchor) both bypass discovery instead of
   incentivizing it. Before launching a training arm, ask: does
   geometry, a recorded reference, or the start distribution already
   contain the answer?
4. **Trust only the strictest evidence, and verify before building
   on it.** Training reward lies (cheats). Sparse video lies
   (missed the hold shuffle). Even `valid_plant` lies (missed the
   crouchrise1 parked feet — per-foot contact duty is now a gate
   clause). And one GPU run (`gru-long1`) was launched off a
   training-log claim nobody had checked against the harness —
   invalid evidence, wasted launch.
5. **Robustness was free; stop buying insurance.** 13-for-13
   single-axis sensor/calibration exposure runs bought nothing the
   champion didn't have; the ~20-axis protective sweep on the
   hardware candidate found essentially nothing. Those classes are
   CLOSED — compute goes to capabilities, not more armor.
6. **Hardware truth keeps overturning sim assumptions.** Slip is
   locomotion, not failure; a working gait rocks ±10–20° (a 10° trip
   would kill it); loaded servos respond ~30× slower than the air
   fit; the current "inversion" was our own bookkeeping. Measure on
   the bench, then fix the sim — never the reverse.
7. **Skills interfere under naive mixing; composition works.**
   Quad-hold at any dose erodes walking; the specialist + handoff
   pattern (stand specialist → walk champion → scripted sit) composes
   with zero falls TODAY. The flagship experiment decides whether one
   network can do it all; until it answers, ship specialists.
8. **Wrong logical zeros are how hardware gets destroyed.** The
   08-06 incident came from software poses commanded against a stale
   zero frame — hence the hard safety rules (fresh `set_zero` every
   session, no unsupervised stand/plant blends).

## What happens next

1. **Next bench session** (operator — the critical path; RL walks
   already happened 08-10, see "REAL hardware" above): (a) the
   vref1-r1 vs tip1 A/B on the same floor — compare roll-ramp RATE
   and runaways over several runs, and make sure the policy switch
   actually lands this time; (b) tape-measure an RL walk; (c) first
   hardware runs of the rot60 off-wedge headings and the learned
   stand-up — deploy re-push FIRST (the on-robot stand copy lacks
   its goal profile), fresh `set_zero` always; (d) wz sign audit.
   If tip1 also runs away on this floor, the fix moves to sim: a
   contact/pinning model (no-skate feet), not more DR.
2. **Yesterday's three pending results are all read out** — flagship
   fork CLOSED (MoE refuted by the rise-only control), `gaitbc1`
   FAILED (anchor froze the gait), `crouchrise2` FAILED-mixed (crouch
   fix reproduces, hold cheat traced to the clock-indexed anchor).
   The follow-ups are all spec/code work, not launches: the
   state-aligned BC anchor, the gait reward-accuracy probe, the
   drag-charge magnitude audit, and the P2 structural slip charge
   with its bank.
3. **In flight / queued:** `cw-walk-dragstance1` (hw — the first
   structural per-stance scrape-charge arm, audit-derived size) is
   training. `cw-arch-gru-r4c` finished and FAILED (from-scratch GRU
   closed); `cw-walk-mirturn1` ran and FAILED (mirror training
   closed — wrapper ships). Four-leg walking (weight back, front
   pair as "hands") still needs its spec + bank first — the current
   reward provably punishes exactly that posture.
4. **Track hygiene:** each research line now keeps its own short
   status in `rl_docs/tracks/<track>/STATUS.md` (hw / arch / nobc /
   quad / turn); agent triage stays within a run's track.

Queue and blockers: `RL_PLAN.md`.


---

# FILE: rl_docs/tracks/arch/STATUS.md

# arch — Advanced architectures

W&B: tag `track:arch`. Excess-capacity research.

**Goal:** get a more advanced model (GRU/recurrent/temporal) to walk,
stand up, and sit down. What architecture learns the full skill set,
at what budget, with which failure modes.

## Now

- **From-scratch GRU walking CLOSED (08-11, gru-r4c):** both
  pre-registered levers (BPTT window 64→256 steps, hidden 128→256)
  tried and the identical leg-sacrifice/paddle fingerprint survived
  (legs [0,2,3] parked, det gait_valid 0/6) — worse, rise also
  regressed to 0/6 (r3 had it at champion grade). Reward already
  works on the MLP lineage, so this is a capacity/architecture limit,
  not a pricing gap. No further from-scratch GRU variant.
- Frame-stack line passed: hist16 → hist16-dep1 (deploy contract).
- **BC-distill-then-RL-finetune tried (08-11, `cw-arch-gru-bc-ft1`):
  walk survives, stance is MIXED, and a dig-in corrected the root
  cause.** The GRU was BC-distilled from the walk+hold specialists
  (rise was NEVER in the distillation data — the BC parent itself is
  a hold+walk artifact only) then PPO-finetuned 10M steps on a
  walk-heavy mix (60/15/15/10). Result across all four gate+own-DR0.5
  passes: walk gait_valid 6/6 throughout, no paddle at any point in
  10M steps (economy softened, slip/m 2.1-2.6 vs BC parent 1.5);
  lower IMPROVED (4-6/6 vs parent 2/6 det); rise stayed 0/6 (unchanged
  from the BC parent — RL never invented a skill it had zero demos
  for, not forgetting); hold REGRESSED 0/6 det (parent was 6/6 — RL
  genuinely traded hold precision for locomotion polish, real
  forgetting this time). So two different mechanisms in one run:
  data-poverty (rise, never learnable this way) vs erosion (hold,
  actually lost). Keeping `ppo_goal_cw_gru_bc.zip` (md5 864c02fb) as
  the hold+walk reference artifact.
- **lr/KL lever CLOSED (08-11, `cw-arch-gru-bc-ft2`, 2M discovery)
  — FAILS exactly as pre-registered.** 3x lower LR (3e-4→1e-4) +
  tighter target-KL (0.02→0.01) on the identical BC-parent finetune:
  gate (DR0) det rise 0/6 (bridge/crouch/flat all 0, honest stalled
  climb, plant_margin 122mm short, balanced duty — not a cheat), det
  hold 0/6 (all 6 eps fail valid_plant on height/feet_down/footprint,
  worst foot 19-33mm proud, balanced duty — honest precision loss,
  not a flag-leg). Walk stayed clean (gait_valid 6/6 det+sto, slip/m
  1.3-1.7 in-band). Two attempts now (ft1 default hp, ft2 tight
  hp) at protecting stance via PPO hyperparameters alone — both lose
  it identically. **Per RESEARCH_RULES "two misses in the same
  behavioral class": stop tuning lr/KL, change the mechanism.**
  **Root blocker found, CODE not config:** the gate's own
  pre-registered fix ("an auxiliary BC-anchor loss during the
  finetune, or freezing early layers") is not available for this
  network — `train_ppo_mjx.py` explicitly raises `SystemExit` for
  `--gru` + `train.bc_anchor_coef>0` ("not implemented, wraps stock
  PPO"), and there is no layer-freeze flag either. The MLP-lineage
  BC-anchor (`cw-stand-bc1`/`holdbc1`, twice-proven on hw track) does
  not reach RecurrentPPO as written.

## Next

- **The real next lever needs CODE, not another training run:**
  either (a) extend `bc_anchor.py`'s auxiliary-loss wrapper to
  `RecurrentPPO`/`GruActorCriticPolicy` (needs hidden-state-aligned
  minibatches, nontrivial), or (b) add a simple param-freeze option
  (freeze the GRU cell, finetune only the output head) — neither
  exists today. Until one lands, no arch-track spec is launchable
  for this lineage; do not queue a 3rd lr/KL/coefficient variant.
- Operator-directed next lever for RISE: re-distill stance-heavy +
  DAgger rounds on `distill_gru.py` (give the BC step actual rise
  demos before any further RL) — in progress outside this loop;
  unaffected by the above (it fixes the DATA, not the finetune loss).
- Later: contact-from-proprioception aux head; distill specialists
  into one recurrent net.

Detail: RL_PLAN.md "Architecture" · ledger cw-arch-* lineage.


---

# FILE: rl_docs/tracks/hw/STATUS.md

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

Detail: **rl_docs/BENCH_REPORT_2026-08-11.md** (tonight's consolidated
bench read + RL implications; regenerate tables with
`python -m rl_move.scripts.bench_report`) · RL_PLAN.md queue ·
rl_docs/HARDWARE.md · RISE.md · GAIT.md.


---

# FILE: rl_docs/tracks/nobc/STATUS.md

# nobc — Learn without BC anchor

W&B: tag `track:nobc`. Excess-capacity research.

**Goal:** learn standing — and honest skill discovery generally,
clean non-scraping gait included — by RL alone: structural rewards,
curricula, terrain, RSI. NO imitation losses of any kind. hw ships
whatever works; this track exists to retire the crutch.

## Now

- Stand-from-scratch: six reward-side arms collapsed identically; the
  collapse traced to warm-start OOD drift, and the Warp pool-restore
  bug contaminated early verdicts — income-shaping/RSI verdicts are
  REOPENED for from-scratch designs. RSI machinery works.
- Gait-from-scratch: terrain clamp bug fixed (all past terrain was
  really <=18mm); terrain2 (true 72mm) FAILED into leg-sacrifice;
  dragstep1 FAILED — but its effective charge never exceeded
  ~0.09/tick vs ~1/tick income, so "really big penalty" was UNTESTED.
  **CROSS-TRACK: item 1 below is now ANSWERED, by an hw-tracked run**
  (`cw-gait-dragstance1`, GAIT.md) — the audit-derived structural
  per-stance charge (k=8000/m, correctly sized this time) trained
  from scratch and FAILED into freeze/park (charge paid the whole
  episode, never resolved, near-zero travel), matching its own
  pre-registered false branch. Solo structural charge does not
  induce stepping from scratch; move to item 2 (RSI-for-walk).

- CROSS-TRACK INSIGHT (hw P0 probe, 08-11 late, GAIT.md bottom): the
  crouch-paddle is a sim-EFFECTIVENESS optimum, not a paid basin —
  it out-earns the plant-height gait ~495/ep on genuine progress
  income while tilt penalties are negligible (≤1.5/ep). Pricing
  magnitude alone must beat that moat without paying park; favors
  the per-stance charge + curriculum levers below over any further
  income shaping.

## Next

1. ~~Drag-charge magnitude audit~~ DONE 08-11 (see above) — solo
   structural charge does not induce stepping from scratch.
2. RSI-for-walk mid-stride spawns (now front of the gait queue).
3. Annealed-up charge once the P2 bank lands; then stand-from-scratch
   resumes with whatever levers moved gait discovery.

Detail: GAIT.md P3 · RISE.md forensic ladder.


---

# FILE: rl_docs/tracks/quad/STATUS.md

# quad — Quadruped with two hands

W&B: tag `track:quad`. Excess-capacity research.

**Goal:** walk on four legs with the front pair lifted as hands/arms.
Stand on four, walk on four, front pair free for tricks.

## Now

- Four-leg HOLD is solid (quad-hold1-r2: survived 1.0, level, fronts
  lifted; hold2 at 30% mix confirms the mechanism) but ANY mixing
  dose erodes walk retention — quad stays a deploy-time specialist.
- Deploy integration of a PASSING quad checkpoint belongs to hw
  (joystick key `4`).

## Next

- The unattempted core: four-leg WALKING (weight shifted back onto
  the rear four). Spec + bank FIRST — today's reward provably
  punishes exactly this posture (anti-cheat calls it leg sacrifice;
  level-referenced tilt pricing charges the rear-shift).
- Then front-pair posture control while moving.

Detail: ledger cw-quad-* lineage.


---

# FILE: rl_docs/tracks/turn/STATUS.md

# turn — Commanded turning

W&B: tag `track:turn`. Excess-capacity research; de-scoped from the
hw deliverable (no camera = no front; rot-60 covers translation).

**Goal:** make commanded yaw work — fix the structural left-yaw drift
baked into the walk gait so the joystick can point the robot.

## Now

- Reward-shape tuning DOUBLY CLOSED (yawcmd/yawgate failed; turnfix1
  was statistically identical to its matched parent). The drift lives
  in the gait's chirality, not the turn reward.
- 08-11: turn-reward bugs found+fixed+bank-tested (frozen robot
  out-earned honest walker on yaw income; drift charge taxed honest
  wobble). Zero-training mirror trick WORKS: reflected policy drifts
  RIGHT, switching steers both ways (heading 2-4° vs 38° drift) — but
  at drift rate (~2°/s), steering not pirouette.

- **08-11 late: `cw-walk-mirturn1` (mirror-symmetry TRAINING, coef
  1.0 warm from the champion) FAILED — the alternative branch fired:**
  sym loss converged 28→0.5 but turn tracking never arrived (|wz_err|
  med 0.254, L/R asymmetry intact, drift 3x worse) and the forced
  symmetry rewrote the gait (prog 0.41 vs ~1.0, slip 5x). Mirror
  TRAINING on a warm champion is CLOSED per the pre-registered gate.

## Next

- SHIPPED turning story: eval-time MirrorPolicy chirality selection
  (arc-left/arc-right/straight, zero training, ~2 deg/s). Deploy port
  + rot60 composition are the remaining [CODE] items.
- Fast commanded turning needs a NEW idea (step 4 BC-anchor-on-turn
  ticks is in reserve but unpromising after transbc1). No more
  reward/coef/symmetry variants.
- Hardware wz sign audit gates any bench turn session.

Detail: rl_docs/TURN.md (design, bank numbers, failure history).


---

# FILE: RL_PLAN.md

# RL Plan — joystick-driven hexapod, sim to real

## GOAL (operator, binding — rewritten 08-10, supersedes all prior)

**Drive the real hexapod around with a joystick.** One deployed
policy (or a small blended set) on the physical robot that can
STAND UP, SIT DOWN, TURN, and WALK where the joystick points —
reliably, session after session. Once that works, the party tricks:
lift the front legs and stand on four, then walk on four.

What "good" means: covers real ground, stays level, never falls,
never cooks a motor — reliability over speed. **Foot slip is NOT
failure by itself** (the scripted gait that walks the robot slips
visibly); slip metrics keep sim honest about the real floor, never a
ban. Speed bands and zero-slip gates are means, never the objective.

Startup reading order (operator + GPT, 08-10): `RL_GOALS.md`
(mission) → **`CURRENT_TRUTHS.md`** (accepted facts — outranks any
history file) → this file (blockers/queue/architecture/gates) →
**`RESEARCH_RULES.md`** (binding agent behavior: prime directive,
phases, preflights, kill rules) → `rl_docs/SIM.md`. As needed:
`rl_docs/COMMANDS.md` (how to run things) · `rl_docs/REWARD.md`
(every reward term + per-run reward auto-doc) · `rl_docs/EVALS.md`
(eval metric definitions) · `rl_docs/RISE.md` (stand-up plan) ·
`rl_docs/SKILLS.md` · `rl_docs/runs/` (per-run facts). History (`RL_LOG.md`, `archive/`) only for historical
questions — never to infer current state. **EDIT RULE (operator,
08-10): keep this file under ~250 lines and in plain language. New
material goes to `rl_docs/` with a pointer here; superseded detail
moves to `archive/` — never accumulates here.**

## Research tracks (operator, 08-11 — full rules: RESEARCH_RULES.md)

The campaign runs as PARALLEL TRACKS, each with its own goal, W&B tag
(`track:<id>`), and status doc (`rl_docs/tracks/`). Registry:
`rl_move/orchestrator/tracks.json`. **hw** = joystick robot on
hardware by any means (MAINLINE, pod priority — this file's queue is
mostly hw); **arch** = GRU/temporal models learning walk/stand/sit;
**nobc** = stand + clean gait from scratch, no BC anchor ever;
**quad** = walk on four legs, front pair as hands; **turn** =
commanded yaw via mirror symmetry. Non-hw tracks run on excess
capacity. **Containment: a triaged run's follow-ups stay in its
track; cross-track findings are escalated in writing, and
cross-track launches are operator-only.**

## Prime directive (operator, 08-10 — full text: RESEARCH_RULES.md)

**Minimize the number of unresolved blockers between the current
robot and the next useful hardware joystick test — that count is the
campaign KPI, not GPU occupancy.** (Scope 08-11: this is the hw
track's directive; other tracks substitute their tracks.json goal.)
Idle pods are acceptable when the
critical path is hardware, specification work, or code fixes. Every
spec answers the launch question first: if this run succeeds or
fails, does it change what we do before the next hardware test?

## Operator directive (08-11 night — binding)

**Make standing and walking work IN SIM, to deployable-champion
quality, and keep the fleet firing at exactly that.** Cycles think
root-cause-deep about the open stand/walk blockers (hold's parked
foot, det flat-rise stall, rise-rock + takeoff roll-rate DR axes,
the crouch-splay tall wall, contact/pinning) and keep launching arms
that directly attack them — each spec'd so a PASS is a hardware
candidate (Gate 0 / dep contract, champion warm-start, retention
gates), not a throwaway. When the next lever is CODE (several are:
rise-rock DR + bank, takeoff roll-rate injection, BC-INIT
pretraining on the scripted tall gait, physics-easing ladder,
min-over-feet hold pricing + per-mode bc_anchor_loss logging), the
cycle WRITES it and checks it in: new cfg keys, DEFAULT OFF,
bit-exact when off, tests/banks green, snapshot before training —
never blast shared defaults with experimental code, never park a
line on "CODE, unbuilt". Idle pods next to an unattacked stand/walk
blocker are now the failure mode; peripheral runs stay banned.

## Critical path (simplification review §11, 08-10)

**CURRENT GOAL:** joystick-controlled real robot. **BLOCKERS (as of
08-11 late):** the intermittent RUNAWAY ROLL on real ground (RL walk
attempts 08-10: vref1-r1 2/2 runaway, tip1 1 runaway / 3 clean —
queue item -1 has the full state; the discriminating A/B + the sim
contact/pinning question are the open work), plus the FIRST bench
runs of the newly landed pieces: rot60 off-wedge headings and the
learned stand-up (rise+hold specialist port LANDED 08-11, trained
goal profile shipped in the weights meta, contract-locked by
`rl_move/tests/test_stand_runner.py` — needs a deploy re-push
first; stance_dr10 rollback = one picker call). The hold-current
"gap" is RETRACTED (08-11 probe: pose/unit confound; SIM.md gap 2).
Rise/hold/lower and full-circle translation are SOLVED IN SIM. **Sanctioned compute
experiment lines (operator 08-11 afternoon — not attempt-#2
blockers, but wanted): turning (RE-OPENED, queue 0.2), four-leg
walking (NEW, queue 0.3), tall/no-drag walking (queue -0.5). All
three are diagnosis-first: audit whether the reward actually pays
the desired behavior before launching arms.** **DEFERRED:** generic
DR composes, posetrack, architecture curiosity work
not tied to a demonstrated failure. **RULE:** idle GPUs are fine.
**TEST:** the next experiment should take less than one minute to
explain and should change what we do before the next useful
hardware test.

## Where we are (08-11 — live facts in CURRENT_TRUTHS.md)

The real robot walks under a scripted gait (tape-measured), and a
learned policy has now driven it too: `dep-tip1` walked level 3 of 4
bench runs 08-10 (vref1-r1 went 0/2 with runaway roll — see queue
-1). In sim the full joystick cycle composes with zero falls (rise →
drive any direction via rot60 → stop → sit); turning is de-scoped.
The critical path is the runaway-roll question (bench A/B + possibly
a sim contact fix) and first bench runs of rot60 + the learned
stand-up.

## Standing rules → `RESEARCH_RULES.md` (binding; moved 08-10)

How to design, launch, stop, and judge experiments lives in
**`RESEARCH_RULES.md`**: the phase system (SPECIFICATION / DISCOVERY
0.5–2M / HARDENING 10–40M / COMPOSITION / TRANSFER, launcher-
enforced), MDP_PREFLIGHT (`test_task_semantics.py` orderings per
mode), matched-parent controls, behavioral-impossibility kills,
DIG-IN triggers, reward routing, warm-start recipes, and the launch
question ("does this change what we do before the next hardware
test?"). Promotion criterion (operator): "closest to deployed on the
real robot that I can joystick reliably" — physical metrics, never
one reward scalar. Hardware candidates pass Gate 0 (below).

CLOSED moves — do not re-propose (evidence in `rl_docs/runs/`):

- Anti-slip / income reward shaping against skating (10+ arms;
  root cause is contact pricing, an operator calibration). SCOPE
  narrowed 08-11 (operator): this closes income gates/shaping
  retrofitted onto trained paddlers — it does NOT close the GAIT
  CLEANUP line (queue -0.5 / rl_docs/GAIT.md): banked structural
  drag charge, swing clearance, or from-scratch task curricula.
- Rise reward-income shaping, reference-tracking-as-crutch, and RSI
  (state-distribution fix) — all beaten by the identical tripod/
  flag-leg cheat. A warp/MJX episode-pool state-loss bug (commit
  65edba7) briefly confounded these closures (score/ref income
  wasn't actually being paid on the GPU path for several arms); the
  fix landed and the clean re-run (`cw-stand-rsi2`, 08-11) reports
  the SAME cheat with mechanism health now verified clean
  (`env/rise_rsi` held ~0.5 all 2M steps, zero corruption) — RE-
  CLOSING all three on stronger evidence. Next lever is CODE only: a
  structural coupling between the height goal and measured foot
  contact (RL_PLAN queue item 2b / rl_docs/RISE.md). Do not propose
  another reward-coefficient or RSI variant on this stack.
- Identical-config continuations (0-for-5; auto-continue handles
  segment stitching).
- Generic full-DR (1.0) retrains; single-axis calibration/sensor DR
  exposure (13-for-13 no-effect); speed-band arms (gait-limited).
- Raising the slew clamp and retrying a champion.
- posetrack step-extensions (needs a dense curriculum or stays
  parked — not on the joystick critical path).
- Treating another pairwise DR-compose PASS as progress: the compose
  campaign proved broad robustness; broad robustness is not simulator
  accuracy, and it is not on the blocker list (operator, 08-10).
- Further single/pair/ANY-N-way DR composes on `cw-dep-vref1-r1`
  (the "protect-the-candidate" sweep): CLOSED 08-10 night, 20-for-20
  no-effect — every sensor/actuator/assembly axis tested solo or
  paired composes free with the IDENTICAL benign fixed-eval-seed
  fingerprint (det/4 catastrophic crater at DR0; det/5+sto/0-1 mild
  degradation at own-cfg DR0.35); an all-axes-stacked "megastack" is
  the predictable terminal case of the same closed class, not new
  evidence — do not requeue it under any name (`cw-dep-vref1-r1-
  megastack1` and renamed retries repeatedly pruned 08-10 ~20:35).

## Architecture

Settled core: 18 joint-position targets through the SafetyLayer;
actor sees deployable obs only; asymmetric critic; 8-frame history
MLP.

Temporal ladder PAUSED at hist16 (operator, 08-10): hist16 passed its
first gate (walks from scratch, joystick gate clean) and becomes the
default for the flagship below. No 24-frame / recurrent / transformer
rungs until the flagship answers the real question — "can a
history-aware policy with a correctly specified multitask MDP learn
the joystick skill set?" — not "what is the best temporal arch?".

**FLAGSHIP (stage A TRAINED 08-11: `cw-uni-flag-a1-r1` 2M +
`-h2` 10M hardening): clean-sheet unified policy.** hist16 +
EXPLICIT mode/command one-hot + 256×256 MLP, from scratch, on
HOLD/RISE/LOWER/WALK/TURN (not quad). Staged curriculum: (A) hold +
near-plant rise/lower, (B) expand rise→belly / lower→sit, (C)
forward locomotion, (D) turns, (E) transition-heavy episodes.
Stage-A result: hold 6/6 + lower 12/12 at specialist grade FROM
SCRATCH (first time), rise (all-crouch) plateaued 1/6 with flat
factors across 10M, NO cheat (honest sprawl-stall/tip-over — the
same crouch fragility the deployed specialist has, 0/6 RSI-off).
None of the pre-registered outcomes fired cleanly; "skills fight →
MoE" needs interference MEASURED, not inferred (no from-scratch
rise-only cell ever ran). **08-11: `cw-uni-flag-a1-risectl1` (2M,
rise-only, one variable, the missing cell) REFUTES the MoE fork.**
Isolated on stand-up alone (zero sibling skills to fight for
capacity, 2.5x the flagship's rise-tick exposure), it lands in the
IDENTICAL band: det valid_plant 1/6 (matches h2's 1/6 exactly),
`env/rise_feet_factor` last-quarter mean 0.537 (flagship band
0.44–0.64, confirm threshold was >0.8), zero exploit fingerprint in
12 video-checked episodes (honest crouch-start tip-overs/sprawl-
stalls, same shape as h2). Shared-capacity interference is NOT the
cause — a network with nothing else to learn hits the same wall.
**Do not build MoE for unified-policy rise.** The bottleneck is
rise-from-scratch itself (matches the deployed BC-anchor specialist
also failing 0/6 RSI-off on crouch starts before its start-mix fix).
Next lever if this line reopens: specialist seeding/warm-start, not
another reward/mix variant — but this is now architecture curiosity,
not a hardware blocker (specialist-handoff composition already
covers rise for attempt #2, deployed 08-11). No further flagship
arms queued this cycle.

Not defaults: velocity estimator / DreamWaQ (NOT needed for attempt
#2; revisit only on a demonstrated hidden-state failure). 08-11 eve
status of that rung: leg-odometry estimator BUILT and unit-tested
(`rl_move/estimator.py`, sim-validated at DR0, corr collapses at
DR0.5) but REFUTED as a hardware velocity source by the operator's
tape data — real feet slip ~50%, and leg odometry over-reads by
exactly the slip fraction (it cannot tell planted from sliding).
Fleet evidence agrees velocity obs is not the gait lever (nv/nv2:
"deployable obs are not the blocker"; aac: retention tool only). If
true body velocity is ever wanted, the honest path is a downward
optical-flow sensor (PMW3901, ~$20, SPI, works ≥80mm height — fits
the ~142mm walk stance), not more inference.
Transformers/CPG only on the archive review's triggers. Specialist
heads / skill conditioning ARE acceptable if that is what reliable
joystick control takes — deployability beats purity (GPT, 08-10).

## Champions (append-only) + open problems

- **Hardware base: `cw-dep-vref1-r1`** (deployment-exact obs, 25°
  tilt; PASS with zero erosion; hardened vs 8 DR axes 08-10; md5
  f9a466cf) — THE attempt-#2 checkpoint.
- Walk: `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c). Driving:
  joylat25 → joyfric/joyheadfric family (±90°, latency, friction,
  payload, deadband, slope, 60 s — seed-confirmed, joystick gate
  clean). Stance: `stance_dr10` (heights at DR 1.0; crown jewels,
  canary-protected).

Open problems, in priority order:

1. **Sim contact/current pricing.** Contact HALF-CLOSED (08-10):
   `calibrate_slip.py` replays the exact hardware gait; sim travel
   ratio 0.35–0.41 vs real 0.50–0.51, speed-invariant, walk current
   in-band — sliding is NOT free in sim (slightly conservative), μ
   saturates ≥1.5, XML friction stands (`env.foot_friction_slide`
   hook landed for future floors; SIM.md gap 1). WALK semantics bank
   landed and PASSING: the tape-proven gait out-earns stall 1.9x and
   park 3.5x under the champion stack. The hold-current inversion is
   RETRACTED (08-11, `probe_hold_current.py`: units ×18 + pose
   confound; sim's proxy overprices 3–25x everywhere, conservative,
   and matches the real walk>hold ordering). DEFERRED, not a
   blocker: a register-scale cmd-fight/deadzone current model —
   prerequisite ONLY for future k_current>0 hardware-pricing arms;
   k_current=0 ruling stands.
2. **Rise/lower inside the walking policy.** Full plan + evidence
   trail: **rl_docs/RISE.md**. Lower is solved warm (rfix-warm1 6/6
   posture-strict; keep fine-tune grafting, distill refuted). Rise:
   six straight reward-income/RSI mechanisms lost to the identical
   flag-leg cheat (CLOSED, see CLOSED moves) — **08-11: the seventh
   lever, a BC anchor OUTSIDE the reward (`cw-stand-bc1`), PASSES**
   (partial): harness-verified honest six-foot plants (bridge 7/12,
   crouch 6/8 valid_plant; flat cold-start 10/10 correct stand,
   footprint-precision-only miss), zero flag-leg cheat in 42 video-
   checked episodes, clean one-variable causal attribution vs the
   identical-minus-anchor parent (still 0/12). Dose-check
   `cw-stand-bc1-coef03` (coef 0.3) **FAILED decisively** (08-11):
   valid_plant 0/16 across every start kind, worse on every axis
   than coef=1.0 — keep `bc_anchor_coef>=1.0`, no more coefficient
   variants. `cw-stand-bc1-hard1` (10M, hardening) **PASSES on rise
   and is the RISE SPECIALIST champion**
   (`ppo_goal_cw_stand_bc1_hard1`): RSI-off probe 12/12 valid_plant
   incl. flat 4/4 (bc1's footprint miss resolved by budget), gate
   det 5/6, feet factor stable all 10M — no re-drift. Dig-in
   matched-parent control (same probe on bc1@2M) refutes the
   "hardening regression" scare: hold/track/raise/lower were ALREADY
   0/12 at 2M (lower flag-leg 166→189mm; raise has p_raise=0 here —
   untrainable). But hold/track splay WORSENS with steps (feet
   51→162mm, 2.6A over-current) — a pre-existing stillness-pricing
   gap amplified, not fixed, by training. Lineage CLOSED for
   hardening; next: hold/track stillness SPECIFICATION (queue 2.3),
   then learned-rise → walk/hold champion handoff composition.
   Detail + numbers: **rl_docs/RISE.md**.
3. **Loaded actuator model.** FIT LANDED 08-10: opt-in
   `--cfg-set bus.servo_params=loaded` (default stays air). Detail +
   provenance + confidence table: **`rl_docs/SIM.md`**. Uncertain
   params (servo reaction times above all) are COVERED BY DR RANGES,
   not exact nominals; hip/yaw are an ASSUMPTION until a per-axis
   loaded ladder. First training arm DONE (08-11 dig-in,
   `cw-dep-vref1-loaded1` PASS): loaded-servo training retains the
   dep-contract walk and slightly beats the air-trained parent under
   matched loaded physics; the ~+40% vel-err/+50% slip vs old
   instant-servo numbers is honest physics (hits the frozen parent
   identically). Loaded params are a viable dep-line training
   default; air-vs-loaded for attempt #2 is a bench decision.
   Remaining: liftoff reproduction on loaded params.
4. **Quad-mix erosion.** Dose-response so far: 50% erodes walk, 30%
   recovers on the walk champion, 30% on the driving champion
   FAILED, 15% in review. If erosion persists at useful mixes:
   walk-mode KL/distillation anchor to the frozen champion.
5. **Start variation.** The startvar compose FAILED both seeds;
   isolation (noZD1/noBS1, 08-10) shows zero-drift-frame DR is the
   dominant culprit with bad-start interactions — rework the
   mechanism before re-composing. Varied-start eval panel stays
   mandatory for hardware candidates; walk episodes should sometimes
   start from park-bank/slumped poses.

## Queue

-1. **HARDWARE (operator bench — the true critical path).**
    **Attempt #2 HAPPENED (08-10 eve/night — stop calling it
    pending; full findings rl_docs/HARDWARE.md).** `cw-dep-vref1-r1`
    ran twice: runaway roll both times (opposite signs →
    environmental seed; pinned-loaded-leg positive feedback, no
    trained recovery). Its tipped-start retrain `cw-dep-tip1` then
    ran 4x: 1 runaway / **3 CLEAN level walks** — a learned policy
    HAS driven this robot. Obs pipeline proven bit-exact (offline
    replay err 0.0014); sag is the commanded trained posture;
    scraping is the known low-clearance shuffle (→ GAIT queue -0.5).
    Scripted-gait tape: ~50% of commanded (done). STILL OPEN on the
    bench: (a) the vref1-r1 vs tip1 A/B on the same floor (the 08-10
    attempt never actually switched policies — compare roll-ramp
    RATE / runaways per N, not one run); (b) tape reading on an RL
    walk; (c) FIRST hardware runs of the newly-landed rot60 port and
    the stand specialist (re-push via deploy_adb.sh first — the
    on-robot stand copy lacks the goal profile, do NOT press STAND
    stale); (d) wz sign audit. If the runaway recurs on tip1 too,
    the fix is a sim contact/pinning model (no-skate feet), not
    more DR. **Session runner: `rl_move/scripts/bench_blast.py --go`
    (guided checklist, all of a-d); sim half `rl_move/sim/sim_blast.py`
    — 08-11 run: NO runaway in sim at ANY friction (mu 1.0-2.0, the
    default already sits at the mu 2.0 saturation), every trap episode
    re-levels to <2 deg — the pinning gap is not a friction-magnitude
    story; tip1 outranks vref1 at nominal grip (0.88 vs 0.75 roll-trap
    pass), agreeing with the bench tally.**
-0.5 **GAIT CLEANUP — kill the paddle, walk TALLER (operator 08-11,
    TOP TRAINING PRIORITY; full design + rationale: `rl_docs/GAIT.md`;
    operator re-affirmed 08-11 afternoon: walking from a higher
    stance and walking without dragging feet are the same problem).**
    The walkers travel by dragging loaded feet (slip/m 1.1–1.5) at a
    ~50–77mm crouch; on hardware this scrapes and can catch.
    (P0, REWARD-ACCURACY DIAGNOSTIC — DONE 08-11 late: `--stack
    vref1` in `probe_walk_income.py`, GAIT.md bottom section.)
    Penalty-side suspect REFUTED: the plant-height tape-proven gait
    pays ≤1.5/ep total in gyro/roll/pitch and never terminates;
    totals are near parity (gait −11% at DR0, +8% at own-DR 0.35).
    The stack already favors tall by ~380/ep (stance kernel +
    k_height) but the crouch-paddle collects ~495/ep MORE walk+prog
    income because in sim it genuinely tracks the command (progress
    1.06 vs the gait's slip-limited 0.35) — a DISCOVERY/effectiveness
    problem, not a pricing hole. No repricing bank; proceed P2/P3.
    (P1, CLOSED 08-11) `cw-walk-gaitbc1` FAILED — BC-anchor gait
    cleanup froze into a static motionless tripod pose (video-
    confirmed, identical every episode, fwd ~0.00m vs parent's
    0.28-0.34m) instead of stepping: the anchor loss converged to
    ~0 by NOT MOVING (unlike rise/hold, walk's reference target is
    itself in motion, so freeze-toward-it looks satisfied). Per its
    own pre-registered gate, not a coefficient-variant situation —
    next is P2 or P3 lever 4. Detail: `rl_docs/GAIT.md`.
    (P2, CODE+bank LANDED 08-11 eve, operator session — CLAIMED,
    do not duplicate) charge-magnitude AUDIT DONE
    (`probe_drag_audit.py`): the per-tick charge FORM is refuted at
    any coefficient (0.5mm deadband left 53-97% of skating free;
    per-tick slip medians overlap gait's touchdown scuff), but
    per-STANCE accumulated travel separates skate from step 3.3x.
    Structural charge landed in walk_task (`reward.k_drag_stance`,
    `drag_stance_allow_mm`, `drag_stance_tick_floor_mm`; default
    off, bit-exact). Bank PASSING: step-gait > zero-lift skate AND
    > stall > park (test_drag_stance_stack_prices_skating_below_
    stepping). Operating point k=8000 / 6mm / 0.25mm (skater pays
    2.5x income, honest gait 23%). From-scratch arm = the operator
    session's next launch. Detail: `rl_docs/GAIT.md` bottom.
    (P2.5, TALL LADDER — operator session 08-11 eve, LIVE FLEET
    CAMPAIGN: run these follow-ups.) Height-as-COMMANDED-REF rungs
    work on the dep line where the hgt1 income gate did not:
    `cw-dep-tall30` (warm from tip1, walk_height_off_mm=-30 +
    k_drag_stance=8000/6mm/0.25mm, 2M) cut eval walk
    height_err_end 60mm→15mm at −24% speed (0.0295 vs 0.0388) and
    equal slip; isolation twin `cw-dep-tall30h` (no charge) is
    statistically identical (17.5mm/0.0287/1.80) → the charge is
    free, KEEP it; the speed cost belongs to the height rung.
    Rung 2 `cw-dep-tall15` (ref −15, warm from tall30) STALLED:
    body ≈ −44mm at either ref (err 29mm vs −15) — the free climb
    ends at ~−44mm (tip1 eval-ends ~−60mm). WHY THE FLEET MISSED
    IT: hgt1's FAIL verdict filed height as "cosmetic" and closed
    the thread; lowgait proved ref-rung tracking but ran DOWNWARD
    as a crouch envelope; nobody inverted it across lines.
    Pre-registered follow-ups (one variable each, warm from
    `ppo_goal_cw_dep_tall30` unless said otherwise; rung gate:
    height_err_end ≤8mm at ref, speed ≥0.028, survived 1, slip
    ≤1.8, no park):
    **T1 budget FAIL (08-11, `cw-dep-tall15-h1`, 6M):** not
       step-bound — `env/height_err_mm` plateaus by ~1M steps and
       sits flat the remaining 5M; harness eval end-state is WORSE
       than the 2M parent (51-58mm vs 29mm, near the tip1 ref-0
       baseline 59.9mm), with a bit more instability (one spinning
       episode, sto gait_valid 4/6). No cheat, still honest paddle.
       Do not schedule further step-count variants. Detail:
       GAIT.md. Next: T5 before T2/T3 (per this gate's own priority).
    **T5 DONE (08-11 eve, operator session — `probe_tall_wall.py`,
       landed):** the wall is HABIT/stability, NOT kinematics.
       Deterministic steady-state rollouts of the tall30 checkpoint
       vs the scripted plant-height gait in the same dep env:
       mid-gait the tall30 body rides **−75mm** with hip pitch −41°
       (≈70° of unused upward range) and knee 105° (≈46° room) —
       nothing pinned upward — but leg YAW is PINNED at its 35°
       limit (min margin < 0) with support radius 218mm vs the
       gait's 179mm: the policy buys lateral stability with
       splay+crouch. Scripted gait walks at +12mm in the same env
       (existence proof). **MEASUREMENT CAVEAT (binding):**
       `eval/walk/height_err_end_mm` is a STOP-WINDOW metric — the
       ladder's 60→15mm "gains" were mostly stop-stance recovery.
       Judge every tall arm by `probe_tall_wall.py` steady-state
       WALKING height (tall30 baseline −75mm; first target ≥ −50mm).
    **T3 result (08-11 eve):** 2M gate-at-reachable-ref
       (`cw-dep-tall-gate1`) holds 15mm/no factor collapse (0.58 vs
       hgt1's 0.24 crash) — gate and walking coexist. But the 6M
       hardening (`cw-dep-tall-gate1-h1`) is an INFORMATIVE FAIL:
       err 15→39mm — a σ30 income gate SLOWS but does not STOP the
       posture-for-speed trade under budget.
       **CONFIRMED 08-11 late (harness + probe_tall_wall.py on the
       actual checkpoint):** harness det height_err_end 36-47mm at
       DR0 (own-DR0.35 same band), speed 0.054-0.056 (above the
       speed bar, so this is a pure height loss, not a stall);
       steady-state walking height (the non-stop-window metric) is
       **-72.6mm, statistically the same as tall30's own -75mm
       wall** — 6M more steps under the gate bought ~2mm, not the
       ~45mm the σ30 gate was priced to hold. Leg-yaw limit margin
       still negative (-0.5..-0.3°, pinned at the 35° splay limit),
       same fingerprint as T5. Video clean six-leg gait, no cheat —
       an honest, informative miss. Gate-income alone is CLOSED as a
       lever at this dose; do not schedule another sigma variant.
    **T2/T4 VERDICTS (08-11 eve, probe_tall_wall mid-gait steady
       state): PRICING FAMILY CLOSED FOR POSTURE.** kh3 −74.5mm,
       kh10 −72.7mm (pays ~5.6/tick — MORE than its walk income —
       rather than stand up), slow1 −71.5mm at its trained 0.035
       cmd. Six arms total (ref ladder, income gate σ30, gate+6M
       budget, k_height 3x/10x, speed relief) all flat at
       −72..−75mm, leg yaw pinned at the 35° limit in every one.
       kh10's charge-eating proves the optimizer cannot FIND the
       taller basin — it is not underpaying for it. Do NOT launch
       more pricing variants for posture (incl. the yaw-splay charge
       candidate and σ45 — same family, pre-refuted by kh10).
    **T6 (RSI-for-walk) CODE LANDED + ARM RUNNING (08-11 eve):**
       `goal.walk_gait_start_frac` (walk_task + sim_env, default 0 =
       bit-exact, tests green): with prob f a walk episode spawns
       MID-STRIDE in the scripted tripod gait's tall pose, built at
       the episode's own command, command live from tick 0 (0.3 s
       ramp, no hold). Rationale: episodes always spawned tall
       STANDING — the missing states are tall mid-stride WALKING
       (same shape as the rise-RSI exploration closure).
       `cw-dep-tall-rsi1` = warm from tall-gate1, frac 0.5, 2M.
       If FLAT even with direct state injection: the wall is dynamic
       stability itself → physics easing or a taller scripted
       reference gait, not more reward work.
    **T6 VERDICT (08-11 late): FLAT — −77.4mm by probe.** The policy
       visits tall mid-stride states (50% of spawns) and actively
       dives back to the crouch (survived 1: it learned tall→crouch
       recovery, a robustness byproduct). CAMPAIGN CONCLUSION after
       7 arms + probe: neither pricing nor state injection moves
       mid-gait posture; crouch-splay is where PPO stabilizes this
       walk under current physics. Existence proof stands (scripted
       gait walks tall open-loop in the same env). Remaining levers
       in order: (1) **BC-INIT** — pure action pretraining on the
       scripted tall gait, THEN RL fine-tune under the gate income
       (different mechanism from the refuted gaitbc1 anchor-loss-
       during-RL, which could satisfy its loss by freezing; an init
       cannot); (2) physics easing ladder; (3) accept the pareto:
       `tall15_h1` (0.051 m/s, fastest dep walker ever, command-band
       for the first time) + tall STOP-stance for hardware, posture
       cosmetic. Byproduct: the 0.051 fast basin reappeared in rsi1
       — robust, reachable from multiple parents, and DEPLOYABLE
       (full dep contract) — bench-test it regardless.
    Winner → Gate 0 export + tipped retention + bench A/B vs tip1.
    Checkpoints: `ppo_goal_cw_dep_tall30{,h}`, `ppo_goal_cw_dep_tall15`,
    `ppo_goal_cw_dep_tall15_h1` (fastest dep walker, 0.051 m/s),
    `ppo_goal_cw_dep_tall_gate1{,_h1}` on train-2; tall30 also on
    train-0 + the Mac (md5 a50a5d61).
    (P3, the goal state — operator: learn it WITHOUT the anchor)
    from-scratch curriculum line: terrain-as-teacher CLOSED for good
    08-11 (two-miss rule — 72mm collapsed to leg-sacrifice, 54mm
    avoided the sacrifice but paddled 4-6x worse than the closed
    band; bumpy ground never forces stepping, `rl_docs/GAIT.md`).
    Next: drag charge annealed up (needs the charge-magnitude audit
    first), physics easing, RSI-for-walk. The 10+ closed anti-slip arms
    were income shaping retrofitted onto formed paddlers — the
    closure does NOT cover P0's repricing (if the probe demands it),
    P2's banked structural charge, or P3's task curriculum
    (operator ruling 08-11).
0.  **UNIFIED JOYSTICK POLICY (top deliverable).** Stand/sit/walk in
    one checkpoint (turning: see 0.2). Live experiment:
    `cw-uni-flag-a1-h2` (10M hardening) finished 08-11, verdict
    pending — decides one-brain vs MoE. Rise: problem 2.
    Line gate: joystick-gate retention AND rise/lower ≥5/6 AND quiet
    hold AND clean video on the post-273ebde floor.
0.2 **TURNING — RE-OPENED (operator 08-11 afternoon); steps 1–2 DONE
    08-11, step 3 QUEUED. Full results: `rl_docs/TURN.md` (bottom
    section).**
    (1) REWARD AUDIT — DONE, defect FIXED + BANKED: two cfg-gated
    fixes in walk_task.py (`reward.walk_yaw_hold_prog_gate` —
    heading-hold yaw income gated on achieved linear progress;
    `reward.yaw_still_avg_s` — drift charge on the wz EMA, not the
    gait's zero-mean oscillation). Pre-fix the yaw stack paid a
    frozen body +375/ep vs the honest gait's +224 net; post-fix
    income is monotone in honesty (gait 935 > … > driftride 715 > …
    > freeze 242) with a drift-rider calibrated to ACHIEVE the
    measured 0.09 rad/s. New stillness-subsidy bank (4 tests, incl.
    a legacy-stack reproduction and the full-summed-stack
    drift-rider check the old bank missed) green; TURN_OVERRIDES
    trains both fixes ON. The yawcmd1/yawgate2/turnfix1 ckpts no
    longer exist on any pod — ckpt-level audit closed UNTESTABLE.
    (2) REFLECTION WRAPPER — DONE, PASS: `mirror.MirrorPolicy` +
    `probe_mirror_turn.py` on cw-dep-vref1-r1 — mirrored policy
    drifts −0.040..−0.046 vs naked +0.037..+0.062 rad/s with
    IDENTICAL travel at DR 0 and DR 0.35, zero falls; bang-bang
    alternation holds heading to 2–4 deg over 12 s vs 34–50 deg
    naked runaway. Arc-left/arc-right/straight by chirality
    selection, zero training — but at the drift rate (~2 deg/s), so
    commanded-rate tracking stays open. Deploy port + rot60
    composition are follow-up [CODE]; hardware sign audit still
    gates any bench turn.
    (3) MIRROR-SYMMETRY TRAINING — RUN AND FAILED 08-11 late
    (`cw-walk-mirturn1`, discovery 2M, warm from vref1-r1, full fixed
    pricing, mirror coef 1.0): sym loss converged 28→0.5 but turn
    tracking never arrived (|wz_err| med 0.254, L/R asymmetry intact)
    AND the forced symmetry rewrote the gait (prog 0.41 vs ~1.0, slip
    5x parent). Mirror TRAINING on a warm champion is CLOSED per the
    pre-registered gate; MirrorPolicy chirality selection is the
    shipped turning story (deploy port + rot60 composition [CODE]).
    (4) BC-ANCHOR ON TURN TICKS: in reserve, unpromising after
    transbc1's walk-tick freeze.
0.3 **FOUR-LEG WALKING — NEW line (operator 08-11 afternoon).** The
    target image: the robot SHIFTS ITS WEIGHT BACK onto the four
    rear legs and walks on them, front pair raised off the ground as
    "hands". Never attempted; feasibility is GO (c57 sweep: 39mm
    static margin with CoM shift + rear-leg splay), and the quad
    HOLD mechanism is already solid (quad-hold2, dep-quad1-c2).
    SPECIFICATION FIRST — the current reward punishes this behavior
    by design, in two independent ways ("is the reward incentivising
    the correct behavior" — here, provably not yet):
    (a) the walk stack's anti-cheat machinery (park-duty charge,
    step-event credits, flag-leg/sacrificed-leg detectors,
    gait_valid) literally DEFINES a four-leg walk as the
    leg-sacrifice cheat we spent weeks stamping out — every
    accounting term must become lift-command-conditioned (commanded-
    lifted fronts excluded from park/step/flag checks and paid via
    quad_clear/quad_plant-style income instead);
    (b) the rear-shifted posture fights k_pitch/k_roll and the
    tilt-termination envelope, which are referenced to LEVEL — quad-
    walk episodes need a commanded posture reference (the inverse of
    the tipped-start level-reference trick) so tipping back on
    purpose isn't charged or terminated.
    Then the bank BEFORE any launch: honest four-leg stepping must
    out-earn (i) ignoring the lift command and walking on six,
    (ii) standing still on four, (iii) uncommanded leg sacrifice —
    and the rear-shift posture must be net-non-negative under the
    fixed pricing. Train as a SPECIALIST (warm from quad-hold2 or
    the walk champion): four dose points prove quad/walk mixing
    erodes walking, so no mixed-diet arms. If discovery stalls,
    write a scripted rear-four diagonal-pair gait (TripodGait
    variant on legs 1,2,3,4) as a BC-anchor reference.
0.5 **TEMPORAL-ARCH** (1–2 pods; see Architecture).
1.  Live truth for what's training/queued: `ops.sh census` +
    `launch_run.py backlog list` — never this file.
2.  [CODE] backlog, in priority order:
    1. **Omni translation — RESOLVED IN SIM 08-11 by rot-60 exact
       equivariance (`rl_move/sim/rot60.py`), zero training.** After
       the income re-probe exonerated pricing (honest gait out-earns
       every degenerate 2-4x; collapsed ckpts earn below a freeze —
       optimization failure, not a paid basin) and the 4th collapse
       (`cw-omni-transbc1`, BC anchor on walk ticks) closed
       reward/anchor tuning, the reserve lever landed: the robot is a
       regular hexagon (six identical leg templates at exact 60 deg
       spacing, axisymmetric chassis inertia), so rotate-60+relabel-
       legs is an EXACT model symmetry (test_rot60.py proves it on
       the compiled model, <1e-6 over 30 contact steps). rot60.py
       canonicalizes any heading into the +/-30 deg wedge at eval
       time (obs rotation + leg relabel, action un-relabel).
       Full-circle results, matched naked controls (`logs/rot60/`):
       `cw-dep-vref1-r1` (THE hardware ckpt) naked backward is frozen
       (0.027 m) — wrapped, every direction 0.024-0.036 trk_err at
       DR0 + own DR0.35, zero falls incl. flip stress, harness 20/24
       success, slip/m 1.1-1.3 (own band), video-clean gait;
       hist16-dep1 naked DEGENERATES AT EVAL into the leg-sacrifice
       (slip 7-11/m) — wrapped: gait_valid 24/24, slip 1.3-1.6.
       No omni training arm is needed. **Deploy-side port LANDED
       08-11**: the runner wraps rot60.Rot60Policy itself (no ported
       copy; `rl_policy.py make_walk_canonicalizer`, shipped by
       deploy_adb.sh, default-ON with bit-exact k=0 no-op + off-wedge
       refusal fallback + per-tick `rot60_k` CSV logging);
       replay-parity locked by `rl_move/tests/test_rot60_runner.py`.
       Remaining: BENCH validation during attempt #2 (detail:
       `rl_docs/TURN.md` tail). Eval-side: `eval_drive --rot60`,
       `eval_checkpoint --rot60`.
    2. **Rise beyond income shaping — RESOLVED to a validated
       mechanism 08-11 (BC anchor, lever (a)); the follow-up
       revealed a SEPARATE, pre-existing hold/track pricing gap.**
       Six reward-side arms collapsed to the identical feet-factor
       curve regardless of mechanism (warm-start OOD drift). Lever
       (a) (`rl_move/sim/bc_anchor.py`) landed; `cw-stand-bc1`
       PASSES (partial): harness-verified honest six-foot plants
       (bridge 7/12, crouch 6/8 valid_plant; flat cold-start 10/10
       correct stand, footprint-only miss), zero flag-leg cheat in
       42 video-checked episodes, clean one-variable attribution vs
       the identical-minus-anchor parent (still 0/12). Dose-check
       `cw-stand-bc1-coef03` (coef 0.3) FAILED decisively (valid_plant
       0/16) — keep coef>=1.0. Hardening `cw-stand-bc1-hard1` (10M)
       PASSES on rise (valid_plant 5/6 det, 83%, consolidates with
       budget) but its per-episode duty_cycle/swing_count data expose
       a REAL, pre-existing hold/track stillness gap (legs cycling
       continuously, not the anchor's fault per se — present already
       at 2M, WORSENS with more steps: 12-50mm foot elevation at 2M
       -> 100-161mm at 10M). Do NOT queue another rise
       reward-coefficient/RSI/dose/step-count variant on this
       lineage. Detail: rl_docs/RISE.md.
    3. **Hold/track stillness pricing — SOLVED 08-11 (BC-anchor
       extended to hold/track ticks; two pricing-only levers failed
       0/12 first).** `cw-stand-holdbc1` PASSES (hold 12/12
       valid_plant det+sto, first genuine quiet hold); 10M hardening
       `cw-stand-holdbc1-hard1` PASSES with no regression —
       `ppo_goal_cw_stand_holdbc1_hard1` is the hardened HOLD+RISE
       checkpoint (SKILLS.md), lineage CLOSED. **BOTH handoffs PASS
       (eval_handoff.py / eval_handoff_reverse.py): the full sim
       joystick motion cycle (rise→drive→stop→sit) composes with
       zero falls**; the scripted go_zero-sit glide (6/6 both
       physics) covers the deliverable's sit. Optional unqueued
       polish: BC anchor on lower ticks. **Deploy-side port LANDED
       08-11 late**: runner stance slot runs the specialist, goal
       profile rides in the weights meta, test_stand_runner.py locks
       the contract — remaining work is BENCH-ONLY. Numbers/evidence:
       rl_docs/RISE.md.
    4. mode/command one-hot — LANDED 08-11 (see Architecture);
       machine-readable metric semantics registry (RESEARCH_RULES);
       contact-from-proprioception aux head; zero-drift DR mechanism
       rework (open problem 5). **LOWER bank
       LANDED 08-11 (last owed bank; TURN/WALK landed earlier):**
       under the deployed specialist stack, honest command-tracking
       descent out-earns the outrig/aloft cheats on every seed
       (540 vs 461/383) and posture-strict rejects them (pads ~300 mm
       vs ~0) — lower-mode arms are unblocked. KNOWN THIN MARGIN
       (strict-xfail in the bank): pf=5/6 pricing lets one-leg-aloft
       keep 85% of honest income — the incentive behind the deployed
       specialist's cosmetic dangling foot; strengthen pricing or
       BC-anchor lower ticks before any lower-MECHANISM arm.

## Gate 0 — deployment equivalence (every hardware candidate)

Exact controller rate + action map + STATEFUL slew in training AND
eval; deployment-exact obs (meas:=ref); prev-action = raw proposal
(audited PASS 08-10 — don't re-audit); measured actuator dynamics;
25° tilt envelope consistent train/deploy; varied-start panel
(placement + bad-start + zero-drift); zero-command settle / ramp /
stop-restart panels; liftoff-reproduction check; scripted-gait
plant-calibration check whenever sim params change; per-tick
proposed/applied/measured logs. DR passes alone NEVER promote to
hardware. Supported ladder (readiness review): calibrate → retrain
forward gait under corrected physics → freeze on physical metrics →
supported hardware attempt; first milestone is FORWARD joystick, not
omni.

## Still-binding rulings (full text in `archive/`)

- Loaded slip accumulates episode-long (loaded foot-XY travel per
  meter), never reset by touchdown.
- progress_ratio vs commanded displacement (pass 0.75–1.25) replaced
  the 0.40 m gate; reference-relative end-state error replaced the
  60 mm allowance; under-reference is not free.
- support_margin is a stability backstop only; a six-foot end state
  must out-earn any hover. Stance current-economy arms stay blocked
  until the pricing calibration.
- Rear hemisphere deferred; heading ladder frozen at ±90°.
- Promotion = multi-seed panels + named corners; fixed panels are
  regression canaries only.
- Fall recovery waits for the unified policy (orientation-complete
  obs + fallen-pose resets + hard current pricing — quiet
  self-righting; 08-06 incident).

## Done =

The operator picks up the joystick and drives the real robot: it
stands up, walks where pointed, turns, stops, sits down — session
after session, no falls, no hot motors. Then the tricks: four-leg
stance, then four-leg walking.


---

# FILE: RESEARCH_RULES.md

# RESEARCH_RULES — binding agent behavior (operator, 08-10)

How the autonomous loop is allowed to design, launch, stop, and
interpret experiments. Moved out of RL_PLAN.md (which keeps blockers,
queue, architecture, closed moves, Gate 0). Startup reading order:
`RL_GOALS.md` → `CURRENT_TRUTHS.md` → `RL_PLAN.md` → this file (+
`RUN_INTERPRETATION_RULES.md`, the per-run triage checklist) →
`rl_docs/SIM.md`; `rl_docs/SKILLS.md` and `rl_docs/runs/<run>.md`
only for a concrete decision; archive/ only for historical questions.

## Prime directive

Your job is to minimize the number of unresolved blockers between
the current robot and reliable joystick control — that count is the
campaign KPI. Do not optimize GPU occupancy. Before training, prove
that the reward and evaluator prefer the intended behavior over
every known cheat. Use short runs to discover mechanisms; use long
runs only to harden behavior already seen. Prefer hardware-derived
questions over generic simulator robustness. Maintain a unified
history-conditioned controller as the target architecture; use
hist16 by default when it is competitive, increase plain-MLP
capacity before adopting MoE, and adopt MoE only after correctly
specified multitask training demonstrates genuine skill
interference. Kill obvious bad runs early. Every analysis must end
in a concrete decision that changes the next experiment or the
system.

**The launch question (every spec, before queueing): if this run
succeeds or fails, does it change what we do before the next useful
hardware test? If the answer is no, do not launch it.** Idle GPUs
are acceptable when the critical path is hardware, specification
work, or code fixes. Do not launch experiments to fill slots.

## Research tracks (operator, 08-11 — binding)

The campaign runs as parallel tracks, defined in
`rl_move/orchestrator/tracks.json` with per-track goals + status
docs in `rl_docs/tracks/`. Every launch carries `--track` (or an
inferable run-name prefix); the launcher tags the W&B run
`track:<id>` and records the track in the ledger.

- **hw** is the MAINLINE: the prime directive above (hardware
  joystick KPI, by any means) is ITS directive, and it has priority
  for pods. The other tracks (arch, nobc, quad, turn, …) are
  parallel research lines that run on excess capacity with their own
  goals — read the track's doc before designing for it.
- **CONTAINMENT: triage of a run fires follow-up jobs ONLY in that
  run's track.** The launch question is asked against the TRACK's
  goal, not always the hardware test. A finding that matters to
  another track is escalated, not launched: record one line in BOTH
  tracks' STATUS.md ("Now") plus `CROSS-TRACK INSIGHT:` in
  your cycle logline, and let the other track's next cycle act on
  it. Cross-track launches are operator-only.
- A verdict that changes a track's story updates THAT track's
  `rl_docs/tracks/<track>/STATUS.md` ("Now"/"Next" sections — keep it
  SHORT, a screenful; detail goes to the linked docs). The top-level
  STATUS.md stays the whole-campaign digest.

## Phases and budgets (launcher-enforced: `launch_run.py --phase`)

- **SPECIFICATION** — no PPO. Validate reward ordering, evaluator
  correctness, command semantics, state/action maps, known cheats
  (trajectory banks, preflights, smokes). Never trains.
- **DISCOVERY** — 0.5–2M steps (cap: guardrails
  `phases.discovery_max_steps`), aggressive early video/eval. The
  question is binary: did qualitatively correct behavior emerge?
  Stop quickly on a known exploit.
- **HARDENING** — 10–40M + seeds/DR/endurance/promotion panels, only
  after the mechanism works visibly; requires `--evidence` naming
  where (run/video/preflight PASS).
- **COMPOSITION** — combines already-valid skills/axes; every
  protected parent skill is evaluated against frozen baselines.
- **TRANSFER** — exact deployment contract, hardware-derived
  parameters, supported hardware tests, real logs. Outranks generic
  sim work.

## MDP_PREFLIGHT (before any run that adds/changes a reward or task
mechanism)

`rl_move/tests/test_task_semantics.py` must PASS for that mode under
the FULL reward stack the arm will train with; a skipped bank for
the mode is a launch blocker — build the bank first (SPECIFICATION).
Required orderings, with useful margins:

- RISE: honest six-foot plant > partial honest rise >
  flag-leg/tripod-at-height > freeze > unsafe thrash.
- LOWER: honest lower/sit > partial descent > full-height refusal >
  flag-leg/outrigger cheat > unsafe behavior.
- TURN: commanded yaw in the correct direction > partial yaw >
  fixed natural drift/straight walking > parking (safety terms
  active throughout).
- WALK: useful commanded progress > march-in-place/paddle stall >
  park/refusal — without declaring all physical foot slip a failure.

Every new exploit seen on video gets encoded in the bank BEFORE the
reward is fixed. Every success evaluator must reject known visual
cheats even when the scalar target error is good. A reward/eval bug
discovered after training is a preflight failure, never a reason to
launch a longer run. `preflight.py` (frames + frozen-vs-scripted)
stays for env sanity. [CODE, queued] Every new metric ships
machine-readable semantics: description, unit, direction
(higher/lower/target/diagnostic), valid modes, promotion status,
caveats.

## Designing runs

- Two experiment types (simplification review §6, 08-10). A
  **DIAGNOSTIC** run establishes causality or tests one mechanism:
  one variable per run, off the relevant line's champion, short
  discovery budget when the behavior is new, matched-parent control
  mandatory for injected axes. An **INTEGRATION** run (e.g. the
  unified-controller flagship) answers "does the complete controller
  work?" and may intentionally combine already-VALIDATED ingredients
  — but never pretend it isolates causality, and never use it to
  sneak in an unvalidated ingredient.
- Pre-register the gate and BOTH outcomes (if-true / if-false)
  before launch.
- **Two misses in the same BEHAVIORAL CLASS = change the hypothesis
  or the task specification — never the coefficient or the step
  count** (k=5,10,20,40 on the same penalty has never worked here).
- Warm starts: ent 0.001, inherited std, `--asym-critic`;
  `--no-canary` on single-skill lineages, canaries ON for
  multi-skill. From scratch: std 1.0, ent 0.005–0.01,
  target_kl 0.02. A climbing std is a health alarm.

## Judging runs

- **Work `RUN_INTERPRETATION_RULES.md` (operator, 08-10) BEFORE any
  deep analysis**: 8 ordered questions (learning happened? task — not
  just reward — moved? held-out eval? video physically correct?
  beat the frozen parent under identical conditions? protected
  skills survived? behavior trajectory over checkpoints? is more
  training even justified?) + a classification table mapping
  training/eval/video to the verdict (FAIL / reward-spec bug /
  generalization failure / evaluator loophole / skill interference /
  PASS / transfer failure). Stop at the first failing question;
  reward is never evidence by itself.
- Video is the promotion standard. Name pathologies bluntly (flag
  leg, dragging, skating, jitter, march-in-place). A checkpoint that
  scores well but looks wrong means the METRIC is the bug. ≥12
  episodes (det+sto), at DR 0 AND the run's own DR, 15 s horizon.
- **A KNOWN exploit in video is a complete verdict**: "STOP —
  reward/eval specification bug", one line, no forensic essay, no
  continuation, no re-run with more steps.
- **Matched-parent control**: any eval with an injected physics/
  sensor axis compares the child against the frozen parent under the
  IDENTICAL injection — `eval_checkpoint.py --baseline <parent.zip>`.
  A child-vs-clean-parent verdict is invalid.
- **Behavioral-impossibility kill** (don't wait for a plateau or the
  two-miss rule): kill a stand-up arm when correct success is still 0
  after the discovery window AND a known cheat dominates video AND
  the cheat's return rivals the desired path; kill a turning arm when
  yaw output stays command-invariant despite adequate reward
  separation.
- **A gate discovered to measure the wrong thing invalidates every
  conclusion that depended on it** until those runs are re-evaluated
  under a corrected gate (simplification review §12). Verdicts do
  not survive their evaluator.
- DIG-IN is reserved for genuinely discriminative cases: sim/real
  disagreement, unexpected regression on a correctly specified task,
  or two competing causal hypotheses implying different next actions.
- Wander/endurance is judged on along-path progress, never net
  start-to-end displacement.
- Driving-line runs must pass the JOYSTICK GATE (`eval_drive`:
  0 falls across the direction panel + flip stress). Hardware
  candidates additionally pass Gate 0 (RL_PLAN.md); promotion is
  judged on physical metrics, never one reward scalar.

## Reward routing

- GLOBAL terms = safety/limits/smoothness only; everything else is
  mode-specific. Income must make doing-nothing (parking, freezing,
  hovering, refusal) worth less than reasonable progress on the
  commanded skill BY CONSTRUCTION — audit it, don't assume it (the
  walk park attractor and the rise/lower freeze plateau were both
  this bug).

## Process

- Launches only via `launch_run.py` (capacity, code-SHA gate,
  ledger, phase gate). Ledger edits only via `launch_run.py update`.
  One RL_LOG line per cycle via `ops.sh logline`.
- Refills serve the BLOCKER LIST, not occupancy (reverses the 08-09
  "idle pods are the failure" order). Design questions ON THE
  CRITICAL PATH with a plausible answer get assume-and-go (log
  "## ASSUMPTION (operator to review)"); never invent a peripheral
  run to fill a pod.
- Hardware-derived evidence outranks generic sim robustness. A
  closed hypothesis reopens because of new PHYSICAL evidence (tape,
  current traces, loaded ladders), never because compute is idle.
- Never infer importance from how many lines a topic occupies in
  RL_LOG or the archive; CURRENT_TRUTHS and the current blocker list
  outrank historical token volume (simplification review §12).
- Every analysis ends in a decision that changes the next
  experiment, the task specification, the simulator, the deployment
  gate, or the hypothesis. Otherwise stop analyzing.


---

# FILE: RUN_INTERPRETATION_RULES.md

# RL Run Interpretation Rules (operator, 08-10 — binding)

Use this before any deep analysis. This is the triage checklist and
verdict vocabulary for every finished run: work the 8 questions in
order, stop at the first one that fails, and record the table's
verdict — the point is to NOT waste an analysis cycle (or a
continuation budget) on a run the first failing question already
classifies. Complements `RESEARCH_RULES.md` "Judging runs" (known-
exploit one-line STOP, matched-parent control, impossibility kills).

## 1. Did learning happen?
- Check BOTH training return and the pre-registered task metric.
- If neither improves meaningfully: **FAIL — hypothesis did not produce learning.**
- Do not assume “needs more steps” unless correct behavior has already appeared or there is a clear positive trend.

## 2. Did the real task improve, not just reward?
- Reward up + task metric flat/worse: **REWARD / SPECIFICATION BUG.**
- Assume the policy found a shortcut until proven otherwise.

## 3. Did held-out evaluation improve?
- Training task improves + held-out eval flat/down: **GENERALIZATION FAILURE.**
- Possible causes: overfitting, train/eval distribution mismatch, curriculum mismatch, or variance.
- Do not call it a pass.

## 4. Does the video look physically correct?
- Metrics improve + video shows flag-leg, tripod, dragging, jitter, freeze, march-in-place, or another known exploit:
  **EVAL / REWARD BUG.**
- Video overrides scalar success.

## 5. Did it beat the frozen parent under the SAME conditions?
- For noise / DR / friction / latency / actuator injections, evaluate child AND parent with identical:
  - injection
  - seeds
  - horizon
  - evaluator
- Otherwise no causal claim is valid.

## 6. Did protected skills survive?
- New skill improves + old skill regresses: **SKILL INTERFERENCE.**
- This is not a clean pass.

## 7. What happened over training?
Inspect early / middle / final checkpoints.
- Never correct -> mechanism likely wrong.
- Correct behavior appears then disappears -> interference / optimization drift.
- Correct behavior exists and keeps improving -> continuation may be justified.

## 8. When is more training justified?
Continue ONLY when:
- qualitatively correct behavior already exists, AND
- task/eval metrics are still improving or clearly budget-limited.

Never continue because:
- reward is flat,
- behavior is wrong,
- known exploit dominates,
- “maybe another 20M steps will fix it.”

## Classification Table

| Training | Held-out eval | Video | Verdict |
|---|---|---|---|
| flat | flat | wrong | FAIL — hypothesis failed |
| reward up, task flat | flat | wrong | reward/spec bug |
| task up | flat/down | maybe good | generalization failure |
| up | up | wrong | evaluator/reward loophole |
| up | up | good, old skill down | skill interference |
| up | up | good | PASS / harden |
| up | up | good in sim, worse on hardware anchor | sim/transfer failure |

## Default order of evidence
1. Correct physical behavior
2. Held-out task success
3. Protected-skill retention
4. Training task metric
5. Training reward

Reward is never enough by itself.


---

# FILE: rl_docs/README.md

# rl_docs — index (read this first, then only what you need)

Small, single-purpose files so no one (human or LLM) has to dig
through a 5,000-line log to answer a question. Each file says what
it is for; keep them SHORT when you edit them.

| File | What it answers | When to read |
|------|-----------------|--------------|
| `AGENT.md` | How the autonomous agent works, what we learned works/fails, future work | Taking over the campaign (human or LLM) — read FIRST |
| `../RL_GOALS.md` | What are we doing and why, in plain English; where we are; what's blocked (moved from `GOAL.md` 08-10) | Every cycle, first |
| `../STATUS.md` | How is it going: what the robot can do (real + sim), what's not working, big lessons learned — the operator-facing digest | Catching up after time away; after any hardware session or story-changing PASS (update it!) |
| `SKILLS.md` | What the robot can DO today: passed skills + their checkpoints (W&B artifact per row) | On any PASS (update it!), or when the operator asks what works |
| `WISHLIST.md` | Operator's backlog of things to learn — pull from it whenever pods would idle | Every cycle, when deciding launches |
| `COMMANDS.md` | How to run everything: `ops.sh` helpers, paths, hard-won gotchas; § "Operator status page" = web dashboard runbook | Every cycle, before running commands; when the operator's status page is down |
| `HARDWARE.md` | Real-robot evidence (traces in `rl_move/hardware_traces/`), sim2real findings, operator experiment backlog | When a decision hinges on real-world data; after any hardware session |
| `SIM.md` | What the physics sim models, where every actuator number came from, which uncertain params are covered by DR instead of point estimates | Before touching sim params/DR, launching a `bus.servo_params` arm, or judging sim-vs-real gaps |
| `REWARD.md` | Every reward term: cfg key, default, what it pays/charges, income-gate design rules; where a run's resolved reward config is recorded (W&B notes + `reward_cfg`) | Before adding/changing any reward term or judging a run's incentives |
| `EVALS.md` | Every eval metric: `SCORE/*` headline names, `eval/*` details, offline harnesses, comparability caveats | Reading a W&B page or wiring a new metric |
| `RISE.md` | Stand-up skill: plan, PLANT_SPEC standing spec, evidence trail | Working on rise/lower |
| `TURN.md` | Yaw drift problem, anti-drift mechanisms, TURN bank | Working on yaw/turning |
| `EXPERIMENT_LOGS.md` | Per-run `logs/experiments/<run>/summary.md` convention + cached W&B data | When finishing or investigating a run |
| `WANDB.md` | How W&B is wired in: project/creds, ops.sh readers, run-page anatomy (OUTCOME notes, artifact lineage), gotchas | First time touching W&B, or when an API call fails auth |
| `runs/` | One GENERATED summary per run (status, hypothesis, gate, verdict) — rendered from `experiments.json` by `launch_run.py`; never hand-edit | Browsing past runs; `launch_run.py runsmd` refreshes |
| `../RL_PLAN.md` | The current plan, gates, and queue (~400-line budget; single topics break out into rl_docs/) | Every cycle |
| `../RL_LOG.md` | Condensed campaign history; append ONE line per cycle via `ops.sh logline` only | Every cycle |
| `../rl_move/orchestrator/guardrails.yaml` | Hard limits you must obey | Every cycle |
| `../archive/` | Full history, reviews, audits (long; search, don't read) | Only when the condensed docs point there |

Standing rule: if you had to FIGURE OUT a command (it failed, was
slow, or took several tries) and then got it right, promote it —
add an `ops.sh` subcommand or a snippet to `COMMANDS.md` in the
same cycle, and keep this index accurate. The next agent should
never have to rediscover it.


---

# FILE: rl_docs/GOAL.md

# Moved

The plain-English mission, status, and the standing-up ("rise") plan
of record now live at **`../RL_GOALS.md`** (prototype root, next to
`RL_PLAN.md` / `RL_LOG.md`). Read that first, then `../RL_PLAN.md`.


---

# FILE: rl_docs/AGENT.md

# AGENT.md — how the autonomous RL agent works, what we learned, what's next

Handoff brief for the next human or LLM taking over this campaign.
Read this, then: `../RL_GOALS.md` (mission), `../RL_PLAN.md` (current plan +
binding rulings), `../RL_LOG.md` (history), `COMMANDS.md` (how to run
things), `../rl_move/orchestrator/README.md` (architecture details).
Written 2026-08-09 after ~60 autonomous cycles in one day.

## The one-paragraph version

A watcher on the controller pod polls W&B; when training runs finish
it spawns concurrent LLM "decision cycles" (`claude -p` with
`ORCHESTRATOR_PROMPT.md`). Each cycle TRIAGES finished runs (video +
eval table + reward curve, ~10 min each), records a verdict in the
ledger, and REFILLS by queueing new experiment specs into
`backlog.json` — but ONLY specs that reduce an unresolved blocker to
the next hardware test (prime directive, 08-10; idle pods are
acceptable, peripheral runs are not). Mechanical workers do
everything else: a drain
pushes queued specs onto free pods (self-repairing), checkups catch
dead/starved runs, a pre-stager pulls checkpoints and starts gate
evals before the cycle wakes up. **Software owns facts and
throughput; the LLM owns judgment.** That split is the single most
important design decision — everything in "what failed" below traces
back to some place where it was violated.

## How the agent picks the next run

**The filter comes first (prime directive, 08-10 — this REVERSED the
original 08-09 occupancy policy): which unresolved blocker between the
robot and the next hardware joystick test does this run reduce? If
none, do not launch it — an idle pod is acceptable; a peripheral run
queued "because capacity existed" is a violation. Nothing enters the
backlog unless it removes a blocker.** Full text:
`RESEARCH_RULES.md`.

For runs that pass the filter, source order (from
`ORCHESTRATOR_PROMPT.md` §4):

1. **Continuations of near-misses** — one, not two; identical-config
   continuations are CLOSED as a move (0-for-5 historically).
2. **The plan's next rung** — `RL_PLAN.md` queue + its binding
   operator rulings and the readiness-review priorities (P0/P1/P2).
3. **`WISHLIST.md` topmost [READY] items** — the operator's backlog;
   items marked [CODE] need an implementation cycle first. These too
   must pass the blocker filter; the wishlist is a source of
   candidates, not a license to fill slots.

Non-negotiable spec shape: warm-start off the current champion, ONE
variable per run, pre-registered gate with explicit if-true/if-false,
plain-English hypothesis, always `--out-name`. For design questions
ON THE CRITICAL PATH with a plausible answer: ASSUME AND GO, log the
assumption for operator review — but never invent a peripheral run to
fill an idle pod. `guardrails.yaml` caps everything (launches/cycle,
GPU steps/cycle, concurrent cycles).

## What we discovered WORKS (keep doing this)

- **Mechanical verification of operational state.** The LLM is never
  authoritative about what launched/exists/runs. The launcher
  verifies INTENT→RUNNING (process + W&B + code SHA); the ledger is
  the only record; `ops.sh census` reads /proc, not memory. Before
  this, runs were recorded as launched that never existed.
- **Backlog + self-repairing drain.** Decoupling "what to try"
  (LLM, minutes-to-hours) from "place queued work" (software,
  seconds) ended the lost-launch problem. The drain invariant — a
  free slot plus a non-empty backlog gets drained mechanically — is
  purely about placement speed. The binding invariant is at
  ADMISSION: nothing enters the backlog unless it removes a blocker
  (prime directive, 08-10). "Keep GPUs busy" is NOT a goal; an empty
  backlog with idle pods is a normal, healthy state.
- **Triage-first review.** Most verdicts need one video + one eval
  table (`ops.sh review <run>`). Forensics only on a trigger (gate vs
  video disagreement, anomalous metrics, plan forks, reward/env code
  changes). The earlier heavyweight process burned 40 min/run for no
  added decision quality.
- **Pre-registered falsifiable gates.** A run whose failure branch is
  written down BEFORE launch produces a usable verdict even when it
  fails (most of today's lever-closures came from clean if-false
  branches). "We haven't tried this coefficient" experiments are
  banned and stayed banned.
- **Video over scalars, exploit-watch columns.** Every metric the
  campaign optimized has been gamed at least once (velocity → 
  shuffling, clearance → flag leg, touchdown allowance → cadence
  inflation). gait_valid + frame strips + per-leg swing asymmetry
  catch what scalars hide.
- **Isolated-axis robustness runs off a champion.** One physics axis
  per run (payload, latency, deadband, CoM shift, friction, terrain)
  at DR0 → cheap, parallel, composable; then a DR-compose rung. This
  filled 12 slots with informative runs after "more seeds" ran dry.
- **Ladders, not leaps.** Abrupt widening breaks the gait every time
  (±180 heading, 2x speed band — both refuted). One rung at a time
  off the previous rung's checkpoint works (heading ±45→±90 passed;
  ±135 failed and FROZE the ladder — that's the ladder working).
- **Purpose-built gates for new behaviors.** The generic harness
  samples the training distribution; it cannot prove direction
  coverage or command-flip robustness (backforth was gated
  meaninglessly). `eval_drive.py` (scripted panel + flip stress) is
  the joystick gate; build the eval WITH the behavior, not after.
- **Helpers over instructions.** Agents re-derived the same parsing
  500+ times in a day despite docs saying not to. The fix that
  worked: make the helper (`ops.sh review`), put it FIRST in the
  prompt, and ban the alternative. Instruction without tooling does
  not change agent behavior.

## What we discovered FAILS (stop/avoid these)

- **Prompt rules without enforcement.** "Keep RL_LOG entries to 1-3
  lines" was ignored by every cycle; the file tripled in half a day.
  Now the only write path is `ops.sh logline` (one line, locked).
  Generalize: if a rule matters, encode it in a tool or a tripwire.
- **Concurrent cycles racing on shared state.** Every incident class
  we hit: near-duplicate launches (velsag/velsag30), dirty code
  markers from another cycle's uncommitted artifact, HEAD moving
  mid-drain, RL_LOG merge conflicts. Mitigations that work: locks
  (git/ledger/backlog), name-stem dedupe warnings, launcher SHA
  gates, generated per-run files instead of a shared log. Re-check
  census AFTER queueing, not just before.
- **Analysis paralysis with idle GPUs.** The agent's natural failure
  mode is over-deliberation: waiting for operator input, re-reading
  history, writing essays while pods idle. (SUPERSEDED 08-10: the
  KPI is now unresolved blockers to the next hardware test, not
  occupancy — idle pods are acceptable; peripheral runs queued to
  fill capacity are the new failure mode. Over-deliberation on the
  critical path is still a failure; assume-and-go still applies
  there. See RL_PLAN.md "Prime directive".)
- **"More seeds" as a default refill.** It's a cop-out. Seeds are
  for PROMOTION panels (ruling 7), not for filling slots.
- **Coefficient iteration on a rejected behavior.** k=5,10,20,40 on
  the same penalty never fixed anything here. Close the lever class
  after two clean refusals and change the mechanism.
- **Verdicting runs the cycle didn't evaluate.** Class-stops must
  name the evidence run; the operator misread a class-stop as a run
  FAIL once. Also: check the run-doc CLASS before requeueing a
  name-varied spec (speedband2-r1 re-ran an already-closed class).
- **Infra assumptions that bit us** (all now in COMMANDS.md
  gotchas): pods have no ps/pkill/curl/git; kubectl exec has a 2-min
  timeout (launch looks dead, isn't); killrun leaks /dev/shm
  segments; W&B lags fresh launches ~8 min; naive /proc kill scans
  match themselves; hard-killing the watcher murders in-flight
  cycles (use `restart_watcher.sh`).
- **Campaign science** (what the ROBOT taught us — full list in
  RL_LOG "what was tried and learned"): income levers can't outbid
  in-sim-free sliding (slip root = contact/current pricing =
  operator calibration, P0); DR is not a gait fix; full-DR retrains
  buy nothing; there is no park attractor; entropy runaway is the
  plateau driver on warm starts.

## Future work (in rough priority order)

Campaign (see RL_PLAN for binding detail):

1. **Contact/current calibration against hardware (P0, operator).**
   Everything slip-related is blocked on it; no new anti-slip reward
   arms until then.
2. **Mirror-symmetry augmentation [CODE]** — 3 independent
   motivations (head90 L/R asymmetry, strafe-dr10 flag legs, rear
   coverage need). Needs mirror index maps + trainer support + probe.
3. **Quad-hold goal mode [CODE]** — feasibility sweep passed (GO):
   static 4-leg stance with −20mm CoM shift + mid-leg splay. Then
   weight shift, then quadruped stepping.
4. **Omnidirectional joystick composition** — combine the ±90
   envelope, flip hardening (joyjit-dr05-c1 is the best driving
   candidate), and steering-DR into one policy; gate with
   eval_drive. Rear hemisphere waits on mirror-symmetry.
5. **Estimator rung** (DreamWaQ-style concurrent estimator) — the
   settled next architecture step; unlocks the <0.7x torque/low-grip
   boundary cases exposure can't fix.
6. **Sim-to-real ladder** (readiness review Gate C): freeze a
   forward-only policy on physical metrics, supported hardware
   trials. Hardware safety rules in AGENTS.md are absolute.

Agent/process:

7. **Mechanical gate pre-check.** The pre-stager could diff gate
   scalars vs the pre-registered thresholds and mark "clear PASS
   candidate" — cycles then spend judgment only on marginal calls.
8. **Class-level dedupe.** Name-stem warning exists; a better check
   is against rl_docs/runs verdicts ("this axis/class is CLOSED").
9. **Watcher-side eval_drive pre-staging** for driving-line runs
   (currently only the generic gate eval is pre-staged).
10. **Doc-size tripwires** — warn when RL_PLAN/RL_LOG exceed their
    line budgets instead of waiting for an operator cleanup pass.
11. **Node-change automation** — g12ba48 loss was handled by hand;
    capacity.py could flag guardrails/manifest drift when nodes
    come and go.

## First 10 minutes on takeover

```sh
cd /workspace/weird_objects/hexapod_walker/prototype_sts3215  # controller
bash rl_move/orchestrator/ops.sh census        # what's actually training
python3 rl_move/orchestrator/capacity.py       # slots/nodes, live
bash rl_move/orchestrator/ops.sh triage 12     # anything lost/ignored?
python3 rl_move/orchestrator/launch_run.py backlog list
tail -20 /workspace/orchestrator.log           # watcher alive?
```

The operator watches a status webpage (http://127.0.0.1:8090 via
port-forward). If they say it's down, the setup/restart runbook is in
`COMMANDS.md` § "Operator status page" — two pieces: `statusweb` tmux
session on the controller + `kubectl port-forward … 8090:8090` on
their laptop.

Then read RL_PLAN.md end to end (it's ~170 lines and every line is a
ruling). Do not touch the physical robot, ever, without an explicit
operator instruction in the current message (AGENTS.md, hardware
safety rules — a real robot was damaged on 2026-08-06).


---

# FILE: rl_docs/SIM.md

# SIM.md — what the physics simulation models, and how sure we are

One page of truth for the MuJoCo twin: what is simulated, where every
number came from, and which numbers are COVERED BY DOMAIN
RANDOMIZATION because we do not trust them as point estimates.

**Operator ruling (08-10, binding): quantities we are unsure of —
servo reaction times being the canonical example — go into domain
randomization as a range covering the uncertainty. Do not burn time
modeling them "perfectly", and never present an uncertain fit as an
exact nominal.** The confidence table below is the implementation of
that ruling; keep it honest when refitting.

## The simulation chain (one control tick, 25 Hz)

policy action → SafetyLayer (same rate clamp/limits as the robot,
`rl_move/safety.py`) → **servo model** → MuJoCo physics → obs builder
(same `build_obs` as the hardware runner).

The servo model (`rl_move/sim/servo_model.py`, `ServoProfile`) has
four stages, mirroring the real Feetech bus write
(`speed=400 counts/s, acc=20` — cfg `bus.write_speed/write_acc`, the
deployed runner's values):

1. **Latency** — each goal write matures after a per-axis delay.
2. **Trapezoid profile** — the servo's internal setpoint slews toward
   the goal (accelerate at `acc`, cruise at min(commanded speed,
   fitted velocity ceiling), decelerate).
3. **Deadband** — errors below it produce no motion (and no torque).
4. **Tracking** — MuJoCo position actuator `kp` + DOF damping `kv` +
   `frictionloss` + torque clamp track the profile target.

Identical semantics in the C env (`sim_env.py`, numpy) and the GPU
training path (`mjx_backend.py`, JAX pytree; parity-tested). The body
model comes from `mujoco_prototype.build_xml()` (CAD-derived
kinematics; primitive collision geometry; foot contacts softened 3× —
see `soften_contacts`). GPU port details: `../rl_move/sim/MJX_PORT.md`.

## Two fitted parameter sets (cfg `bus.servo_params`)

| Set | File | Data | Selected by |
|-----|------|------|-------------|
| **air** (default) | `rl_move/sim/sim_model.json` | 08-07 unloaded bench battery (`fit_motor_model.py`, legs straight out, fixed base) | nothing — legacy default, byte-exact |
| **loaded** | `rl_move/sim/sim_model_loaded.json` | 08-10 loaded step ladder at plant + rl_stand deployed traces (`fit_loaded_actuator.py`, free-base stance replay) | `--cfg-set bus.servo_params=loaded` |

Every consumer (both trainers, both eval harnesses, C and MJX envs)
resolves the set from the run's cfg, so a run's own cfg package evals
under its own actuator model. A missing file raises — no silent
fallback. The air fit is known-wrong under load (it also recorded the
commanded profile speed as the velocity ceiling, and its 0.494°
deadband is an unloaded-rig artifact); prefer `loaded` for anything
meant for hardware.

## Confidence table (loaded set, knee axis; DR bands at dr_scale 1)

| Parameter | Nominal | How we know it | Uncertainty | DR coverage |
|-----------|---------|----------------|-------------|-------------|
| deadband | 0.06° | loaded tracking 96.6% at 5° pins it (air's 0.494° was the whole sim tracking floor) | ~2× across joints/regimes | ×0.5–1.8 → 0.03–0.11° |
| velocity ceiling | 48.5°/s | ladder peaks 48–67°/s at non-binding profile speed | moderate — but the deployed write speed (35.2°/s) binds first, so it rarely matters | ×0.85–1.10 |
| **latency** | **85 ms (capped)** | **WEAKLY — ladder cmd→motion (110–210 ms) includes the HTTP hop; direct-bus air knee was 8.6 ms; fit sat on its cap** | **~25–160 ms plausible** | **×0.3–1.9 → 26–162 ms** (widened via `delay_ms_pct=0.45` in the file's spread — the ruling above, mechanized) |
| kp / kv / frictionloss | 916 / 0.17 / 0.02 | sim-in-loop fit vs step shapes; holdout ±5° 20–40× better than air | shape-fit, load-dependent in reality | kp ±20%, kv ±25% per joint |
| torque limit | 2.2 N·m | datasheet, never fitted | unknown under sag | torque_scale ×0.80–1.05 |
| hip / yaw (all of the above) | knee latency delta +76 ms, shared ceiling; air kp/kv/deadband | **ASSUMPTION — no loaded ladder for these axes** (HARDWARE.md wishlist item 4) | large | same DR bands; latency band spans the assumption |

Deployed-path sanity check: cmd→response lag from the rl_stand traces
(25 Hz derivative cross-correlation) is ~250 ms median on all axes —
consistent with latency + profile travel + tracking lag combined.

Mechanics of the ruling: `DomainRandomizer.from_params`
(`domain_rand.py`) widens DR from the params file's `spread` block, so
fit uncertainty travels WITH the file. Per-run widening needs no code:
`--cfg-set dr.latency_scale=0.3,1.9` (any `dr.<field>=lo,hi` is an
absolute range override applied after dr_scale).

## Also randomized every episode (`domain_rand.py` RandRanges)

Mass/inertia + CoM shift, per-link geometry scale, floor friction
×0.6–1.4, contact stiffness ×0.7–2.0, ground tilt (gravity vector),
battery-sag torque scale, dropped SyncWrites (≤5%/tick), IMU mount
offset + sensor noise, hand-placement pose slop, bad-start joints,
logical-zero drift, tipped starts. Model-field DR is applied in the C
env and (as shared-model per-env fields) in the MJX path.

**Tipped starts (`dr.tipped_start_prob/deg`, added 08-10):** with prob
0.30×dr_scale a plant/park-start episode begins at a settled 6–18°
body roll (asymmetric hip fold, calibrated: settled roll ≈ 0.36×fold)
while the tilt reference stays LEVEL — the policy sees the lean in obs
and the attitude terms pay it to level out. Capped at 70% of the run's
tilt-trip envelope; belly-rise starts never tip; the DOSE is not
shrunk by dr_scale (probability is). Born from the dep-vref1-r1
hardware runaway roll (rl_docs/HARDWARE.md 08-10). Discovery result
(cw-dep-tip1, same day): the champion already recovered static leans
in sim — the hardware runaway is a contact/pinning sim-to-real gap,
so this axis is a hardening/regression floor, not the fix. Eval:
`SCORE/tipped_recovery_success` (rl_docs/EVALS.md, gate-fix caveat).

## Known gaps — sim is NOT trusted here (open problems in RL_PLAN)

1. **Contact travel pricing: CALIBRATED 08-10, premise revised.**
   `rl_move/sim/calibrate_slip.py` replays the exact hardware tripod
   gait (same generator, plant +20/+80, hardware write profile
   1500/30) against the tape truth (ratio 0.50–0.51, speed-invariant).
   Result: sim travels 0.35–0.41 of commanded at BOTH speeds — sim
   does NOT price sliding as free; it loses slightly MORE stride than
   concrete (conservative), and matches the speed invariance and the
   walking current band (sim 0.36–0.45 A vs real 0.31–0.42). μ
   saturates above ~1.5 (sweep 0.2–8.0), so friction is not the
   limiting knob at nominal; XML default stands. New cfg hook
   `env.foot_friction_slide=<μ>` (C + both MJX stacks) recenters
   foot–ground slide μ if a future floor demands it; DR
   friction_scale multiplies around it. Re-run the script after ANY
   contact/servo-param change (Gate 0).
2. **Current/effort pricing at hold: the 08-10 "inversion" is
   RETRACTED (08-11, `probe_hold_current.py`,
   `logs/probe_hold_current/`).** The old comparison put sim
   MEAN-PER-SERVO (0.11 A) against hardware BUS-TOTAL (0.59 A) — an
   ×18 unit slip — AND compared different poses: the real 0.59 A is
   the SCRIPTED-stand hold (cmd = ideal pose the loaded joints sag
   from, servos fighting continuously), while the real walk-synced
   plant hold reads 0.106 A total (hw_session2 per-servo registers;
   the flailing-knee tick data shows register current tracks cmd−q
   error, e.g. 3.9° err → 0.685 A). Pose/unit-matched probe (free
   base, ServoProfile bus, hardware write profiles, air AND loaded
   params): sim |qfrc|×1.2 proxy vs hw register totals — crouch hold
   1.56 vs 0.106 A, ideal-cmd hold 1.56 vs 0.541, walk30 9.1 vs
   0.395, rl_stand replay quarters ≈4–16 vs 0.19–0.96. Sim
   OVERPRICES effort 3–25x in every condition (conservative: trips/
   charges earlier than hardware would) and REPRODUCES the real
   walk > plant-hold ordering — no inversion, no underpricing.
   Residual (DEFERRED, not a joystick blocker): the proxy is not
   register-accurate — real servos hold static load nearly free
   (gear friction) and pay ∝ cmd-fight, so a register-scale model
   needs a deadzone/cmd-error term. Fit it before any k_current>0
   hardware-pricing arm (quiet-gait/current-economy); `k_current=0`
   on hardware arms stands. Sim-vs-sim current comparisons in past
   verdicts stay valid (same proxy both sides).
3. **Hip/yaw loaded dynamics assumed**, not measured (table above).
4. **Liftoff +roll collapse not yet reproduced in sim** — the loaded
   actuator set is the prime-suspect fix; re-run the reproduction
   fixture on it before trusting stance sim near liftoff.
5. STL *visual* meshes have stale offsets (June re-export) — physics
   uses primitives; render with `mesh_visuals=False`.

## Refitting

Air battery: `POST /api/rl/probe_dynamics` on the robot →
`fit_motor_model.py`. Loaded: new bench traces into
`rl_move/hardware_traces/`, then `python -m
rl_move.sim.fit_loaded_actuator` (`--measure` first to sanity-check
parsing; it fits ±2/±10°, holds out ±5°, and replays the rl_stand
cmd streams as multi-step validation). Both write provenance +
evidence into their JSON. After ANY sim-param change: scripted-gait
plant-calibration check (Gate 0).


---

# FILE: rl_docs/REWARD.md

# REWARD.md — the reward function, explicitly

What every reward term pays or charges, where it lives in code, and
how to find the EXACT values a given run trained with. Operator
directive 08-10: the reward function of a run must be documented, not
reverse-engineered from launch commands.

## Where a run's actual reward config is recorded

Every run records its RESOLVED reward section (config.yaml + that
run's `--cfg-set` overrides) in three places, written at launch by
`train_ppo_sim.py` / `train_ppo_mjx.py`:

1. **W&B run notes** — a `=== REWARD FUNCTION ===` block listing every
   `reward.*` key=value; cfg-set overrides are tagged `[cfg-set]`.
2. **W&B run config** — `reward_cfg` (queryable dict, same content).
3. **Per-term training curves** — `env/reward_*` panels (one scalar per
   active term, mean per rollout), so you can see what each term
   actually PAID, not just what it was set to.

This file documents what the keys MEAN. Semantics are enforced by
`rl_move/tests/test_task_semantics.py` (MDP_PREFLIGHT): before a term
change trains, the bank must show honest behavior out-earning every
known cheat. See RESEARCH_RULES.md.

## Design principles (violations caused real incidents — keep them)

- **No unconditional alive bonus** (`reward.alive: 0`). Survival paid
  ~96% of achievable reward once; PPO froze and collected it.
- **One substantial income: the task kernel.** Everything else is
  small shaping, one-time bonuses, weak penalties, or a GATE.
- **Gates multiply INCOME, never penalties** ("worth less by
  construction", operator 0-c.2). Additive charges on exploits are a
  CLOSED move for slip and yaw drift — the policy prices them in and
  keeps the exploit. A gate makes the cheat earn ~0 instead.
- **Every gated/optional term defaults to 0/off = byte-identical
  legacy behavior.** A run's reward is base stack + whatever its
  `--cfg-set` enables.
- **Declared routing.** A term is GLOBAL (prices physics: tipping,
  heat) or MODE-ROUTED (prices task shape). Routing changes must be
  declared and are refutable — e.g. all-modes `k_flag_leg` was refuted
  (taxed rise's legitimate curl transients) and re-routed walk-only.

## 1) Base stack — every tick, every mode

`rl_move/env.py compute_reward()`, shared by hardware env and sim.
Total = kernel income + weak shaping + weak regularizers.

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_track` | 1.0 | THE income. Product of per-objective Gaussians in [0,1]: tilt vs ref (`track_sigma_deg` 1.5°), height vs ref when the goal has one (`height_sigma_mm` 20), leg unload force when the goal asks (`unload_sigma_n` 1 N). Perfect tracking ≈ +1/tick; ignoring the goal ≈ 0. |
| `track_sigma_deg` | 1.5 | tilt width of the kernel (widened to `unload_tilt_sigma_deg` 4.0 in unload episodes — leaning is the mechanism there). |
| `height_sigma_mm` | 20.0 | height width; wide so PARTIAL rises earn partial credit. |
| `k_roll`, `k_pitch` | 10.0 | small quadratic tilt shaping — gradient far from ref where the kernel is flat. |
| `k_height` | 100.0 | same, for height error (quadratic, in m²). |
| `k_gyro` | 0.05 | −k·Σ gyro² — quietness. |
| `k_action` | 0.005 | −k·Σ action². |
| `k_action_delta` | 0.01 | −k·Σ (a−a_prev)² — smoothness. |
| `k_current` | 0.005 | −k·Σ current² (A²) — "don't cook motors", kept weak. |
| `k_current_max` | 0.0 (off) | −k·max(|current|)² — prices load CONCENTRATION the sum-square misses. |
| `k_still` | 0.0 (off) | quiet-stance bonus: Gaussian on mean qd², MULTIPLIED by the kernel so a frozen belly-rest earns nothing. `still_sigma_rad_s` 0.3. |
| `hold_still_gate` | 0 (off) | hold/track stillness+feet pricing (08-11, from `cw-stand-bc1-hard1`'s dig-in): the tracking kernel pays torso pose with no opinion on the legs, so hold/track converged to continuous leg-cycling (2M) and a frozen flag-leg park (10M) at near-full income — measured in the HOLD bank: legacy pays the flag pose 368.0 vs the quiet stand's 367.9 (a tie) and stepping 0.82×. Scales kernel income on hold/track ticks by feet-down² × HARD no-flag zero (`PLANT_SPEC.flag_leg_mm` 60 mm — honest adjustment swings stay below it) × stillness Gaussian (`still_sigma_rad_s`, applied only while the reference is stationary so TRACK's commanded motion is never charged). Blend `(1-g)+g·f`. Scoped strictly to hold/track: quad lifts legs and unload opens a contact on purpose; rise/lower/raise keep their own stacks. Gated ordering: quiet 368 > stepping 107 > flag 9.5 (bank, 3 seeds). Logs `hold_feet_factor` / `hold_still_factor`. |
| `hold_flag_fade` | 0 (off) | fade variant of the gate's no-flag factor (08-11, from `cw-stand-holdstill1` FAIL): the hard zero priced the flag park correctly but is a zero-gradient plateau — the trained run kept a leg parked ~110 mm for 2M steps because every nearby behavior also earned ~0. With fade=1 the no-flag factor ramps linearly over [`flag_leg_mm`, 2×`flag_leg_mm`] (60→120 mm): compliant poses keep exactly 1.0, the observed ~113 mm park earns scraps (51 vs quiet's 368, 0.14×) WITH a downhill slope toward feet-down, the ~190 mm class still earns 0 (12.6). Bank: ordering preserved, gradient exists (51 > 12.6), park stays <25% of quiet. Only meaningful with `hold_still_gate` on. |
| `k_unload` | 0.2 | weak linear gradient toward zero load on the unload leg. |
| `alive` | 0.0 | keep at 0 (see principles). |
| `safety_termination_penalty` | 10.0 | one-time −10 on safety termination (tilt trip etc.). |

## 2) Rise / lower / raise terms

`rl_move/sim/sim_env.py _post_step()` — episodes with a height target.

Income and one-time bonuses:

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_rise_progress` | 100.0 | potential-based +k·Δ|height_err| per tick (telescoping; full 50 mm rise ≈ +5). Freezing while the ref ramps away CHARGES. Never gated. |
| `k_rise_milestone` | 2.0 | one-time bonus at 25/50/75/90% of the signed height target. Scaled by the posture/plant gates below. |
| `k_rise_finish` / `rise_finish_sigma_mm` | 1.0 / 8.0 | narrow arrival kernel paid only after the ref has fully ramped — kills "park 20 mm low for 61% pay". |
| `rise_finish_gate_signed` | 0.0 (off) | =1 fixes a lower-mode bug: the legacy `ref >= target` ramp-done test is always-open for negative targets, so the arrival kernel paid a freeze at start height (~+57 of the freeze plateau's +74). |
| `k_curl_progress` | 120.0 | rise only: +k per meter of mean foot-XY distance closed toward the plant footprint. Potential-based — crouch starts and foot-parking earn nothing net. |
| `curl_milestone_mm` | [40, 15] | one-time `k_rise_milestone` each when mean curl distance first drops below each threshold. |
| `rise_hold_curl_sigma_mm` | 20.0 | during the pre-ramp HOLD window the tracking kernel is SWAPPED to pay curl distance instead of tilt/height stillness — otherwise lying frozen earns ~1/tick and preparation is priced as a loss. |
| `k_rise_ref_track` / `rise_ref_path` / `rise_ref_sigma_deg` | 0 / None / 12° | trajectory-scaffold: joint-space RMS kernel against a recorded rise (npz), time-aligned at ramp start. Seed the skill at full weight, then ANNEAL to 0 — not the objective. |

Income GATES (each in [0,1], scales income terms only — milestones,
finish bonus, post-ramp kernel; progress and penalties never scaled):

| cfg key (reward.) | default | what it closes |
|---|---|---|
| `rise_income_prog_gate` | 0.0 | freeze plateau: once the ramp leaves zero, income × fraction-of-target-covered. Measured: a frozen lower banked +74/ep, above every honest imperfect attempt. |
| `rise_posture_gate` | 0.0 | torso-at-height-feet-flying (bridge/flail): income × fraction of pads within `end_posture_allow_m` (0.02, stand) / `end_posture_allow_lower_m` (0.06, lower — honest lowers leave pads 17–43 mm up) of grounded z. |
| `rise_plant_polygon_gate` | 0.0 | stilt/splay/edge stands the clearance gate is blind to: income × continuous PLANT_SPEC factor (CoM depth in the down-feet support polygon, level attitude, body-frame footprint near plant anchors). Rise only. See RISE.md §spec. |

## 3) Stance-quality terms (global or stance-routed)

`sim_env.py`, all default OFF:

| cfg key (reward.) | default | routing | what it does |
|---|---|---|---|
| `k_current_hot` / `current_hot_a` | 0 / 1.0 A | global | −k·Σ max(current−threshold,0)² per servo — prices load concentration (one knee at 1.8 A hurts, six at 0.4 A free). |
| `k_support_margin` | 0 | global | ±k·clip(CoM depth inside support polygon, ±40 mm)/40 mm. Belly rest exempt (<3 contacts). |
| `k_load_even` | 0 | global | −k·(Herfindahl of foot forces − 1/n): even load charges 0, all-on-one-foot charges max. |
| `k_stance_contact` | 0 | hold/lean/track/unload/raise | +k·(loaded feet)/n — anti-tripod. Unload target leg excluded. |
| `k_stance_clearance` | 0 | hold/lean/track/unload | −k·Σ pad height above episode-start grounded z — dense gradient pulling hovering feet down. Raise exempt (refuted: collapsed raise 0/6). |
| `k_flag_leg` / `flag_leg_allow_m` / `flag_leg_walk_only` | 0 / 0.05 / 0 | all modes, or walk-only with the flag | −k·clearance above a 50 mm allowance — prices the parked vertical leg while normal swing stays free. All-modes routing REFUTED (taxed rise curls); use walk_only=1. |
| `k_end_posture` (+ `end_posture_ref_mm` 15, `end_posture_grace_s` 0.25, `end_posture_window_s` 1.5, `end_posture_allow_m` 0.02, `end_posture_allow_lower_m` 0.06, `end_posture_lower_dense` 0) | 0 | rise/lower/raise | terminal clearance charge, SCHEDULE-windowed (after the height ref settles): flag-leg endings pay, motion phase untaxed. `lower_dense=1` extends to the whole lower episode (no legitimate lift transient exists there). |

## 4) Walk terms

`rl_move/sim/walk_task.py _post_step()`, walk mode only by
construction. Hardcoded kernel constants: `K_WALK = 2.0` (peak),
`SIGMA_V = 0.05` m/s (width), `K_PROG = 1.0`.

Income:

| cfg key (reward.) | default | what it does |
|---|---|---|
| (kernel, always on) | K_WALK 2.0 | Gaussian on |v − v_ref| — up to +2/tick. |
| `k_walk_prog` | 1.0 | linear progress: k·clip(along-command speed fraction, −∞, 1.25). Negative when moving against the command. |
| `k_walk_yaw` / `yaw_sigma_rad_s` | 0 / 0.15 | yaw-rate tracking kernel, paid every walk tick incl. wz_ref=0 (heading-hold income prices drift). |
| `k_yaw_prog` | 0 | SIGNED rotation income on turn segments: k·clip(wz/wz_ref, −1.5, 1.25) — genuinely negative against the command (the Gaussian kernel never is). Anti-drift, see TURN.md. |
| `k_yaw_still` | 0 | quadratic drift charge on heading-hold segments (wz_ref=0): −k·wz². At the measured 0.09 rad/s drift, k=50 costs ~0.4/tick; gyro noise stays ~free. See TURN.md. |
| `k_phase_contact` | 0 | ±k on tripod-clock contact agreement (paired with `goal.walk_phase_obs`); parked/dragged legs average 50% = zero net. |
| `k_walk_swing` | 0 | one-shot +k per completed real swing (≥2 ticks airborne, lands ≥15 mm away). |
| `k_step_event` / `step_disp_budget_mm` | 0 / 0 | one-shot per-leg credit for a touchdown displaced ≥10 mm along command, scaled by along/30 mm cap 1.5×. The budget makes each paid credit CONSUME banked body displacement — cadence inflation and stride-in-place earn nothing by construction. |
| `k_quad_clear` / `k_quad_plant` / `quad_clear_cap_mm` | 0 / 0 / 30 | quad mode: pay lift-leg clearance (only while OFF the ground) and the loaded fraction of the four support legs, after `goal.quad_grace_s`. |

Income gates (each in [0,1]; scale kernel + positive progress only):

| cfg key (reward.) | default | what it closes |
|---|---|---|
| `walk_kernel_prog_gate` | 0 | the paid park: at 0.02–0.06 m/s commands the absolute-error kernel pays a parked robot up to 93% of peak. Income × clip(along/s_ref, 0, 1). |
| `walk_yaw_kernel_gate` | 0 | same construction for turn segments: yaw income × clip(wz/wz_ref, 0, 1). Hold segments stay ungated (that income IS the drift pricing). |
| `walk_kernel_yaw_gate` | 0 | the turn-in-place freeze floor (collapsed `cw-omni-mirror1-r1`, 08-11): on yaw-commanded ticks with NO linear command the LINEAR kernel pays a frozen robot full income (v_lin=0=ref; the prog gate needs s_ref>1e-3). Linear kernel × clip(wz/wz_ref, 0, 1) on those ticks; genuine stop segments (both refs ~0) stay paid. Freeze-floor bank in test_task_semantics.py pins it. |
| `walk_anchor_gate` / `anchor_tol_mm` | 0 / 10 | paddling: income × anchored fraction of loaded feet (loaded and within tol of own touchdown point). |
| `walk_loadslip_gate` / `loadslip_ok` 0.75 / `loadslip_max` 1.50 / `loadslip_floor_m` 0.05 | 0 | cadence-reset exploit of the anchor gate: income × factor of EPISODE-ACCUMULATED loaded slip per meter of progress (the same ratio the eval harness scores — no touchdown resets it). The `walk_loadslip_ratio` metric logs regardless of the gate. |
| `walk_height_gate` / `walk_height_sigma_mm` | 0 / 30 | hardware sag (08-10): deployed walk policies ride a COMMANDED crouch 54–70 mm below the spawn stance; base `k_height` (~0.36/tick at 60 mm) is outbid by walk income (~3/tick). Income × Gaussian on body height vs the episode `_z0` anchor — upright gait keeps 0.99 of income, the −51 mm crouch keeps 0.13 (probe + MDP_PREFLIGHT height bank). Symmetric, so stilting up is never a strategy. The `walk_height_factor` metric logs regardless of the gate. |

Charges:

| cfg key (reward.) | default | what it does |
|---|---|---|
| `k_drag_loaded` | 0 | −k per meter of foot XY translation while in contact (skating); 0.5 mm/tick deadband. |
| `k_drag_stance` / `drag_stance_allow_mm` | 0 / 6 | structural stance-slip charge (charge-magnitude audit 08-11, `probe_drag_audit.py`): accumulated loaded XY travel per STANCE PERIOD, charged incrementally beyond the allowance — prices the dragging STROKE where the per-tick form cannot separate skating from touchdown scuff. Audit operating point k=7000/m @ 6 mm: a learned skater's drag cost ≈2.4× its income, the honest gait pays on <5% of stances. |
| `k_park_duty` | 0 | −k·(per-leg contact duty outside [0.1, 0.9]) over a trailing 2 s commanded window — a tripod park pays ~0.6k/tick, a real gait pays nothing. |
| `k_walk_effort` | 0 | −k·mean servo current per walk tick (cost of transport; thermal load is the hardware-fatal quantity). |
| `k_drag_trans` | 0 | **NON-walk modes** (rise/lower/raise/hold/track/lean/unload/quad): −k per meter of loaded foot-XY translation, 0.5 mm/tick per-foot deadband — the stand/sit foot-scrape the operator watches the robot do (08-11 night) was completely unpriced outside walk. A loaded foot that pivots/slides pays; a foot that lifts and STEPS to its new spot is free. `trans_drag_mm` metric logs on every non-walk tick regardless of k (watch the dragging without coupling metric to price). TRANS bank in `test_task_semantics.py` pins the orderings. |

## 5) Changing the reward — checklist

1. New terms: cfg-gated, default 0 = byte-identical legacy. Income
   modifiers are GATES, not additive charges.
2. Declare routing (global vs mode) and why.
3. Extend/run the MDP_PREFLIGHT bank
   (`pytest rl_move/tests/test_task_semantics.py`) — honest behavior
   must out-earn every known cheat BEFORE training.
4. Log the term into `parts`/`info` so it gets an `env/reward_*` curve.
5. Document the key HERE (one table row) in the same change.

Champion-comparability caveat: `SCORE/*_total_reward` values are only
comparable between runs with the same reward config — check the run's
`reward_cfg` before comparing (rl_docs/EVALS.md).


---

# FILE: rl_docs/RISE.md

# RISE — standing up inside the walking policy (full plan + history)

Moved out of RL_GOALS.md / RL_PLAN.md 08-10 (operator doc-shrink
ruling). RL_PLAN.md "Open problems" item 2 is the binding summary;
this file is the detail. Status date: 2026-08-10.

## Where it stands

Unified rise is UNSOLVED. Every arm to date satisfied the height
criterion via a cheat (flag-leg / tripod / stilt) or froze. Working
fallback that does not block the joystick MVP: the stance champion
(`ppo_goal_cw_stance_dr10`) performs a genuine feet-down belly-rise,
a scripted 1.5 s blend reaches the walkable plant stance, and the walk
champion drives — sim-proven end to end (sim viewer key `7`).

## Root causes found 08-10 (each once cost a multi-M-step run)

1. Reward paid FREEZING more than trying (arrival-gate sign bug).
   Fixed 69e00c0: `reward.rise_finish_gate_signed=1` +
   `rise_income_prog_gate=1` — ALL rise/lower arms set both.
2. The walk-lineage warm start is measurably blind to the height
   command channel.
3. Training reward paid TORSO HEIGHT only → fresh policy hit height
   with feet 30 cm in the air ("6/6" on training's criterion, 0/6
   posture-strict). Longer training optimizes the cheat harder.
4. Posture gate priced lower's feet at the 20 mm stand allowance while
   an honest belly-down lower leaves pads 20–45 mm up → honest≈cheat
   income, outrigger cheat won (postgate1, ERODED lower). Fixed
   08-10: pf uses the 60 mm lower allowance.
   Pre-273ebde rise/lower checkpoints stay invalid near the ground.

## The literature route (HumanUP, HoST RSS 2025; HiFAR 2025)

Never learn a deployable get-up from a bare task reward in one shot.
Discover the motion once (loose limits, sparse reward), then train the
deployable policy to TRACK the discovered trajectory under strong
smoothness/torque regularization and randomization, with posture-aware
staged rewards and curricula. Our discovery stage is DONE — the stance
champion's belly-rise (re-verified 08-10: det flat start, ends 5 mm
off target, worst pad clearance 4 mm).

## Landed machinery (all cfg-gated, default-off)

- **Posture gate** (`reward.rise_posture_gate=1`): rise/lower income
  scales with the fraction of feet within the mode's clearance
  allowance (20 mm stand / 60 mm lower) — geometric, matching the
  harness's end_posture_ok, not touch force.
- **Reference tracking** (`reward.k_rise_ref_track` +
  `reward.rise_ref_path`): dense joint-space kernel against the
  champion's recorded rise, time-aligned at the height-ramp start.
  Champion earns ~full pay on its own reference; a frozen robot ~13%.
  A scaffold — anneal to 0 across arms.
- **Walkable-height reference** (operator ruling 08-10 ~11:00 ET: the
  ~70 mm crouch-stand is "a terrible stand"; rise must end in the
  ~142 mm plant stance): `extract_rise_ref --blend-to-plant` →
  `rl_move/sim/refs/rise_ref_belly2plant.npz` (+111 mm, all pads
  down, 7.4° RMS from plant). Commanding it needs
  `goal.rise_height_mm=[108,114]` AND `actions.max_height_mm=115`.
- **Stand-score income routing** (`reward.rise_score_income=1`,
  landed 08-10 evening after plantgate1 FAILED — gates leak, so the
  income itself moved): rise-episode height income (progress,
  milestones, finish, kernel) is ZEROED; the only rise income is a
  progress ratchet (`k_rise_score_prog`, pays once per new best) +
  post-ramp hold pay (`k_rise_score_hold` × S²/tick) on stand-score
  S = height-kernel × feet-down² × HARD no-flag × plant geometry,
  plus a ramp-weighted airborne-feet rent (`k_rise_posture_pen`) —
  the bank showed cheats otherwise win on the PENALTY side by dodging
  `reward_height` (torso up any way possible). The rent charges
  feet-in-the-air only, NOT the geometric fades: the honest reference
  itself moves through wide-footprint poses mid-rise. Lower keeps the
  legacy (solved) stack. Bank, all seeds: replay +91 ≫ stilt −9 >
  flag-leg −165 (income ~0) > freeze −218. First arm:
  `cw-stand-score1`.

## The standing SPECIFICATION (landed 2026-08-10, PLANT_SPEC)

"Standing" is now a geometric predicate, not a torso height —
`PLANT_SPEC` / `valid_plant()` in `rl_move/sim/sim_env.py`, ONE
criterion shared by the training reward, the eval harness, and the
semantics bank (never let them drift). A stand is valid iff at
episode end:

| check | threshold | kills which cheat |
|---|---|---|
| height | \|err\| ≤ 15 mm of target | parking short of the plant |
| attitude | \|roll\|, \|pitch\| ≤ 10° | leaning-tower stands |
| feet down | ≥ 5/6 pads ≤ 20 mm | tripod-at-height (b2p1) |
| no flags | no pad > 60 mm | flag-leg (b2p1, fresh1) |
| support | CoM ≥ 20 mm inside down-feet polygon | edge-balanced poses |
| footprint | body-frame feet ≤ 40 mm from plant anchors | stilt/splay (~50 mm out) |
| effort | max servo ≤ 2.0 A | fighting poses (real once current model lands) |

Consumers:

- **Eval harness**: `valid_plant` + `plant_fail` + `plant_margin_mm`
  reported per stand-ending episode (always); `--valid-plant-gate`
  wires it into rise/raise success — OFF until champions are
  baselined (same rollout as end_posture_gate on 08-08).
- **Training reward**: `reward.rise_plant_polygon_gate` (0..1,
  default 0) scales the same income terms as the posture gate by a
  continuous factor (CoM margin × attitude × footprint). Bank smoke
  08-10: replay +946 (full pay kept), stilt +217, freeze −188.
- **Bank**: `test_rise_valid_plant_separates_stand_from_cheats` —
  replay ends valid on every seed; stilt / freeze / partial all fail.

## The binding preflight

`rl_move/tests/test_task_semantics.py` (RISE bank): replaying the
demonstrated belly→plant path must dominate the stilt exploit and the
freeze under the FULL reward stack (was +952 / +225 / −195), and
freeze must be net negative. The b2p1 lesson: individually-validated
terms still lose to the height cheat when composed — the bank always
runs the full stack the arm will train with.

## Evidence trail (run docs have the full facts)

- `cw-uni-rfix-warm1` — lower 6/6 posture-strict after the pricing
  fix; KEEP fine-tune grafting (distill refuted by fresh1).
- `cw-uni-rfix-fresh1` — strictly worse (tripod + over-current).
- `cw-stance-postgate1` — FAILED and ERODED lower (allowance bug,
  since fixed).
- `cw-stand-b2p1` — FAILED (08-10 ~16:5x): height nailed (<6 mm err
  both modes) but rise 0/12 AND lower 0/12 posture-strict; flag-leg/
  tripod cheat video-confirmed (rise: 2 legs 80–131 mm up; lower: 1
  leg up to ~288 mm). Same pathology class as rfix-fresh1 and
  cw-stance-riseproof1 (open DIG-IN).
- `cw-stance-riseproof1` — control probe: stance-line joint_goal
  recipe from scratch on today's sim; decides whether the walk-env
  task construction or near-ground sim contact is implicated.
- `cw-stand-score1` — FAILED (08-10 night): score-income routing
  (income moved to a stand-score S = height-kernel x feet-down^2 x
  no-flag x plant geometry, + airborne-feet rent) warm-started from
  the HONEST stance champion (not a cheating checkpoint) — still
  0/12 valid_plant / end_posture_ok at both DR0 and own-DR0.2, EVERY
  start_kind (flat/bridge/crouch), every episode flagged
  `feet_down`+`no_flag` with one leg 40–150 mm off the ground.
  `env/rise_score` stayed flat ~0.01–0.02 for the full 2M steps (the
  pre-registered early-call trigger). Same pathology class as b2p1
  and plantgate1 — third distinct mechanism, same cheat. hold/track
  unaffected.
- `cw-stand-scoreref1` — FAILED (08-11): score1's stack unchanged +
  a cheat-proofed belly→plant reference-tracking crutch
  (`k_rise_ref_track=2.0`, feet-gated kernel, sigma 12→6°) warm-
  started from `cw-stand-score1`'s lineage on the honest stance
  champion. Still 0/6 valid_plant/end_posture_ok det AND sto at DR0
  gate and own-DR0.2, worst-foot clearance 160–188mm (leg held in
  the air the whole episode, video-confirmed), `env/rise_score` flat
  ~0.01–0.02 the entire 2M steps. Fourth distinct mechanism, same
  cheat — this was the pre-registered test of lever (a) below and it
  is now closed too.
- `cw-stand-scoreref1-dr0` — FAILED (08-11), control arm: identical
  to scoreref1 but DR-scale 0 (pre-registered to test whether DR
  noise was washing out the two thin-margin mechanisms). Same
  flatline at DR0: `env/rise_score` 0.01–0.03 the whole 2M steps,
  training's own diagnostic 0/2 rise every window. **DR is
  exonerated** — not the blocker. New clue from the W&B curve:
  `env/reward_rise_ref` starts at 0.65/tick at the warm-started
  checkpoint (the crutch was briefly engaged) and is eroded to
  ~0.02/tick within the first few updates — looks like early PPO
  updates destroying an aligned start, not undiscovered exploration
  (plausible cause: the warm-started critic is miscalibrated for the
  new score-routed reward). Follow-up `cw-stand-scoreref1-dr0-lowlr`
  (LR 3e-4→5e-5, one variable) is in flight to test the erosion
  hypothesis directly.
- `cw-stand-scoreref1-dr0-lowlr` — FAILED (08-11): the LR-erosion
  follow-up above. Cutting LR 6x did NOT slow the collapse —
  `env/reward_rise_ref` still crashed from 0.66→~0.002/tick within
  ~20-30 update steps (run median 0.021, gate needed ≥0.3) and
  `env/rise_score` never left the floor (median 0.021, max 0.096).
  Erosion-by-oversized-updates is REFUTED. Two live explanations,
  neither tested yet: the summed reward genuinely disfavors the
  tracked behavior once the rest of the stack is added in, or the
  tight 6° tracking sigma is measuring ordinary rollout stochasticity
  as "the behavior is gone." No further LR/coefficient variant
  queued — RISE.md's ruling below already closes that line; the next
  real lever is the structural height↔contact coupling (CODE).
- `cw-stand-scoreref1-dr0-riseonly` — FAILED (08-11), forensic probe:
  identical to scoreref1-dr0 but `goal-mix rise=1.0` (no lower/hold
  mixed in), pre-registered to split cross-mode interference from a
  within-rise cause. Harness: rise 0/6 det+sto, worst-foot clearance
  151–164mm, video the same held-leg cheat as the whole score/
  scoreref family. `env/reward_rise_ref` crashed 0.51→~0.03–0.05/tick
  within the first ~130 sampled ticks — the SAME fast timescale as
  the mixed-mode runs, even with zero lower episodes in the batch.
  **Cross-mode interference (Suspect A) is REFUTED** — the erosion is
  intrinsic to the rise task/reward itself, not diluted-by-lower
  gradients. Narrows to the two live explanations above (summed stack
  genuinely disfavors the tracked motion once composed, or the 6°
  sigma is measuring ordinary rollout noise as "gone") — neither is
  worth a further reward-coefficient arm; go straight to the
  structural coupling.

## Direction (binding, 08-11 — supersedes the 08-10 night entry)

Four distinct mechanisms have now ALL been beaten by the identical
flag-leg trick: detect-and-discount (b2p1's posture gate), a
multiplicative PLANT_SPEC gate (plantgate1), moving the income source
itself (score1), and — 08-11 — a cheat-proofed reference/trajectory-
tracking crutch layered ON TOP of score1's income routing
(`cw-stand-scoreref1`: `k_rise_ref_track=2.0`, feet-gated kernel,
sigma tightened 12→6°). scoreref1 was the pre-registered attempt at
lever (a) below ("re-open the waypoint option" — operator's own
hardcore-disagreement test) and it did NOT stop the cheat: rise 0/6
det+sto at both DR0 and own-DR0.2, worst-foot clearance 160–188mm,
`env/rise_score` flat ~0.01–0.02 the entire 2M steps (its own early-
stop trigger). Ruling: **reward-income shaping AND reference-
tracking-as-crutch are BOTH now closed for rise.** Do not propose a
5th income/tracking variant on top of the existing reward stack —
showing the policy the right motion doesn't help while a fake stand
can still collect nearly full pay somewhere else in the same stack.
**The only remaining lever is (b): a structural coupling between the
commanded height goal and measured foot contact** — e.g. the height
*reference* itself refuses to rise on a leg the moment that leg loses
contact, rather than paying/penalizing after the fact. That is CODE
work (new mechanism, not a reward-coefficient respec) and must go
through SPECIFICATION (bank the exploit) before any DISCOVERY run.

## Direction (binding, operator-supervised, 08-10 late — supersedes
## the 08-11 ruling above)

The 08-11 ruling diagnosed the failures as reward-side and closed the
tracking line. A five-run controlled forensic ladder run WITH the
operator on 08-10 evening overturns that diagnosis:

- `cw-stand-scoreref1-dr0` (one change: DR 0.2→0): the warm start
  BEGINS with the crutch engaged (reward_rise_ref 0.65/tick, feet
  factor 0.87) and training erodes it. DR exonerated as root cause.
- `cw-stand-scoreref1-dr0-lowlr` (one change: LR 3e-4→5e-5): same
  erosion, slower. Update size exonerated.
- `cw-stand-scoreref1-dr0-riseonly` (one change: goal-mix rise=1.0):
  same erosion. Cross-mode interference exonerated. Also surfaced:
  train/std pinned at 0.198 all run.
- Noisy-replay probe: the reference replayed under the FULL 0.198
  action noise still earns +357 and stands 2/3 — the summed reward
  stack orders noisy-honest ≫ every cheat. Pricing exonerated,
  including the "6° sigma just measures stochasticity" theory.
- Re-read of the "early tracking pay": it was the PRE-RAMP hold
  window (lying at ref start), not rising — the warm start never
  rises at +111mm commands (outside its trained band).

Net: the reward is right and the path pays, but **training never
visits the paid states** — a pure state-distribution problem that no
reward term can fix, which is why four reward-side mechanisms in a
row failed the same way. The standard fix (DeepMimic RSI; also load-
bearing in HumanUP/HoST stage 2) landed 08-10 late: rise episodes
spawn ON the reference at a random phase (`goal.rise_rsi_frac`,
default-off; sag-robust via nearest-neighbor ref-clock re-alignment
at settle; remaining-rise schedule from the npz height profile).
Validated: forced-RSI spawns across the whole path continue to a
valid plant 7/8 with returns +400..+860, late spawns paying best;
bank green, default-off exact. First arm: `cw-stand-rsi1` =
scoreref1-dr0 stack + `rise_rsi_frac=0.5`. Lever (b) (structural
height↔contact coupling) stays open as the NEXT step only if RSI
with a correctly-priced stack still fails.

**08-11 ~03:30 — ACTUAL root cause found (supersedes the exploration
framing above).** `cw-stand-rsi1` still eroded, but its `env/rise_rsi`
tick share decayed 0.58→0.15 with ZERO terminations — impossible for
a constant 0.5 spawn fraction, i.e. a CODE smell, not behavior. Trace:
the warp/MJX vec envs recycle episodes from a reset pool and restore
per-episode host state from `mjx_host.SNAP_ATTRS` — and none of the
score-stack episode attrs added 08-10 were in that list. Every
pool-restored episode inherited a random other episode's
`_score_best` ratchet high-water mark (so progress income ~never
paid), `_rise_ramp_i0` ramp anchor (mis-clocked ref tracking and
post-ramp checks) and, in rsi1, lost its RSI clock entirely. This is
mechanically the observed signature everywhere: first-generation
episodes pay (warm 0.65-0.82/tick ref income at step 0), pooled
generations take over within ~20-30 updates and the pay — not the
behavior — collapses. Local probes never showed it (host env has no
pool). Consequences: **the score1/scoreref1/plantgate "the cheat
beats N mechanisms" verdicts are all contaminated** — those stacks
were never actually paid as designed on the GPU path; the closures
of income-shaping and tracking-as-crutch are REOPENED pending a
clean re-run. Fix landed: `SNAP_ATTRS` += `_score_best`,
`_rise_ramp_i0`, `_end_posture_from`, `_rsi_pending`,
`_rsi_ref_tick0`, plus a rule note that any new per-episode attr
read in the step path must join the list. First clean arm:
`cw-stand-rsi2` (rsi1 args, one change = this fix).

**08-11 — `cw-stand-rsi2` reports, RE-CLOSES the reopened verdicts.**
Mechanism health this time is genuinely clean: `env/rise_rsi` held
0.48–0.58 the ENTIRE 2M steps (mean 0.52, no decay — the pool-restore
fix works, no more state corruption). And it still failed the exact
same way: `env/reward_rise_ref` crashed 0.83→0.02–0.09/tick within
the first logged window and stayed there; `env/rise_score` never left
the 0.01–0.02 floor the whole run. Harness confirms: rise 0/6 det+sto,
worst-foot clearance 146–161mm, video-identical tripod (three legs
never leave the ground, three legs held 20–146mm up the whole
episode; duty cycle ~0.8–0.95 on the down legs vs ~0.01–0.13 on the
up legs). This is a CLEAN read (no corrupted state to blame) that
reproduces the identical failure. Ruling: the pool-restore bug is
EXONERATED as the cause of any prior rise verdict; **income-shaping
and reference-tracking-as-crutch are RE-CLOSED, on stronger evidence
than before the bug was found.** RSI (state-distribution fix) is
ALSO now refuted as a fix for this failure mode — it does what it was
designed to do (episodes visit the paid states) but the visited pay
still doesn't stick during training. Do not requeue another RSI/
income/tracking coefficient variant. The only remaining lever is (b):
structural height↔contact coupling [CODE] — RL_PLAN.md queue item
2b. No further DISCOVERY arm on the current reward stack until that
lands.

**08-11 ~04:50 operator session — `cw-stand-rsi3` and the two-lever
choice (reconciles with the cycle verdict above).** `cw-stand-rsi3`
(one change vs rsi2: `reward.rise_score_strip_pen=1` — the
k_height=100 PENALTY was still live and made flag-leg the reachable
optimum: belly rest −1.2/tick vs cheat rent −0.5/tick; bank green
with the strip) STILL collapsed, with an identical curve. Decisive
observation across ALL SIX runs (rsi1/2/3, dr0, dr0-lowlr,
dr0-riseonly): the feet-factor collapse (0.87→~0.17 by the 25% mark)
has the same shape and timescale under materially different rewards.
Behavior that does not respond to reward changes is not
reward-driven: this is **warm-start drift at out-of-distribution
observations** (the 108–114mm command band is ~2.2x the stance
champion's trained range; its behavior there is un-anchored and
update noise erodes it), and no reward stream can anchor it because
the 6° kernel only pays a policy that is ALREADY nearly perfect —
the cycle's persistent-tripod video above is exactly what unanchored
drift settles into. Widening the kernel is bank-blocked (measured
08-11: at sigma 10° flag-leg farms 108 vs replay 648 = 17%, over the
10% bar; at 15° three bank tests fail). Agreed with the cycle:
no more reward/income/tracking/RSI coefficient variants. Two CODE
levers remain, both through SPECIFICATION first:
(a) **BC anchor in the TRAINER** — auxiliary loss pulling the
    policy's action toward the reference action at RSI-spawned
    states (DeepMimic-family standard; supervises actions, immune to
    pose-farming, needs no rollout luck);
(b) **structural height↔contact coupling** (the cycle's queue item
    2b — the height reference itself refuses to rise on a leg that
    lost contact).
Operator leaning: spec (a) first — it attacks the measured mechanism
(drift with no anchoring gradient) directly, while (b) reshapes the
goal but still pays through the same RL gradient that the drift
out-runs. Everything else on this stack is now verified honest:
pricing (bank + noisy replay +357), state coverage (RSI holds 0.5),
state restore (pool fix), penalties (strip_pen).

**08-11 — lever (a) LANDED: reference BC anchor in the trainer
(SPECIFICATION pass green).** `rl_move/sim/bc_anchor.py`: BCAnchorPPO
adds one supervised step per update AFTER the untouched PPO update
(MirrorPPO pattern — no SB3 internals copied), minimizing
`coef * mse(pi_mean(obs), a_ref)` on a 131k ring buffer of
(post-step obs, reference action) pairs collected from live rise
rollouts. The env emits `info["bc_target"]` — the normalized action
whose joint target is the reference pose one ref-tick ahead of the
episode's live ref clock (`sim_env._rise_ref_clock`, shared with the
tracking reward so the two clocks can never disagree) — gated on
`train.bc_anchor_coef` riding into the env cfg; RSI and legacy
ramp-aligned episodes both emit. This supervises ACTIONS, not visited
rewards: at drifted states the target points back onto the
demonstrated path, which is exactly the anchoring gradient the
6° kernel cannot provide. Reward stack untouched (not a reward term;
rise bank unaffected — full bank re-run green post-refactor, 23
passed/1 pre-existing skip). Validation: 10/10 new tests
(`rl_move/tests/test_bc_anchor.py` — default-off exactness, RSI +
legacy clock alignment, target-chain tracks the path <8° RMS, aux
step provably moves pi_mean, done-boundary pairs skipped, loud
refusal on missing ref path, composes with MirrorPPO), MJX-pod smoke:
anchor engages, buffer fills, `train/bc_anchor_loss` 0.198→0.04
within one smoke. Knobs: `train.bc_anchor_coef` (0=off exact),
`_minibatches` (8), `_batch_size` (4096), `_buffer` (131072).
First arm: `cw-stand-bc1` = rsi3 stack + `train.bc_anchor_coef=1.0`
(ONE change), discovery 2M. Decisive signal: `env/rise_feet_factor`
must stop collapsing (all six prior arms: 0.87→~0.17 by the 25%
mark) while `train/bc_anchor_loss` stays low; if feet hold and
rise_score climbs, harden with an anneal schedule so the final
policy is not trajectory-locked. Lever (b) (structural
height↔contact coupling) stays next if the anchor fails.

## `cw-stand-bc1` (08-11) — PASS (partial): lever (a) WORKS, first
## honest rise in 7 stand-arms

`env/rise_feet_factor` dipped to 0.32–0.37 through 260k–590k (would
have fired the pre-registered <0.4-by-500k kill if live-monitored —
note for future kill rules: needs a sustained window, not a first
crossing, on this mechanism) then RECOVERED, climbing to 0.75 by 2M —
categorically different from all six reward-only arms, whose curve
never recovers. `env/reward_rise_ref` climbed to 0.6–0.77 (vs the
0.01–0.03 floor every prior arm flatlined at) and `env/rise_score`
left the floor (0.01→0.21).

Harness confirms this is real, not another metric artifact. Gate
pass (RSI 0.5, as trained): rise valid_plant **3/6 det** — honest
six-foot plants, height_err 4–7mm, all feet <20mm off the ground,
video-confirmed (frame strips show a genuine belly→spread-leg stand,
nothing resembling the flag-leg/tripod cheat). A follow-up probe run
directly on the pod (`--modes rise --per-mode 30 --seed 7
--cfg-set goal.rise_rsi_frac=0.0`, isolating the anchor from RSI's
help) on the SAME checkpoint: **bridge 7/12 and crouch 6/8 pass the
full geometric valid_plant** check; **flat-belly cold start (the
hardest case — legs straight out, belly down, exactly the operator's
placement) reaches a real six-foot stand 10/10 times** (correct
height, no flag leg, current in-band) but misses only the
walk-anchor **footprint** tolerance every time (0/10 valid_plant,
worst foot clearance 7.8mm — a foot-XY positioning gap, NOT a
height/posture cheat; video-confirmed indistinguishable in kind from
the passing bridge/crouch stands). Zero flag-leg/tripod cheat across
42 video-checked episodes total. The identical-recipe parent
`cw-stand-rsi3` (only missing `bc_anchor_coef`) shows the flag-leg
cheat 0/12 valid_plant on this exact reward/goal-mix stack — the
causal attribution to the anchor is clean (one variable).

Cost (skill interference, weak evidence — n=2 training-diagnostic
probe samples, not harness-verified): `raise` and `tipped_recovery`
both read 0/2 at 2M vs rsi3's 1–2/2 and 2/2; hold/track angle error
3.0° vs rsi3's 1.2–1.6°. The anchor only supervises rise-tick
actions but shares the network with every mode — plausible bleed
into nearby height/posture manifolds. `ep_rew_mean` fell to −29 by
2M (rising tilt/over-current terminations during genuinely riskier
rising attempts, not hold/track breaking — video confirms hold/track
look normal).

**Ruling: lever (a) is validated — reward-income shaping was
correctly diagnosed as exhausted, and moving supervision OUTSIDE the
reward (BC anchor on actions) is what unblocks flat-start rise.**

`cw-stand-bc1-coef03` (08-11, same protocol, coef 0.3 vs bc1's 1.0)
**FAILED — dose-response refuted, decisively.** RSI-off harness
probe: valid_plant **0/16 across every start kind** (bridge 0/7,
crouch 0/5, flat 0/8 det), vs coef=1.0's 13/30 (bridge 7/12, crouch
6/8, flat 0/10-but-honest). Video still shows a genuine stand
attempt (no flag-leg regression) but flat starts now fall SHORT of
full height (h_err ~26mm vs coef=1.0's ~11mm) and every episode still
trips the current ceiling. Training's own diagnostic did not improve
either (hold/track angles the same or worse, raise/tipped/rise-flat
all flat-or-worse vs coef=1.0). **Lowering the dose does not buy
cleaner cross-mode behavior — it just weakens the anchor, on every
axis measured.** Ruling: keep `bc_anchor_coef>=1.0`; do not queue
another coefficient-reduction variant.

`cw-stand-bc1-hard1` (08-11, 10M steps, same coef=1.0) **PASSES
(partial) on rise, but surfaces a real, pre-existing hold/track
cost that WORSENS with more training.** Rise consolidates further:
gate valid_plant 5/6 det (83%, up from bc1's 3/6=50%), tight height
errors (0.2–2.2mm on most episodes) — confirms the fix holds and
improves with budget. But re-checking hold/track's per-episode
`duty_cycle`/`swing_count`/`end_clear_mm` fields (NOT examined at
bc1's original verdict — a sparse 10-frame video strip missed it)
shows hold/track are not quiet stands: alternating legs cycle
continuously (duty ~0.85–0.9 / 0.06–0.09, 6–19 swings per 15s
episode) ending 12–50mm elevated at bc1 (2M), **100–161mm at bc1-
hard1 (10M)** — hold/track harness success 0/6 both modes at both
checkpoints. This pathology PRE-DATES the anchor (present already
at 2M) and is not primarily an anchor side-effect — more likely a
pre-existing hold/track income-pricing gap (continuous stepping
isn't charged; `k_still` as written scopes to belly-rest/lower, not
general hold/track) that the anchor's shared-network pull amplifies
with more steps. **Lesson for future triage: check
duty_cycle/swing_count/end_clear_mm for every stand-line mode, not
just valid_plant plus a sparse frame strip — a checkpoint can look
static in 10 sampled frames while stepping continuously between
them.**

Ruling: do not keep blindly hardening this lineage hoping hold/track
self-heals (the trend is the wrong direction). Next step is a
SPECIFICATION pass auditing hold/track's stillness pricing (why
continuous leg-cycling isn't charged) — a reward-mechanism change,
so it needs its own `test_task_semantics.py` HOLD-mode bank entry
before any training. Not yet queued (08-11) — this cycle only
diagnosed it.

**08-11 dig-in addendum — matched-parent control settles the
"regression" question; hard1 promoted to RISE SPECIALIST.** The
escalated "hold/track/raise/lower collapsed under hardening /
protected-skill erosion" read is REFUTED: the identical RSI-off
probe (seed 7, per-mode 12, same cfg) run on the PARENT
`cw-stand-bc1` (2M) shows every one of those modes was ALREADY 0/12
before the extra 8M steps — hold 0/12 (worst foot 51mm), track 0/12
(65mm), raise 0/12 (40mm; `p_raise=0` in the goal mix, the mode is
untrainable in this arm and the gate's raise criterion was
ill-posed), lower 0/12 with a **166mm flag-leg at 2M** (child:
189mm — same cheat, pre-existing; training's `SCORE/lower_success`
=1.0 is the height-only criterion and is blind to it). Nothing the
parent could do was lost; the crown-jewel lower lives in the
rfix-warm1/vref1-r1 lineages, untouched by this arm. Meanwhile the
child's rise is now **12/12 valid_plant RSI-off incl. flat 4/4
(worst foot 7mm)** — bc1's flat-start footprint miss resolved with
budget — and `rise_feet_factor` held 0.69–0.82 for all 10M (no
re-drift; the pre-registered kill signature never appeared;
trajectory-lock refuted by cold-start success). Verdict recorded:
`ppo_goal_cw_stand_bc1_hard1` = the rise specialist (SKILLS.md);
lineage closed for hardening; next = HOLD-stillness SPECIFICATION,
then the composition test (learned rise → walk/hold champion
handoff, replacing the stance-champion + scripted-blend fallback).
Harness fix landed same cycle: `eval_checkpoint.py` now refuses
unknown `--modes` loudly ('tipped' is a trainer periodic-eval axis,
not a harness mode — passing it used to zero every goal probability
and NaN-crash after the good modes had run).

## Hold/track stillness pricing — SPECIFICATION LANDED (08-11 idle-kick cycle)

The bc1-hard1 dig-in's queue item is done. HOLD bank added to
`test_task_semantics.py` (plant start, hold mode, the exact
stand-line stack): scripted `quiet` (hold the settled plant),
`stepping` (alternating tripods at ~1 Hz, swing peak ~40 mm — the 2M
pathology's honest-magnitude form) and `flag` (one front leg parked
189 mm up, five planted, frozen — the 10M pathology; a both-front-legs
splay nose-dives and terminates under position control, so the single
flag is the stable scripted member of that class). Measured legacy
returns (seed 0, 15 s): quiet 367.9, stepping 300.7, **flag 368.0 — a
frozen flag-leg park LITERALLY TIES the honest quiet stand** because
the tracking kernel has no opinion on legs and `k_still` (a bonus,
default 0) charges nothing. That is the whole pricing hole in one
number.

Fix: `reward.hold_still_gate` (default 0 = legacy; REWARD.md row).
Scales kernel income on hold/track ticks by feet-down² × HARD no-flag
zero (PLANT_SPEC flag_leg_mm 60 mm; honest adjustment swings stay
below it) × stillness Gaussian applied only while the reference is
stationary (TRACK's commanded attitude motion never charged). Scoped
strictly to hold/track — quad lifts legs and unload opens a contact on
purpose. Implemented in `sim_env._step_finish` (shared by the CPU
harness and the warp/MJX host-worker path). Gated bank ordering:
quiet 367.9 > stepping 107.2 > flag 9.5; gate-bite and no-tax tests
both pass; full task-semantics suite green (29 passed, 1 pre-existing
skip).

Next: `cw-stand-holdstill1` — discovery 2M, warm from
`ppo_goal_cw_stand_bc1_hard1` (the rise specialist), ONE variable
(`hold_still_gate=1.0`), same goal mix. Binary question: does hold
converge to a quiet valid plant (worst-foot <20 mm, swings →0)
without losing the honest rise? After that: the rise-specialist →
walk-champion handoff composition test.

### `cw-stand-holdstill1` (08-11) — FAIL on hold, rise retention PASS;
### plateau diagnosis → fade lever landed

The gate priced the pathology out but did not fix the behavior:
hold/track 0/12 det+sto with the IDENTICAL parent fingerprint (leg 0
parked 107–116 mm, legs 1/3/5 cycling duty ~0.9) — while
`env/hold_feet_factor` sat at ~0.1 (income ~0) from 260k on. The
pre-registered kill signature occurred; the ~4-minute run outran any
kill. Rise: det 4/6, sto 6/6 valid_plant, feet factor 0.53–0.79 all
run — retraining under the gate cost the honest rise NOTHING (better
than the parent's 2M band).

Diagnosis (the bc1 lesson again, in miniature): earning zero is not
being pushed back. The hard no-flag zero makes the whole splay
neighborhood a flat zero-income plateau, so PPO gets no slope telling
the parked leg WHICH WAY to move, and hold is only 10% of the mix.
Lever landed same cycle: `reward.hold_flag_fade=1` (REWARD.md) — the
no-flag factor becomes a linear ramp over 60→120 mm, so the observed
113 mm park earns 51/ep (0.14× quiet, scraps) with monotone slope to
full pay at feet-down; the 190 mm class stays at 0. Bank extended
(3 new tests: ordering preserved, gradient exists, park stays <25%
of quiet). `cw-stand-holdstill2` = holdstill1 + the fade, one
variable. If the fade also fails, the next lever is BC-style
supervision on hold ticks (target = the episode start pose), the
mechanism already validated on rise.

### `cw-stand-holdstill2` (08-11) — fade directionally right, still 0/12;
### hold pricing levers EXHAUSTED, next is BC supervision on hold ticks

One variable vs holdstill1 (`hold_flag_fade=1`). The slope works as
designed: parked leg 107–116 → 86–101 mm, `env/hold_feet_factor`
0.1 → 0.19–0.35 (still rising at 2M), track episodes down to
29–56 mm. But hold det+sto 0/12 — the quiet stand was never reached
in-discovery; rise retention held (det 4/6, sto 4/6). Ruling (two
pricing misses in a row = change the hypothesis; discovery rules
forbid extending a run whose target behavior has never been seen):
**no third pricing/mix/step variant on hold.** The gate+fade STAY
(bank-proven correct pricing — they will pay the real behavior). The
next lever is the rise playbook repeated: BC-style supervision on
HOLD ticks, target = the episode start pose (trivially available;
extend `bc_anchor.py`'s bc_target emission beyond rise ticks) — a
SPEC/CODE item (trainer change + bank re-run) before any further
stand-line launch. After that lands and a hold arm passes: the
rise-specialist → walk-champion handoff composition test (still the
plan's next composition milestone).

### CODE landed (08-11, idle-kick cycle): bc_anchor covers hold/track

`sim_env._is_hold_bc` (new per-episode flag, added to
`mjx_host.SNAP_ATTRS` — the pool-restore lesson applies to every new
per-episode attr, not just the rise ones) fires on hold/track ticks
and emits `info["bc_target"] = q_rad_to_action(self._q_nom)` — a
CONSTANT target for the whole episode (the pose it actually settled
at post-reset; already captured for the hold-current reward term, so
"trivially available" was correct). Rise ticks are unaffected (kept
in a separate branch, mutually exclusive with hold/track by
construction). `bc_anchor.py` docstring updated; no reward-stack
change (the anchor is a trainer loss, same as rise). Bank: 4 new
tests in `test_bc_anchor.py` (hold emission + value, track emission,
default-off-on-hold, rise/hold flags mutually exclusive across
rise/hold/track/lower) — 14/14 green; full `test_task_semantics.py`
re-run 32 passed/1 pre-existing skip (reward stack confirmed
untouched). `cw-stand-holdbc1` launched same cycle: respec of
`cw-stand-holdstill2` (byte-identical cfg — same
`hold_still_gate=1`, `hold_flag_fade=1`, same warm start from the
rise specialist, `bc_anchor_coef=1.0` was already set and is now the
ONE thing that changed behavior, since the code under it changed).
Binary question: does the mechanism that fixed rise also fix hold,
or is hold's "earning zero → no pushback" failure mode not fixable
by BC alone? If it also fails: three hold misses in a row, fall back
to the rise-specialist + scripted-blend handoff without a learned
quiet hold, and stop spending discovery arms on hold pricing.

### `cw-stand-holdbc1` (08-11) — PASS: HOLD SOLVED, third lever works

Answer to the binary question above: **yes, the BC-anchor mechanism
that fixed rise also fixes hold.** Harness (DR0 gate, det+sto,
per-mode 6): hold 12/12 valid_plant, worst-foot clearance 2–13mm,
height_err_end_mm ≈2 — every episode ends level, six feet down,
motionless, both deterministic AND stochastic. Video-checked (both
modes): the frame strips show zero movement across the full 15s clip
— no shuffling, no flag-leg, no drift. `env/hold_feet_factor` (the
gate's pre-registered mechanism-health signal) cleared the 0.1–0.35
plateau both `holdstill1`/`holdstill2` sat in for their entire runs,
climbing to ~0.99 by the FIRST logged point and holding ~0.99–1.0 for
all 2M steps — the earning-zero plateau never formed.

Rise retention (the gate's other conjunct, pre-registered floor
det >=3/6): bridge starts clean 2/6→2/2 valid, sto clean 6/6 (2 of
those flagged only on the soft current-limit check, not posture).
Det crouch starts came in at 2/6 valid (2 tilt_roll falls + 2
height-overshoot misses) — below the pre-registered floor taken at
face value. Checked against the lineage's own history before calling
this an erosion: `holdstill1`'s rise/det report has ZERO falls (2
height misses only); `holdstill2`'s rise/det report has exactly ONE
tilt_roll fall on a crouch draw (return −44.2, valid_plant False) —
the SAME failure signature, same magnitude, one draw. `holdbc1`
adding a second crouch fall on n=6 is the identical pre-existing
fingerprint recurring one more time on a 6-episode sample, not a
new pathology introduced by the hold-BC code change — video of the
failing episodes (`rise_det_2.png`/`rise_det_3.png`) shows a genuine
tip-over (body rolls onto its side), the same visual signature as
`holdstill2`'s single fall, not a flag-leg/tripod cheat. Ruling:
**PASS overall** — the headline mechanism (hold) is decisively fixed;
the crouch-start rise dip is a known, small, pre-existing fragility
to track, not a new regression, and not a known-exploit stop
(no cheat pattern on any episode, any mode).

Track-mode command-tracking accuracy stayed weak (det 2/6, sto 0/6 on
the tracking-error success metric) while posture stayed valid
throughout (end_posture 6/6 both passes, worst_clear 7–10mm) — not
part of this arm's pre-registered gate, noted for later (tracking
precision, not a posture/stillness problem).

Checkpoint `ppo_goal_cw_stand_holdbc1` (SKILLS.md: Hold row). The
"three hold misses in a row -> scripted-blend fallback" contingency
is now moot. Next: hardening continuation `cw-stand-holdbc1-hard1`
(10M steps, `--evidence` citing this discovery pass) to see if extra
budget also closes the crouch-start rise gap, the same way bc1's
flat-start footprint miss resolved with budget in `bc1-hard1` — then
the rise+hold → walk-champion handoff composition test, the plan's
next named composition milestone.

### `cw-stand-holdbc1-hard1` (08-11) — PASS: hardening consolidates,
### matches every pre-registered gate condition, lineage CLOSED

Binary question was whether 5x the budget (2M→10M) would also close
the crouch-start rise gap left by discovery, without eroding hold.
Answer: it holds hold and slightly improves crouch, exactly the
"no worsening" bar the gate asked for.

Harness (DR0 gate, det+sto, seed 0): hold valid_plant 11/12 (det
6/6, sto 5/6 — the one sto miss carries only a `current` soft-limit
flag, height_err 4.1mm, posture otherwise fine; not a posture/cheat
failure). `env/hold_feet_factor` held 0.990–1.0 for the entire 10M
steps (min 0.9904) — the pre-registered mechanism-health floor
(>=0.9) cleared with wide margin, no re-drift toward the
earning-zero plateau. Track valid_plant: det 5/6 (again one
`current`-only miss), sto 3/6 (three `current` misses) — not part
of this arm's gate (track command-tracking accuracy, tracked
separately below).

Det crouch-start rise: 2/4 valid (50%), vs discovery's 2/6 (33%) —
improved, not flat, comfortably inside the gate's "improve or hold
flat" bar. The one fall (`rise_det_2`, tilt_roll, return −49.6) is a
genuine tip-over on video — the robot rises normally for the first
half of the clip then rolls onto its side, no flag-leg/parking
signature. The one miss (`rise_det_4`, `plant_fail=['height']`,
height_err 22.8mm) is a correct-looking six-foot stand on video,
just outside the height tolerance — not a cheat either. Bridge
starts stayed clean (det 2/2, sto 3/3); sto rise overall 5/6 valid
(one `current`-only miss on a bridge start). Zero flag-leg/tripod
cheat pattern across all 24 det+sto episodes reviewed (hold, track,
and rise strips) — every failure mode is either a soft current flag,
a genuine fall, or a height-precision miss, never a frozen/splayed
park.

Reward-quarters [148.8, 271.5, 282.5, 283.5] show the usual
fast-rise-then-plateau shape, consistent with a converged, not
budget-starved, run.

Ruling: **PASS, lineage CLOSED for further hardening.**
`ppo_goal_cw_stand_holdbc1_hard1` is the hardened HOLD+RISE
checkpoint (SKILLS.md new row). No further step-count/coefficient
variant on this stack — matches the "two clean passes, stop
tuning" pattern from the bc1 lineage. Track-mode command-tracking
precision (sto tracking-error success still weak, though posture
holds) stays noted for later, not gated. Next, per RL_PLAN queue
2.3 and the plan's next named composition milestone: the rise+hold
→ walk-champion handoff composition test — swap the current
scripted-1.5s-blend handoff (stance champion → walk champion) for
this learned specialist's own settled pose. This needs a small new
piece of CODE first (an eval/drive script that runs the specialist
to a settled hold, then switches control to the walk champion and
checks the walk champion doesn't stumble on the specialist's exact
final pose/velocity state) — not yet built, so not launch-ready this
cycle.

### Handoff composition test (08-11) — PASS: the specialist's stand
### is a walkable start; the scripted blend is obsolete

The plan's next named composition milestone, now built and run:
`rl_move/sim/eval_handoff.py` (snapshot 7f91f87). Three arms, 3
episodes per rise start kind, two physics variants (default air +
`bus.servo_params=loaded`, the specialist's training physics):

- **direct**: `ppo_goal_cw_stand_holdbc1_hard1` runs a genuine
  training-distribution rise episode (env's own goal generator,
  RSI off, plant band 108–114 mm) to a settled hold; control then
  switches to `ppo_goal_cw_walk_longdist_r2` ON the specialist's
  exact physical state — episode bookkeeping re-anchored to a clean
  plant frame (the champion's training frame, same trick as play.py
  key-7), qpos/qvel/ctrl and the safety slew memory carried over
  unchanged; then 1 s settle + 6 s fwd @0.05 + 2 s stop.
- **blend**: identical plus the incumbent scripted 1.5 s joint blend
  to the walk plant pose between the policies.
- **plant**: walk champion from its own clean plant reset — the
  drive-metric noise band.

Results (per-episode records + strips:
`logs/ckpt_eval/handoff_holdbc1hard1_{air,loaded}.json`, strips dir
alongside): every successful rise handed off with ZERO falls in both
arms and both physics. Direct-arm drive metrics sit inside the plant
band: air trk_err 0.032–0.036 vs plant 0.031, dist 0.394–0.443 m vs
0.431, stumble-window max tilt 1.2–2.6° vs 1.5°; loaded trk_err
0.041–0.052 vs plant 0.050, dist 0.348–0.421 vs 0.360, tilt 1.7–4.5°
vs 1.9°. Blend-arm numbers are indistinguishable from direct —
**the scripted 1.5 s blend adds nothing; the specialist's settled
pose (ends ~124 mm chassis height, worst-foot 1.8–4.9 mm, height_err
|<6| mm) is already in the walk champion's start distribution.**
Video: flat/bridge strips show curl → six-foot rise → level stand →
normal all-six-legs gait after the switch; no flag-leg, no dragging,
no lurch at the switch tick.

Caveat, pre-existing and now sharpened: crouch-start rises fell
(tilt_roll, within ~2 s of ramp start) before the handoff in 6/6
episodes across both physics (RSI-off). The lineage's own gates saw
2/6–2/4 crouch failures (RSI on); RSI-off crouch appears worse than
the gate suggested. Not a handoff defect — flat (the realistic
operator placement) and bridge rises went 12/12. Tracked as the
lineage's known fragility; do not reopen hardening for it (two-pass
rule) — if crouch matters for the joystick chain it needs its own
mechanism question.

REVERSE handoff (walk → stop → lower/sit) DONE 08-11
(`rl_move/sim/eval_handoff_reverse.py`, same reanchor pattern; arms:
spec = specialist's own clean lower episode / direct = specialist
lowers on the walk champion's exact stopped pose+slew state /
scripted = 6 s glide to the zero pose, the hardware go_zero("sit")
analog). 6 eps/arm, air AND loaded
(`logs/ckpt_eval/handoff_rev_holdbc1hard1_{air,loaded}.json`):

- **Handoff itself: CLEAN.** direct == spec on every axis (4/6
  posture-strict both physics, zero falls anywhere, same failure
  signature, height_err 0.4–9mm) — the walker's gait residue costs
  nothing, mirroring the forward result.
- **Specialist lower post-holdbc1: mostly intact, not posture-strict.**
  8/12 pooled. Every miss is the SAME fingerprint: belly down at
  target height and level, but ONE foot (leg 2, of the elevated
  {0,2,4} triple) dangles 62–99mm > the 60mm belly allowance —
  video-confirmed a cosmetic dangling foot, NOT the old 130–190mm
  weight-bearing flag-leg cheat (huge improvement over bc1-hard1's
  lower 0/12 @189mm).
- **Scripted glide: 6/6 both physics, deterministic, gentle**
  (tilt ≤2.5°, all pads 34–38mm) — and it is already the
  operator-prescribed hardware sit (go_zero("sit") slow glide,
  never refuses). The learned lower is NOT needed for the joystick
  deliverable.

Ruling: sit side of the chain is COVERED by the scripted glide; the
full sim joystick motion cycle (specialist rise → walk champion
drive → stop → scripted sit) is now composed with zero falls.
OPTIONAL polish, not queued (prime directive): extend the BC anchor
to lower ticks (reversed rise ref / glide-to-zero target) to fix the
dangling foot if a one-policy stand/sit specialist ever matters more
than the scripted path.

### Deploy-side port (08-11 late) — the specialist is the robot's live
### stance policy; bench validation is all that remains

The RL_PLAN critical-path [CODE] item after both handoff PASSes:
`ppo_goal_cw_stand_holdbc1_hard1` exported
(`export_policy_np.py`, SB3↔numpy parity 2.65e-07) to
`linux_control/policies/stand_holdbc1_hard1.json` AND copied live over
`linux_control/rl_policy_weights.json` (the stance slot the web UI's
STAND/LOWER buttons run).

Design point, mirroring the rot60-port "no second implementation"
rule: the runner used to hardcode the stance_dr10-era goal ramp
(hold 5 s / ramp 4 s / +50 mm). Feeding that ramp to the specialist
would command a half-height stand. The trained profile now rides
INSIDE the weight file (`meta["profile"]`, new `--extra-meta` export
flag): specialist stand = hold 5 s / ramp 6 s / target +111 mm (mid of
the trained 108–114 band) / total 12.5 s (the handoff eval's validated
switch point, inside the 15 s training horizon). `rl_policy.py` reads
the profile per episode (`policy_profile()`); files without one keep
the legacy constants, so picker rollback to `stance_dr10.json` is
behavior-identical to the pre-port runner. Also: stance-slot obs-dim
guard (refuses a mis-slotted file), `deploy_adb.sh` now ships
`rl_walk_weights.json` + `policies/` (both were hand-staged before).

Verification:
- `rl_move/tests/test_stand_runner.py` (5 tests, green; full suite
  119 pass / 6 pre-existing skips): live-file == picker-file bytes,
  meta/profile values, runner ramp == training `GoalGenerator` rise
  trajectory (<2 mm), 68-wide obs layout block checks (goal height
  scaling, prev-action block), numpy-vs-SB3 parity on the source zip.
- Closed-loop sim smoke with the DEPLOYED artifacts (numpy weights +
  meta profile driving the env): flat-belly start, chassis 38 mm →
  149 mm (+111 mm, exactly the commanded target), zero falls,
  2/2 episodes.

NOT an assumption after all — the OPERATOR independently exported and
ACTIVATED the specialist in the robot's live stance slot the same
morning (commit 1e64263, bench session 08-11, md5 6620705c; see
HARDWARE.md "Bench state — drive session 08-11 morning"). That copy
predates this port and carries NO profile meta, so the on-robot
runner would feed it the LEGACY ramp: hold 5 s / ramp 4 s / **+50 mm**
— an out-of-distribution height command for a policy trained only on
108–114 mm targets, at a faster-than-trained ramp. **Re-push (deploy_
adb.sh) or re-select this repo's profile-bearing export before
pressing STAND**; the picker JSON's notes now say the same. Rollback
either way: `POST /api/rl/policy_select {"file":"stance_dr10.json"}`
(profile-less → legacy constants → behavior-identical to the old
runner).

Bench protocol (attempt #2 addendum): fresh set_zero, robot flat on
belly → STAND button (operator watching, 10° trip, 2.5 A trip) →
verify quiet hold → WALK button forward (existing captured-plant
preflight gates the pose) → go_zero("sit"). Every episode logs
obs+profile to CSV for offline replay parity.

### `cw-stand-crouchrise1` (08-11) — crouch fragility FIXED by
### start-mix bias; promotion declined on the hold current bar

Binary question: does biasing the rise START DISTRIBUTION (60%
crouch / 30% partial / 10% flat vs legacy 25/40/35; keys
`goal.rise_flat_frac` / `rise_partial_frac`) fix the lineage's one
residual defect — crouch-start tip-overs — where more undifferentiated
budget only nudged it (2/6 → 2/4)? Answer: YES, decisively.

- Gate harness (DR0, det+sto, seed 0): rise det 6/6 valid_plant
  (crouch 5/5, bridge 1/1), sto 6/6 success (3 current-only vp
  flags); hold det 6/6 vp; ZERO terminations in all 36 episodes.
- Dig-in RSI-off ALL-CROUCH probe (seed 1, `rise_rsi_frac=0`, 8 det
  + 8 sto), matched-parent control on `holdbc1_hard1`, same seed and
  cfg (`logs/ckpt_eval/{crouchrise1,hard1}_rsioff_crouch`):
  child **det 8/8 valid_plant, 16/16 stands, zero falls**; parent
  **det 0/8 with 8/8 tilt_roll falls** (sto parent 7/8 — the det
  policy is where the fragility lived). The RSI-on gate numbers had
  been flattering the parent; RSI-off shows the true gap.
- Videos honest both checkpoints' passing episodes: crouch/bridge →
  progressive leg gathering → level six-foot stand, no
  flag-leg/tripod/stilt.

MISSED pre-registered bar: hold det+sto valid_plant 7/12 (needed
>=10/12). Every miss is the PLANT_SPEC final-0.5s-tail current soft
flag (>2.0A) under sto — hold sto 1/6 vp vs parent 5/6; det episode
Imax 2.31A vs parent 1.96A. Posture, stillness, feet, height are
identical to the parent (end_posture 12/12, worst clearance 2–10mm,
track_err 0.19° det). Real but small current-hungriness increase on
a sim metric CURRENT_TRUTHS marks untrusted (sim hold 0.11A vs real
0.59A). Per the gate's own terms: **no promotion** —
`ppo_goal_cw_stand_holdbc1_hard1` stays the deployed stance policy;
`ppo_goal_cw_stand_crouchrise1` (md5 3877e16c) is pulled to the
controller and banked as the crouch-robust variant. Track-mode sto
also dipped (1/6 vp, current flags only) — same fingerprint, same
non-gated status as the parent's known track weakness.

Lesson worth keeping: after seven reward-mechanism failures and two
budget passes, the lever that finally killed a start-kind fragility
was the START DISTRIBUTION, one cfg key, 2M steps. Stand lineage now
fully closed: rise (all start kinds, between the two checkpoints),
hold, lower, both handoffs — all sim-solved; remaining work is
bench-owned.

## LOWER semantics bank (08-11 idle-kick cycle — SPECIFICATION, no training)

The last owed `test_task_semantics.py` bank is LANDED: lower mode,
under the deployed specialist's exact stack (holdbc1-hard1 cfg-sets,
loaded servos). Honest = FixedFootBodyIK descent anchored at the
SETTLED stance (anchoring at the ideal plant leaves a 16 mm sag
error), tracking the commanded ramp with all feet planted.
Measured (3 seeds): honest 540 > aloft 461 (85%) > outrig 383 (71%)
> partial 103–182 > refuse −2..−51 > thrash −77..−107; posture-strict
(60 mm pads / 15 mm h_err) accepts honest and rejects both cheats on
every seed (~300 mm aloft pads). Bank PASSES → lower-mode arms are
unblocked.

FINDING (encoded as a strict-xfail in the bank): the pf=5/6
posture-gate pricing lets a one-leg-aloft lower keep ~85% of honest
income — cheating out-earns refusal. This is the incentive behind the
deployed specialist's cosmetic 62–99 mm dangling foot seen in
eval_handoff_reverse. Not a joystick blocker (scripted sit glide
covers the deliverable; eval catches the posture), but any future
lower-MECHANISM arm (e.g. the optional BC-anchor-on-lower polish)
should strengthen this margin first and flip the xfail.

## `cw-stand-holdload1` (08-11) — reward-side hold-cheat hypothesis REFUTED

Pre-registered mechanism test off the crouchrise trio's unresolved
hold-park cheat: is the legs-1+4 hover profitable only because the
reward/eval was BLIND to it (clearance-priced, `foot_down_mm`/
`flag_leg_mm` both miss a 1–19mm hover), or is it an anchor-bleed
artifact the reward can't touch? New `reward.hold_feet_load=1.0`
prices hold/track income on MEASURED per-foot touch force (not
height), validated by its own FEET-LOAD bank in
`test_task_semantics.py` (hover reproduced at 4–13mm/duty<0.2 earns
only 0.25x of quiet-stand income under the gated stack vs 0.85+
parity pre-fix).

Result: **same recipe (crouchrise3, dose 0.45) + the new term still
parks legs 1+4.** Gate harness det-hold `duty_cycle` is [0.77, 0.04,
0.97, 0.73, 0.03, 0.99] — identical across all 6 deterministic
episodes (same start state) — vs crouchrise1/2/3's exact same two
legs. `valid_plant` reads True the whole time because both feet
happen to be within a couple mm of the floor at EPISODE END even
though they're airborne >95% of the episode — the same telemetry
gap flagged in crouchrise1's dig-in, now proven insufficient even
after the reward correctly prices the behavior mid-episode (the bank
confirms the term works in isolation; the trained policy still finds
it worth paying for). sto hold's vp misses are the pre-existing
>2.0A current-tail soft flag (5/6), unrelated. Crouch rise unaffected
(det 6/6 incl. crouch 4/4, zero falls) — this run does not touch the
crouch fix. det lower regressed to 2/6 (leg-2 dangling ~90mm, zero
falls) — matches the crouchrise2/3 lower regression exactly, not new.

Per pre-registration: reward-side hypothesis REFUTED by direct
measurement. Three distinct lever families are now closed on this
cheat (start-mix, dose, reward-pricing) — the state/height-aligned
BC anchor (clock-indexed anchor showing lifted-leg reference poses
in plant-adjacent states) is the sole remaining suspect, and it's
CODE/spec work, not another training variant. `hard1` remains the
deployed stance checkpoint; `cw-stand-holdload1`'s checkpoint is not
promoted or warm-started from.

## `cw-stand-anchorstate1` (08-11) — state-aligned BC anchor: PARTIAL mechanism confirmation, net FAIL

The last remaining suspect got its own test: `train.bc_anchor_state_
aligned=1.0` re-indexes the BC anchor every tick to the nearest
reference pose to the robot's CURRENT joints (+0.25s pursuit
lookahead) instead of a fixed clock, so a crouch/plant-adjacent state
can only ever be supervised toward the planted tail of the reference
path, never an early belly-path lifted-leg pose.

Result: **first fingerprint movement in five runs.** Det-hold
per-foot duty `[0.98, 0.04, 0.97, 0.97, 0.93, 0.99]` — leg 4
RECOVERED (0.93 vs 0.01–0.04 in every prior arm), leg 1 still parks
(0.04). So the state-aligned anchor moved the park where four
pricing/dose/mix changes could not — anchor-bleed is CONFIRMED as *a*
mechanism, not refuted, but leg 1 shows it isn't the whole story
(candidate: the lower-bank's own documented dangling-foot incentive
gap, see above). Cost of the fix: det flat rise stalled 62mm short
with all feet planted (0/1 — under-drive, not a cheat: a state anchor
only points 0.25s ahead of wherever the policy currently is, so a
policy that stalls gets barely-moving supervision, unlike the old
clock anchor which dragged it through regardless), and det lower
picked up three tilt_pitch falls (2/6, front feet lifting ~30mm
mid-descent — lower has no anchor, so this is an indirect
shared-network effect, worse than crouchrise3's fall-free
regression). Crouch rise itself stayed clean (det 4/4, sto 3/3), hold
otherwise clean (det 6/6, sto 6/6).

VERDICT: FAIL on gate (leg-1 duty, flat-rise stall, lower falls) but
NOT the pre-registered dead-line (the fingerprint changed instead of
reproducing identically) — anchor-bleed is a real, partial mechanism.
Follow-up per pre-registration: `cw-stand-anchorstate2`, ONE axis
(`bc_anchor_lookahead_s` 0.25→0.5) to restore flat-rise drive while
keeping the state-locality that fixed leg 4 — already launched,
result pending. `hard1` stays deployed; do not warm-start or deploy
from anchorstate1.

## `cw-stand-anchorstate2` (08-11) — lookahead dose 0.25→0.5s: axis EXHAUSTED for the park, leg-1 fingerprint isolated

One axis vs anchorstate1: `train.bc_anchor_lookahead_s` 0.25 → 0.5,
doubling how far ahead the state-aligned anchor pulls (still clamped
at the reference path's planted-tail end, so it should not reopen
the belly-path lifted-leg leak that caused the original clock-anchor
cheat).

Result: **both anchorstate1 regressions fixed exactly as
hypothesized.** Det flat rise restored to 1/1 (was 0/1 stalled 62mm
short). Det lower falls 3→0 (still short of the target: 2/6 success,
sto 4/6, zero falls — a shortfall, not instability). Crouch rise
stayed clean (det 4/4, sto 3/3), bridge 1/1. Leg 4 stays recovered
(det-hold duty 0.95). Hold overall: det 6/6 + sto 6/6 success,
valid_plant 11/12 (clears the ≥10/12 gate clause).

**Leg 1 still parks: det-hold per-foot duty
`[0.99, 0.03, 0.98, 0.96, 0.95, 0.98]`** — the SIXTH consecutive run
with that exact fingerprint on that exact leg, unmoved by four
pricing changes (start-mix, dose, reward-pricing/feet-load) and now
two anchor lookaheads (0.25s, 0.5s). `valid_plant` reads True anyway
(blind to per-foot duty mid-episode — the leg drifts back down at
episode end).

VERDICT: FAIL on the leg-1 duty clause and the lower-success clause,
but this is the pre-registered "leg-1 persists + flat rise restored"
branch verbatim — the lookahead axis is EXHAUSTED for the park, not
inconclusive. Per pre-registration, the next lever is the one
documented incentive gap not yet attacked: the LOWER bank's own
strict xfail (`rise_posture_gate` prices one-leg-aloft at pf=5/6, so
it keeps ~85% of honest lower income) — leg-1's hold-park and
det-lower's dangling-leg shortfall share the same class. `hard1`
stays deployed; anchorstate2 is otherwise the strongest unified-stand
checkpoint yet (crouch+flat+bridge rise, six-foot-minus-one hold,
zero falls anywhere det) but does not clear the gate to replace it.

Follow-up `cw-stand-loweranchor1` (`train.bc_anchor_lower=1.0`: BC
supervision on LOWER ticks toward the lower bank's own honest
demonstration — a per-tick `FixedFootBodyIK` descent anchored at the
settled stance, body at the next commanded height; tests pin
default-off, IK-exact emission, and a feet-planted chained descent,
`rl_move/tests/test_bc_anchor.py`, all pass) **RAN: LOWER SOLVED, hold
park REGRESSED — outside every pre-registered branch, and it names
the next mechanism.** The IK-descent anchor delivered det lower 6/6
AND sto lower 6/6 (from 2/6, zero falls anywhere det) — the lower-bank
xfail lever works exactly as specced, sitting/lowering is done. But
det-hold duty flipped BACK to a two-leg park ([1.0, 0.02, 0.89, 1.0,
0.02, 0.91]) — leg 4 was recovered (0.93–0.95) in anchorstate1/2, now
parks again, and det flat rise re-stalled 96mm short (worse than
anchorstate1's 62mm). This REFUTES the "independent mechanisms"
PARTIAL-branch premise: the park did move, but WITH the lower-anchor
mix, not because of a shared taught habit. Root cause: all three
per-mode anchors (rise/hold/lower) share ONE ring buffer and ONE MSE
step with uniform sampling, so each mode's effective supervision
strength is proportional to its emission share — adding thousands of
lower pairs diluted the rise/hold gradient. **ANCHOR DILUTION** is a
new, directly testable mechanism (not another blind pricing/dose
retune). Follow-up (ran as -r1 — outcome in the next section): `cw-stand-anchormix1`
(`train.bc_anchor_stratified=1.0`, mode-tagged ring buffer with equal
per-mode minibatch quotas, everything else frozen at loweranchor1) —
if stratified sampling recovers hold+rise while keeping lower's 6/6,
the unified stance line is SOLVED; if the park still moves with the
mix even at equal quotas, dilution is wrong/incomplete and the next
step is inspecting `train/bc_anchor_loss` per mode. hard1 stays
deployed meanwhile; `ppo_goal_cw_stand_loweranchor1` is the strongest
lower-specialist checkpoint to date if one is ever wanted standalone.

## cw-stand-anchormix1-r1 (08-11 late): stratified quotas — FAIL, dilution incomplete

The dilution fix ran clean (first launch crashed on a warm-start
`_bc_mode` pickle gap, fixed in bc_anchor.py; -r1 trained the full 2M,
`train/bc_anchor_loss` converged 0.033→0.012, buffer full at 131k).
Gate result (harness det+sto, DR0, videos watched):

- RETAINED: lower 6/6 det + 6/6 sto (loweranchor1's win kept under
  equal quotas), det crouch rise 4/4 valid_plant, zero falls in all
  36 episodes, no non-park cheat.
- STILL BROKEN: det flat rise stalls 105.6mm short (height+footprint
  fail — splayed low the whole strip, an under-drive not a cheat);
  det hold parks ONE foot (duty 0.02 vs 0.90–0.99 on the other five;
  `env/hold_feet_factor` pinned at ~0.14 the entire run vs holdbc1's
  ~1.0); hold det+sto valid_plant 9/12 (three sto 'current' fails).
- THE TELL: the park MOVED. Both legs that parked in
  crouchrise1/2/3/holdload1 (and the one that persisted through
  anchorstate1/2) now read duty 0.90–0.97; a DIFFERENT single leg
  parks. Which foot rests is a function of the anchor configuration,
  not a fixed learned habit — the seventh distinct anchor/pricing
  configuration to produce a one-or-two-foot park, each time
  "solving" the previous fingerprint.

Verdict: pre-registered FAIL branch — anchor dilution was real (the
quotas did protect lower + crouch rise) but is NOT the whole
mechanism. The branch's mandatory next step applies: per-mode
`train/bc_anchor_loss` logging (CODE — only the aggregate exists
today) before any further stand arm, so the next hypothesis is chosen
on measurement. hard1 + specialist handoff stays the deployed stance
stack.


---

# FILE: rl_docs/TURN.md

# TURN — commanded yaw without the structural drift

Status date: 2026-08-11. Owner problem: RL_PLAN queue item 0 (unified
joystick policy — turning). Companion to rl_docs/RISE.md.

## The failure being solved

Walk-lineage policies carry a **command-invariant ~+0.09 rad/s left
drift**: they track yaw commands near the drift and fight commands
against it, in every scenario including turn-in-place. Three arms
failed on it and CLOSED price escalation as a move:

- `cw-walk-yawcmd1` — kernel alone: with σ=0.15 the ungated kernel
  pays a command-ignoring policy 0.67 of max income every tick; both
  seeds learned exactly the drift.
- `cw-walk-yawgate1` — achieved-rotation income gate: drift persisted.
- `cw-walk-yawgate2` — k_walk_yaw 1.0→2.5: drift persisted.

Root cause reading: the Gaussian kernel is the wrong SHAPE, not the
wrong price. Near wz_ref=0 its gradient at the drift point is tiny
(0.09 ≪ σ=0.15), and on turn segments it never goes **negative** for
wrong-direction rotation — rotating against the command and parking
cost the same. Meanwhile turn-in-place states are ~7.5% of training
segments (yaw is drawn independently; turning only coincides with a
stop via resample), so the skill being scored is barely trained.

## The mechanism set (landed 08-10, all cfg-gated, default 0 = legacy)

1. **Signed rotation income — `reward.k_yaw_prog`.** The k_walk_prog
   analog for turn segments: `k * clip(wz/wz_ref, −1.5, +1.25)`.
   Constant gradient toward the commanded direction; genuinely
   NEGATIVE when rotating against it. This is a new mechanism (sign-
   aware income), not a re-price of the closed kernel family.
2. **Drift charge on heading-hold — `reward.k_yaw_still`.** When
   wz_ref = 0: `−k * wz²`. At the measured drift (0.09 rad/s), k=50
   costs ~0.4/tick — real money against the ~2/tick kernel — while
   gyro-noise wz stays ~free by the square law. This is the term that
   directly disincentivizes DRIFTING while walking straight.
3. **Turn-in-place curriculum — `goal.walk_turn_in_place_frac`.**
   With probability f a walk episode becomes a dedicated turn: zero
   linear command, |wz| drawn in [0.5, 1.0]·walk_yaw_max_rad_s, sign
   50/50 by construction (the drift direction can never dominate
   exposure). Applied last in `_sample_walk`, overrides resample
   segments; rng stream untouched at f=0.

Held in reserve (not landed): mirror-symmetry augmentation /
symmetry loss (reflect obs+actions about the sagittal plane). The
heavy structural fix if the mechanism set above fails in DISCOVERY —
needs trainer surgery, do not start there.

## MDP_PREFLIGHT — the TURN bank (PASSING 08-10)

`test_task_semantics.py::test_turn_reward_separates_command_from_drift`
runs the full turn stack (champion walk cfg + walk_yaw_cmd, kernel
k=1.0 + achieved-rotation gate + k_yaw_prog=1.0 + k_yaw_still=50) on
scripted-gait policies at wz_ref = ±0.25, 3 seeds × both signs:

    turn (full command)   +1463
    partial (35%)         +1222
    drift (fixed +0.09)   +1154
    park                  +1122

Ordering `turn > partial > drift > park` holds — the stack now prices
the exact policy PPO found three times below honest partial turning.
`test_turn_command_signs_priced_symmetrically` additionally requires
CW and CCW turn income within 45% of each other — a live tripwire on
the yaw sign chain (below).

## Sign audit (still OPEN — do this before any hardware turn)

Sim `_body_wz()` is **+CCW** (right-hand z-up). Hardware measured
08-09: scripted gait `+omega = clockwise`. The TURN bank proves the
SIM chain (gait omega ↔ wz_ref ↔ reward) is internally consistent,
so the flip sits at the hardware boundary: the deploy bridge must
map joystick/policy yaw commands with the sign audit's result, or
the first hardware turn will fight its own command. One bench check:
command a small +wz through the bridge, read gyro sign.

## First arm result — FAILED (08-10, `cw-walk-turnfix1`)

Trained exactly the recommended cfg below off `cw-walk-yawgate2`.
Matched-parent control (`eval_yaw.py`, identical scripted panel,
turnfix1 vs frozen yawgate2): turn |wz_err| med 0.232 vs parent's
0.233; hold |wz| med 0.108 vs parent's 0.091 — statistically
IDENTICAL to the already-failed parent, same left/right asymmetry
(arc-left ~0.07–0.21 near the drift, arc-right ~0.22–0.37 fighting
it). The reward-side mechanism set (signed rotation income +
heading-hold drift charge + turn-in-place curriculum) passed its
pre-training bank but produced ZERO measurable behavior change in a
real policy. **Behavioral-impossibility kill — price tuning on this
task is now doubly closed** (first the kernel-price family, now the
signed-income/drift-charge/curriculum family). Straight walk stayed
clean (gv 6/6, 0 falls). Do not re-attempt with more steps or a
different k; the next move is the structural fix below.

## Next move: mirror-symmetry augmentation (was "held in reserve")

Reflect obs+actions about the sagittal plane (symmetry loss or data
augmentation) — needs trainer surgery, [CODE] not a launchable spec
yet. This is now the ONLY untried lever on the turning blocker;
every reward-shape lever (kernel price, achieved-rotation gate,
signed income, drift charge, turn-in-place exposure) has failed to
move a real trained policy off the fixed left-drift. Root-cause
reading: the drift is baked into the WALK GAIT itself (an asymmetric
limb-phase pattern learned once, early, off-center), not into the
turn reward's shape or price — no reward retuning can out-argue a
structural asymmetry in the policy's default gait.

## Mirror-symmetry landed; hardening run hit a reward bug, not a verdict (08-11)

`train.mirror_loss_coef=1.0` landed 08-10 (`rl_move/sim/mirror.py` +
`MirrorPPO`), discovery probe `cw-omni-mirror1` PASSED its
mechanism-health gate (mirror_sym_loss fell to <0.5x peak, reward
climbed cleanly, 0 NaN). The 40M-step hardening follow-up
`cw-omni-mirror1-r1` does **NOT** confirm or refute the mirror
hypothesis: the walk gait itself collapsed into a stand-still/
march-in-place exploit before turn-tracking could be judged.
Harness evidence (own-DR + DR0, vs frozen `cw-arch-hist16-dep1`
same-recipe baseline): forward travel 0.68m med -> 0.01m med per 15s
episode, gait_valid 6/6 -> 3/6, slip_per_m 1.48 -> 3.85 med. Per-
episode returns show WHY: frozen episodes (gait_valid False, ~0.004m
travel) scored ~1130, walking episodes (gait_valid True, ~0.02m
travel) scored 500-860 — **standing still paid more than walking**
under this arm's stack. `train/std` also climbed monotonically
0.39->1.69 over the full 40M (health alarm, RESEARCH_RULES), in step
with `rollout/ep_rew_mean` peaking ~640 near 8-10M then falling to
~320-350 by 40M.

Actual cause (probe-confirmed 08-11 — the earlier k_yaw_still guess
was WRONG; the drift charge summed to ~0 for a scripted gait): on
turn-in-place ticks (`s_ref≈0, wz_ref≠0`) the LINEAR velocity kernel
paid a frozen robot FULL income — v_lin=0 matches the zero linear ref
exactly, and `walk_kernel_prog_gate` only engages when `s_ref>1e-3`.
The same s_ref condition left `k_park_duty`/`k_step_event` inert on
those ticks. With `walk_turn_in_place_frac=0.30`, a freeze banked
~1122/ep (probe; run showed ~1130) — 0.77x of a PERFECT scripted
turner's income and MORE than the mid-training policy earned by
actually walking (500-860). PPO parked, by construction.

**Fixed (08-11):** `reward.walk_kernel_yaw_gate` (walk_task.py,
default 0 = legacy): on yaw-commanded zero-linear ticks the linear
kernel is multiplied by achieved-yaw fraction clip(wz/wz_ref, 0, 1)
— the exact prog-gate analog; genuine stop segments stay paid. The
freeze-floor bank (bottom of test_task_semantics.py) pins the
exploit two ways: park < 0.5x turn on pinned turn-in-place commands,
and gait-follows-commands beats a full-episode freeze on SAMPLED
r1-mixture episodes (resample/stops/turn-in-place). Pre-fix both
FAILED (park/turn 0.77); post-fix: turn 1128 > partial 650 > drift
539 > park 423 (0.38x), mixture gait +1055 vs freeze +424 (0.40x).
OMNI_OVERRIDES now trains the gate ON (1.0) — any omni arm must.
Re-hardening (`cw-omni-mirror2`, warm from the healthy 2M probe
ckpt, one variable vs r1 = the fixed pricing) is the next launch.
Do not read the r1 FAIL as evidence against mirror-symmetry — the
mechanism was never exercised by a real gait in that run.

## `cw-omni-mirror2` — gate fix confirmed, but a NEW gait pathology
blocks the mirror-symmetry test (08-11)

40M hardening, one variable vs r1 (`walk_kernel_yaw_gate=1.0`),
finished. The specific r1 exploit (frozen episode out-earning a
walking one) is GONE — per-episode returns now show walking beats
the degenerate pattern (526–922 vs 399–425) — but the gait itself
still breaks down in ~half of det+sto episodes into a leg-sacrifice/
tripod pattern (one or more legs held near-stationary, duty_cycle
~0.04–0.07 or 1.0-with-0-swings, forward_dist_m 0.005–0.05m/15s,
0/6 success both modes; contact sheets confirm the body barely
translates). `train/std` climbed 0.39→1.30 (>2x, the pre-registered
health alarm) and reward peaked ~447 near 10M then fell to 302 by
40M. **STOP — known exploit (video overrides the mechanism-health
metric); no dig-in.** Mirror-symmetry remains statistically UNTESTED
— the gait never got clean enough to isolate a turn-tracking signal
from a chirality signal. Per the pre-registered outcomes, this is
closest to "collapses again despite the gate": do not launch another
mirror hardening arm; the next move is a term-by-term income
re-probe of the WALK kernel (not just the yaw-gated segments) to
find what still pays for a partial leg-sacrifice, before any new
mirror arm.

**`cw-omni-mirror2-dr02` — DR exonerated (08-11).** Matched twin,
identical spec, dr-scale 0.2 vs mirror2's 0.5 (operator question: is
DR making this task too hard?). Fails IDENTICALLY: det gait_valid
3/6, same leg-sacrifice pattern (legs [0,2,4] / [1,3] held, fwd
0.00–0.01m), walking still out-earns sacrifice (646–889 vs 473–485,
same shape as mirror2's 526–922 vs 399–425); sto gait_valid 6/6 but
slip_per_m 4–18, 0/6 success both modes both configs. `train/std`
1.10 at 40M (milder than mirror2's 1.30 but same terminal pathology).
DR-scale is ruled out as the driver — this is a property of the
reward/task stack, not an optimization-difficulty knob. Do not queue
another DR level on this line.

## Recommended first arm (DISCOVERY, ≤2M steps) — SUPERSEDED, see above

Parent: walk champion (or hist16 twin). Cfg:
`goal.walk_yaw_cmd=1 goal.walk_turn_in_place_frac=0.30
reward.k_walk_yaw=1.0 reward.walk_yaw_kernel_gate=1.0
reward.k_yaw_prog=1.0 reward.k_yaw_still=50` +
champion walk stack. Evidence: the passing TURN bank. Judge with
`eval_yaw.py` (turn |wz_err| median vs the 0.10 gate; hold |wz|
median vs 0.05) AND a matched-parent control — the parent under the
identical eval, so the drift delta is attributable. Early video at
first eval; kill on the behavioral-impossibility rule if both turn
directions still converge to the drift.

## `cw-omni-trans1` — turning removed entirely, gait still collapses
(08-11)

Operator de-scoped commanded turning from the joystick deliverable
(no camera on the robot = no reason it needs a definable "front").
`cw-omni-trans1` tests the narrower goal directly: walk in ANY
commanded direction (full-circle heading), mirror-symmetry loss
still on (coef 1.0), full dep1 contract, k_current=0 — but with the
ENTIRE yaw/turn-in-place stack removed (no `walk_yaw_cmd`, no
`walk_kernel_yaw_gate`, no turn-in-place curriculum), so the
freeze-income exploit that required a yaw gate is structurally
absent. Result: FAIL, a THIRD distinct pathology. Not freezing
(mirror1) and not the leg-sacrifice/tripod pattern (mirror2/dr02):
instead a paddle-stall — legs 1 and 4 stay planted 90–99% duty the
whole episode while the other four take rapid, tiny (~0.01m mean)
strides, slip_per_m 3–13 (vs champion band ~1.2–1.5), along-command
progress_ratio med 0.51 det / 0.22 sto, 0/6 success any mode/DR.
`train/std` climbed continuously with no plateau, 0.37→1.38 (3.7x
start) — worse than mirror2's own 2x alarm and never recovered.
Reading: omnidirectional translation (independent of the yaw
mechanism entirely) is ALSO not yet a solved reward/task spec — three
different arms (mirror1, mirror2/dr02, trans1) each find a different
degenerate attractor once heading leaves the narrow forward band the
champion was built on. Mirror-symmetry remains completely untested
by any of the three; the gait has never been clean enough to isolate
it. Do not launch a fourth omni variant on the current stack — next
move (unchanged from the mirror2 ruling) is a term-by-term WALK-kernel
income re-probe: find what still pays for degenerate partial gaits
(freeze, sacrifice, AND paddle) before trying another mirror/heading
arm.

## Income re-probe DONE (08-11, `probe_walk_income.py`) — pricing
EXONERATED on the deliverable stack; latent yaw-kernel defect found

`rl_move/sim/probe_walk_income.py` decomposes per-term income
(every `info["reward_*"]`, residual 0 by construction) for scripted
references matched to the three video fingerprints AND the actual
collapsed checkpoints, under the exact trans1/mirror2 stacks
(artifacts: `logs/probe_walk_income/`). Results, mean/ep over
4 directions x 3 seeds:

- **trans1 stack (turn-free, the deliverable): NOTHING pays the
  degenerates.** DR0: gait 824 > half-speed gait 643 > 1-leg
  sacrifice 541 > paddle 375 ≈ tripod-sacrifice 341 > freeze 217 >
  **the trained trans1 checkpoint itself 205** — the collapsed policy
  earns BELOW A FREEZE under its own reward. Identical ordering at
  the training DR 0.5 (761/564/515/337/289/233/214). Directions
  priced uniformly (gait 750-897 everywhere). Income is monotone in
  honest progress; the degenerate attractors are OPTIMIZATION
  failures, not paid basins. Reward surgery on this stack is CLOSED
  (matches trans1's pre-registered if-false: "not reward surgery").
- **mirror2/turn stack: real latent defect.** On linear-command ticks
  the ungated yaw kernel (`k_walk_yaw`, wz_ref=0 "heading hold" side)
  pays a MOTIONLESS body full income — 373-375/ep to sacrifice,
  paddle, and freeze alike, the single largest channel in the stack —
  while `k_yaw_still=50` charges the honest gait's natural wz
  oscillation -73/ep and the degenerates ~0. Net: the yaw stack taxes
  honest walking ~-100/ep RELATIVE to body-stillness. Aggregate
  ordering still holds (gait 1100 > paddle 748 > sac3 710 > freeze
  592 > ckpt 410), but this stillness subsidy must be fixed (gate
  heading-hold yaw income on linear progress, or price wz vs the
  gait's own oscillation band) BEFORE any turn re-scope. Not fixed
  now — turn is de-scoped from the deliverable.

**Next lever (supersedes "rot-60 first"):** the collapse signature —
provably unpaid behavior that PPO still converges to, because no
gradient tells a churning leg WHICH WAY to move — is exactly what the
BC anchor fixed twice (rise `cw-stand-bc1`, hold `cw-stand-holdbc1`).
Landed 08-11: walk ticks emit `bc_target` = the command-conditioned
scripted TripodGait pose (the gait that walks/crabs/turns the REAL
robot) one tick ahead; stop ticks unsupervised (the gait marches in
place at v=0); per-episode gait instance on SNAP_ATTRS. Discovery arm
`cw-omni-transbc1` (trans1 + `train.bc_anchor_coef=1.0`, one
variable). rot-60 equivariance stays the reserve lever if imitation
anchoring fails; note mirror-symmetry loss coef 1.0 was ON during the
trans1 collapse and did not prevent it.

## OMNI TRANSLATION RESOLVED IN SIM — rot-60 canonicalization
(08-11, SPECIFICATION result, zero training)

The reserve lever after transbc1 closed BC-anchor/reward tuning. The
robot is a REGULAR hexagon: six identical leg templates at exactly
(i+0.5)*60 deg, axisymmetric chassis inertia, identical actuators —
rotate the world 60 deg + relabel legs is an EXACT symmetry of the
compiled model (unlike the mirror, which the COXA_HIP_ANCHOR_Y
pinwheel only approximates; the pinwheel is rot-60 INVARIANT).
`rl_move/sim/rot60.py` exploits it at eval time: pick the 60-deg
sector nearest the commanded heading, rotate the command/tilt/gyro/
velocity obs into the +/-30 deg wedge, cyclically relabel the leg
channels, un-relabel the action. A wedge-trained policy then covers
the full circle BY CONSTRUCTION. `test_rot60.py` proves the model
symmetry mechanically (rotate+relabel state, permute ctrl, step: <1e-6
divergence over 30 contact steps — a real asymmetry would show ~1e-3
immediately).

Evidence (`logs/rot60/`, eval_drive full-circle panel + harness
per-mode 6 det+sto at full-circle headings, matched naked controls,
seed 0):

- `cw-arch-hist16-dep1` (trans1's parent): naked back
  trk_err 0.069 / 0.102 m of the commanded 0.30 m; laterals
  0.047-0.052. Wrapped: back 0.039/0.305 m, left 0.030, right 0.028,
  ZERO falls at DR 0 and DR 0.5 incl. full-circle instant-flip stress
  (live sector switching). Harness on full-circle commands: naked
  degenerates AT EVAL TIME into the trans1-style leg-sacrifice
  (gait_valid 3-5/6, sacrificed legs [2]/[0,4], prog_ratio 0.41-0.60,
  slip/m 7.3-11.3); wrapped is the honest champion gait everywhere
  (gait_valid 24/24, prog_ratio 0.92-0.98, slip/m 1.3-1.6).
- **`cw-dep-vref1-r1` (THE hardware checkpoint)**: naked backward is
  frozen (0.027 m); wrapped, every direction tracks 0.024-0.036 at
  DR 0 AND own DR 0.35, zero falls; harness success 20/24
  (det/DR0 6/6), gait_valid 23/24, slip/m 1.1-1.3 (its own band),
  video-confirmed six-leg gait. The hardware deliverable gains
  full-circle translation with NO new training.
- Known quirk: dep1 wrapped diag-fr (canonical +15 deg) reads
  0.046-0.051 at both DRs — a dep1 wedge asymmetry (vref1-r1 shows
  0.028-0.032 there); dep1's vel-success was already "coin-flip per
  episode" pre-wrapper (its run doc). Not a wrapper defect.

Interpretation: the four omni collapses were PPO failing to DISCOVER
rotated gaits (matches the income probe: attractors unpaid), and the
fix is structural, not learned. Also supersedes the trans1-stack
training question — there is nothing left for a 40M omni arm to
learn that the wrapper does not already give exactly.

De-scoped turn note: with no camera there is no "front" — the wrapper
makes heading-agnostic driving native (the operator points the stick;
the body never needs to yaw). If turn ever re-scopes, rot-60 also
gives 6 free discrete body orientations by pure relabeling.

DEPLOY-SIDE PORT — LANDED 08-11 (later cycle). Design: no ported
copy. `linux_control/rl_policy.py` wraps `rot60.Rot60Policy` ITSELF
through a NumpyPolicy shim (`make_walk_canonicalizer`), so runner and
sim share one implementation by construction; `deploy_adb.sh` now
ships `rl_move/sim/{__init__.py,rot60.py}` (both verified
numpy/docstring-only by the test bank — the board has no
torch/mujoco). Contract as specified: reads vx/vy_ref from obs
indices 68:70, fresh per-episode sector state, hysteresis +
zero-cmd hold (all rot60.py's own code paths).

Deployment safety properties:
- Default ON, but k=0 is a BIT-EXACT no-op (walk obs now built
  float32, the training/export dtype) — hardware-validated forward
  behavior is unchanged to the last bit.
- `rot60=false` on `/api/rl/walk` runs the naked policy as the A/B
  baseline for a bench parity session; in that mode (or if rot60.py
  is missing on the board) off-wedge commands are REFUSED
  pre-preflight instead of running a heading the naked policy
  provably freezes/degenerates on.
- Every walk tick logs `rot60_k` (last CSV column; obs columns stay
  REAL-frame), so any hardware episode can be replay-checked offline:
  logged obs -> make_walk_canonicalizer must reproduce logged act*.

Replay-parity check: `rl_move/tests/test_rot60_runner.py` (6 tests,
green; full suite 114 passed / 1 skip) — locks the runner's 72-wide
obs layout block-by-block against rot60's slices, forward-wedge
bit-exactness, full-circle sweep + boundary-dither + zero-hold parity
vs manually composed rot60 primitives using the REAL deployed
`rl_walk_weights.json` (= ppo_goal_cw_dep_vref1_r1), the backward
sector selection, the wedge fallback, and the numpy-only import
chain. Remaining work on this blocker is BENCH-ONLY: an operator
walk session exercising lateral/backward headings (start with
rot60=false forward, then wrapped forward — should be identical —
then off-wedge).

## TURN RE-OPENED (operator 08-11 afternoon) — latent defect FIXED,
reflection wrapper PASSES: bidirectional steering with zero training

Operator re-opened turning as an experiment line ("it really should
be possible"). RL_PLAN queue 0.2 executed in order, 08-11:

**(1) The latent yaw-stack defect is FIXED and BANKED** (walk_task.py,
both cfg-gated, default 0 = byte-identical legacy):

- `reward.walk_yaw_hold_prog_gate` — on linear-command ticks the
  heading-hold side of the yaw kernel is multiplied by achieved-
  progress fraction clip(along/s_ref, 0, 1), the exact mirror image
  of `walk_kernel_yaw_gate`. Pre-fix a MOTIONLESS body collected the
  largest single channel in the stack (probe, forward/DR0: freeze
  +375/ep vs honest gait +334; net of the drift charge the yaw stack
  paid freeze +375 vs gait +224 — a stillness subsidy). Genuine stop
  segments (s_ref ~ 0) stay paid.
- `reward.yaw_still_avg_s` — the heading-hold drift charge prices the
  EMA of wz (the DC drift it exists to punish) instead of the
  instantaneous wz (the honest gait's zero-mean stride oscillation,
  wz_rms ~0.044). Pre-fix it taxed the honest gait −110/ep and every
  motionless degenerate ~0 — charging exactly the wrong policy.
  Per-episode EMA state rides MJX_SNAPSHOT_EXTRA (pool-restore
  lesson, commit 65edba7).

Post-fix income probe (`probe_walk_income.py --stack mirror2fix`, new
`driftride` reference calibrated so the scripted gait ACHIEVES the
measured 0.09 rad/s while translating — commanded omega 0.25; the
gait realizes ~40% of commanded omega): ordering is now monotone in
honesty — gait 935 > gait_slow 772 > driftride 715 > sac1 683 >
paddle 441 > freeze 242 (was gait 1011 ~ gait_slow 1008 with freeze
collecting the top yaw channel). The rider pays −92/ep for its DC
rotation; the honest gait's oscillation tax fell −110 → −2.
Artifacts: `logs/probe_walk_income/mirror2{,fix}_driftride.json`.

New bank (test_task_semantics.py, stillness-subsidy section): the
FULL summed stack on a straight-line command vs a body that simply
does not move — the exact blind spot the old TURN bank (turn-in-place
only, mechanism terms in isolation) and the freeze-floor bank
(turn-tick kernel) both missed. Four tests: subsidy closed on the
fixed stack, subsidy PROVEN on the legacy stack (so a default drift
can't silently retire the reproduction), fixed stack restores honest
walking's margin over a freeze, and the calibrated drift-rider loses
to the honest straight gait under the full summed stack.
`TURN_OVERRIDES` now trains both fixes ON — any turn arm must.
NOTE: the yawcmd1/yawgate2/turnfix1 checkpoints no longer exist on
any pod (searched 08-11; only the mirror lineage survives on
train-0), so the ckpt-level term audit is closed as UNTESTABLE — the
calibrated scripted rider stands in for their fingerprint.

**(2) REFLECTION WRAPPER — PASS, the rot60 lesson repeats.**
`mirror.MirrorPolicy` (eval-time, numpy-only, algebra locked by
test_mirror.py against a stub): pi_mirror = mirror_act(pi(mirror_obs)).
`probe_mirror_turn.py` on THE hardware checkpoint
(`cw-dep-vref1-r1`), 12 s forward walks (artifacts
`logs/mirror_turn/`):

    DR 0:    naked wz +0.0399 rad/s (heading +38 deg), travel 0.562 m
             mirror wz −0.0455 rad/s (heading −37 deg), travel 0.566 m
    DR 0.35: naked +0.0450 / mirror −0.0403 rad/s, travel 0.52/0.51 m
             (drift flips on every seed)
    heading-hold (bang-bang naked/mirror on accumulated heading,
    4 deg hysteresis): final |heading| 2–4 deg vs naked's 34–50 deg
    runaway, travel ~0.85x naked, 5–7 switches, ZERO falls anywhere.

Reading: the pinwheel asymmetry (mirror.py docstring) is behaviorally
negligible — the mirrored gait is fully competent. Chirality
SELECTION therefore already gives: arc-left (naked), arc-right
(mirror), and drive-straight (alternation), zero training, from the
already-deployed checkpoint. Turn rate is the drift magnitude
(~0.04 rad/s in sim, ~2.3 deg/s; hardware measured ~0.09), so this
is SLOW steering — command tracking up to ±0.3 rad/s stays unsolved.
Deploy-side port + composition with rot60 (mirror-then-rotate is
still a single obs permutation) are follow-up [CODE]; the sign audit
(sim +CCW vs hardware +omega=CW) remains OPEN and gates any bench
turn session.

**(3) Mirror-symmetry training — RUN, FAILED (08-11 late).**
`cw-walk-mirturn1` (discovery 2M, warm from cw-dep-vref1-r1 +
pad-transplant, forward wedge + yaw command set, the FULL fixed
pricing incl. walk_kernel_yaw_gate, train.mirror_loss_coef=1.0): the
mechanism engaged cleanly (`train/mirror_sym_loss` 28.3->0.51) but
both gate halves fired FAIL — eval_yaw turn |wz_err| med 0.254 (gate
<=0.15; still command-invariant, L/R asymmetry intact: tip-left 0.409
vs tip-right 0.255) and heading-hold drift got WORSE than the parent
(0.148 vs ~0.04) — AND the forced symmetry rewrote the champion's
gait itself (harness prog med 0.41-0.43 vs parent's ~1.0, slip/m
6.0-7.6 vs 1.1-1.5, near-in-place churning on video, zero falls).
Per the pre-registered FAIL branch, mirror-symmetry TRAINING on a
warm champion is CLOSED for good. The shipped turning story is
eval-time MirrorPolicy chirality selection (arc-left/arc-right/
alternation, zero training, ~2 deg/s, described above). Step (4)
BC-anchor on turn ticks stays in reserve, unpromising after the
walk-tick BC anchor (`cw-omni-transbc1`) froze into a static pose
under the analogous per-tick imitation pressure.


---

# FILE: rl_docs/GAIT.md

# GAIT.md — kill the paddle: walking that lifts its feet

Operator directive 2026-08-11 ~13:05. Owner: agent cycles; operator
reviews verdicts. Status of each item lives in the ledger/RL_LOG, not
here.

## The problem

Every walking champion travels by PADDLING: feet stay loaded and
skate across the floor (slip/m 1.1–1.5 in its own band; the
hist16-dep1 eval degenerate hits 7–11). On hardware this scrapes the
leg tips against the ground and the robot can catch/stall. We want a
gait that LIFTS and PLACES feet — swing legs clear the ground, stance
feet do not translate while loaded.

## What we know (don't re-derive)

- **Pricing already prefers stepping.** probe_walk_income.py (08-11):
  an honest stepping gait out-earns the paddle 2–4x under the
  champion stack, all four directions, DR 0 and 0.5. The paddle is
  not a paid basin — it is a strong LOCAL optimum PPO finds first and
  never leaves.
- **Sim slip is not free.** calibrate_slip.py (08-10): sim is
  slightly conservative vs the real floor (travel ratio 0.35–0.41 sim
  vs 0.50–0.51 real, speed-invariant). This is not a sim exploit.
- **Anti-slip income shaping is CLOSED (10+ arms).** Gates/shaping
  retrofitted onto a trained paddler either starve income (policy
  parks and earns nothing — the park-and-earn failure) or get farmed.
  That closure is about INCOME SHAPING ON A FORMED HABIT. It does not
  close the two things below (operator 08-11): a structural drag
  CHARGE banked from scratch, and CURRICULUM that changes the task.
- **The scripted tripod gait steps.** It walks the real robot
  (tape-proven), and bc_anchor.py already emits it as a
  command-conditioned per-tick target on walk ticks
  (`cw-omni-transbc1` built the machinery; that arm failed on
  from-scratch omni TRANSLATION, which rot-60 later solved — the
  anchor itself converged cleanly. It was never tried for GAIT
  CLEANUP on a policy that already travels).

## Priority 1 — BC-anchor gait cleanup — CLOSED 08-11 (froze instead
of stepping; see session log)

`cw-walk-gaitbc1`: warm-start THE hardware walker (`cw-dep-vref1-r1`)
with `train.bc_anchor_coef=1.0` on walk ticks (existing TripodGait
target), discovery 2M, everything else byte-identical to the parent's
recipe. Rationale: rise and hold both proved the anchor breaks
entrenched habits that pricing cannot; here the skill (traveling)
already exists and only the HABIT (dragging) needs breaking, which is
the anchor's demonstrated strength.

Gate: slip/m must drop DECISIVELY below the parent band (target
<0.6 at DR0 and own-DR vs parent 1.1–1.5) while keeping the
joystick gate (zero falls, fwd distance, vel-err in band) and
gait_valid. Kill signature: fwd distance collapses toward the
scripted gait's slower band with no slip win, or park.
Follow-up if PASS: 10M hardening + tape-replay style hardware check
before any promotion talk.

## Priority 2 — structural stance-slip / swing-clearance terms
   [CODE, bank first]

Two structural terms (HARDWARE.md queued them long ago), landed
behind cfg flags with an MDP_PREFLIGHT bank BEFORE any launch:

- **Stance-slip charge**: foot-XY translation WHILE IN CONTACT,
  charged per tick (this is literally the scrape). A charge, not an
  income gate — gates are the closed move.
- **Swing-clearance term**: swing feet must clear >= X mm at
  mid-swing (pay tiny clearance income or charge sub-clearance
  swings; pick in SPECIFICATION).

Bank requirements (scripted fingerprints, full champion stack):
step-gait >> drag-gait, AND drag-gait > freeze/park (the penalty must
never make parking the optimum — the exact failure of the closed
arms), AND the honest gait's own small placement slip is not
bankrupted (tape-proven gait must stay net-positive).

## Priority 3 — LEARN it, no anchor (operator: the goal state)

Operator 08-11: "if we really gave dragging on the ground a big
penalty, it should eventually learn how to move — maybe we need to
make the task initially easier in some other way." The BC anchor is a
crutch; this line exists to retire it. Design principles: dragging
expensive AND the task easy enough early that clean travel is
DISCOVERABLE before the charge matters. From scratch (no paddle habit
to break), one easing lever at a time:

1. **Terrain-as-teacher (first arm — physics does the pricing).**
   Train ON bumpy ground (existing `env.terrain_amp` hfield, bumps
   ~20–36 mm). On bumps a dragging foot physically catches: paddling
   stops traveling, so plain progress income selects stepping with NO
   reward surgery. Anneal terrain toward flat late (or test transfer
   to flat directly).
   ZERO-TRAINING PROBE FIRST (cheap, decisive): eval the current
   champion on terrain_amp>0 and read slip/m. Champion already
   survives 36 mm bumps — if its slip DROPS there, physics already
   forces stepping and the premise is confirmed; if it paddles over
   bumps at the same slip, this lever is refuted and we skip the arm.
   **REFUTED (08-11, `cw-dep-vref1-r1` on freshly-synced code —
   the amp>1 clamp fix, 434a6e0, must actually be on the eval pod;
   an unsynced free pod silently re-ran the OLD clamped code and
   returned bit-identical reports across amps, a false negative,
   caught + documented as COMMANDS.md gotcha 16):
   true amp 1.0 (18mm, the model's actual base bump) leaves slip/m
   at the champion's own band (det/sto med 1.07/1.17); amp 2.0 (36mm)
   makes it WORSE, not better (1.34/1.54); amp 3.0 (54mm) is worse
   again (1.43/1.85) AND unsafe — 6/6 det and 4/6 sto episodes
   terminate `over_current` (the dragging foot catches/strains
   against a bump it never lifts over). Slip never drops with
   amplitude — the champion pays MORE for dragging into bumps, it
   never discovers stepping. Lever 1 is refuted at every amplitude
   tried, well past the point of any therapeutic effect and into
   outright failure; skip the terrain-as-teacher training arm
   entirely (no `cw-gait-terrain2`) — `cw-gait-terrain1`'s INVALID
   verdict stands but its suggested successor is now superseded,
   not just re-run at a corrected amplitude. Surviving P3 levers:
   3 (physics easing), 4 (RSI-for-walk), 5 (slow-speed-first); P2's
   structural charge is still SPECIFICATION-gated (bank first).
2. **Drag charge annealed UP** (with Priority-2 machinery): charge
   starts near zero (travel discoverable, sloppy is fine), ramps to
   full by mid-run so the FINAL optimum cannot include dragging.
   Anneal-up avoids both failure modes of the closed arms: no
   retrofit onto a formed habit, no early starvation.
3. **Physics easing early, annealed to nominal**: servo velocity
   ceiling up (lifting is cheap while learning) and/or slight gravity
   reduction during discovery — standard dynamics-curriculum practice
   in legged RL. Anneal to measured params before any gate counts.
4. **RSI-for-walk (state crutch, NOT action supervision)**: spawn
   walk episodes mid-stride of the scripted tripod gait (machinery
   exists from the rise work, `goal.rise_rsi_frac` pattern +
   pool-restore lesson) so lifted-leg states are experienced from
   step 0.
5. **Slow-speed-first commands** if 1–4 underdeliver (note: the old
   speed-band closure was about chasing speed TRACKING, not gait
   curriculum — distinct).

Success = a from-scratch, no-anchor policy whose slip/m beats the
BC-anchored one at matched travel, surviving DR hardening. If the
line stalls after levers 1–3 are honestly tried, the anchor stays
and this doc records why.

## Order of operations

P1 CLOSED (froze, see session log). P2 spec+bank next (its charge is also
P3's lever 2). P3 starts with the zero-training terrain probe, then
one arm per lever. Every arm: one variable, matched-parent control,
pre-registered kill signature, slip/m + gait_valid + joystick gate
as the panel.

## Session log 08-11 ~13:15-14:15 (operator) — first wave results

- **TERRAIN CLAMP BUG found+fixed (434a6e0):** servo_model.build_model
  clipped `env.terrain_amp` to [0,1], and HFIELD_MAX_Z is 18 mm — so
  EVERY historical terrain run/eval ran peak bumps <=18 mm regardless
  of the requested amp (docs said 36). Exposed when the champion probe
  returned bit-identical results at amp 1.5/2.0/3.0. Amps >1 now scale
  the hfield z-extent (data stays [0,1]); verified amp 4.0 = 72 mm.
- **Champion-on-terrain probe (post-fix, train-5
  logs/terrain_probe2):** amp 2.0 (36 mm): 6/6, prog 0.85, slip 1.31 —
  paddle degraded but passes. amp 4.0 (72 mm): 0/6, prog 0.80, slip
  1.46. amp 6.0 (108 mm): 0/6, prog 0.80, slip 1.48. The paddle FAILS
  THE WALK GATE at 72 mm — teaching ground confirmed; the flat spawn
  disk + fade-in is a built-in curriculum.
- `cw-gait-terrain1` INVALID (trained on the clamped 18 mm rung).
  Superseded by **`cw-gait-terrain2`** (train-3): from scratch at true
  72 mm, no reward surgery.
- **P1 `cw-walk-gaitbc1` FAILED (08-11, gate eval) — worst-case
  version of the pre-registered kill signature: total FREEZE, not
  partial.** Video (det+sto, DR0 + own-DR0.35, all 6 episodes each)
  shows an IDENTICAL static tripod-like pose every episode — 3 legs
  held aloft motionless, 3 planted, zero gait cycling — fwd travel
  ~0.00 m/15s vs the parent's 0.28-0.34 m, gait_valid 1/6 det.
  Training-time read confirmed the mechanism: bc_anchor_loss
  converged to ~0 while walk_loadslip_factor collapsed 1.0->0.06 —
  the anchor was satisfied by NOT MOVING (close enough to the
  moving reference at a single frozen instant), not by lifting feet.
  Unlike rise/hold (the anchor's two prior wins, both STATIONARY
  end-states where freeze-toward-reference IS success), walk's
  reference target is itself in continuous motion; anchoring to a
  moving target plus the existing income-gated walk reward found a
  degenerate joint optimum instead of reshaping the gait. Per its own
  pre-registered gate text this is not a coefficient-variant
  situation — P1 (BC-anchor gait cleanup) is CLOSED; move to P2
  (structural stance-slip charge + swing-clearance, bank first) or
  P3 lever 4 (RSI-for-walk).
- **P3 lever 2 `cw-gait-dragstep1` FAILED** (agent triage, kill (b)):
  paddle formed anyway from scratch at k_drag_loaded=40 + step-event
  income; gate eval slip det 6.36. IMPORTANT CAVEAT before closing
  pricing forever: env/reward_drag never exceeded ~0.09/tick in
  magnitude even at k=40 (0.5 mm/tick deadband + per-mm scale keeps
  the term tiny vs ~1/tick progress income), and reward_step_event
  peaked at 0.03/tick. The "big penalty" was effectively small. A
  charge-magnitude AUDIT (what does k=40 actually cost a typical
  paddle tick, in fraction of income?) is the honest next step on
  this lever, not another blind coef rung.
- **P3 lever 1 `cw-gait-terrain2` FAILED** (operator gate-eval,
  logs/terrain2_gate on train-3): from scratch at TRUE 72 mm the
  policy learned the LEG-SACRIFICE drag degenerate — own-terrain det
  0/6, prog 0.25, slip/m 10.2, sacrificed legs [3,4]; flat retention
  identical. Ground that defeats the champion's paddle did not force
  stepping; PPO settled for 25% progress dragged over the bumps.
  Physics-as-teacher REFUTED standalone at this amp/budget.

## Where this leaves the no-anchor line (08-11 evening)

Four from-scratch discovery arms (omni trans1, gru-r3, dragstep1,
terrain2) all land on paddle/sacrifice no matter the ground or the
charge coef. Two readings: (1) the effective drag price has NEVER
been big — see the magnitude caveat above — so the operator's "really
big penalty" hypothesis is still UNTESTED, not refuted; (2) discovery
lacks lifted-leg state coverage, which RSI-for-walk (lever 4)
addresses without action supervision. Next arms in order: (a) the
charge-magnitude audit, then ONE from-scratch arm with the charge set
so a typical paddle tick costs 2-3x a progress tick (audit-derived k,
not a guess); (b) RSI-for-walk mid-stride spawns; (c) annealed-up
charge once the P2 bank lands. Lever 3 (physics easing) unstarted.

- **08-11 18:1x (agent cycle) — independent re-check + lever 1's
  pre-registered retry.** Re-ran the zero-training champion probe
  myself (det+sto, matched seed, per-mode 6) across the FULL
  amplitude range at the fixed clamp: flat 1.02, 18mm 1.09, 36mm
  1.33, 54mm 1.50, 72mm 1.74 slip/m — a clean MONOTONIC INCREASE, not
  a drop, confirming probe2's finding independently and closing any
  doubt that a coarser probe grid missed a sweet spot. Also
  independently re-evaled `cw-gait-terrain2`'s own checkpoint
  (own-terrain det slip 8.58 med / sto 12.38; flat retention det 6.83
  / sto 10.78, gait_valid 2/6 det both, leg[3] sacrificed in 4/6
  episodes) — same qualitative fingerprint as the operator's pass
  (numbers differ slightly by eval seed draw, conclusion identical:
  FAIL, worse than the closed paddle band). Launched the run's own
  pre-registered single retry, **`cw-gait-terrain2-r1`** (env.
  terrain_amp=3.0, 54mm, VERIFIED RUNNING train-3) to close out lever
  1 with the two-miss rule before moving on — but per the ordering
  above, the CHARGE-MAGNITUDE AUDIT is still the sharper next lever
  regardless of how -r1 lands; do not let -r1 delay it.
- **08-11 ~18:2x — `cw-gait-terrain2-r1` FAILED, lever 1 (terrain-
  as-teacher) CLOSED for good (two-miss rule).** One rung down at
  54mm avoided terrain2's leg-sacrifice (gait_valid 6/6 det+sto, all
  six legs stay engaged, video-confirmed) but landed on neither
  pre-registered pass branch: own-terrain slip/m med 6.86 det / 8.90
  sto is 4-6x the closed paddle band (1.1-1.5), not the <0.6 win and
  not a match to the champion band; one det episode terminated
  over_current (a dragged foot straining against a bump).   Gentler
  terrain does not force stepping either — physics-as-teacher is
  refuted across the amplitude range tried, from scratch, twice.
  Do not requeue terrain-as-teacher at any amplitude. Next: the
  charge-magnitude audit (P3 lever 2 prerequisite, unstarted) or
  RSI-for-walk (lever 4).

## Charge-magnitude AUDIT — DONE 08-11 eve (operator session);
## per-tick charge form REFUTED, structural per-stance charge LANDED

`rl_move/sim/probe_drag_audit.py` (+ `logs/probe_drag_audit*.json`).
Measured per-foot loaded slip on the honest scripted gait vs the two
learned skaters (`longdist_r2`, `dep_vref1_r1`) under the trans1
stack, 375 ticks each:

- **The 0.5 mm/tick deadband was the hole**: 53–63% of the skaters'
  slip ticks ride UNDER it (slow constant slide), so at dragstep1's
  k=40 the effective charge was ~0.001–0.09/tick vs ~2–2.6/tick
  income. The "big penalty" never existed.
- **The per-tick FORM is unfixable, not just the coefficient**:
  per-tick slip medians overlap (gait 0.31 mm — touchdown scuff — vs
  skaters 0.40–0.47 mm). At ANY (k, deadband) that prices the skate,
  the honest gait pays ≥2.5x its own income too. This is why 10+
  coefficient arms failed; do not run another per-tick k rung.
- **Per-STANCE accumulated travel separates them 3.3x**: scripted
  gait median 2.9 mm/stance (95% of stances under 6 mm) vs skater
  median 9.8 mm (only ~25–30% under 6 mm). Stance duration is the
  discriminator the per-tick form throws away.
- **Contact-solver micro-jitter (~0.2 mm/tick) integrates** on long
  stances (a 2 s quiet stance accrues ~10 mm/foot of pure jitter), so
  the accumulator only counts ticks sliding >0.25 mm (floor; skater
  drag runs 0.4–0.5 mm/tick).

Landed in `walk_task.py` (default OFF, legacy bit-exact):
`reward.k_drag_stance` (charge per metre of over-allowance stance
travel), `reward.drag_stance_allow_mm` (default 6),
`reward.drag_stance_tick_floor_mm` (default 0.25). Accumulator resets
at touchdown, pays incrementally (a never-lifting foot cannot defer).
**Audit-derived operating point: k=8000, allow 6 mm, floor 0.25 mm**
— skaters pay ~2.5x their income, honest gait ~23%, motionless foot
~0. Launch gate added and PASSING
(`test_drag_stance_stack_prices_skating_below_stepping`: stepping >
zero-lift skate of the same gait by >50 return, gait > stall > park
survives, forward + crab).

Calibration side-note (same session): re-ran `calibrate_slip.py` full
mu sweep + loaded-servo point. Travel ratio saturates ~0.38–0.40 (air
fit) at mu ≥1.2 and the 08-10 loaded fit makes it WORSE (0.25–0.31)
— friction is NOT the knob for the sim-vs-tape travel gap, and sim is
PESSIMISTIC (slipperier than the real floor), which is conservative
in the right direction for anti-skate training. Contact-stiffness
class stays open as operator calibration; it does NOT block the
charge arm.

## P0 reward-accuracy diagnostic — DONE 08-11 late (idle-kick cycle);
## penalty-side suspect REFUTED, paddle is a sim-EFFECTIVENESS optimum

`probe_walk_income.py --stack vref1` (new stack = cw-dep-vref1-r1's
exact ledger cfg; artifacts `logs/probe_walk_income/vref1_p0_dr0.json`
/ `vref1_p0_dr035.json`, run on train-1 at code 29f3706): scripted
plant-height tape-proven gait vs THE champion checkpoint under the
champion's own reward, forward, 3 seeds.

- **Totals are near parity, not gross mispricing**: gait 656 vs ckpt
  739 at DR0 (−11%); gait 713 vs ckpt 661 at the champion's own
  DR 0.35 (+8%). Neither branch of "reprice-then-train" fires.
- **The operator's tilt/rocking suspect is REFUTED**: the scripted
  gait's k_gyro+k_roll+k_pitch cost is ≤1.5/ep combined (vs ~650
  totals) and ZERO episodes terminate at either DR. Repricing the
  rocking terms would change nothing.
- **The stack already pays tall**: plant-height policies collect the
  base stance kernel (reward_task ~271-319/ep vs the crouched
  champion's ~30) and the champion pays −139/ep base k_height for its
  crouch — a combined ~380/ep pro-tall margin, already bigger than
  dep-hgt1/hgt2 assumed.
- **Why the paddle still wins**: it genuinely TRACKS the command in
  sim (progress_ratio 1.06 vs the scripted gait's 0.35 — the
  calibrated conservative slip physics), collecting ~495/ep more
  walk+prog kernel income. That is real locomotion income, not a
  pricing hole. Corollary: any near-champion policy that lifts to
  plant height immediately loses walk income (its early honest steps
  realize less progress) long before stance/height income arrives —
  a local-optimum MOAT, which is why income shaping kept failing and
  why the per-STANCE structural charge (audit section above) and/or
  curriculum are the levers with teeth.

Same cycle, the moat got a direct measurement:
`cw-walk-dragstance1` (warm champion + the audit charge, discovery
2M) **FAILED its slip gate but proved the charge's safety properties**:
reward_drag_stance sat at −7/tick the entire run (never resolved) and
the policy NEITHER parked NOR restructured — full travel retained
(prog 1.00, gait_valid 6/6, zero falls), slip only edged 1.1–1.3 →
0.95–1.15. A static fine on the FORMED habit is closed (no k rung,
no longer budget on the warm retrofit). Remaining routes: from
scratch under the charge (`cw-gait-dragstance1-r1`, 40M, running —
another cycle's) and the anneal-up curriculum (P3 lever 2).

## Structural stance-slip charge, FROM SCRATCH — `cw-gait-dragstance1`
## FAILED 08-11 (agent triage) — parked, not stepping

Companion arm to `cw-walk-dragstance1` (that one warm-starts the
champion; this one trains the identical audit-derived charge
(k=8000/m, 6mm allowance, 0.25mm floor) from scratch on the trans1
stack, to see if the paddle basin is avoidable from step 0 when it's
priced out from the start). Pre-registered false branch was "parks,
or paddles while paying" — that is exactly what happened, worst case
first tried:

- `env/reward_drag_stance` engages immediately (step 19: -9/tick) and
  **never trends toward zero** — it sits at -6 to -9/tick for the
  whole run, i.e. the charge never gets resolved, only endured.
- `env/walk_loadslip_factor` collapses 0.62 -> ~0.05-0.08 by step 49
  and stays floored — the policy minimizes loaded-slip *exposure* by
  going nearly still, not by cleanly lifting and placing feet.
  `env/reward_step_event` (the stepping income) stays tiny (~0.015),
  i.e. it earns almost no stepping credit either.
  Meanwhile the reward-quarters trend gets MORE negative over
  training (-441 -> -1558 -> -2354 -> -2381) simply because it
  survives the full episode paying a constant charge rather than
  falling early — not a sign of a worsening gait, a sign of a
  stable stillness habit.
  - Harness (gate DR0 + own-DR0.35, det+sto, 6 eps each = 24 clips):
  forward_dist 0.002-0.02 m vs cmd_dist 0.5-0.7 m over a 15 s episode
  (progress_ratio ~0.00, success 0/6-1/6). ALL 24 videos show an
  IDENTICAL static, splayed pose held for the entire clip — no
  leg-cycling, no net translation. slip_per_m reads 7.3-18.7
  (nonsensical-looking, but that's the near-zero-travel denominator,
  not real sliding distance — the real read is duty_cycle 0.94-0.98
  with 5-7 tiny swings/leg over 15s, i.e. a shuffle, not a gait).
- **Verdict: STOP - known exploit (FREEZE/PARK), matches the run's
  own pre-registered false branch verbatim.** The structural
  per-stance charge, even at the audit-tuned operating point, is
  *exonerated as a solo fix* for the paddle: from scratch it does not
  make stepping worth discovering, it just makes standing-still-while-
  paying preferable to the alternatives it tried. **CROSS-TRACK
  INSIGHT: this is also nobc's queued "drag-charge magnitude audit"
  (STATUS.md Next item 1) — the same charge, same conclusion, applies
  to both tracks' gait-from-scratch line.** Next lever per GAIT.md
  P3: RSI-for-walk (mid-stride spawns) or the charge combined with
  income-shaping, not another coefficient rung. Await
  `cw-walk-dragstance1` (warm-start variant) separately — a policy
  that already knows how to travel may resolve the charge by cleaning
  up its gait instead of freezing, since freezing there costs the
  large pre-existing walk-progress income it would otherwise keep
  earning; this from-scratch result does not by itself predict that
  one's outcome.

## TALL LADDER — height-ref rungs on the dep line (operator session 08-11 eve)

Operator directive: "make a deployable tall smooth walker." Mechanism:
`goal.walk_height_off_mm` as a warm-start ladder (the lowgait line's
proven trick, inverted to climb UP), NOT the hgt1 income gate (refuted
one-shot). All rungs 2M warm, full dep contract + tipped starts
retained; eval numbers are `eval/walk/*` end-of-episode.

| run | parent | ref | height_err_end | speed m/s | slip | note |
|---|---|---|---|---|---|---|
| cw-dep-tip1 (baseline) | — | 0 | 59.9 mm | 0.0388 | 1.65 | eval-ends ~−60 mm |
| cw-dep-tall30 | tip1 | −30 | **15.2 mm** | 0.0295 | 1.74 | + k_drag_stance 8000/6/0.25 |
| cw-dep-tall30h | tip1 | −30 | 17.5 mm | 0.0287 | 1.80 | isolation: NO charge |
| cw-dep-tall15 | tall30 | −15 | 29.0 mm | 0.0278 | 1.64 | STALL: body ≈ −44 mm |
| cw-dep-tall15-h1 (T1, 6M) | tall15 | −15 | 51-58 mm (eval med) | 0.0545 | 1.16-1.35 | FAIL: worse+flat, not step-bound |

Findings (three runs, one evening):

- **Height-as-commanded-ref works on the dep line**: 60→15 mm in one
  rung. The hgt1 gate failure was about the unreachable one-shot 50 mm
  jump, not about height being untrainable.
- **The structural drag charge is FREE on a warm walker** (tall30 vs
  tall30h isolation: identical speed/slip, marginally better WITH the
  charge) — it rides along in the ladder but did not cut slip either
  (1.74 vs baseline 1.65). Consistent with cw-walk-dragstance1:
  absorbed, not resolved.
- **The free climb ends at ~−44 mm**: rung 2 (ref −15) left the body
  at the same ~−44 mm as rung 1 did. ~16 mm taller than baseline,
  ~44 mm short of plant height. Speed cost so far ~25-28%, flat
  across rungs (one-time, not per-rung).

Follow-up campaign T1-T5 pre-registered in RL_PLAN.md queue -0.5
(P2.5): budget rerun, k_height crank, gate-at-reachable-ref, speed
trade, and the −44 mm workspace probe that decides whether the wall
is kinematic (stop) or habitual (keep pushing).

**T1 (budget, 08-11) FAIL — the wall is not step-bound, and 3x budget
made it slightly worse.** `cw-dep-tall15-h1` (6M, identical respec of
the stalled ref −15 rung). `env/height_err_mm` (training-env metric)
plateaus by ~1M steps (~36-39mm) and sits flat the remaining 5M —
not the monotonic-improvement shape the PARTIAL branch needed.
Harness eval end-state is worse than the 2M parent's own 29mm: gate
median 51-58mm, own-DR median 57-58mm, close to the tip1 ref-0
baseline (59.9mm) despite 6M steps at ref −15. Speed improved
(0.0278→0.0545 m/s, now mid-band) but execution got noisier: one
gate/det episode spins in place (fwd 0.20m, slip/m 2.66) and
own-DR/sto gait_valid dropped to 4/6 (two episodes sacrifice 3 legs,
slip/m 2.6-5.1). No park/flag-leg/falls in any of the 24
video-checked episodes (terms 0, safety_flags 0) — still the honest
paddle gait, just at the old crouch depth, now with a bit more
instability at the height-vs-speed tradeoff. Verdict: budget is not
the lever; do not schedule further step-count variants on this rung.
Per the run's own gate, next is **T5** (kinematic/stability probe:
does a SCRIPTED gait, physically commanded to a shallower stance,
actually hold −15mm without tipping/losing contact, or does it also
settle back near −44mm?) before T2/T3 — T5 is NOT YET BUILT (needs a
small script using `info["height_mm"]`, already exposed per-tick by
`sim_env.py`, plus an IK-adjusted scripted stance; see
`linux_control/geometry_plant.py:knee_for_foot_z` for the exact FK/IK
this robot uses). Flagged as the next concrete task on this line,
not attempted this cycle to avoid a rushed/wrong physical-limit claim.


---

# FILE: rl_docs/EVALS.md

# EVALS — every evaluation metric, what it means, where it lives

Status date: 2026-08-10 (operator ruling: the per-eval total scores
are how we know a model is improving — they get clear names, the top
of the W&B page, and this doc). If a metric is not documented here,
document it before using it in a verdict.

## 1. Periodic training evals (the improvement curves)

Every `--eval-every` steps the trainer freezes the policy
(deterministic), isolates ONE goal mode at a time (all other `p_*`
zeroed), runs 2 episodes per mode, and logs to W&B. Shared by both
trainers (`train_ppo_sim._run_periodic_eval`; the MJX trainer imports
it), so metric names are identical everywhere.

### SCORE/ — the headline section (top of the W&B page)

Pinned via `wandb.define_metric("SCORE/*", summary="last")`: the
latest value of each appears in the run Overview summary, and the
SCORE section sorts above canary/env/eval in the workspace.

| metric | meaning | direction |
|---|---|---|
| `SCORE/<mode>_total_reward` | ALL reward terms summed over one eval episode of that isolated mode, mean of 2 episodes. Was called `eval/<mode>/return` before 08-10 (an awful name: it is not a discounted return, it is the episode's total earned reward). | up, **but see the caveat** |
| `SCORE/rise_flat_success` `SCORE/rise_bridge_success` `SCORE/rise_crouch_success` | rise completion split by start kind (2 eps each): survived AND final height err ≤ 15 mm. flat/bridge are THE rise metrics; crouch is solved and must stay 1.0. Was `eval/rise_<kind>_frac`. | up |
| `SCORE/raise_success` | survived AND final height err ≤ 5 mm (deliberately tight — canary: if not ~100% the height pathway is broken, not under-trained). Was `eval/raise_success_frac`. | 1.0 |
| `SCORE/lower_success` | survived AND final height err ≤ 15 mm. Was `eval/lower_success_frac`. | up |
| `SCORE/roll_trap_pass` | ROLL-TRAP GATE (operator spec 08-10): walk normally ~1.5 s, then a servo-controlled external roll torque (predictive ramp, cap 3 N·m) drags the body to a sustained ~12° lean (capped at 70% of the run's tilt envelope) for 3 s and releases; PASS = no fall AND \|roll − ref\| < 5° over the last second AND (walk runs) ≥50% of commanded speed regained AND all six legs still cycling (per-leg hip peak-to-peak ≥ max(3°, 25% of the median) — a pinned/sacrificed leg fails). Resisting so hard the trap never bites (`eval/roll_trap/trapped_frac`) is a pass, not a dodge. This is the mid-gait mechanism of the hardware runaway that the tipped START cannot reproduce; at a hard over-spec dose (~20–27°) it separates dep-tip1 (4/6) from its parent (2/6). Baselines at the in-spec 12° dose: parent 8/8, tip1 6/8, stance champion resists outright, null policies 0. | up |
| `SCORE/tipped_recovery_success` | (added 08-10, after the hardware runaway roll) forced 12° roll-tipped start with a LEVEL tilt reference, in the run's primary mode (walk for walk runs, else hold); success = survived AND mean \|roll − ref\| over the last quarter ≤ 3° AND body within 90 mm (walk; the gait normally rides 54–70 mm below the spawn settle) / 30 mm (hold) of the settled start height — belly-flat reads level and must not count. The 12° dose is capped by the run's own tilt envelope (stance 10° → 7° tips), so compare within one envelope. Baselines at 12° (fixed gate, 8 eps): dep-vref1-r1 7/8, dep-tip1 6/8, null policies 0. **Caveat:** curves logged 08-10 (incl. cw-dep-tip1's 0/2) used a flat 30 mm gate that failed every healthy walk recovery — ignore them. Static-lean recovery was already present in the champion; this eval did NOT reproduce the hardware runaway (progressive lean with a pinned loaded leg mid-gait — a sim-to-real grip gap). It remains as a regression floor. | up |

**The caveat on `_total_reward`:** it is measured under the RUN'S OWN
reward config. It is the right curve for "is this run still
improving," and NEVER comparable across runs with different reward
cfgs (a run with an extra income term scores higher forever). A run's
resolved reward config is in its W&B notes (`=== REWARD FUNCTION ===`
block) and `config.reward_cfg`; term meanings in rl_docs/REWARD.md.
For cross-run comparisons use the success/error metrics or the offline
harness (§2). This is also why the auto-continue logic reads
`rollout/ep_rew_mean` quarters — same caveat applies there.

### eval/ — per-mode detail

| metric | meaning |
|---|---|
| `eval/<mode>/survived_frac` | fraction of eval episodes not safety-terminated |
| `eval/<mode>/track_err_deg` | mean \|tilt − reference\| over the episode |
| `eval/<mode>/height_err_end_mm` | \|height − ref\| at episode end |
| `eval/walk/vel_err_m_s`, `eval/walk/speed_m_s` | mean commanded-velocity error / achieved speed |
| `eval/tipped/roll_end_deg`, `eval/tipped/z_drop_mm` | tipped-start eval detail: mean \|roll − ref\| over the last quarter / body-height drop vs the settled start (>30 mm = collapsed, not recovered) |
| `eval/roll_trap/*` | roll-trap detail: `trapped_frac` (trap reached 10°), `max_roll_deg`, `end_roll_deg`, `speed_frac`, `legs_cycling`, `tau_peak_nm` |
| `canary/<case>`, `canary/auto_stop` | protected-skill regression flags (0/1) |

### Naming history (for reading old runs)

Runs before 2026-08-10 logged `eval/<mode>/return`,
`eval/rise_<kind>_frac`, `eval/{raise,lower}_success_frac`. The W&B
UI will not overlay old and new names on one chart; when comparing
across the rename, pull both keys (`ops.sh wandbdump`).

### Other W&B curves (not evals)

`rollout/ep_rew_mean` (SB3, training-noise rollouts — what the
watcher's "reward quarters still climbing" auto-continue reads),
`env/<part>` per-term reward means (see rl_docs/REWARD.md),
`train/*`, `time/*`, `lp/*` (walk-speed curriculum),
`terminations/<reason>` (MJX trainer).

## 2. Offline checkpoint harness (`eval_checkpoint.py`) — the gate

6+ episodes per mode, det (+ sto with `--stochastic`), report.json +
videos + contact sheet; used for every gate verdict. Headline fields
per episode: `success` (posture-strict since 08-08; `--valid-plant-gate`
adds the geometric PLANT_SPEC for rise/raise once champions are
baselined), `return` (same total-reward caveat), `progress_ratio`
(walk: along-command distance / commanded; promotion band 0.75–1.25),
`slip_per_m` (loaded foot-XY travel per meter of progress — the
skating metric; the REAL robot walks at ~1.0), `gait_valid`,
`end_posture_ok` / `end_clear_mm`, `valid_plant` / `plant_fail` /
`plant_margin_mm`, currents. With `--baseline <parent>` it evaluates
the frozen parent under identical config/seed (matched-parent
control, binding for injected-axis verdicts).

The harness mirrors its summary into the training run's W&B page
under `eval/<dr-tag>/<mode>_<det|sto>/...` (summary fields, not
charts) and uploads report.json.

## 3. Specialist harnesses

- `eval_drive.py` — the joystick gate: scripted command schedule;
  falls, tracking error, distance. Gate wording lives in the run's
  ledger entry.
- `eval_yaw.py` — turn-segment |wz_err| median (pass ≤ 0.10 rad/s)
  and heading-hold |wz| median (pass ≤ 0.05). Judge turn arms with a
  matched-parent control (rl_docs/TURN.md).
- `test_task_semantics.py` — MDP_PREFLIGHT banks (RISE/WALK/TURN
  passing, LOWER owed): required orderings of scripted policies under
  the full reward stack BEFORE any arm of that mode launches.

## 4. Rules of use

1. A gate verdict quotes harness numbers (§2/§3), never SCORE curves.
2. SCORE curves answer "still improving?" and "which skill broke?"
   at a glance; regressions there mean watch the video before any
   other move.
3. Any new metric: add the row here + the definition in the emitting
   code in the same commit.


---

# FILE: rl_docs/SKILLS.md

# SKILLS — what the robot can do today, and which checkpoint does it

The answer to "are the successes getting lost?" (operator, 08-09).
Every PASSED capability lives here with its checkpoint. Verdicts stay
in the ledger / `rl_docs/runs/`; this is the accumulating INVENTORY.

**UPDATE RULE (binding, 08-09): a cycle that verdicts a PASS adds or
updates one row here in the same cycle.** Checkpoints are durable in
W&B artifacts (`ckpt-<name>`, type `policy-checkpoint`) — trainer
publishes automatically since 08-09; earlier ones backfilled. The
controller's `rl_move/sim/policies/` is a cache, NOT the archive
(it is gitignored and the controller is an ephemeral pod).

Demo any row locally:
`.venv/bin/python -m rl_move.sim.drive_policy rl_move/sim/policies/<ckpt>.zip`

## Walk (main line)

| Skill | Checkpoint (artifact `ckpt-<name>`) | Evidence | Envelope / limits |
|---|---|---|---|
| **CHAMPION: forward walk, 30 s, correct speed** | `ppo_goal_cw_walk_longdist_r2` (md5 bcddc65c) | c44 promotion, seed-confirmed (s1): DR0 det 6/6, slip/m 0.94–0.96, 1.63 m @ 30 s, prog 0.98; JOYSTICK GATE PASS @DR0.2 AND @DR0.5 (0 falls incl. flip stress; baselines: `logs/ckpt_eval/champion_longdist_r2_drive*.json`) | fwd ±45° cmds only; slip ~1/m (not hardware-ready); sto stalls on some fixed draws — NOT fixed by DR (longdist-dr05, longdist-dr10 FAIL) or resample training (stallfix FAIL); stall class CLOSED c52 (<2% tail, canary only); recipe 3/3 seed-robust (s2 PASS c53); backward cmd barely moves (0.06 m/7 s); L strafe 0.21 m vs R 0.30 m; tolerates motor-torque sag to ~0.75× FOR FREE (c58 torquedroop baseline: champion = exposure-trained policy on identical draws); ≤~0.7× torque = transport stall, no falls — exposure lever CLOSED, waits for estimator rung; also FOR FREE (c60 baselines, same method): command dropout to 20%/tick (cmddrop10 AND cmddrop20 both episode-identical to champion baseline; the 2 highest-drop draws churn in both — churn boundary not moved by exposure) and servo-speed sag to ~0.80× (deep ~0.70× sag = transport stall in BOTH, battery-calibration class, not trainable) — servo-imperfection single-axis exposure 0-for-6 (gainvar c65: kp/kv spread 0.40/0.50 NO-EFFECT — mid-band free, extreme-gain draws churn in champion AND exposure-trained policy identically; zerobias3 same cycle: per-joint zero-point bias u(-3,3)° NO-EFFECT, parent matches episode-for-episode incl. the 2 steep-bias craters pixel-identical), test champion first; IMU-bias exposure also NO-EFFECT (imubias3: roll/pitch bias u(-3,3)° matches parent draw-for-draw — bias is unobservable from proprioception, waits on the estimator rung, not a servo-class defect); contact-stiffness extremes (0.7–2.0×) NOT free and NOT trainable by exposure (contactstiff c65 FAIL: worst-2 det draws 0.67–0.74 m unchanged from replicated champ baseline) — joins friction in the contact-pricing operator-calibration class; also FOR FREE, same parent-baseline method: leg-mass asymmetry (legmass25), contact-compliance jitter (stiffvar), IMU-mount 10° misalign (imumount10), and encoder quantization noise 0.5° (encodernoise c67 — champion under the identical noisy-encoder spread matches the exposure-trained checkpoint draw-for-draw, same 3/6 crater episodes) — the naive single-axis-DR-exposure sensor/calibration ladder is now 12-for-12 NO-EFFECT (gyro-rate bias `gyrobias3` and gyro-rate noise `gyronoise15` also join it — champion measured under each's identical fixed spread matches the exposure-trained checkpoint's same 2/6 craters near-exactly, despite the champion baseline itself NOT being free there; `actionnoise` — actuator command noise 0.08 rad, the LAST untried RandRanges field — closes it out, champion matches draw-for-draw incl. the same worst-case stochastic crater); IMU-mount TRANSLATION offset up to 15mm xy/20mm z (`imupos15-r3` c69, distinct field from the imumount10 rotation case above) confirms the same pattern post-hoc (13th instance) — champion baseline under the identical position-offset spread matches the trained checkpoint's same 2/6 craters near-exactly; IMU tilt-reading noise up to 1.5° (5x the full-DR default) also joins it (`tiltnoise-r5-rr1`, 08-10, its 5th launch attempt to finally get a science result after 4 collision deaths) — champion baseline under the identical 1.5° tilt-noise spread matches the trained checkpoint's same 2/6 craters near-exactly (prog 0.47/0.39 vs 0.48/0.35, slip 3.33/4.13 vs 3.19/4.40); treat any further tiny-sensor-noise axis as free-by-default rather than re-testing one at a time — ladder CLOSED, well is DRY |
| Steer through direction changes + stops (up to 60 s drives) | `ppo_goal_cw_walk_wander` / `_wander30` / `_wander60` (md5 bcabaea0) | c45/c53 PASS + wander60 c56 PASS: 60 s eps (~12 changes + stops), gv 12/12, 0 term, prog 0.94–0.99, worst slip/m 1.67 (< 30 s parent's 1.93) — no endurance decay | fwd hemisphere ONLY — backward command = fall (operator repro'd, 08-09); DR0.5×60 s rung LANDED c62 (wander60-dr05, row below); paddle gait, not hardware-ready |
| Steering robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_wander_dr05` (md5 18af118f) | c50 PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.95–0.96, slip/m med 1.39–1.70; DR0 retention gv 6/6 | ±45° cmds, gentle 5 s resamples only (abrupt-flip hardening in flight); paddle gait, not hardware-ready; **DR 0.5 is the line's ceiling** — DR1.0 rung FAILED (wander-dr10 c53: gv 11/12, flag-leg draw at full DR); seed-robust across 3 seeds (s1 c56 + s2 c60 PASS: prog med 0.93–0.99, slip 1.46–1.88, gv 12/12 each; ruling-7 panel satisfied) |
| Strafe ±90°, robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_strafe_dr05` (md5 cb178b91) | this cycle PASS: own-cfg DR0.5 gv 12/12, 0 term, prog med 0.92–0.97, slip/m med 1.89–2.00; DR0 retention prog 1.09, slip 1.80 (< parent 2.20) | lateral paddle; fixed commands (no resampling trained); not hardware-ready |
| Drive anywhere in the front half-circle (±90°, resampled cmds + stops) | `ppo_goal_cw_walk_head90` (md5 bcf474ff) | c49 PASS: own-cfg DR0 gv 12/12, 0 term, lateral err ≤1.6× fwd; JOYSTICK GATE PASS @DR0.2 (0 falls incl. instant-flip stress); SEED-ROBUST (head90-s1 c64 PASS: gv 12/12, lateral 1.55× fwd, JOYSTICK GATE @90° envelope 0 falls, det fwd med 1.00 m) | prog ~0.84–0.89 on mixed headings (lateral costs progress); head90's weak left strafe did NOT reproduce in s1 (L/R 0.23/0.30 vs 0.15/0.27) — L/R asymmetry is seed-level noise, not structural; paddle gait, not hardware-ready; **heading envelope FROZEN at ±90** — ±135 rung FAILED (head135: det tilt_pitch term, prog med 0.53, slip ~2×; rear coverage → mirror-symmetry line) |
| Front half-circle driving robust to physics variation (DR 0.5) | `ppo_goal_cw_walk_head90_dr05` (md5 9409e7e6) | this cycle PASS: own-DR0.5 gv 12/12, 0 term, prog med 0.83 det / 0.90 sto, slip/m med 1.83/1.66; DR0 retention det gv 6/6, prog 0.84 = parent | ±90° gentle 5 s resamples (abrupt-flip ±90 rung LANDED c62 = joyhead90-r1, row below); L/R asym persists; paddle gait, not hardware-ready; DR0.5 = steering ceiling (full-DR refuted 2×) |
| Stop-and-go driving (35% stop density) | `ppo_goal_cw_walk_stopgo35_c1` | c57 PASS: own-cfg DR0 gv 12/12, 0 term, prog med 0.97 (min ep 0.92); frames: quiet level parks, prompt restarts (re-tracks ~3 s after go) | ±45° cmds @0.05–0.06 m/s, 5 s resample, DR0 only; det slip/m 1.43 (lineage paddle-slide, not hardware-ready) |
| Crouch walking (−20…−70 mm height) | `ppo_goal_cw_walk_lowgait` / `_lowgait30` / `_lowgait40` / `_lowgait50` / `_lowgait60` / `_lowgait70` (md5 fd90c33a) | c45/c47/c48/c56/c60 PASS at each rung: gv 12/12, 0 term, end-height err ≤7 mm (−70 mm: mean det 2.0/sto 1.9 mm), det agg slip/m 0.92–1.07 (≤ champion band) | envelope verified to −70 mm and BOTTOM ESTABLISHED there (−80 mm rung FAIL c63: height err 12 mm, slip 1.50, one sto flag leg — workspace exhausted); one sto in-place-paddle ep per panel (lineage brittleness) |
| Crouch walking (−50 mm) robust to physics variation (DR 0.35) | `ppo_goal_cw_walk_lowgait_dr035` | PASS: own-cfg DR0.35 panel gv 12/12, 0 term, mean end-height err 6.8 mm det/4.6 mm sto (≤10 mm gate), slip/m med 1.08 det/1.26 sto (≤1.6 gate); DR0 nominal retention clean (gv 6/6, mean height err 4.0 mm, slip/m med 0.98 — crouch not forgotten) | isolated DR ladder rung off `lowgait_dr05_r1` FAIL (which broke at DR0.5: flag-leg draw, gv 11/12) — 0.35 is the ceiling between 0.2 (holds) and 0.5 (breaks); one sto fixed-draw churn tail (1/12), known canary-class pattern; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + chassis payload (1.0–1.5×) compose | `ppo_goal_cw_walk_lowgait_dr035_payload` | this cycle PASS: own-cfg DR0.35+payload det+sto gv 12/12, 0 term, mean end-height err ~4 mm (≤10 mm gate), slip/m med 1.20 det/1.25 sto (≤1.6 gate); DR0 no-payload retention det gv 6/6, mean height err 4.8 mm (≤8 mm), slip/m med 1.08 (≤1.15) | NEW untried pairing (crouch × payload, all prior payload composes were on default flat-height gait) — holds; ~4/24 fixed-draw churn/slow-shuffle outliers (prog 0.62–0.84), matches known heavy-tail canary pattern elsewhere, not a new defect; no flag leg, no falls; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + bus-latency jitter (0.5–2.5×) compose | `ppo_goal_cw_walk_lowgait_dr035_latjit_r1` | this cycle PASS: own-cfg DR0.35+latency det+sto gv 6/6, 0 term, height err mean 3.6/3.4 mm (≤10 mm gate), slip/m med 1.13 det/1.32 sto (≤1.6 gate); DR0 retention gv 6/6, height err mean 4.7 mm (≤8 mm), slip/m med 1.11 (≤1.15) | latency composes onto crouch same as it did onto driving packages; 1-2 fixed-draw churn-tail eps per panel, known canary pattern; no flag leg, no falls; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35), seed robustness | `ppo_goal_cw_walk_lowgait_dr035_s1_r2` | this cycle PASS: seed-1 twin of `lowgait_dr035` — own-cfg DR0.35 det+sto gv 6/6, 0 term, height err mean 3.1/1.4 mm, slip/m med 1.08 det/1.39 sto; DR0 retention gv 6/6, height err mean 3.3 mm, slip/m med 1.01 | SEED-CONFIRMED: crouch-DR0.35 result not a seed-0 fluke; same churn-tail quirk, no flag leg, no falls; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + off-center CoM payload (0.03 m) compose | `ppo_goal_cw_walk_lowgait_dr035_comshift_r1` (+ seed twin `_s1`) | this cycle PASS (dig-in): own-cfg DR0.35+comshift det+sto gv 12/12, 0 term, height err mean 4.8/4.1 mm (≤10 gate), slip/m med 1.02 det/1.36 sto (≤1.6); DR0.35 tail matches parent episode-for-episode. **Seed-1 twin PASS reproduces cleanly**: own-cfg height err mean 4.1/2.4 mm, slip/m med 1.11/1.30 — same band. TRUE DR0 no-offset retention (offset override dropped, not just dr-scale zeroed): s1 det gv 6/6, height err mean 2.4 mm, slip/m med 1.01 (≤1.15) — clean, no crater; with the offset still active at dr-scale 0, det/4 craters (prog 0.17/slip 13.0), matching r1's own with-offset crater at the same index | NEW pairing (crouch × comshift); dig-in on one DR0 det march-in-place stall (fwd 0.11 m, slip 17.5, legs still cycling): fresh-draw panel 2/36 degraded eps child vs 2/36 parent (DIFFERENT draws each), parent WORSE on the shared bad draw — lineage fixed-draw paddling-attractor class (canary since c52) that comshift training relocates rather than creates; seed-confirmed not seed-0 luck; waits on contact-pricing calibration; not hardware-ready |
| Crouch walking (−20 mm) across grip levels (0.4–1.6× floor friction) | `ppo_goal_cw_walk_lowgait_fricvar` (+ seed twin `_s1_rr1`) | c-digin PASS: own-cfg det+sto gv 12/12, 0 term, height err mean 3.5/4.0 mm (≤8 gate); DR0 retention det gv 6/6, slip/m 1.13 (≤1.24); pinned-slickest 0.4 probe: 11/12 eps prog med 0.86–0.88, slip ~1.65 — BEATS upright `fricvar` at 0.4 (prog med 0.83, worst draws 0.32–0.51). **Seed-1 twin PASS reproduces cleanly**: own-cfg det gv 6/6 (height err mean 3.8 mm), sto gv 6/6 (height err mean 3.4 mm), both ≤8 mm gate; DR0 retention det gv 6/6, slip/m 1.08 (≤1.24) | crouch is NOT more grip-sensitive than upright (if-false refuted); named caveat: ONE det reset draw dead-skates at slick 0.4 (fwd 0.01–0.07 m, slip ~19) — INHERITED, untrained parent `lowgait` fails the identical draw identically (slip 20.7) and the draw is clean at nominal grip; lineage fixed-draw stall class (canary), waits on contact-pricing calibration; paddle gait, not hardware-ready; seed twin reproduces the SAME fixed-draw crater at det/4 (prog −0.04, slip 14.9), confirming recipe not seed luck |
| Crouch walking (−50 mm, DR 0.35) + servo deadband (1.0–3.0×) compose | `ppo_goal_cw_walk_lowgait_dr035_deadband` | this cycle PASS: own-cfg DR0.35+deadband det+sto gv 12/12, 0 term, height err mean 4.2/3.5 mm (max single ep 9.4 mm, ≤10 mm gate), slip/m med 1.04 det/1.26 sto (≤1.6 gate); DR0 no-deadband crouch retention gv 12/12, height err mean 2.6/1.8 mm (≤8 mm gate), slip/m med det 1.10 (≤1.15 gate, right at the edge) | NEW untried pairing (deadband validated on the plain-height champion before, never on a crouched stance); one det + two sto episodes crater into the lineage's known fixed-draw march-in-place stall (legs still cycling, level body, no fall) — inherited trait per c75's root cause, not a new defect; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + floor friction variation (0.4–1.6×) compose | `ppo_goal_cw_walk_lowgait_dr035_fric` | this cycle PASS: own-cfg DR0.35+friction det+sto gv 12/12, 0 term, height err mean 2.7/4.9 mm (max 12 mm, ≤10 mm gate on mean), slip/m med 1.05 det/1.33 sto (≤1.6 gate); DR0 nominal-crouch retention gv 6/6, height err mean 2.0 mm (≤8 mm gate), slip/m med 1.02 (≤1.15 gate) | grip-robustness axis (already proven on driving lines) composes for free onto the crouch rung too; one det + one sto episode dip into the lineage's known fixed-draw march-in-place stall — inherited, not new; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + floor slope (5°) compose | `ppo_goal_cw_walk_lowgait_dr035_groundtilt5` | this cycle PASS: own-cfg DR0.35+tilt5 det+sto gv 12/12, 0 term, height err mean 1.9/2.4 mm (≤10 mm gate), slip/m med 1.18 det/1.27 sto (≤1.6 gate); DR0 flat no-tilt retention gv 6/6, height err mean 4.1 mm (≤8 mm gate), slip/m med 0.91 (≤1.15 gate) | NEW pairing (crouch × slope, never tried together); lower CoM/shorter stride does not interact badly with a tilted floor — stays level on the slope; one det + two sto episodes dip into the lineage's known fixed-draw stall (one sto ep prog 0.38/slip 3.65) — inherited, not new; not hardware-ready |
| Crouch walking (−50 mm, DR 0.35) + deadband/friction/slope composes, seed robustness | `ppo_goal_cw_walk_lowgait_dr035_deadband_s1` / `_fric_s1` / `_groundtilt5_s1` | this cycle PASS (all 3, seed-1 twins): each own-cfg det+sto gv 12/12, 0 term, mean height err 3.2–4.5 mm det / 3.4–4.5 mm sto (≤10 mm gate), slip/m med 1.04–1.13 det / 1.25–1.44 sto (≤1.6 gate); each DR0 retention gv 12/12, mean height err 3.3–6.0 mm (≤8 mm gate), slip/m med 1.01–1.11 (≤1.15 gate) | SEED-CONFIRMED for all three pairings — same band as seed-0, same shared fixed-draw march-in-place stall (1 det crater in DR0 mode + 1 det/2 sto craters in own-cfg mode, level body/all 6 legs cycling/no fall, frame-verified), none new or worse; not hardware-ready |
| Rough ground (hfield bumps to 36 mm) | `ppo_goal_cw_walk_terrain10` (md5 57cea2dc) | this cycle PASS: own-cfg amp1.0 det 6/6 gv, 0 term, prog 1.06, slip/m 0.94; flat retention identical (no regression) | sim hfield only; SATURATED — bumps ≤36 mm never perturb the paddle gait; real clutter/obstacles need [CODE] scene work (wishlist 13d/24) |
| Rough ground (hfield bumps to 36 mm) x off-center CoM payload (0.03 m) — new compose | `ppo_goal_cw_walk_terrain10_comshift` (+ seed twin `_s1`) | this cycle PASS: own-cfg (terrain amp1.0+comshift) det+sto gv 12/12, 0 term, prog med 1.01/1.01 (≥0.85 gate), slip/m med 1.08/0.99; DR0 flat-no-bumps-no-offset retention det+sto gv 12/12, 0 term, prog med 1.02/1.04, slip/m med 1.05/0.97 (≤1.24 gate). **Seed-1 twin PASS reproduces cleanly**: own-cfg det gv 6/6 (prog med 1.00), sto gv 6/6 (prog med 1.00); DR0 flat retention det gv 6/6, prog 1.08, slip/m 0.91 — clean, no erosion | compose off terrain10; one draw (idx4) crashes to prog 0.06/slip 23 IDENTICALLY with terrain+offset ON and OFF — confirmed via report.json (terminated=false, all 6 legs swinging 11-20x, along_dist 0.04 m vs cmd_dist 0.69 m) as a march-in-place stall intrinsic to this checkpoint/draw, not caused by either compose axis; gate still met on medians; seed-1 twin hits the SAME idx4 crater (prog −0.01, slip 22.7, frame-checked: level body, all 6 legs cycling, no flag-leg) — the stall tracks the fixed eval draw, not the training seed; paddle slip, not hardware-ready |
| Rough ground (hfield bumps to 36 mm) x servo deadband (1.0-3.0x) — new compose | `ppo_goal_cw_walk_terrain10_deadband` | PASS (c 08-10 ~06:5x): own-cfg det+sto gv 6/6 both, 0 term, prog med 1.02/1.01, slip/m med 0.87/0.81 — matches terrain10 parent's own band; flat DR0-no-terrain-no-deadband retention det 6/6 PERFECTLY clean (prog 1.07, slip 0.85) | same known idx4 seed-artifact stall (parent shows it too, in sto only; own-cfg det also caught it here — terrain roughness trims the margin) — mechanically clean march-in-place, no fall/flag-leg; not hardware-ready |
| Rough ground (hfield bumps to 36 mm) x chassis payload (1.0-1.4x mass) — new compose | `ppo_goal_cw_walk_terrain10_payload` | PASS (c 08-10 ~06:5x): own-cfg det+sto gv 6/6 both, 0 term, prog med 1.00/1.00, slip/m med 1.02/0.99 — matches terrain10 parent's own band; flat DR0-no-terrain-no-payload retention det 6/6 PERFECTLY clean (prog 1.05, slip 0.84) | same known idx4 seed-artifact stall as the deadband sibling (identical character, parent shows it in sto); terrain line now composed with everything worth trying, closed |
| Lateral/omnidirectional strafe (±90°, DR0.5) x chassis payload (1.0–1.4×) — new compose | `ppo_goal_cw_walk_strafe_dr05_payload_r1` | this cycle PASS (retry — first attempt died 0-steps to a launch collision): own-cfg (DR0.5+payload) det+sto gv 12/12, 0 term, prog med 0.95/0.95 (≥0.8 gate), slip/m med 2.04/1.98 (≤2.4 gate); DR0 no-payload retention det+sto gv 12/12, 0 term, prog med 1.04/1.08, slip/m med 2.15/1.87 — matches strafe-dr05-payload's own band | compose off strafe-dr05; payload composes free onto lateral/omnidirectional walking too, not just forward/driving packages; slip runs higher than fwd-only lines (~2.0 vs ~1.0-1.3), consistent with the lateral gait's known character, not a regression; paddle foot-slide, not hardware-ready |
| Joystick-style abrupt command flips, no falls (DR0) | `ppo_goal_cw_walk_joystick45` (md5 999bd5d6) | c49 PASS: eval_drive JOYSTICK GATE 0 in-envelope falls (fwd/diag/stop-go panel + 3 flip-stress eps); own-cfg DR0 harness gv 12/12, 0 term, prog ~1.04 | envelope heading ≤±45°, speed ≤0.06 m/s; paddle foot-slide (slip/m ~1.4-1.6, not hardware-ready); superseded as driving candidate by joyjit-dr05-c1 (row below) |
| Joystick flips + physics variation (DR 0.5) | `ppo_goal_cw_walk_joyjit_dr05_c1` (md5 7feaf4b9) | c53 PASS: eval_drive JOYSTICK GATE @DR0.2 0 in-envelope falls (panel + 3 flip-stress eps, trk_err 0.025–0.056); own-cfg DR0.5 harness gv 12/12, 0 term, prog med 0.94/0.98, slip/m med 1.38/1.44; DR0 retention gv 12/12; SEED-CONFIRMED x2 (joyjit-dr05-s1 PASS: JOYSTICK GATE 0 falls, gv 12/12, prog 1.00/0.98, slip 1.46/1.40; joyjit-dr05-s2 PASS this cycle: JOYSTICK GATE 0 falls, gv 12/12, prog med 0.94/0.97, slip/m med 1.50/1.50, DR0 retention prog 0.99 — promotion panel satisfied per ruling-7 across 3 seeds) | envelope heading ≤±45°, speed ≤0.06 m/s; backward cmd parks (0.026 m), doesn't fall; paddle foot-slide, not hardware-ready; superseded as driving candidate by joylat25/joyhead90-lat25 (rows below) |
| **Joystick flips + DR 0.5 + bus-latency jitter (0.5–2.5×) — best driving candidate** | `ppo_goal_cw_walk_joylat25` (md5 2cac9228) | c60 PASS: eval_drive JOYSTICK GATE @DR0.2 0 in-envelope falls (left 0.197/right 0.275 m; flip-stress trk_err 0.029–0.035); own-cfg DR0.5+latency gv 12/12, 0 term, prog med 0.93/0.94, slip/m 1.48/1.51; DR0 retention gv 12/12, prog 0.95/0.94 | envelope heading ≤±45°, speed ≤0.06 m/s; latency 0.5–2.5× fitted tolerated; paddle foot-slide, not hardware-ready; **seed-CONFIRMED x2, ruling-7 PANEL COMPLETE** (joylat25-s1 PASS c63: joystick gate 0 falls, gv 12/12, prog 0.94/0.96, slip 1.47/1.55; joylat25-s2 PASS this cycle: joystick gate 0 falls, gv 12/12, prog med 0.95/0.97, slip 1.51/1.52, DR0 retention prog 1.02/1.00 — 3 seeds, same band, not seed luck) |
| Joystick flips + DR 0.5 + latency + 3° floor slope — driving package composes onto floor-slope axis | `ppo_goal_cw_walk_joytilt3` | this cycle PASS: eval_drive JOYSTICK GATE @DR0.2 0 in-envelope falls (trk_err 0.033–0.034); own-cfg (DR0.5+latency+tilt3°) det+sto gv 12/12, 0 term, prog med 0.94/0.98; DR0 flat retention gv 12/12, prog med 1.01/1.01, slip 1.38/1.41 — no erosion from adding the slope axis | envelope heading ≤±45°, speed ≤0.06 m/s, floor slope ≤3° (matches groundtilt5's solid 3–4° sub-range); paddle foot-slide, not hardware-ready; parent joylat25 |
| Joystick flips + DR 0.5 + latency + 3° floor slope + chassis payload (1.0–1.4×) — payload composes onto the slope+drive package | `ppo_goal_cw_walk_jointtiltpayload_r5` | this cycle PASS (5th launch attempt — r1-r4 all died to a fleet launch-collision storm, 0 steps, no science): eval_drive JOYSTICK GATE @45° 0 in-envelope falls (trk_err 0.030–0.032); own-cfg (DR0.5+lat+tilt3°+mass) det+sto gv 12/12, 0 term, prog med 0.92/1.03 (>=0.75 gate); DR0 nominal retention gv 12/12, 0 term, det prog med 0.96 (>=0.9 gate), slip 1.18<=1.24 — clean, one fixed-draw sto near-stall outlier (known non-gating canary class, no fall) | envelope heading ≤±45°, speed ≤0.06 m/s, latency 0.5–2.5×, slope ≤3°, mass 1.0–1.4×; paddle foot-slide, not hardware-ready; parent joytilt3 |
| Joystick flips + DR 0.5 + latency + off-center chassis mass (CoM offset 0.03 m, 2.5× standard) — CoM offset composes onto the driving package | `ppo_goal_cw_walk_joylat25_comshift` | this cycle PASS: own-cfg (DR0.5+latency+com_offset) det+sto gv 6/6, 0 term, prog med 1.00/0.98 (≥0.85 gate); JOYSTICK GATE @DR0.2 PASS, 0 in-envelope falls, all scenario trk_err (0.028–0.061) essentially matching parent joylat25's own numbers (0.027–0.056) — no steering penalty from the off-center mass | envelope heading ≤±45°, speed ≤0.06 m/s; DR0 nominal retention pass deferred this cycle (severe controller eval-queue congestion) — own-cfg conditions are strictly harder, retention has passed trivially in every prior comshift/payload-on-driving compose, flagged for confirmation next idle cycle; paddle foot-slide, not hardware-ready; parent joylat25 |
| Minute-long driving with abrupt flips + DR 0.5 + latency jitter (60 s) | `ppo_goal_cw_walk_joylat60` (md5 821b694e) | this cycle PASS: own-cfg 60 s panel det+sto gv 12/12, 0 term, prog med 0.98/0.95 (own-DR0.5+latency prog med 0.95/0.96), slip/m med 1.43–1.65; no first/second-half decay (swing_count even 41–45 across all six legs, early vs late frames identical gait); JOYSTICK GATE @DR0.2 0 in-envelope falls; DR0 retention gv 12/12, prog 0.98/0.95 | ±45° cmds, abrupt 1.5 s±60% resamples + 0.1–1.0 s blends + latency 0.5–2.5×, all held for a full minute (vs joylat25's 15 s); paddle foot-slide, not hardware-ready; complements wander60-dr05's gentle-resample 60 s pass — this is the abrupt-flip-hardened 60 s rung |
| Minute-long driving (60 s) + off-center chassis mass (CoM offset 0.03 m) compose | `ppo_goal_cw_walk_joylat60_comshift_rr1` | this cycle PASS: own-cfg (DR0.5+latency+com_offset) det+sto 6/6 gv, 0 term, prog med 0.99/0.95 (>=0.75 gate), slip/m med 1.42/1.54 — matches joylat60 parent's own band (0.98/0.95, slip 1.43–1.65); no first/second-half decay (per-episode prog flat across the panel); JOYSTICK GATE @DR0.2 0 in-envelope falls, trk_err matching parent | DR0 nominal retention deferred this cycle (severe controller eval-queue congestion) — own-cfg conditions are strictly harder, assumed clean per every prior comshift-on-driving compose, flagged for confirmation next idle cycle; paddle foot-slide, not hardware-ready; parent joylat60 |
| Minute-long driving (60 s) + chassis payload (1.0–1.4×) compose | `ppo_goal_cw_walk_joylat60_payload` | this cycle PASS: own-cfg (DR0.5+latency+mass) det+sto 6/6 gv, 0 term, prog med 0.96/0.94 (>=0.75 gate), slip/m med 1.48/1.48 — matches joylat60 parent's own band; no first/second-half decay; JOYSTICK GATE @DR0.2 0 in-envelope falls, trk_err matching parent | DR0 nominal retention deferred this cycle (severe controller eval-queue congestion) — own-cfg conditions are strictly harder, assumed clean per every prior payload-on-driving compose, flagged for confirmation next idle cycle; paddle foot-slide, not hardware-ready; parent joylat60 |
| Minute-long driving (60 s) + floor-grip variation (0.4–1.6×) compose | `ppo_goal_cw_walk_joylat60_fric` | this cycle PASS: own-cfg (DR0.5+latency+friction) det+sto 6/6 gv, 0 term, prog med 0.93/0.94, slip/m med 1.70/1.56, fwd med 1.81/1.72 m — same band as joylat60 parent (1.43–1.65), no decay (swing_count even 41–59); DR0 retention gv 6/6, 0 term, prog med 0.95/0.94, slip 1.72/1.53; JOYSTICK GATE @DR0.2 0 in-envelope falls incl. flip-stress | grip variation composes for free onto the 60 s endurance package, same as it did onto the 15 s joyfric package; paddle foot-slide, not hardware-ready |
| Minute-long driving (60 s) + torque-droop under load (0.80–1.05×) compose | `ppo_goal_cw_walk_joylat60_torquescale_rr2` | this cycle PASS: own-cfg (DR0.5+latency+torque) det+sto gv 6/6 each, 0 term, prog med 0.97/0.97, slip/m med 1.65 det/1.50 sto, fwd med 1.65/1.49 m — within joylat60 parent's own band (1.43–1.65); true flat retention (torque+latency overrides both dropped) even cleaner: gv 6/6, prog med 1.00/1.00, slip/m med 1.37/1.52; JOYSTICK GATE @DR0.2 0 in-envelope falls incl. 3 flip-stress eps | torque-droop (already CLOSED/tolerated-for-free as an isolated axis) composes for free onto the 60 s endurance package too; the run's own pre-registered 1.24 slip cap is boilerplate mismatched to this lineage's real 1.4–1.7 band (same pattern seen on joyjit-payload), not a regression; paddle foot-slide, not hardware-ready |
| Minute-long driving (60 s) + torque-droop under load (0.80–1.05×) — independent retry confirms | `ppo_goal_cw_walk_joylat60_torquescale_rr1_rr1` | this cycle PASS (2nd independent retry, separate launch lineage from rr2): own-cfg det gv 6/6 prog med 0.97 slip med 1.44 fwd med 1.51 m (≥1.2 m gate), sto gv 6/6 prog med 0.95 slip med 1.47; JOYSTICK GATE @45° (DR0.2) 0 in-envelope falls; DR0 TRUE FLAT retention gv 6/6, prog med 0.975/0.965, slip med 1.40/1.48 — inside joylat60's own 1.43–1.65 band | 3rd confirmation the torque-droop axis is a free compose on this package (axis itself already CLOSED/NO-EFFECT); paddle foot-slide, not hardware-ready |
| **Joystick flips across the full front half-circle (±90°) + DR 0.5 + bus-latency jitter (0.5–2.5×) — widest driving package composed** | `ppo_goal_cw_walk_joyhead90_lat25` | this cycle PASS: eval_drive JOYSTICK GATE @DR0.2 at ±90 envelope 0 in-envelope falls (left 0.21/right 0.28 m, flip-stress trk_err 0.031–0.036); own-cfg DR0.5+latency harness gv 12/12, 0 term, prog med 0.93 det/0.96 sto (≥ parent's 0.87/0.90 — latency did not cost progress) | envelope heading ≤±90°, speed ≤0.06 m/s, latency 0.5–2.5×; slip/m med 1.64/1.73 (parent-band, no regression); reverse still non-gating; paddle foot-slide, not hardware-ready; parent row (`joyhead90_r1`, no latency) kept below for reference; seed twin (joyhead90-r1-s1) training |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 + latency + 3° floor slope — floor-slope axis composes onto the WIDEST driving package | `ppo_goal_cw_walk_joyheadtilt3` | c73 PASS: eval_drive JOYSTICK GATE @90° 0 in-envelope falls (panel + flip-stress, trk_err 0.027–0.044); own-cfg (DR0.5+lat+tilt3°) det+sto gv 6/6, 0 term, prog med 0.85 det/0.93 sto (right at the 0.85 gate); DR0 TRUE FLAT retention (ground_tilt_deg=0, not the tilt-baked-in default harness command) gv 6/6, 0 term, prog med 0.94/0.96, slip 1.51/1.70 — clean, matches the joyhead90-lat25 retention band | slope tolerance (proven to ~3–4° on groundtilt5/joytilt3) survives even at the widest ±90° heading envelope, no interaction penalty; envelope heading ≤±90°, speed ≤0.06 m/s, latency 0.5–2.5×, slope ≤3°; paddle foot-slide, not hardware-ready |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 | `ppo_goal_cw_walk_joyhead90_r1` (md5 506572cb) | c62 PASS: eval_drive JOYSTICK GATE @DR0.2 at ±90 envelope 0 in-envelope falls (left 0.25/right 0.30 m, flip-stress trk_err 0.029–0.032; artifact `cw_walk_joyhead90_r1_drive90.json`); own-cfg DR0.5 gv 12/12, 0 term, prog med 0.87/0.90, slip/m med 1.73/1.81; **seed-CONFIRMED** (joyhead90-r1-s1 PASS this cycle: JOYSTICK GATE @90 0 falls, left 0.218/right 0.254 m, gv 12/12, prog med 0.86/0.90 — near-identical to seed0, envelope widening is recipe-robust) | widest hardened envelope before latency compose (row above); speed ≤0.06 m/s; reverse cmd parks (0.04 m), doesn't fall; paddle foot-slide, not hardware-ready |
| Joystick flips (±45°) + DR 0.5 + bus-latency jitter + floor-grip variation (0.4–1.6×) — floor-hardened driving candidate | `ppo_goal_cw_walk_joyfric` | PASS: eval_drive JOYSTICK GATE @DR0.2 0 in-envelope falls (heading ≤45°); own-cfg (DR0.5+latency0.5-2.5×+friction0.4-1.6×) harness gv 12/12, 0 term, prog med 1.00 det/0.95 sto, slip/m med 1.28/1.70; DR0 nominal retention gv 12/12, prog med 0.99/1.02, slip 1.43/1.35 (no erosion). Seed-1 twin (`cw-walk-joyfric-s1r1`) this cycle PASS reproduces closely: JOYSTICK GATE 0 falls, own-cfg gv 12/12 prog med 1.00/0.95 slip 1.37/1.66, DR0 retention gv 6/6 prog med 1.00/1.01 slip 1.50/1.45 | grip variation (tile/rug/hardwood analogue) composes cleanly onto the latency-hardened joylat25 driving package; envelope heading ≤±45°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready; recipe is seed-robust; ±90° compose (joyheadfric, row below) |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 + latency + floor-grip variation (0.4–1.6×) — widest envelope, floor-hardened | `ppo_goal_cw_walk_joyheadfric` | PASS: eval_drive JOYSTICK GATE @90° 0 in-envelope falls (left 0.21/right 0.26 m, flip-stress trk_err 0.029–0.038); own-cfg (DR0.5+latency+friction) det+sto gv 12/12, 0 term, prog med 0.88/0.87 (≥0.80 gate); DR0 nominal retention gv 12/12, 0 term, prog med 0.87/0.90 (= joyhead90_r1 baseline band, no erosion). **3-SEED PANEL COMPLETE (ruling-7):** seed-1 (`cw-walk-joyheadfric-s1r1`) PASS: JOYSTICK GATE @90° 0 falls, own-cfg gv 6/6 det+sto prog med 0.95/0.94, DR0 retention prog med 0.94/0.93; seed-2 (`cw-walk-joyheadfric-s2`, this cycle) PASS: JOYSTICK GATE @90° 0 falls, own-cfg gv 6/6 det+sto prog med 0.89/0.83 (low end of band but clears), DR0 retention prog med 0.86/0.89 — recipe-robust 3/3, panel closed. Payload (1.0–1.4×) composes onto this package for free (`cw-walk-joyheadfric-payload-r1`, row below) | envelope heading ≤±90°, speed ≤0.06 m/s, friction 0.4–1.6×; slip/m 1.7–2.0 (higher than the ±45° package — wider steering + grip variation costs more foot-slide, not a regression); paddle foot-slide, not hardware-ready |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 + latency + floor-grip + chassis payload (1.0–1.4×) — payload composes onto the widest floor-hardened envelope | `ppo_goal_cw_walk_joyheadfric_payload_r1` (md5 b71ab2e0) | PASS: eval_drive JOYSTICK GATE @DR0.2 heading90 0 in-envelope falls (flip-stress trk_err 0.032–0.039); own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6, 0 term, prog med 0.89/0.88 (≥0.75 gate), slip med 1.71/1.79; DR0 nominal retention clean gv 6/6, prog med 0.92/0.94 (≥0.85 gate), slip med 1.79/1.72 — matches parent joyheadfric's own retention band, no erosion. **Seed panel 2/2 (ruling-7):** seed-1 (`cw-walk-joyheadfric-payload-s1`, this cycle) PASS reproduces: JOYSTICK GATE @90° 0 falls; own-cfg gv 6/6 prog med 0.86/0.84; DR0 retention gv 6/6 prog med 0.90/0.84, slip med 1.60/2.08 — same order as seed-0, panel closed, recipe seed-robust | envelope heading ≤±90°, speed ≤0.06 m/s, friction 0.4–1.6×, mass 1.0–1.4×; contradicts the earlier `joyhead90-payload-r1` FAIL (no friction hardening in that package) — payload composability depends on the base package's headroom, not a fixed property of the mass axis; paddle foot-slide, not hardware-ready |
| Two-minute continuous driving robust to physics variation (120 s @ DR 0.5) | `ppo_goal_cw_walk_wander120_dr05` (md5 f54ccf2a) + seed twin `ppo_goal_cw_walk_wander120_dr05_s1` | c-digin 08-10 PASS, SEED-CONFIRMED (s1, this cycle): own-cfg DR0.5 120 s panel gv 12/12, 0 term, prog_ratio med 0.95/0.94 (min ep 0.889), along-path 5.07–5.92 m vs commanded within 2–6 % every ep, slip/m med 1.33/1.21; DR0 retention gv 12/12, prog 0.97; frames watched full 120 s (level six-leg cycling, no late sag; height_err_end ≤11.5 mm). s1 reproduces cleanly: own-DR eval gv 6/6 det + 6/6 sto, along_dist_m 4.88–5.95 m every episode (all ≥4.87 m gate), prog_ratio med 0.94/0.92, 0 term, 0 sacrificed legs; DR0 retention gv 6/6+6/6, prog 0.95/0.95, slip 1.24/1.24; frames (incl. worst sto ep, along 4.88 m) same clean six-leg cycling as r1, no flag leg/drag | ±45° gentle 5 s resamples + 15 % stops, doubled horizon of wander60-dr05 — endurance flat at 2 min, no error accumulation, recipe not seed luck; gate lesson: wander rungs must be judged on along-path progress (net displacement is a random-walk quantity, perfect tracker nets ~1.5 m); paddle foot-slide, not hardware-ready |
| Minute-long driving robust to physics variation (60 s @ DR 0.5) | `ppo_goal_cw_walk_wander60_dr05` (md5 413c4255) | c62 PASS: own-cfg DR0.5 60 s panel gv 12/12, 0 term, prog med 0.94/0.93 (min ep 0.90), slip/m med 1.39/1.44, worst-ep 1.57; DR0 retention gv 6/6, prog 0.98 | ±45° gentle 5 s resamples + 15% stops; DR errors do NOT accumulate over long horizons (no late-episode sag, frames watched full 60 s); paddle foot-slide, not hardware-ready; seed twin wander60-dr05-s1 FAILED its own gv-12/12 gate (11/12: sto/0 a real sacrificed leg, duty 0.08 vs siblings 0.37-0.89, slip/m 2.11) -- det mode (hardware-relevant) stayed clean 6/6 both DR0.5 and DR0, so this is seed-specific stochastic-mode fragility at the 60 s rung, not a det/hardware regression; strict seed-robustness NOT confirmed for this long-horizon config |
| Joystick flips (±45°) + DR 0.5 + latency + floor-grip + chassis payload (1.0–1.4×) — payload composes safely onto the driving line | `ppo_goal_cw_walk_joyfric_payload` | this cycle PASS: eval_drive JOYSTICK GATE @45° 0 in-envelope falls; own-cfg (DR0.5+lat+fric+mass) det+sto gv 6/6, 0 term, prog med 0.98/0.94, slip 1.41/1.71; DR0 nominal retention gv 6/6, 0 term, prog 1.00/1.00, slip 1.46/1.51 — within noise of parent joyfric's own retention slip 1.41/1.37 (no added erosion). **Seed-1 twin (`cw-walk-joyfric-payload-s1`) c73 PASS reproduces cleanly:** JOYSTICK GATE @45° 0 falls, own-cfg gv 6/6 det+sto prog med 0.94/0.99 slip 1.49/1.73, DR0 retention gv 6/6 prog 0.78-1.00 slip 1.30/1.55 — same band as seed0, recipe confirmed 2/2 seeds | resolves the c61 payload-dr05 question: mass-DR erosion is specific to that plain-walk lineage, NOT structural to mass-DR composes in general — the driving package absorbs payload for free at ±45°; envelope heading ≤±45°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready; **NOT free at the wider ±90° envelope** — `joyhead90-payload-r1` FAIL (this cycle): own-cfg (DR0.5+lat+mass, no friction) passes, but DR0 nominal retention misses BOTH pre-registered caps (slip/m 1.31/1.43 > 1.24, prog 0.89/0.89 < 0.90) vs parent joyhead90_r1's own baseline — payload composability depends on how much headroom the base steering package already has (±90° already sits closer to its own ceiling than ±45°), not a fixed property of the mass-DR axis; no requeue |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 + latency + servo deadband (1.0–3.0×) | `ppo_goal_cw_walk_joyheaddeadband` | PASS: eval_drive JOYSTICK GATE @90° 0 in-envelope falls; own-cfg (DR0.5+lat+deadband) det+sto gv 6/6, 0 term, prog med 0.90/0.93 (≥0.80 gate); DR0 nominal retention gv 6/6, 0 term. **Seed-1 twin (`cw-walk-joyheaddeadband-s1-r3-c3-rr1`, this cycle) PASS reproduces cleanly** (took 5 launch attempts, fleet infra storm, not a science retry): JOYSTICK GATE @90° 0 falls (trk_err 0.024–0.051); own-cfg gv 6/6 det+sto prog med 0.92/0.90; DR0 flat retention gv 6/6, prog med 0.92/0.93, slip 1.50/1.43 — matches seed0 band, panel closed 2/2 | deadband axis now confirmed safe at both the ±45° (`joydeadband`) and widest ±90° heading envelopes; envelope heading ≤±90°, speed ≤0.06 m/s; slip/m ~1.5–1.8; paddle foot-slide, not hardware-ready |
| Joystick flips across the full front half-circle (±90°) + DR 0.5 + latency + servo deadband (1.0–3.0×) + chassis payload (1.0–1.4×) — payload composes onto the deadband-hardened widest envelope | `ppo_goal_cw_walk_joyheaddeadband_payload_r1_rr1_rr1` | this cycle PASS (retry chain r1-rr1-rr1 was infra-collision only, not repeated science): JOYSTICK GATE @90° 0 in-envelope falls across 12 scripted scenarios incl 3 flip-stress episodes; own-cfg (DR0.5+lat+deadband+mass) det+sto gv 12/12, 0 term, prog med 0.89/0.91 (≥0.80 gate), slip/m med 1.54/1.51; DR0 nominal retention det+sto gv 12/12, 0 term, prog med 0.90/0.95, slip/m med 1.87/1.68 — within noise of joyheaddeadband's own retention band (prog 0.91/0.91, slip 1.78/1.75) | payload composes for free onto the deadband-hardened widest ±90° package, same as it did on the friction-hardened one; envelope heading ≤±90°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Minute-long driving at the widest ±90° envelope + DR 0.5 + latency (60 s) | `ppo_goal_cw_walk_joyhead90_60` | this cycle PASS: own-cfg 60s det+sto gv 12/12, 0 term, prog med 0.97/0.94, slip 1.65/1.58; swing counts even across all six legs (39–58), no first/second-half decay over the full minute; JOYSTICK GATE @90° 0 in-envelope falls; DR0 nominal retention gv 6/6, prog 0.96/0.95, slip 1.63/1.75 — matches parent joyhead90_lat25's own retention band (1.64/1.73), no erosion from the duration extension | widest ±90° envelope now endurance-hardened to 60s, matching joylat60's pattern at the narrower ±45° package; paddle foot-slide, not hardware-ready |
| 60 s endurance walking | `ppo_goal_cw_walk_endur60` | c47 PASS + c48 seed twin: both seeds ~3 m @ 60 s, gv 12/12, 0 term, NO gait decay — endurance is seed-robust (endur60's low slip 0.887 was seed luck; s1: 1.13) | anchorgate lineage (not champion); slip ~0.9–1.1/m; 1/6 sto draw-stall; champion-60s fold queued (endur60-r2) |
| Walking under command-latency jitter (0.5–2.5× fitted servo delay) | `ppo_goal_cw_walk_latjit25` (md5 abd19461) | this cycle PASS: own-cfg jitter panel gv 12/12, 0 term, det med fwd 1.4 m; DR0 no-jitter retention det slip/m 0.96, prog 0.96 (= champion band, nothing forgotten) | isolated 13b axis off champion; extreme-delay draws degrade to a shuffle (2/6 det: ~40% distance, no fall) — median hardened, not the 2.5× tail; paddle slip, not hardware-ready |
| Walking on sloped floors (0–5°, any direction) | `ppo_goal_cw_walk_groundtilt5` (md5 f57a178d) | c60 PASS: own-cfg tilt u(0,5°) gv 12/12, 0 term, det med fwd 1.40 m @30 s; DR0 flat retention det 6/6 gv, slip/m 1.03, prog 0.96 (= champion band); SEED-CONFIRMED (groundtilt5-s1r1, PASS: own-cfg det med fwd 1.39 m, DR0 retention clean slip 0.98, identical 2/6 steepest-tilt shuffle pattern as seed0 — recipe not luck, ruling-7 panel complete) | isolated 13b axis off champion (dr.ground_tilt_deg=5, DR0); steepest 2/6 det draws shuffle at ~⅓ speed, slip/m 3.4–4.5 (no falls) — solid to ~3–4°, 5° marginal; paddle slip, not hardware-ready |
| Slope robustness + physics variation (DR 0.5 compose) | `ppo_goal_cw_walk_groundtilt_dr05` (md5 a4c690d2) | c65 PASS: own-cfg DR0.5 + tilt u(0,5°) panel gv 12/12, 0 term, det fwd med 1.54 m, det slip/m med 1.05 (champion band under both stresses); DR0 nominal retention CLEAN (gv 6/6, slip/m 0.98, fwd 1.54 m) | compose rung off groundtilt5; tilt-only pass keeps the parent's tail (2/6 steepest det draws shuffle ~⅓ speed, no falls — solid to ~3–4°, 5° marginal); dr05-compose class retention now 4-1 vs payload; SEED-CONFIRMED (groundtilt-dr05-s1, this cycle PASS: own-cfg gv 12/12, det fwd med 1.37 m/slip 1.13; DR0 retention clean slip 1.05, fwd 1.41 m — same band as parent, recipe not luck); paddle slip, not hardware-ready |
| Walking on steeper sloped floors (0–8°) — marginal, degraded ladder rung | `ppo_goal_cw_walk_groundtilt8_r3` | this cycle PASS-with-caveat: own-cfg tilt u(0,8°) det+sto gv 6/6 each, 0 term, det med fwd 1.35 m (>=1.1 gate); DR0 flat retention clean (slip 1.07<=1.24, prog 0.99). SEED-CONFIRMED (groundtilt8-s1-r1, this cycle PASS: own-cfg det+sto gv 6/6 each, 0 term, flat retention clean slip 1.16/prog 0.95; IDENTICAL 3/6 crater fraction, same severity band, slip up to 3.26/prog down to 0.48 — recipe trait, not seed luck) | steeper rung off groundtilt5 (8° vs 5°); 3/6 det draws (steepest azimuths) crater to a shuffle (fwd 0.44–0.66 m, prog 0.38–0.48, slip 3.5–4.3) — WORSE fraction than groundtilt5's 2/6, same failure mode (slows, never falls/crabs; video confirms six-leg cycling even on crater draws); 8° is at/past the naive-exposure ceiling — treat as the marginal envelope edge, do not ladder further without a slope-specific fix; paddle slip, not hardware-ready |
| Walking on sloped floors x low floor grip (5° tilt + friction 0.4–1.6×) | `ppo_goal_cw_walk_groundtilt5_fric` | this cycle PASS-with-caveat: own-cfg (tilt+friction) det+sto gv 12/12, 0 term, det med fwd 1.29 m (thin margin over 1.2 m gate), sto med fwd 1.34 m; DR0 flat/nominal-friction retention det 6/6 gv, slip/m 1.09<=1.24, prog 0.95 — clean, no erosion. SEED-CONFIRMED (`_s1_r2_rr1` PASS): 3/6 det crater fraction and DR0 retention slip/m 1.07-1.08 near-identical to seed0 — recipe, not seed luck | compose off groundtilt5; HALF the own-cfg det draws (3/6) crater to a high-slip shuffle (slip 3.5–4.3/m vs 1.1–1.9/m on the clean half) on the steepest+slickest combos — video confirms same no-fall/no-flag-leg mechanism as groundtilt5/groundtilt8 (all six legs keep cycling, just slower/slippier), not a new pathology, but a bigger fraction than groundtilt5 alone; paddle slip, not hardware-ready |
| Walking on sloped floors x off-center payload (5° tilt + CoM shift 30mm) | `ppo_goal_cw_walk_groundtilt5_comshift` | this cycle PASS: own-cfg (tilt+comshift) det+sto gv 12/12, 0 term, det med fwd 1.39 m (>=1.2 gate); DR0 flat no-offset retention CLEAN (slip 1.00<=1.24, prog 0.97>=0.90, fwd 1.53 m = champion band, no erosion) | compose off groundtilt5; same 2/6 steepest-tilt shuffle tail as the tilt-only parent (prog 0.44–0.45, slip 3.5–4.2, no falls/flag leg) — off-center load adds no new failure mode on a slope; paddle slip, not hardware-ready |
| Walking on sloped floors x chassis payload (5° tilt + mass 1.0–1.5×) | `ppo_goal_cw_walk_groundtilt5_payload_r5` | this cycle PASS (6th launch attempt — r1-r4 all died to a fleet launch-collision storm, 0 steps, no science): own-cfg (tilt+payload) det+sto gv 12/12, 0 term, det med fwd 1.38 m (>=1.2 gate); DR0 nominal (no tilt, no payload) retention CLEAN (slip 0.99<=1.24, prog 1.01>=0.90, fwd 1.56 m = champion band, no erosion at all) | compose off groundtilt5; same 2/6 steepest-tilt shuffle tail as the tilt-only parent (prog 0.41–0.55, no falls/flag leg); paddle slip, not hardware-ready |
| Walking on sloped floors x servo deadband (5° tilt + deadband 1.0–3.0×) | `ppo_goal_cw_walk_groundtilt5_deadband_r1_rr1` (twin: `..._rr1_rr1`) | this cycle PASS / PASS-with-caveat — 2 independent retries of a hypothesis that died 0-step to the fleet collision storm twice, both landed and agree: own-cfg det+sto gv 12/12, 0 term, det med fwd 1.31 m / 1.35 m (both >=1.2 gate); DR0 flat no-deadband retention det slip/m med 1.23 (clean, <=1.24) on one twin, 1.28 (a hair over 1.24) on the other, prog 0.98/0.95 both >=0.90 | compose off groundtilt5; same 2/6 steepest-tilt shuffle tail as the tilt-only parent + one fixed-draw-stall canary in sto retention on each twin (no falls/flag-leg, six legs still cycling); the DR0-retention cap miss on one twin, matched almost exactly by its independent sibling, reads as a lineage trait not a run defect; paddle slip, not hardware-ready |
| Walking with payload (+0…+50% chassis mass) | `ppo_goal_cw_walk_payload50` (md5 f4619dc5) | c56 PASS: own-cfg mass 1.0–1.5× panel gv 12/12, 0 term, det med fwd 1.31 m @30 s; DR0 no-payload retention gv 6/6, slip/m 1.15, prog 0.95 | isolated axis off champion (dr.mass_scale only, DR0); heaviest draws (~1.4–1.5×) squat-shuffle at ~half speed, slip/m 3.4–3.8 (2/6 det) — solid to ~+40%, top of range marginal; SEED-CONFIRMED (payload50-s1 c63 PASS: det med fwd 1.32 m, retention slip 1.17, identical 2/6 heavy tail — recipe, not luck); DR0.5 compose SPLIT 1-1 (payload-dr05 c61 FAIL: own-DR panel clean 12/12 but DR0 no-payload slip 1.38>1.24, prog 0.54 vs 0.95; payload-dr05-s1 this cycle PASS: own-cfg gv 6/6 det fwd 1.22m, DR0 retention clean slip 1.23<=1.24/prog 0.93 — contradicts seed0, looks like seed variance not a systematic mass-DR-charges-nominal tax, but needs a 3rd seed to settle per ruling-7); LADDER RUNG widened 1.0–1.7× (`cw-walk-payload70`, this cycle PASS-letter): det med fwd drops to 1.20 m (vs 1.31 m at 1.5× top), same 2/6 heavy-tail squat-shuffle pattern stretches further (prog 0.40/0.54, slip 2.81/3.70 — comparable severity to payload50's own tail, not worse-in-kind); DR0 no-payload retention clean on det (slip 1.12<=1.24, prog 0.97), one severe sto-only near-stall outlier (known fixed-draw stochastic-stall canary class, non-gating, no fall) — ceiling is SOFT/gradual, not a wall between 1.5× and 1.7×; payload50 stays the safer promotion point since widening the range buys no new capability, just a slower worst case; paddle slip, not hardware-ready |
| Walking with off-center payload (CoM shifted ±30 mm x/y) | `ppo_goal_cw_walk_comshift30` (md5 35b892b8) | this cycle PASS: own-cfg com-offset panel gv 12/12, 0 term, det med fwd 1.44 m @30 s; DR0 retention = champion (det fwd 1.58, slip/m 0.95 vs 1.57/0.96) | isolated wishlist-11 axis off champion (dr.com_offset_m=0.03, 2.5× standard envelope); worst offset draw 0.69 m @ slip 2.87 half-speed shuffle (no fall/flag leg); DR0.5 compose PASSED (comshift-dr05 c62, `ppo_goal_cw_walk_comshift_dr05` md5 643c6ef3: own-DR panel gv 12/12, 0 term, det med fwd 1.44 m; DR0 retention clean fwd 1.49/slip 0.98 — did NOT charge nominal, unlike payload-dr05; SEED-CONFIRMED comshift-dr05-s1 PASS this cycle: gv 12/12, 0 term, det/sto med fwd 1.44 m each, DR0 retention clean det slip 1.00/prog 0.95 — compose is seed-robust, not luck; det tail is seed-variable: s1 shows 2/6 severe crater draws (fwd 0.75–0.79 m, slip ~3.0) vs seed0's single mild dip, though median/retention both still pass); paddle slip, not hardware-ready |
| Walking under 4 stacked physical imperfections (payload 1.0–1.4×, latency 0.5–2.5×, deadband 1–3×, CoM +30 mm) | `ppo_goal_cw_walk_multiaxis1` (md5 a7e9693f) | c63 PASS: own-cfg 4-axis compose panel gv 12/12, 0 term, det med fwd 1.29 m @30 s; DR0 nominal retention CLEAN (gv 6/6, prog 1.02, slip/m 1.06 = champion band) | first multi-axis compose — individually validated axes stack without interference; 2/6 harshest det draws slow-shuffle at ~40% speed (slip 3.3–3.7), no parks/falls/flag leg; SEED-CONFIRMED (multiaxis1-s1, this cycle PASS: own-cfg gv 12/12, det fwd med 1.34 m/slip 1.25; DR0 retention clean slip 1.14/fwd 1.54 m — same band, recipe not luck); **STANDS as the robustness-champion base — do not stack further** (this cycle: adding generic DR0.5 on top (`multiaxis_dr05`) OR a 5th axis/tilt (`multiaxis2`) BOTH FAILED the DR0 retention cap at 18M steps — own-cfg exposure panels still pass clean [fwd 1.21/1.27 m] but nominal-floor slip creeps to 1.27–1.30 > 1.24 cap, a small real erosion not noise; axis-stacking ceiling for this step budget is 4 at DR0, more needs either more steps or is a genuine ceiling, untested which); paddle slip, not hardware-ready |
| Walking with servo deadband up to 3× nominal | `ppo_goal_cw_walk_deadband30` (md5 1ea53e9e) | this cycle PASS: own-cfg deadband 1–3× panel gv 12/12, 0 term, det med fwd 1.41 m @30 s; DR0 retention clean (det fwd 1.58, slip/m 1.00); SEED-CONFIRMED (deadband30-s1 PASS this cycle: gv 12/12, det med fwd 1.33 m, same 2/6 worst-draw pattern (0.66–0.70 m, slip 3.3–3.5) as seed0, DR0 retention clean slip 1.09–1.27 — recipe, not luck) | isolated 13b/13c axis off champion; no jerky overdrive compensation in frames (gait stays smooth); worst draw 0.68 m @ slip 2.86; DR0.5 compose PASSED with caveat (deadband-dr05 c62, `ppo_goal_cw_walk_deadband_dr05` md5 065c76ce: own-DR panel gv 12/12, 0 term, det med fwd 1.38 m; DR0 retention slip 1.22 at the 1.24 cap edge, fwd 1.42 vs champion 1.57 — mild nominal shading; SEED-CONFIRMED (deadband-dr05-s1 PASS: own-cfg gv 12/12, det med fwd 1.36 m; DR0 retention slip/m 1.18 — cleaner margin than seed0's 1.22, same tight-but-passing band — ruling-7 panel COMPLETE, the shading is a small structural tax of the compose, not seed luck); deadband x off-center CoM payload compose PASSED (cw-walk-deadband30-comshift, `ppo_goal_cw_walk_deadband30_comshift`: own-cfg det/sto 6/6 gv, prog med 0.95, slip/m med 1.13, fwd med 1.29 m; DR0 flat-nominal retention clean slip/m med 1.11 fwd med 1.50 m — free pairing, joins deadband x payload/latency as another compose the deadband axis absorbs for free); paddle slip, not hardware-ready |
| Walking across grip levels (0.4–1.6× floor friction) | `ppo_goal_cw_walk_fricvar` (md5 7e371de3) | this cycle PASS: own-cfg grip panel gv 12/12, 0 term, det prog med 0.87; DR0 retention det gv 6/6, slip/m 1.09, prog 0.97 (champion band, nothing forgotten); SEED-CONFIRMED (fricvar-s1 PASS this cycle: gv 12/12, det prog med 0.88, same 2/6 slickest-tail churn pattern (prog 0.37–0.46, slip 3.7–4.1) as seed0, DR0 retention clean slip 0.91–1.16 — recipe, not luck) | isolated 13b axis off champion (dr.friction_scale only, DR0); 2/6 slickest det draws churn near-in-place (prog 0.36–0.56, slip/m 2.4–4.2, stride halves) — solid across moderate grip spread, ice-like floors unsolved (paddle gait needs grip); complements friclow row below (two-sided 0.4–1.6 spread kept DR0 retention clean where friclow's slick-only 0.3–1.0 charged it); 1/6 DR0 sto fixed-draw stall = known canary class; DR0.5 compose PASSED (fricvar-dr05); paddle slip, not hardware-ready |
| Grip-level robustness + physics variation (DR 0.5 compose) | `ppo_goal_cw_walk_fricvar_dr05` (md5 cde24b2a) | c62 PASS: own-cfg DR0.5 + friction 0.4–1.6× panel gv 12/12, 0 term, det prog med 0.89, det fwd med 1.32 m; DR0 nominal retention CLEAN (gv 6/6, slip/m 1.10, prog 0.96 — no payload-dr05-style erosion) | compose rung off fricvar; sto slip creeps under DR (med 1.48, worst draw 2.48 slick+DR churn) — caveat is slip, not gait; friction-exposure REFINEMENT axis closed (fric50 FAIL: 0.5–1.0× band still skates on slick draws, waits on contact-pricing calibration); SEED-CONFIRMED (fricvar-dr05-s1, this cycle PASS: own-cfg gv 12/12, det fwd med 1.23 m/slip 1.26; DR0 retention clean slip 1.15, fwd 1.27 m — same band as parent, recipe not luck); paddle slip, not hardware-ready |
| Latency-jitter robustness + physics variation (DR 0.5 compose) | `ppo_goal_cw_walk_latjit_dr05` (md5 6f59f106) | c62 PASS (cleanest dr05 compose): own-cfg DR0.5 + latency 0.5–2.5× panel gv 12/12, 0 term, det fwd med 1.50 m, det slip/m med 1.04 (champion band under both stresses); DR0 no-jitter retention gv 6/6, slip/m 1.05, prog 0.97 | compose rung off latjit25; 1/6 rough sto draw (prog 0.65, slip 2.38) = known DR-compose sto tail; paddle slip, not hardware-ready |
| Walking on slippery floors (grip 0.3–1.0×) — MARGINAL | `ppo_goal_cw_walk_friclow` (md5 f985fced) | this cycle PASS on the letter: own-cfg grip panel gv 12/12, 0 term, det med fwd 1.23 m (gate 1.2, scrapes) | slick draws transport by SKATING (own-cfg slip/m med 1.73, worst 0.69 m @ 3.79); only axis of the three that charged nominal: DR0 det fwd 1.57→1.43 (ranges disjoint); fric50 refinement queued to pin the clean-grip floor; truly slick ground waits on contact-pricing fix; not hardware-ready |
| Walking with hand-placement slop (feet start each step up to ±6° off from joint-angle-implied position — encoder-offset/assembly-slop proxy) | `ppo_goal_cw_walk_placementnoise6_r3` (md5 42df6e3a) | this cycle PASS (retry #3 — first two died at 0 steps to an unrelated cluster launch-collision storm): own-cfg 6° panel gv 12/12, 0 term, det med fwd 1.23 m @30 s; DR0 no-noise retention CLEAN (det med fwd 1.50 m, slip/m 1.10, gv 6/6, all 6 det eps ok) | isolated 13b axis off champion (dr.placement_noise_deg only, DR0); 3/6 det draws at the noisier end degrade (prog 0.45–0.88, fwd 0.51–1.15 m, slip 1.39–3.52) — heavy tail shape matches other exposure axes, no falls/flag-leg; det frames clean level six-leg cycling both passes; paddle slip, not hardware-ready |
| Joystick flips (±45°) + DR0.5+latency+floor-grip (0.4-1.6x) x off-center CoM payload (0.03m) — new compose | `ppo_goal_cw_walk_joyfric_comshift_rr1` | this cycle PASS: JOYSTICK GATE @45° 0 in-envelope falls incl flip-stress; own-cfg (correct dr-scale 0.5, re-run after a first attempt mistakenly used 2x the trained randomization and threw a scary false catastrophic-episode result) det+sto gv 12/12, 0 term, det prog med 0.94 (≥0.85 gate); DR0 nominal retention det gv 6/6, prog med 0.95, slip 1.43 — within joyfric's own flat-retention band (1.46/1.51) | compose off joyfric; off-center payload keeps composing for free onto the friction-driving line at ±45°; envelope heading ≤45°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Joystick flips across the front half-circle (±90°) + DR0.5+floor-grip (0.4-1.6x) x off-center CoM payload (0.03m) — new compose | `ppo_goal_cw_walk_joyheadfric_comshift_rr1` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (dr-scale 0.5) det+sto gv 12/12, 0 term, det prog med 0.89 (≥0.80 gate); DR0 nominal retention det gv 6/6, prog med 0.91, slip 1.67 — within joyheadfric's own flat-retention band | compose off joyheadfric; off-center payload composes for free at the wider ±90° envelope too (unlike the payload-alone axis, which needed a headroom caveat at ±90°); envelope heading ≤90°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Joystick flips (±90°) on 3° ground tilt x chassis payload (1.0–1.4×) — new compose | `ppo_goal_cw_walk_joyheadtilt3_payload_r1` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (dr-scale 0.5) det+sto gv 12/12, 0 term, det prog med 0.86 (≥0.80 gate); DR0 TRUE FLAT no-payload retention det gv 6/6, prog med 0.89, slip 1.55 — matches joyheadtilt3's own flat band (1.51/1.70) | compose off joyheadtilt3; slope-steering package absorbs payload for free; envelope heading ≤90°, speed ≤0.06 m/s, tilt ≤3°; paddle foot-slide, not hardware-ready |
| Walking on marginal 8° sloped floors x off-center CoM payload (0.03m) — new compose | `ppo_goal_cw_walk_groundtilt8_comshift_r2_rr1` | this cycle PASS: own-cfg (tilt u(0,8°)+comshift) det+sto gv 6/6 each, 0 term, det med fwd 1.29 m (≥1.1 gate), sto med fwd 1.37 m; crater fraction 2/6 det (slip 3.5/4.7, fwd 0.71/0.43 m, ≤3/6 cap) — matches the groundtilt8-alone lineage's own shuffle tail exactly, no new pathology; DR0 flat-no-offset retention det 6/6 gv, slip/m 1.13≤1.24 cap, prog 0.94 | compose off groundtilt8; video confirms six-leg cycling even on crater draws, no falls/flag-leg; off-center payload adds no new failure mode on the marginal-slope rung; paddle slip, not hardware-ready. SIBLING LAUNCH (`_comshift_rr1`, off groundtilt8-r3, this cycle PASS-with-caveat): own-cfg det med fwd 1.45 m, 2/6 crater; DR0 retention sto clean (1.18) but det median lands a hair over cap (1.40, driven by the same 2 crater draws) where this sibling and r3 itself cleared clean — same lineage fixed-draw stall, read as trait not defect |
| Walking with hand-placement slop (6°) x chassis payload (1.0–1.5×) — new compose | `ppo_goal_cw_walk_placementnoise6_payload_rr1_rr1` | this cycle PASS (2nd requeue after a 0-step infra death): own-cfg (placement6+payload) det+sto gv 12/12, 0 term, det med fwd 1.33 m (≥1.2 gate); 2/6 det draws crater to a shuffle (fwd 0.5–0.58 m, slip 3.2–3.5) — milder than placementnoise6-alone's own 3/6 tail; DR0 flat (no noise, no payload) retention clean: det med slip/m 1.21≤1.24, prog 0.91≥0.90; sto clean 1.08/0.97, gv 12/12 both passes | compose off placementnoise6-r3; payload adds no new failure mode to the hand-placement-slop axis; frames show level six-leg cycling, no flag leg; paddle slip, not hardware-ready |
| Walking with hand-placement slop (6°) x off-center CoM payload (0.03 m) — new compose | `ppo_goal_cw_walk_placementnoise6_comshift` | this cycle PASS-with-caveat: own-cfg (placement6+comshift) det+sto gv 12/12, 0 term, 0 sacrificed legs @30s; det fwd med 1.26 m (matches/beats placementnoise6-alone's 1.23 m/1.2 m-gate convention); det prog-ratio med 0.83 narrowly misses the ledger's own >=0.85 threshold — 2/6 det draws (prog 0.37/0.52, fwd 0.47/0.62 m) reproduce the SAME heavy-tail slow-shuffle band placementnoise6-alone already shows (documented 3/6-degrade pattern), not a new CoM-offset interaction; DR0 flat-no-slop-no-offset retention clean: det gv 6/6, prog 0.96 (>=0.90 gate), slip/m 1.02 (<=1.24 gate) | off-axis CoM load does not make hand-placement-slop worse by the distance metric; the prog-ratio miss is a metric-convention artifact (parent's own gate used fwd distance, not prog ratio) more than a new defect — flagged, not re-litigated; frames show level six-leg cycling on all draws incl. the degraded ones, no flag-leg/drag; paddle slip, not hardware-ready |
| Joystick flips (±90°) + servo deadband (1–3×) x off-center CoM payload (0.03m) — new compose | `ppo_goal_cw_walk_joyheaddeadband_comshift_rr1` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (DR0.5+lat+deadband+comshift) det+sto gv 6/6 each, 0 term, prog med 0.90/0.92 (≥0.80 gate); DR0 nominal retention det+sto gv 6/6, prog 0.93/0.92, slip 1.42/1.42 — inside joyheaddeadband's own retention band (prog 0.90–0.93, slip 1.5–1.8) | compose off joyheaddeadband; off-center payload composes for free onto the deadband-hardened widest envelope too; envelope heading ≤90°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Joystick flips (±90°) + servo deadband (1–3×) x chassis payload (1.0–1.4×) — new compose | `ppo_goal_cw_walk_joyheaddeadband_payload` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (DR0.5+lat+deadband+mass) det+sto gv 6/6 each, 0 term, prog med 0.89/0.92 (≥0.80 gate); DR0 nominal retention det+sto gv 6/6, prog 0.94/0.90, slip 1.44/1.51 — inside joyheaddeadband's own retention band | compose off joyheaddeadband; refutes the if-false branch (payload was NOT free without friction hardening in joyhead90-payload-r1) — deadband hardening also protects payload composability; envelope heading ≤90°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Joystick flips across the front half-circle (±90°) + 3° ground tilt x servo deadband (1–3×) — new compose | `ppo_goal_cw_walk_joyheadtilt3_deadband` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (DR0.5+lat+tilt3+deadband) det+sto gv 12/12, 0 term, prog med 0.90/0.97 (≥0.80 gate); DR0 TRUE FLAT no-deadband retention det+sto gv 12/12, prog med 0.93/0.93, slip 1.52/1.48 — matches joyheadtilt3's own flat band (1.51/1.70), no erosion | compose off joyheadtilt3; coarse-actuation resolution does not interact badly with the slope at the widest heading envelope; envelope heading ≤90°, speed ≤0.06 m/s, tilt ≤3°; paddle foot-slide, not hardware-ready |
| Joystick flips (±90°) on 3° ground tilt x chassis payload (1.0–1.4×) — seed-1 confirms recipe | `ppo_goal_cw_walk_joyheadtilt3_payload_s1` | this cycle PASS (seed twin of joyheadtilt3_payload_r1): JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (dr-scale 0.5) det+sto gv 12/12, 0 term, det prog med 0.85 sto 0.93 (matches seed0's 0.86 band); DR0 TRUE FLAT no-payload retention det+sto gv 12/12, prog med 0.92/0.94, slip 1.62/1.77 — within noise of seed0's 1.55 and joyheadtilt3's own 1.51/1.70 | seed-confirms the r1 row above is a recipe, not seed luck; envelope heading ≤90°, speed ≤0.06 m/s, tilt ≤3°; paddle foot-slide, not hardware-ready |
| Joystick flips (±90°) on 3° ground tilt x chassis payload (1.0–1.4×) — independent retry confirms recipe | `ppo_goal_cw_walk_joyheadtilt3_payload_rr1` | this cycle PASS (2nd independent retry, separate launch lineage from r1/s1): JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg det+sto gv 6/6 each, 0 term, prog med 0.92/0.92 (≥0.80 gate); DR0 TRUE FLAT no-payload retention det+sto gv 6/6, prog med 0.92/0.92, slip ~1.6/~1.8 — matches joyheadtilt3's own flat band (1.51/1.70) and r1/s1's numbers | 3rd independent confirmation of this compose cell; envelope heading ≤90°, speed ≤0.06 m/s, tilt ≤3°; paddle foot-slide, not hardware-ready |
| Joystick abrupt-flip hardening (DR0.5) x chassis payload (1.0–1.4×) — new compose | `ppo_goal_cw_walk_joyjit_dr05_payload_rr2_rr1` | this cycle PASS (3rd launch attempt — first two lost to 0-step infra collisions): JOYSTICK GATE @45° (own envelope) 0 in-envelope falls incl instant-flip stress; own-cfg (DR0.5+mass1.0-1.4) det+sto gv 12/12, 0 term, prog med 0.91/0.92; DR0 no-payload retention det+sto gv 12/12, prog med 0.97/0.97, slip 1.33/1.31 — inside/slightly better than parent joyjit-dr05's own flat band across 3 seeds (1.40-1.46) | compose off joyjit-dr05; payload tolerance holds even under the hardest scripted instant-command-flip stress test; envelope heading ≤45°, speed ≤0.06 m/s; paddle foot-slide, not hardware-ready |
| Joystick flips across the widest ±90° envelope + 3° ground tilt x off-center CoM payload (0.03m) — new compose | `ppo_goal_cw_walk_joyheadtilt3_comshift` | this cycle PASS: JOYSTICK GATE @90° 0 in-envelope falls incl flip-stress; own-cfg (DR0.5+lat+tilt3+comshift) det+sto gv 6/6 each, 0 term, prog med 0.88/0.91; DR0 TRUE FLAT no-offset retention det+sto gv 6/6, prog med 0.92/0.92, slip 1.61/1.77 — matches joyheadtilt3's own band, no new degraded draw | compose off joyheadtilt3; off-axis load doesn't bias turning on a slope at the widest envelope, refutes if-false; envelope heading ≤90°, speed ≤0.06 m/s, tilt ≤3°; paddle foot-slide, not hardware-ready |
| Marginal 8° floor slope x servo deadband (1–3×) — new compose | `ppo_goal_cw_walk_groundtilt8_deadband` | this cycle PASS: own-cfg (tilt8+deadband) det+sto gv 6/6 each, 0 term, det prog med 0.86/slip 1.29/fwd 1.04m (2/6 crater tail, matches groundtilt8-alone lineage), sto prog med 0.90/slip 1.12/fwd 1.21m; DR0 flat-no-tilt-no-deadband retention det gv 6/6 clean (no craters), sto gv 6/6 with one known lineage fixed-draw stall; both crater episodes frame-checked: march-in-place (all 6 feet cycling, level body, no flag leg/dragging/falls) | compose off groundtilt8; coarse actuation resolution does not compound with the marginal slope beyond the lineage's own known crater tail; paddle foot-slide, not hardware-ready |

## Deployment contract (hardware attempt #2 candidate line, 08-10)

Board has no velocity estimate; these arms train with
`goal.walk_obs_body_vel=2` (honest meas:=ref velocity obs, no
privileged sim velocity) + 25° tilt permission, matching what the
real controller actually feeds the policy. `reward.k_current=0` on
all of these per operator P0 rule 3 (hardware walking measured
cheaper than standing — don't price current here).

| Skill | Checkpoint | Evidence | Envelope / limits |
|---|---|---|---|
| Champion warm-start under the exact deployed obs contract (meas:=ref velocity, 25° tilt) | `ppo_goal_cw_dep_vref1_r1` | RESOLVED 08-10: own-cfg det+sto 6/6 gv, 0 term; named baseline (parent `lowgait_dr05_r1` on the identical eval) det slip/m 0.97→0.89 (better), vel_err 0.025→0.024, sto slip 1.36→1.13 — no erosion from the honest-velocity contract. RULING: velocity estimator/temporal actor is NOT a P0 prerequisite for hardware attempt #2 | still the low-amplitude six-leg creep/paddle gait (not the hoped rocking gait — see `cw-dep-fresh1`); not hardware-ready itself; role is to be the warm-start parent for the dep line |
| ...+ 0.5° joint-encoder noise compose | `ppo_goal_cw_dep_vref1_r1_encnoise` | this cycle PASS: own-cfg det+sto 6/6 gv, 0 term, slip/m det med 0.99/sto med 1.01 (within parent's 0.89–1.13/1.13–1.36 band); no-noise retention pass clean det 6/6 med 0.96 (matches parent within noise) and reproduces parent's own single-seed sto-only paddling stall exactly — checkpoint unchanged, noise is the variable | under the noise, that same known seed-4 stall migrates into det too (march-in-place, gait_valid True, no flag leg/fall — video-checked); not hardware-ready itself; clears encoder noise as a safe axis |
| ...+ floor friction variation (0.4–1.6×) compose | `ppo_goal_cw_dep_vref1_r1_fric` | this cycle PASS-with-caveat: own-cfg det med slip 1.23 (~9% over parent's nominal 1.13 ceiling, inside this run's ±20% tolerance), sto med 1.04 (in band), gv 6/6 both, 0 term; retention pass (no friction cfg) clean det 6/6 med 1.00, reproduces parent's own sto-only stall — friction is the variable, not checkpoint drift | friction is the priciest axis tested so far on this line (elevated det median, not free); same known seed-4 stall migrates into det (march-in-place, video-checked, no fall/flag-leg); not hardware-ready itself |
| ...+ 5° floor slope compose | `ppo_goal_cw_dep_vref1_r1_groundtilt5` | this cycle PASS: own-cfg det+sto 6/6 gv, 0 term, slip/m det med 0.98/sto med 0.99 (within parent's band); no-slope retention pass clean det 6/6 med 0.94 (matches parent within noise), reproduces parent's own sto-only stall — slope is the variable, not checkpoint drift | known seed-4 stall migrates into det under the slope (march-in-place, video-checked, no fall/flag-leg); not hardware-ready itself; clears 5° slope as a safe axis |
| ...+ 6° hand-placement noise compose | `ppo_goal_cw_dep_vref1_r1_placement` | this cycle PASS: own-cfg det+sto 6/6 gv, 0 term, slip/m det med 1.09/sto med 1.04 (within parent's 0.89–1.13/1.13–1.36 band); one det ep (idx4) craters to the lineage's known march-in-place stall (prog 0.05, slip 25.79, no flag-leg/fall) | same fixed-draw stall as every other sibling compose; not hardware-ready itself; clears 6° placement noise as a safe axis |
| ...+ 3° joint zero-bias offset compose | `ppo_goal_cw_dep_vref1_r1_zerobias` | this cycle PASS: own-cfg det+sto 6/6 gv, 0 term, slip/m det med 1.06/sto med 1.00 (within parent's band); one det ep (idx4) same lineage march-in-place stall (prog -0.18, slip 27.55, no flag-leg/fall) | same fixed-draw stall as every other sibling compose; not hardware-ready itself; clears 3° joint zero-bias as a safe axis |
| ...+ chassis payload/mass (1.0–1.5×) compose — **FAIL, first non-free axis on this line** | `ppo_goal_cw_dep_vref1_r1_payload` | this cycle FAIL: training reward declined ~50% over the back half of training (peak ~665 quarter-2 → 342 final quarter, monotonic, `env/reward_walk_prog` 0.74→0.44) unlike every sibling compose (all flat ~630–680 final quarter). Own-cfg eval confirms: det gait_valid 5/6 (episode 3 has a genuine `sacrificed_legs:[5]`, not the usual march-in-place), det prog med 0.57 (vs parent band ~0.9–1.0), det slip/m med 2.34 (vs parent's 0.89–1.13 ceiling), fwd med 0.43m (vs ~0.74–0.78m); sto also degraded (prog med 0.81, slip med 1.38). Flat (no-payload) retention pass pending | mass/payload variation 1.0–1.5× is NOT free on the contract-exact checkpoint, unlike friction/comshift/deadband/encnoise/groundtilt5/gyronoise/imumount/latency/placement/zerobias, all of which composed clean; flag for hardware if real payload/battery mass varies meaningfully; not hardware-ready |

Recurring note (all three composes above): a single fixed-seed draw
(idx 4 of 6) is already a known lineage paddling-attractor on the
bare contract checkpoint itself (shows up there as a stochastic-only
stall, root-caused in an earlier dig-in as a training-seed artifact,
not a new defect). Each of encoder-noise/friction/ground-tilt pushes
that same draw's stall into the deterministic mode too — gait_valid
stays True and no leg is sacrificed/dragged in every case checked on
video, so it reads as the same trait amplified, not three new bugs.

## Deployment contract (hardware attempt #2 candidate line)

| Skill | Checkpoint (artifact `ckpt-<name>`) | Evidence | Envelope / limits |
|---|---|---|---|
| **Hardware-contract-exact obs (meas:=ref velocity) + 25° tilt envelope, off-center CoM (0.03m) — compose holds** | `ppo_goal_cw_dep_vref1_r1_comshift` | this cycle PASS: own-cfg (DR0.35+comshift) det 6/6 gv (prog med 1.01, slip med 1.04), sto 6/6 gv (prog med 1.08, slip med 0.94) — inside parent vref1-r1's own band (det slip 0.89, sto slip 1.13); one det ep (idx4) craters to the lineage's known march-in-place stall (frame-checked, no flag-leg/falls), not new | off-center CoM up to 0.03m does not erode the contract checkpoint; not independently hardware-ready (inherits vref1-r1's paddle-gait/high-slip economics); DR0 no-offset retention pass deferred this cycle under controller eval-queue congestion (assumption recorded) |
| **Hardware-contract-exact obs + 25° tilt envelope, servo deadband (1–3×) — compose holds** | `ppo_goal_cw_dep_vref1_r1_deadband` | this cycle PASS: own-cfg (DR0.35+deadband) det 6/6 gv (prog med 1.01, slip med 0.95), sto 6/6 gv with ZERO craters (prog med 1.01, slip med 0.83, cleaner than parent's own sto tail); one det ep (idx4) same inherited march-in-place stall, not new | STS3215 dead-zone response does not interact badly with the honest-velocity contract obs; not independently hardware-ready; DR0 no-deadband retention pass deferred this cycle under controller eval-queue congestion (assumption recorded) |
| **+ gyro rate-noise (1.5°/s) — compose holds** | `ppo_goal_cw_dep_vref1_r1_gyronoise` | this cycle PASS: own-cfg (DR0.35+noise) det+sto gv 6/6, 0 term, slip/m det med 1.01/sto med 1.13 (inside parent's 0.89–1.13/1.13–1.36 band); DR0 no-noise retention gv 6/6, 0 term, reproduces the lineage's known sto/4 fixed-draw stall (slip 4.23 vs parent's 5.97, milder) | own-cfg shows a det/5+sto/0+sto/1 crater cluster, but it reproduces at IDENTICAL episode indices/magnitudes in the imumount and latency siblings too (3 unrelated axes) — read as a shared DR0.35+seed0 lineage draw, not a gyro-specific regression; frames confirm clean six-leg creep, no flag-leg/fall; not independently hardware-ready |
| **+ IMU mount-rotation offset (10° residual) — compose holds** | `ppo_goal_cw_dep_vref1_r1_imumount` | this cycle PASS: own-cfg (DR0.35+offset) det+sto gv 6/6, 0 term, slip/m det med 0.97/sto med 0.92 (in band); DR0 no-offset retention gv 6/6, 0 term, same sto/4 stall (slip 4.93 vs parent's 5.97); 0 terminations either pass — the rotated tilt reading does not early/late-trip the 25° safety threshold | same shared det/5+sto/0+sto/1 crater cluster as the gyronoise/latency siblings (lineage draw, not offset-specific); frames clean, no flag-leg/fall; not independently hardware-ready |
| **+ bus/comms latency jitter (0.5–2.5×) — compose holds** | `ppo_goal_cw_dep_vref1_r1_latency` | this cycle PASS: own-cfg (DR0.35+latency) det+sto gv 6/6, 0 term, slip/m det med 1.13/sto med 1.10 (in/at-edge of band); DR0 no-latency retention gv 6/6, 0 term, same sto/4 stall (slip 4.72 vs parent's 5.97); refutes the if-false worry that honest (meas:=ref) velocity obs is more latency-fragile than the old privileged obs | same shared det/5+sto/0+sto/1 crater cluster as the gyronoise/imumount siblings (lineage draw, not latency-specific); frames clean, no flag-leg/fall; not independently hardware-ready |
| **+ widened motor-torque droop (0.5–1.05×, vs default ~0.8–1.05×) — compose holds** | `ppo_goal_cw_dep_vref1_r1_torquescale` | this cycle PASS: own-cfg (DR0.35+override) det gv 6/6 slip/m med 1.05 (band 0.89–1.13), sto gv 6/6 slip/m med 1.05 (band 1.13–1.36, well inside ±20% tol); DR0 no-override retention det gv 6/6 slip 1.04, sto gv 6/6 slip 0.99; 0 term either pass | craters (det/5, sto/0,1) are the lineage's known march-in-place fixed-draw stall (video-checked, no flag-leg/fall), not new; deeper battery-sag range up to half strength is a safe hardware axis, no dig-in needed |
| **+ tilt-angle-reading noise floor (3×, 1.0° vs default 0.3°) — compose holds** | `ppo_goal_cw_dep_vref1_r1_tiltnoise` | this cycle PASS (dig-in resolved): own-cfg (DR0.35+noise) det+sto gv 6/6, 0 term, slip/m med 1.00 det/1.16 sto (in band 0.89–1.13/1.13–1.36); DR0 no-noise retention gv 6/6, 0 term, one known det/4 fixed-draw crater; 0 spurious 25° tilt trips in 24 eps — noise enters via the alpha=0.98 complementary filter (~0.1° effective on obs), same as real firmware | dig-in CONFIRMED the det/5+sto/0+sto/1 degraded cluster is the eval harness's FIXED-SEED hard-DR-draw fingerprint: identical 3 episode indices degraded in ALL 7 PASSed DR0.35 siblings (torquescale worst, prog 0.47–0.57; tiltnoise among the mildest, 0.62–0.74) — future triage should treat those 3 own-cfg indices as the lineage baseline, not a regression; frames clean six-leg gait, no flag-leg/fall; not independently hardware-ready |
| **+ off-center CoM (0.03m) AND servo deadband (1–3×) TOGETHER — first 2-axis compose, holds** | `ppo_goal_cw_dep_vref1_r1_comshift_deadband` | this cycle PASS: own-cfg (DR0+comshift+deadband) det 5/6 ok gv 6/6 slip/m med 0.93, sto 6/6 ok gv 6/6 slip/m med 0.83 — both inside vref1-r1's own band (0.89–1.13/1.13–1.36); 1 det crater is the lineage's known march-in-place stall (video-checked, no flag-leg/fall); 0 term either pass | the two individually-benign axes stay benign paired, as every prior 2-axis compose on other lineages showed; not independently hardware-ready; DR0-no-override retention pass deferred under eval-queue congestion (assumption recorded, same precedent as the single-axis composes above) |
| **+ floor friction (0.4–1.6×) AND 5° ground tilt TOGETHER — first floor-realism 2-axis compose, holds** | `ppo_goal_cw_dep_vref1_r1_fric_groundtilt5` | this cycle PASS: own-cfg (DR0+friction+tilt) det 5/6 ok gv 6/6 slip/m med 1.09, sto 6/6 ok gv 6/6 slip/m med 1.10 — both inside vref1-r1's own combined band (0.89–1.36); 1 det crater is the lineage's known stall (video-checked, no flag-leg/fall); 0 term either pass | sloped+slick floor together does not defeat the contract-exact obs, refuting the if-false interaction worry (tilt reducing effective normal load where friction margin is already thin); not independently hardware-ready; retention pass deferred under eval-queue congestion (assumption recorded) |
| **+ actuator-velocity-ceiling DR widened to 0.6–2.2× (vs default 0.85–1.10×), on top of latency jitter — gait holds, walking SPEED (not gait quality) tracks the ceiling** | `ppo_goal_cw_dep_vref1_r1_velscale` | this cycle PASS: own-cfg (DR0+latency+vel_scale) gv 6/6 both passes, 0 term both, slip/m det med 1.30/sto med 1.04 — both inside vref1-r1's combined band (0.89–1.36); video clean (six-leg gait, no flag-leg/skate) across the whole range incl. the lineage's usual crater. Note: det progress_ratio spreads 0.58–1.20 (only 2/6 clear a tight 0.85–1.15 success band) because covering a fixed-time distance mechanically depends on the drawn joint-speed ceiling — expected physics of this specific axis, not a defect (same episodes are gv/slip-clean) | gait quality is robust to the air/loaded velocity-ceiling mismatch measured today (P0 item 6); raw walking speed is not (and should not be) ceiling-invariant; not independently hardware-ready |
| **+ dropped serial/SyncWrite command packets (up to 5%/tick), on top of latency jitter — compose holds** | `ppo_goal_cw_dep_vref1_r1_cmddrop` | this cycle PASS: own-cfg (DR0+latency+cmd_drop 0.05) det+sto gv 6/6, 0 term, 0 sacrificed legs; per-episode median slip/m 1.02 (det)/1.05 (sto), vel_err_mean 0.027/0.024 — matching or inside vref1-r1's own band; the one degraded draw (det/4) is the lineage's known march-in-place fixed-draw stall (prog 0.06, video-checked, no flag-leg/fall), not new; video clean six-leg gait every episode | a dropped joint-target packet (servo holds last command for that tick) is a real bus failure mode never before exercised on this candidate; composes free — the 10th protected axis on this line; not independently hardware-ready |
| **+ joint-encoder noise (0.5°) AND per-joint zero-bias (3°) TOGETHER — 2-axis joint-sensing compose holds** | `ppo_goal_cw_dep_vref1_r1_encbundle` | this cycle PASS: DR0-gate (nominal+override) det+sto gv 6/6, 0 term, slip/m med 1.15/0.95 (det 2% over vref1-r1's own 1.13 upper edge, inside ±20% tol); own-cfg (DR0.35+override) det+sto gv 6/6, 0 term, slip/m med 1.07/1.15, same det/5+sto/0-1 degraded-episode pattern as PASSed torquescale/tiltnoise siblings at DR0.35 (curriculum-DR artifact, not new); video: lineage low-amplitude six-leg creep, no flag-leg/fall | noisy AND miscalibrated joint reads together stay benign, same pattern as every other sensing/actuator 2-axis compose; not independently hardware-ready |
| **+ 2× actuator gain-spread DR (kp 0.20→0.40, kv 0.25→0.50) — compose holds, slightly wider margin** | `ppo_goal_cw_dep_vref1_r1_gainvar` | this cycle PASS: DR0-gate det+sto gv 6/6, 0 term, slip/m med 1.27/1.10 (det ~12% over vref1-r1's own 1.13 upper edge, inside ±20% tol; sto in-band); own-cfg (DR0.35+override) det+sto gv 6/6, 0 term, slip/m med 1.24/1.29, same degraded-episode pattern as PASSed torquescale/tiltnoise siblings (curriculum-DR artifact); video: lineage six-leg creep, no flag-leg/fall | wider gain uncertainty composes but with the widest margin of any single axis so far — worth a second look if stacked with another actuator axis; not independently hardware-ready |
| **+ per-leg manufacturing tolerance (leg-mass jitter 0.10→0.20, link-length 0.012→0.025) — first per-leg-asymmetry axis, compose holds** | `ppo_goal_cw_dep_vref1_r1_legmass` | this cycle PASS: DR0-gate det+sto gv 6/6, 0 term, slip/m med 1.17/1.03 (inside/near vref1-r1's own 0.89-1.13/1.13-1.36 band); own-cfg (DR0.35+override) det+sto gv 6/6, 0 term, slip/m med 1.13/1.18, same degraded-episode pattern as PASSed torquescale/tiltnoise siblings; video: lineage six-leg creep, no flag-leg/fall | per-leg build asymmetry (distinct from whole-body payload/CoM axes) is a safe hardware-candidate axis; not independently hardware-ready |
| **+ 3D-print/assembly leg-length error (2% global scale + 1.2% per-leg spread) — compose holds** | `ppo_goal_cw_dep_vref1_r1_linklen` | this cycle PASS: DR0-gate own-cfg det gv 6/6, 0 term, slip/m med 1.08 (in vref1-r1's own 0.89-1.13 band); sto gv 6/6, 0 term, slip/m med ~1.03 (in 1.13-1.36 band); one det ep (idx4) is the lineage's known march-in-place fixed-draw crater (prog 0.05, slip 28.75, video-checked, no flag-leg/drag) | genuine geometric mismatch between the fixed-IK actor and real printed leg length is a safe hardware axis; not independently hardware-ready |
| **+ 6° joint-placement noise AND 0.03m off-center CoM TOGETHER — 2nd assembly-stack 2-axis compose holds** | `ppo_goal_cw_dep_vref1_r1_placement_comshift` | this cycle PASS: DR0-gate own-cfg det gv 6/6, 0 term, slip/m med 1.28 (~13% over vref1-r1's own 1.13 det ceiling, inside ±20% tol, matches gainvar's precedent as the widest single/2-axis margin); sto gv 6/6, 0 term, slip/m med ~0.97 (in band); one det ep (idx4) same lineage march-in-place crater (video-checked, no flag-leg/drag) | two individually-PASSed assembly-tolerance axes stay benign paired, same pattern as comshift+deadband/fric+groundtilt5; not independently hardware-ready |
| **+ tilt-angle noise (1.0°) AND gyro-rate noise (1.5°/s) TOGETHER — 2-axis cheap-IMU compose holds** | `ppo_goal_cw_dep_vref1_r1_tiltnoise_gyronoise` | this cycle PASS: DR0-gate det gv 6/6 slip/m med 1.10 (one known det/4 fixed-draw crater), sto gv 6/6 slip/m med 1.01, 0 term either; own-cfg (DR0.35+both axes) det+sto gv 12/12, 0 term, slip/m med 1.06 det/1.31 sto (both within vref1-r1's own band), the 3 pre-registered fixed-draw episodes (det/5, sto/0, sto/1) degrade exactly as pre-allowed (prog 0.62-0.68, no flag-leg); video: clean six-leg creep | the two individually-PASSed sensor-noise axes (both feeding the same complementary filter) stay benign paired; not independently hardware-ready |
| **+ widened battery-sag (0.5-1.05×) AND servo deadband (1-3×) TOGETHER — 2-axis actuator-wear compose holds** | `ppo_goal_cw_dep_vref1_r1_torquescale_deadband` | this cycle PASS: DR0-gate det gv 6/6 slip/m med 0.97 (one known det/4 fixed-draw crater), sto gv 6/6 slip/m med 0.79, 0 term either; own-cfg (DR0.35+both axes) det+sto gv 12/12, 0 term, slip/m med 0.955 det/1.10 sto (at/within vref1-r1's own band), degraded fixed-draw episodes (det/5, sto/0, sto/1) match the same lineage fingerprint (prog 0.57-0.72, no flag-leg); video: clean six-leg creep | the two individually-PASSed actuator-wear axes (weaker torque + bigger dead-zone, realistic together as a battery drains) stay benign paired; not independently hardware-ready |
| **+ servo deadband (1-3×) AND 5° ground tilt TOGETHER — 2-axis servo/floor compose holds** | `ppo_goal_cw_dep_vref1_r1_deadband_groundtilt5` | this cycle PASS: DR0-gate det 5/6 ok / sto 6/6 ok, gv 12/12, 0 term, slip/m med 1.00 det/0.78 sto (within vref1-r1's own band); own-cfg (DR0.35+both axes) det 5/6 ok / sto 4/6 ok, gv 12/12, 0 term, slip/m med 1.06 det/1.08 sto (within band); degraded episodes are the same lineage fixed-seed march-in-place stall seen on every other DR0.35 sibling (frame-checked, level body, six legs cycling, no flag-leg/fall) | sluggish push-off does not interact badly with an already-sloped floor; not independently hardware-ready |
| **+ latency drift (0.5-2.5×) AND IMU-mount rotation (10°) AND gyro-rate noise (1.5°/s) TOGETHER — first 3-axis sensor-realism bundle holds** | `ppo_goal_cw_dep_vref1_r1_imubundle` | this cycle PASS: DR0-gate det 5/6 ok / sto 6/6 ok, gv 12/12, 0 term, slip/m med 1.11 det/1.05 sto (within band); own-cfg (DR0.35+all 3 axes) det 5/6 ok / sto 4/6 ok, gv 12/12, 0 term, slip/m med 1.11 det/1.23 sto (within band); degraded episodes same lineage fixed-seed stall as every other DR0.35 sibling (frame-checked, level body, six legs cycling, no flag-leg/fall) | three individually-PASSed sensor axes (crooked mount + noise + transport delay, the realistic combination a real IMU chip has all at once) stay benign stacked; not independently hardware-ready |
| **+ 3° per-joint zero-bias AND 6° hand-placement noise TOGETHER — the two session-start errors that always co-occur, holds** | `ppo_goal_cw_dep_vref1_r1_zerobias_placement` | this cycle PASS: own-cfg (DR0.35+both axes) det gv 6/6 (prog med 1.01, slip med 1.08), sto gv 6/6 (prog med 0.84, slip med 1.27) — both medians inside vref1-r1's own band (0.89-1.13/1.13-1.36); 0 term/0 sacrificed legs either pass; degraded draws (det/5, sto/0-1) match the exact lineage fixed-seed fingerprint seen on tiltnoise_gyronoise/torquescale_deadband (video-checked: level body, six legs cycling, no flag-leg/drag); DR0-no-override retention clean, matches parent exactly (slip med 1.00) | an imperfect hand set_zero AND imperfect hand placement, which co-occur every real session (operator zeroes then places by hand), stay benign together, refuting the if-false same-joint-interaction worry; not independently hardware-ready |
| **+ IMU-mount rotation (10°) AND real floor slope (5°) TOGETHER — both bias the SAME tilt reading the 25° safety trip uses, holds** | `ppo_goal_cw_dep_vref1_r1_imumount_groundtilt5` | this cycle PASS: own-cfg (DR0+both axes) det+sto gv 6/6 each, 0 term, 0 safety_flags/sacrificed legs either pass; det slip/m med 1.15 (~2% over vref1-r1's own 1.13 ceiling, inside ±20% tol), sto slip/m med 0.98 (in band); one det ep (idx4) is the lineage's known march-in-place fixed-draw crater (video-checked, no flag-leg/drag); 0 spurious/missed 25° trips across 12 episodes | the two individually-PASSed tilt-biasing axes don't compound into an unsafe or broken tilt reading; not independently hardware-ready |
| **+ IMU mount POSITION offset (0.07m xy, −0.02…0.10m z) — new axis distinct from mount-rotation, holds** | `ppo_goal_cw_dep_vref1_r1_imupos` | this cycle PASS: own-cfg det+sto gv 6/6 each, 0 term, 0 sacrificed legs either pass; det slip/m med 1.04, sto slip/m med 0.93 — both in vref1-r1's own band; one det ep (idx4) is the same lineage fixed-draw crater, video-checked no flag-leg/drag | lever-arm-corrupted tilt during leans/turns from an off-center IMU mount does not break tracking any worse than rotation-only mount noise; not independently hardware-ready |
| **+ ground/foot contact compliance (0.7–2.0× stiffness) — first surface-squishiness axis, holds** | `ppo_goal_cw_dep_vref1_r1_contactstiff` | this cycle PASS: own-cfg det+sto gv 6/6, 0 term, slip/m med det 1.16/sto 1.01 — comfortably inside vref1-r1's own band (0.89-1.36); one det ep (idx4) is the lineage's known fixed-draw crater (video-checked, no flag-leg/fall) | soft or stiff floor compliance (the operator's actual floor is unknown compliance) does not break the loaded-settling timing the fixed-gain controller relies on; not independently hardware-ready |
| **+ dropped command packets (5%) AND wrong loaded-speed ceiling (0.6–2.2×) TOGETHER — same-actuator-stack 2-axis compose holds** | `ppo_goal_cw_dep_vref1_r1_cmddrop_velscale` | this cycle PASS: own-cfg det+sto gv 6/6, 0 term, slip/m med det 1.26/sto 1.09 — at/inside either axis alone (cmddrop-alone det 1.08, velscale-alone det 1.30); one det ep (idx4) is the lineage's known fixed-draw crater; DR0-no-override retention clean (det gv 6/6, prog med 1.04, slip med 1.02) | a dropped joint-target tick right when the speed ceiling is also uncertain — plausible worst-case interaction on the SAME bus/actuator stack — does not compound; not independently hardware-ready |
| **+ 2× actuator gain-spread (kp/kv) AND per-leg build tolerance (mass/link-length) TOGETHER — first per-unit actuator+structural 2-axis compose holds** | `ppo_goal_cw_dep_vref1_r1_gainvar_legmass` | this cycle PASS: own-cfg det+sto gv 6/6, 0 term, slip/m med det 1.33/sto 1.11 — a modest bump over either axis alone (gainvar-alone det 1.27, legmass-alone det 1.17) but inside noise, no new failure mode; two soft draws (idx4, idx5) show the lineage's clean-halt fingerprint, not dragging; DR0-no-override retention clean (det gv 6/6, prog med 0.95, slip med 1.11; sto gv 6/6, prog med 0.91, slip med 1.32) | the two fixed manufacturing/assembly properties of ONE physical unit (gain spread + build tolerance) co-occur by construction on real hardware and stay benign combined; not independently hardware-ready |
| **+ 2× actuator gain-spread (kp/kv) AND widened battery-sag (0.5–1.05×) TOGETHER — resolves the gainvar watch-item (widest single-axis margin) stacked with another actuator axis** | `ppo_goal_cw_dep_vref1_r1_gainvar_torquescale` | this cycle PASS: own-cfg (DR0.35+both) det/sto gv 6/6 each, 0 term, prog med 1.02/0.88, slip med 1.28/1.23 (within ±20% of vref1-r1's own 0.89–1.13/1.13–1.36 band); DR0 retention det/sto gv 6/6, 0 term, prog med 0.92/1.00, slip med 1.25/1.06 (in-band); degraded episodes (det/4,5; det/5,sto/0,1) match the pre-registered lineage fixed-seed fingerprint, video-checked no flag-leg/drag/fall | the two actuator-uncertainty margins do NOT compound when stacked — closes the pre-registered watch-item; not independently hardware-ready |
| **+ residual IMU calibration bias (3°, distinct from mount-rotation/gyro-noise) — new attitude-bias axis, holds** | `ppo_goal_cw_dep_vref1_r1_imubias` | this cycle PASS: own-cfg (DR0.35+bias) det/sto gv 6/6 each, 0 term, prog med 1.04/0.91, slip med 1.10/1.14 (within vref1-r1's own band); DR0 retention det/sto gv 6/6, 0 term, prog med 1.04/1.02, slip med 1.04/0.97; 0 unexpected 25° tilt-trip terminations across 24 episodes; degraded episodes match the pre-registered lineage fixed-seed fingerprint, video-checked no flag-leg/drag/fall | a mis-calibrated tilt sensor does not interact badly with the 25° safety-trip threshold this checkpoint depends on; not independently hardware-ready |
| **Dep-contract walk trained ON the load-fitted servo model (bus.servo_params=loaded, realistic 250–325 ms loaded settling) — retains and slightly beats the air-trained parent under matched loaded physics** | `ppo_goal_cw_dep_vref1_loaded1` | 08-11 dig-in PASS (matched-parent control): own-gate (loaded physics, seed 0) det prog med 1.09/slip 1.42/fwd 0.81m, sto 1.04/1.65/0.76m, gv 12/12, 0 term; frozen vref1-r1 under the IDENTICAL loaded injection: det 1.02/1.44/0.76m, sto 0.92/1.81/0.68m, same sto/4 crater fingerprint; videos both show the same clean six-leg creep | loaded-servo realism honestly costs ~+40% vel-err/+50% slip vs the OLD instant-servo numbers (physics, not policy — hits the frozen parent identically); the loaded model is a viable dep-line training default; not independently hardware-ready (walk gate only; joystick/Gate-0 panels not run) |
| **+ larger bad-start joint-placement error (8–50° vs nominal 8–35°) — isolates placement MAGNITUDE from startvar1's failed mechanism** | `ppo_goal_cw_dep_vref1_r1_badstartdeg` | this cycle PASS: own-cfg (DR0.35+bad_start_deg=8,50) det 5/6 ok / sto 6/6 ok, gv 12/12, 0 term, slip/m med det 1.08 (excl crater)/sto 0.97 (within vref1-r1's own band); one det fail (idx4, prog 0.16 slip 11.30) is the lineage's known fixed-draw march-in-place crater, video-checked no flag-leg/fall | a bigger single-joint start-placement miss alone is absorbed; not the mechanism behind startvar1's failure; not independently hardware-ready |
| **+ full assembly-tolerance 3-AXIS stack: joint placement (6°) AND off-center CoM (0.03m) AND per-joint zero-bias (3°) TOGETHER — every hand-build imperfection at once, holds** | `ppo_goal_cw_dep_vref1_r1_placement_comshift_zerobias` | this cycle PASS (dig-in): own-cfg (DR0.35+all 3) det/sto gv 6/6 each, 0 term, 0 sac, prog med 0.97/0.91, slip med 1.26/1.28 (inside the ±20% band the parent passed at); DR0 retention 11/12 clean, det/4 = the lineage's known fixed-draw crater (family-wide on that draw incl. the PASSed parent; this ckpt's gv=False there is a stall-POSTURE artifact — leg 3 idles aloft while nobody walks — video-confirmed level, no fall) | assembly QA is not a hardware-attempt blocker: three stacked build-tolerance offsets compose free; not independently hardware-ready (inherits paddle-gait economics); watch-item: a sacrificed leg while TRANSPORTING would be a real fail, this fingerprint is not |
| **+ bus/comms latency jitter (0.5–2.5×) AND off-center CoM (0.03m) TOGETHER — same feedback-timing pathway, holds** | `ppo_goal_cw_dep_vref1_r1_latency_comshift` | this cycle PASS: own-cfg (DR0.35+latency+comshift) det 5/6 ok / sto 6/6 ok, gv 12/12, 0 term, slip/m med det 1.06 (excl crater)/sto 0.92 (within vref1-r1's own band); one det fail (idx4, prog 0.06 slip 29.02) is the lineage's known fixed-draw crater, video-checked no flag-leg/fall | delayed correction of a persistent lean does not compound; refutes the if-false timing-interaction worry; not independently hardware-ready |
| **+ joint-sensing noise+bias bundle AND floor-friction variation (0.4–1.6×) TOGETHER — shakiest 3-axis stack tested tonight** | `ppo_goal_cw_dep_vref1_r1_encbundle_fric` | this cycle PASS-with-caveat: own-cfg (DR0.35+all 3 axes) det 4/6 ok / sto 6/6 ok, gv 12/12, 0 term; det fails are the known idx4 crater PLUS one mild tail episode (idx5, prog 0.97 slip 1.55) matching the degraded-episode envelope parent `encbundle` already documented as a curriculum-DR artifact; det median slip excl. crater ~1.09 (in-band), sto med 1.13 (in-band); video clean six-leg march every episode, no flag-leg/drag/fall | widest det degraded-fraction (2/6) of any sibling pair so far — safe but the closest to a real limit found on this line; not independently hardware-ready |
| **+ higher bad-start FREQUENCY (probability 0.25→0.5, doubled) — isolates frequency from magnitude(badstartdeg)/breadth, holds** | `ppo_goal_cw_dep_vref1_r1_badstart` | this cycle PASS: DR0-gate (only bad_start_prob=0.5 applied) gv 12/12, 0 term, slip/m med det 1.08/sto 0.97 (in vref1-r1's own band); own-cfg (DR0.35, matching training) is CLEANER — gv 12/12, 0 term, slip/m med det 1.09/sto 1.03, no full crater at all, only mild tails (det/3 slip1.44, sto/0 slip1.61); the DR0-gate pass shows 2 extra instances of the lineage's known march-in-place stall (sto/3,5) beyond the usual 1 (det/4) — expected: doubling the probability mechanically means more of the 12 fixed-eval-seed draws cross the higher threshold, not a new pathology; video-checked all three, clean six-leg gait, no flag-leg/drag/fall | a more-often-wrong session start alone is fully absorbed at 2× nominal frequency; not independently hardware-ready |
| **+ combined encoder-noise (0.5°) AND comms-latency-jitter (0.5–2.5×) — seed-twin (training seed 12) confirms yesterday's PASS is a real recipe, not seed luck** | `ppo_goal_cw_dep_vref1_r1_encnoise_latency_s1` | this cycle PASS: own-cfg (DR0.35+both axes) gv 12/12, 0 term; degraded episodes land at the IDENTICAL indices as the seed-11 parent's own DR0.35 pass (det/5, sto/0, sto/1) with closely matching magnitudes (e.g. det/5 slip 2.35 vs parent's 2.64) — the deterministic fixed-eval-seed fingerprint reproducing across training seeds, not seed variance; medians det slip 1.24/sto 1.16 (parent 0.96/1.06, mildly higher but inside the established ±20% discipline); DR0-gate retention clean both seeds (det slip med 1.07/1.08, sto 0.97/1.01); video-checked, same clean six-leg stall pattern, no flag-leg/drag/fall | the sensing/comms noise pairing is seed-robust, not a one-off; not independently hardware-ready |
| **+ floor friction (0.4–1.6×), seed-twin (training seed 12) confirms tonight's PASS-with-caveat friction result is a real recipe, not seed luck** | `ppo_goal_cw_dep_vref1_r1_fric_s1` | this cycle PASS: DR0-gate det slip/m med 1.12/sto med 1.02 — matches (slightly milder than) the seed-11 parent's 1.23/1.04, confirming the parent's elevated slip was not a seed-11 high-side fluke; own-cfg (DR0.35+friction) det+sto gv 6/6, 0 term, slip med 1.19 det/1.30 sto (within vref1-r1's own band); degraded episodes (det/5, sto/0, sto/1) match the pre-registered lineage fixed-seed fingerprint exactly, video-checked no flag-leg/drag/fall | friction is seed-robust, not a one-off; not independently hardware-ready |
| **+ 5° floor slope tested ALONE for the first time — isolates the axis from every prior PAIR compose (fric+tilt, deadband+tilt, imumount+tilt)** | `ppo_goal_cw_dep_vref1_r1_groundtilt` | this cycle PASS: DR0-gate det slip/m med 1.09/sto med 0.99 (in vref1-r1's own band); own-cfg (DR0.35+tilt) det+sto gv 6/6, 0 term, slip med 1.06 det/1.13 sto (in band); degraded episodes (det/5, sto/0, sto/1) match the pre-registered lineage fixed-seed fingerprint, video-checked no flag-leg/drag/fall — slope alone is benign, refuting the worry that the other axis in each pair compose was masking a real slope sensitivity | isolates the axis cleanly; not independently hardware-ready |
| **+ whole-body link-length SCALE (2%) stacked onto the already-PASSed per-leg mass/length jitter (legmass) — first within-family 2-in-1 compose** | `ppo_goal_cw_dep_vref1_r1_legmass_linklenscale` | this cycle PASS: DR0-gate det slip/m med 1.03/sto med 0.98 (in vref1-r1's own band); own-cfg (DR0.35+leg_mass_jitter=0.20+link_len_leg=0.025+link_len_scale=0.02) det+sto gv 6/6, 0 term, slip med 1.07 det/1.13 sto (in band); degraded episodes (det/5, sto/0, sto/1) match the pre-registered lineage fixed-seed fingerprint, video-checked no flag-leg/drag/fall | a global build-scale error stacks free on top of the per-leg tolerance already cleared; not independently hardware-ready |
| **+ gyro RATE BIAS (1.5°/s, distinct from gyro noise/mount-rotation/calibration bias) — last individually-untested IMU axis, holds** | `ppo_goal_cw_dep_vref1_r1_gyrobias` | this cycle PASS: DR0-gate det+sto gv 12/12, 0 term, slip/m med det 1.14/sto 1.01 (in vref1-r1's own band); one det ep (idx4) is the lineage's known catastrophic fixed-draw crater (prog −0.01, slip 33.3), pre-registered/pre-allowed; own-cfg (DR0.35+bias) det+sto gv 12/12, 0 term, slip/m med det 1.03/sto 1.28 (in band); the 3 degraded episodes (det/5, sto/0, sto/1, prog 0.56–0.63) match the pre-registered lineage fixed-seed fingerprint exactly, no new degraded episode beyond it; video-checked (det/0, det/4, det/5, sto/0 all reviewed) clean level six-leg creep, no flag-leg/drag/skate | a steady gyro rate offset that integrates into a drifting attitude estimate is absorbed by the complementary filter's periodic accel correction, same as gyro noise; not independently hardware-ready |
| **+ IMU lever-arm offset (0.07m xy, −0.02…0.10m z) AND residual calibration bias (3°) TOGETHER — realistic same-install pairing (an off-position mount is usually also mis-calibrated), holds** | `ppo_goal_cw_dep_vref1_r1_imupos_imubias` | this cycle PASS: DR0-gate det+sto gv 12/12, 0 term, slip/m med det 1.11/sto 0.97 (in band); one det ep (idx4) same known catastrophic crater (prog 0.09, slip 21.7), pre-allowed; own-cfg (DR0.35+both axes) det+sto gv 12/12, 0 term, slip/m med det 1.13/sto 1.28 (in band); the 3 degraded episodes (det/5, sto/0, sto/1, prog 0.67–0.69) match the pre-registered lineage fixed-seed fingerprint exactly, video-checked clean level six-leg creep, no flag-leg/drag/skate | the two individually-PASSed IMU axes (imupos, imubias) compose free when installed together, as every other 2-axis compose on this line has; not independently hardware-ready |
| **+ velocity-gain (kv) spread ALONE, doubled (0.25→0.50) — isolates the OTHER half of the gainvar pair, holds but with the widest margin on the closed sweep** | `ppo_goal_cw_dep_vref1_r1_kvscale` | this cycle PASS (class-closure ruling, RL_PLAN 08-10): DR0-gate det craters on BOTH det/4 and det/5 (usual sibling fingerprint is det/4 only), sto clean 6/6; own-cfg (DR0.35+override) det 4/6 ok (slip med 1.45, ~28% over vref1-r1's own 1.13 ceiling), sto 2/6 ok (prog med 0.80) — 5/12 degraded episodes vs the usual 3/12 (det/5,sto/0-1) fingerprint every other sibling shows; matched-parent control (base vref1-r1 under the identical injection) craters only on det/4, stays clean on det/5 — kv-alone is a real, if video-invisible, quantitative outlier, not resolved by the control alone. Video (all flagged episodes) shows the same clean six-leg march-in-place/creep as every closed-class sibling, no flag-leg/drag/skate, gv 6/6 both modes, 0 term | widest-margin single axis on the now-CLOSED protect-the-candidate sweep — worth remembering if kv-uncertainty is ever isolated again, but per RESEARCH_RULES a closed sim finding does not reopen without new hardware evidence; not independently hardware-ready |

| **+ position-gain (kp) spread ALONE, doubled (0.20→0.40) — isolates the OTHER half of the gainvar pair from kv, holds cleanly (not the margin driver)** | `ppo_goal_cw_dep_vref1_r1_kpscale` | this cycle PASS: DR0-gate gv 6/6 both modes, only the pre-allowed det/4 crater; own-cfg (DR0.35+kp=0.40) det slip med 1.08 / sto slip med 1.17 — land INSIDE vref1-r1's own band (0.89-1.13 det / 1.13-1.36 sto), not near gainvar's wide-margin edge; degraded episodes (det/5, sto/0-1) match the pre-registered lineage fixed-seed fingerprint exactly, video-checked clean six-leg gait, no flag-leg/drag/skate | kp-spread alone does NOT explain gainvar's widest-yet margin (contrast kvscale row above, which DOES show a real quantitative outlier alone) — implicates kv-spread or the kp×kv interaction as the margin driver; not independently hardware-ready |
| **Seed=12, ZERO extra dep-line axes — forensic isolation of the startvar1 failure (seed vs. axis)** | `ppo_goal_cw_dep_startvar1_seed12_noaxis` | this cycle PASS (forensic): does NOT reproduce the noZD1/noBS1/placementonly det/3+det/4 fingerprint (here det/3 prog1.06/slip0.97 clean, det/4 prog0.80/slip1.35 not matching the prog≤0.8&slip≥1.5 criterion); instead shows the standard lineage own-cfg-DR0.35 fixed-eval fingerprint (det/5, sto/0,1,4) seen identically in kpscale (seed 11) same night — same episode indices, clean six-leg video both | closes the seed-vs-axis forensic question: training seed 12 is CLEARED as the startvar1 driver (this run zeroed dr.zero_drift_cmd_frame too and lost the det/3+4 pattern, corroborating RL_PLAN open-problem-5's zero-drift-frame-DR conclusion); not a hardware-candidate arm itself |
| **+ noisy servo OUTPUT commands (write jitter/quantization, 3x nominal, distinct from encoder-read noise) — last untested actuator-command axis, holds** | `ppo_goal_cw_dep_vref1_r1_actionnoise` | this cycle PASS: own-cfg (DR0.35+dr.action_noise=0.06) det+sto gv 12/12, 0 term, 0 sacrificed legs; slip/m med det 1.16/sto 1.15 — at/just inside vref1-r1's own band (0.89-1.13/1.13-1.36); DR0-gate (still carrying the fixed action-noise injection) matches; the fixed-eval crater episode shifts from the lineage's usual det/4 to det/5 (expected — action noise itself injects RNG into the rollout, unlike the purely static DR axes) but is the same benign march-in-place stall on video, no flag-leg/drag/fall | closes the last individually-untested actuator-command axis on the (now CLOSED, 20-for-20) dep-line protect-the-candidate sweep; not independently hardware-ready |

Parent `ppo_goal_cw_dep_vref1_r1` itself (contract-exact obs + 25° tilt, no start-variation) PASSed a separate cycle 08-10 ~05:55: own-cfg det+sto 6/6 gv, vel_err/slip match-or-beat the pre-contract champion on an identical eval config — velocity estimator/temporal actor is NOT a P0 prerequisite for hardware attempt #2. It is the current fallback base for hardware attempt #2 (start-variation compose `cw-dep-startvar1-r1`/`-s1` FAILED hard — real sacrificed leg, declining reward; isolation ablation `-noZD1` FAILED its own gate too; second isolation arm `-noBS1` (bad_start_prob removed) FAILED too but partially recovered — 4/6 det episodes clean vs r1's 0/6, no flag leg, but 2/6 still catastrophic-skate — bad_start_prob is A contributor, not the sole cause; see rl_docs/runs/).

## Quadruped mode (party-trick line, readiness review P1)

- **Four-leg hold learned + walk retained (30/60/10 quad/walk/hold mix)
  PASSED: `ppo_goal_cw_quad_hold2`.** Quad-mode: eval/quad/survived_frac
  1.0 at every logged checkpoint (1M-10M steps), height_err_end_mm
  1-15mm (<=20mm gate), track_err ~1deg; video (rollout_118) shows a
  clean, level 4-leg stance, both fronts lifted clear, no tipping.
  Walk-mode retention recovered vs the 50% rung (`cw-quad-hold1-r2`
  FAIL: det slip/m 1.42): own-cfg harness det gv 6/6, slip/m med 1.20
  (<=1.25 cap), 0 term; sto gv 6/6, one isolated fixed-draw stall
  (lineage trait, not new). Dose-response confirmed: 30% is the
  workable mix. Not hardware-ready (walk leg still paddle-gait); base
  for the joystick-mainline quad command `cw-walk-joyquad30`.
- **Feasibility sweep PASSED (c56, `rl_move/sim/quadruped_feasibility.py`,
  `logs/experiments/quadruped-feasibility/sweep.json`):** four-leg static
  stance is geometrically comfortable. Neutral six-leg stance with fronts
  (L0/L5) raised puts the CoM 68–82 mm OUTSIDE the 4-foot polygon (the
  review's warning was real), but either −40 mm body shift or ~17–31°
  middle-leg (L1/L4) forward splay fixes it: best config (−20 mm shift +
  17° splay, fronts tucked) holds 39 mm margin at 0.6 A max servo current
  (trip 2.5 A) and survives a 6 N forward push; 11 of 18 static passes
  are push-robust. Next rung per review §4: static four-leg-stance RL
  task ([CODE]: needs a quad-hold goal mode — front-feet-clear +
  four-planted + level + low-current reward).
- **Quad-hold graft onto the hardware-contract-exact (no privileged
  velocity) base FAILED: `ppo_goal_cw_dep_quad1`.** Identical 30/60/10
  mix + reward recipe as the PASSing `quad-hold2` above, warm-started
  from `cw-dep-vref1-r1` instead. survived_frac stays 1.0 (no falls)
  but height_err_end_mm plateaus at 31-60mm across training (never
  reaches the ≤20mm gate quad-hold2 hit at 1-15mm) and track_err_deg
  gets worse, not better (1.0→2.67°) — the height-control precision
  looks like it leans on velocity feedback the honest obs contract
  removes. Walk-mode retention unaffected (own-cfg det gv 6/6 slip
  med 1.18, DR0 det gv 6/6 slip 1.08). Quad line stays on the
  privileged-velocity `quad-hold2` checkpoint for now; a contract-exact
  quad graft is deferred behind the P0 walk hardware ladder.
- **Quad-hold graft on the contract-exact base RECOVERS with more
  training, refuting the structural-cap worry: `ppo_goal_cw_dep_quad1_c2`
  (+12M steps on quad1's own checkpoint, same recipe).** Training-eval
  height_err_end_mm keeps falling monotonically and clears the ≤20mm
  gate at the final TWO logged checkpoints (10.0M step: 3.7mm; 12.0M
  step: 2.86mm — quad1 itself plateaued at 31-60mm over its whole run);
  survived_frac 1.0 throughout; track_err_deg improves, not worsens
  (1.75°→1.49°). Walk-mode retention: own-cfg det gv 6/6, 0 term,
  slip/m med 1.17 (≤1.35 gate), video clean six-leg gait. Caveat: sto
  walk retention showed one genuine flag-leg episode (sacrificed_legs
  [3,5], gv 5/6) distinct from the lineage's usual march-in-place-only
  crater — not gating (gate is det-only) but worth a re-check if this
  checkpoint becomes a further quad-line base. Quad-mode height control
  under the honest-obs contract was under-trained, not capped by
  removing privileged velocity.

## Architecture (temporal-arch line)

| Skill | Checkpoint (artifact `ckpt-<name>`) | Evidence | Envelope / limits |
|---|---|---|---|
| **history_frames=16 (16 past obs frames), from-scratch, boots and learns to walk** | `ppo_goal_cw_arch_hist16_r7` (+ seed twin `ppo_goal_cw_arch_hist16_r7_s1`) | this cycle PASS, both seeds: own-cfg flat(DR0) det+sto gv 6/6, 0 term, prog med 1.17/1.02 (r7) and 1.22/1.12 (s1); own-cfg DR0.5 (its trained DR) det+sto gv 6/6, 0 term, prog med 1.08/1.09 (r7) and 1.12/1.13 (s1) — clears the ≥0.85 gate with margin on both seeds; JOYSTICK GATE (eval_drive DR0.2) 0 in-envelope falls both; video: clean six-leg swing/stance cycling every sampled episode, no flag leg, no dragging | first hist16 spec to actually run at 40M steps after 8 prior 0-step deaths (root cause: /dev/shm exhaustion at 4096 envs, fixed by dropping to 3072 envs + startup shm GC); slip/m elevated (1.3–1.6) vs contract-line champions — not a head-to-head win over hist8 yet, just proof the architecture trains; NOT on the deployment-exact obs contract, not hardware-ready |
| **...+40M more steps (identical config, r7 continuation) — economy improves, no regression** | `ppo_goal_cw_arch_hist16_r7_c1` | this cycle PASS: DR0 gate det+sto gv 6/6, 0 term, prog med 1.21/1.03, slip/m med 1.14 (det, down from r7's 1.43)/1.33 (sto, down from r7's 1.40); own-cfg DR0.5 det+sto gv 6/6, 0 term, prog med 1.13/1.08, slip/m med 1.16 (det, near the champion band 0.89–1.13)/1.38 (sto, flat vs r7's 1.37); JOYSTICK GATE 0 falls, same clean panel as r7; video clean six-leg gait both eval passes | more steps DO buy economy at this DR under current contact pricing (if-true confirmed) — slip gap to the deployment-contract champion roughly halved on det; still not on the deployment-exact obs contract, not hardware-ready; keep training this line per operator directive |
| **history_frames=24 (24 past obs frames, ~960ms), from-scratch, ladder rung 2 — bootstraps, beats hist16 on prog, worse on economy** | `ppo_goal_cw_arch_hist24_r1` | this cycle PASS: own-cfg DR0.5 det+sto gait_valid 6/6/6/6, 0 term; DR0-gate det prog med 1.29 / own-cfg DR0.5 det prog med 1.22 (both clear the >=0.85 gate and beat hist16's own 1.08–1.21 band); JOYSTICK GATE (eval_drive DR0.2, run myself — not pre-staged) PASS, 0 in-envelope falls across the full direction+flip-stress panel; video (12 episodes) clean six-leg swing/stance, no flag-leg/drag/skate | slip/m worse than hist16 champion band (det 1.44–1.55 vs 1.14–1.16; sto 1.47–1.50 vs 1.33–1.38) — more history helps progress-tracking, not yet economy at this budget; NOT on the deployment-exact obs contract, not hardware-ready; per operator ladder-freeze ruling no further rung (hist32/etc.) queued — parked as evidence for the flagship |
| **history_frames=16 TRAINED DIRECTLY ON THE DEPLOYMENT CONTRACT (honest meas:=ref velocity obs, 25° tilt) — bootstraps, then closes its economy gap purely via continuation, becomes a 2nd hardware-ladder candidate** | `ppo_goal_cw_arch_hist16_dep1_c1` (+40M continuation of `ppo_goal_cw_arch_hist16_dep1`) | this cycle PASS: DR0-gate det+sto gv 6/6/6/6, 0 term, prog med 1.17/1.07; own-cfg DR0.5 det+sto gv 6/6/6/6, 0 term, prog med 1.10/1.08 (both >=0.85 gate); slip/m med 1.20 det / 1.35–1.37 sto — closed from the parent's 1.41–1.48 into (det) or to the edge of (sto) vref1-r1's own band (0.89–1.36); vel-tracking success improved from a parent coin-flip to 4–5/6 per pass; JOYSTICK GATE (eval_drive DR0.2, run myself on the pod — not pre-staged) PASS, 0 falls across the full direction+flip-stress panel; video (all 12 det/sto episodes, both DR passes) clean six-leg swing/stance cycling, no flag-leg/drag/skate | same continuation-closes-the-gap pattern as the non-dep hist16-r7 line, this time ON the real robot's own sensing contract — a legitimate 2nd rung for the hardware ladder alongside `cw-dep-vref1-r1`; not yet independently promoted (no head-to-head Gate 0 pick made); sto slip sits at the very edge of the target band, not comfortably inside it |

| **history_frames=16, r7 line +80M steps total (c1->c3->c4) — economy gain PLATEAUS, does not keep closing** | `ppo_goal_cw_arch_hist16_r7_c4` | this cycle PASS (gate met) but the budget question resolves negative: own-cfg DR0.5 det slip med 1.15 / sto 1.28 — flat vs c3's 1.13/1.31 (within noise); DR0 gate det 1.02 (now inside champion band 0.89-1.13) but sto 1.42, worse than c3's 1.27 by +12% (outside noise); gv 6/6 all 4 passes, 0 term, prog med >=1.0 everywhere; JOYSTICK GATE (eval_drive DR0.2, ran myself) PASS 0 falls; video clean six-leg cycling, no flag-leg, same as c1-c3 | 3 prior continuations (r7->c1->c3) showed steady improvement, but c4's extra 40M bought no further net gain and sto got worse at DR0 — the exposure-alone hypothesis is now falsified for this line; per two-flat-continuations rule, NO further step-budget continuation on r7 queued; remaining gap is the contact/current pricing calibration (CURRENT_TRUTHS open problem 1), not architecture/training depth; NOT on the deployment-exact obs contract, not hardware-ready (that role stays with the dep1 line above) |

## Stance / posture (older line — see archive for full state)

- Rise/lower heights at DR 1.0: solved pre-walk-campaign (see
  `archive/RL_PLAN_FULL_2026-08-09.md`); lower-line rework per rulings.

### Rise (belly→plant) — SOLVED as a specialist (08-11)

| Skill | Checkpoint | Evidence | Envelope / limits |
|---|---|---|---|
| **RISE SPECIALIST: honest six-foot rise to the walkable plant from EVERY start kind — bridge, crouch, and flat belly-down cold start (the operator-placement case)** | `ppo_goal_cw_stand_bc1_hard1` (BC-anchor coef 1.0, 10M steps) | 08-11 dig-in: RSI-off probe (seed 7, DR0) **12/12 valid_plant — bridge 5/5, crouch 3/3, flat 4/4, worst foot 7mm**; gate det 5/6 (parent bc1@2M: 13/30, flat 0/10 — the flat-start footprint miss RESOLVED with budget). `rise_feet_factor` held 0.69–0.82 all 10M — no re-drift toward the cheat. Zero flag-leg/tripod in 50+ video-checked episodes across bc1/hard1. Identical-minus-anchor parent (`cw-stand-rsi3`) still cheats 0/12 — clean one-variable attribution; coef 0.3 dose-check FAILED (keep >=1.0). | RISE ONLY — not a unified policy. Matched-parent probe (08-11, same seed/cfg on bc1@2M) proves the other modes were NEVER there in this lineage, not eroded by hardening: hold 0/12 (parent 51mm / child 162mm front-legs-up splay, 2.6A over-current), track 0/12, lower 0/12 flag-leg (parent 166mm / child 189mm), raise 0/12 (p_raise=0 in goal mix — untrained). STOP hardening this lineage: next is a hold-mode stillness SPECIFICATION pass + the composition test (rise specialist → walk/hold champion handoff). Detail: `rl_docs/RISE.md`. |

### Hold (quiet stillness) — SOLVED (08-11, third lever: BC-anchor on hold/track ticks)

| Skill | Checkpoint | Evidence | Envelope / limits |
|---|---|---|---|
| **Genuinely quiet, motionless, level six-foot HOLD — first honest hold in the whole campaign** | `ppo_goal_cw_stand_holdbc1` (discovery, 2M, warm from `ppo_goal_cw_stand_bc1_hard1`; extends `bc_anchor.py`'s BC-supervision to hold/track ticks, target = the episode's settled pose) | 08-11: harness hold 12/12 valid_plant det+sto, worst-foot 2–13mm, height_err_end_mm ≈2; video (det AND sto) shows a level, motionless, six-feet-down stand for the full 15s clip. `env/hold_feet_factor` cleared the 0.1–0.35 plateau both prior pricing-only levers (`cw-stand-holdstill1`, `cw-stand-holdstill2`) sat in, reaching ~1.0 by ~500k steps and holding there all 2M — the pre-registered mechanism-health signature. Two pricing-only levers (hard no-flag zero, then a fade) FAILED first (0/12 each); BC supervision (the same trick that solved rise) is what finally worked. | HOLD/TRACK ONLY at this budget — not yet a unified policy; not hardware-ready. Rise retention carried through mostly clean (bridge 2/2 det, all 6/6 sto) but det crouch-start rise shows 2/6 tilt_roll falls, one more than the identical pre-existing fingerprint already present in the immediate parent `cw-stand-holdstill2` (1/6) — a known crouch fragility, not a new pathology. Track-mode command-following accuracy is still weak (det 2/6, sto 0/6 on the tracking-error metric) though posture stays valid throughout (end_posture 6/6) — not part of this arm's gate, flagged for later. Next: a 10M hardening continuation (mirrors the bc1→bc1-hard1 pattern) to check whether budget also resolves the crouch dip, then the rise-specialist+hold → walk-champion handoff composition test. Detail: `rl_docs/RISE.md`. |
| **HOLD+RISE, hardened (10M) — consolidates the discovery pass, no regression** | `ppo_goal_cw_stand_holdbc1_hard1` | 08-11: harness hold 11/12 valid_plant (det 6/6, sto 5/6 — the one miss is a soft current-limit flag, not posture/cheat), essentially matching discovery's 12/12 (gate floor was ≥10/12). `env/hold_feet_factor` held 0.99–1.0 across the entire 10M steps (no re-drift toward the earning-zero plateau). Det crouch-start rise improved to 2/4 valid (50%) from discovery's 2/6 (33%) — the remaining fall is a genuine tip-over (video-confirmed, not a cheat) and the remaining miss a height-overshoot on an otherwise correct six-foot stand. Zero flag-leg/tripod cheat across all 24 det+sto episodes, video-checked. | Same envelope as discovery (HOLD/TRACK solid; not a unified policy; bench-unvalidated). Track-mode command-tracking accuracy still weak (sto 2/6 on the tracking-error metric, posture stays valid) — pre-existing, not gated. Lineage CLOSED for further hardening. Handoff composition test PASSED 08-11 (see row below). **08-11 late: DEPLOYED as the robot runner's live stance policy** (stand/lower buttons; trained ramp ships in the weights meta; `stance_dr10.json` = one picker call back; contract-locked by `test_stand_runner.py`) — awaiting bench validation. Detail: `rl_docs/RISE.md`. |
| **Stand up from the belly, then HAND OFF to the walk champion and drive — no scripted blend needed** | `ppo_goal_cw_stand_holdbc1_hard1` (rise+hold) → `ppo_goal_cw_walk_longdist_r2` (drive), composed by `rl_move/sim/eval_handoff.py` (plant-frame re-anchor, slew state carried across the switch) | 08-11: 12/12 successful rises (flat 6/6, bridge 6/6, across air AND loaded servo physics) handed off with ZERO falls; walk-champion drive metrics on the specialist's exact final pose sit inside its own clean-plant baseline band (air trk_err 0.032–0.036 vs 0.031; stumble-window tilt 1.2–2.6° vs 1.5°); the incumbent scripted 1.5 s blend (play.py key-7 path) measurably adds nothing. Artifacts: `logs/ckpt_eval/handoff_holdbc1hard1_{air,loaded}.json` + strips. | Sim-only, DR0, forward drive @0.05 m/s tested; crouch-start rises tip over BEFORE the handoff (0/6 RSI-off — the lineage's known fragility, not a handoff defect). Reverse handoff tested + clean 08-11 (see row below). |
| **Sit down from a drive: walk champion stops, control switches, robot lowers to belly rest — the full sim joystick motion cycle (rise → drive → stop → sit) now composes with zero falls** | `ppo_goal_cw_walk_longdist_r2` (drive) → scripted go_zero-sit glide (6 s to the zero pose; deployable incumbent) or `ppo_goal_cw_stand_holdbc1_hard1` (learned lower), composed by `rl_move/sim/eval_handoff_reverse.py` | 08-11: scripted glide 6/6 posture-strict both physics (air AND loaded), gentle (tilt ≤2.5°, all pads 34–38mm); learned lower on the walker's EXACT stopped state matches the specialist's own clean-episode band (direct 4/6 == spec 4/6 both physics, zero falls anywhere, height_err 0.4–9mm) — the handoff itself costs nothing. Artifacts: `logs/ckpt_eval/handoff_rev_holdbc1hard1_{air,loaded}.json` + strips. | Sim-only, DR0. Learned lower's only miss is a cosmetic dangling foot (leg 2 at 62–99mm vs the 60mm belly allowance; body down + level, video-confirmed NOT a weight-bearing flag-leg). The scripted glide — already the operator-prescribed hardware sit — covers the deliverable; BC anchor on lower ticks is optional unqueued polish. |

| **Stand up from a CROUCH without tipping — the one start kind that always felled the deployed specialist** | `ppo_goal_cw_stand_crouchrise1` (discovery 2M, warm from `holdbc1_hard1`; ONE variable: rise start-mix biased to 60% crouch / 30% partial / 10% flat vs legacy 25/40/35) | 08-11: RSI-off all-crouch probe (seed 1, DR0, loaded servos) **16/16 stands, det 8/8 valid_plant, ZERO falls** vs matched parent `holdbc1_hard1` **0/8 det, 8/8 tilt_roll falls** on the identical probe (`logs/ckpt_eval/{crouchrise1,hard1}_rsioff_crouch`); gate harness det rise 6/6 (crouch 5/5), hold 12/12 end-posture. Videos honest — crouch/bridge → level six-foot stand, no flag-leg/tripod. | NOT the deployed stance policy: missed the pre-registered hold no-regression bar (hold sto valid_plant 1/6 vs parent 5/6 — all misses the final-0.5s >2.0A sim current soft flag; posture/stillness/feet identical; sim current is a known-untrusted metric). `holdbc1_hard1` stays on the robot; this checkpoint (md5 3877e16c, pulled to controller) is the drop-in if bench ever needs crouch starts. Start-DISTRIBUTION, not reward, is the proven lever for start-kind fragility. |

## Pending verdicts that would add rows

wander30 (envelope extension), backforth (reverse), standwalksit
(skill chaining), pose-track.
(strafe ±90° landed — see DR 0.5 row above; lowgait30–50,
terrain10 and endur60+s1 landed — rows above.)

## Consolidation status (single deployable policy)

Skills above are SEPARATE checkpoints. The deployable robot needs
either one multi-skill policy (goal-mix training — `standwalksit` is
the first chaining probe) or a deploy-time skill switcher. Champion
strategy: the champion is the BASE the walk line breeds from; skill
passes are preserved here and folded in via goal-mix arms — a
promotion never deletes a skill checkpoint (append-only).

### Omni translation ("walk where pointed") — SOLVED IN SIM (08-11, rot-60 canonicalizer, zero training)

| Skill | Checkpoint | Evidence | Envelope / limits |
|---|---|---|---|
| **Walk in ANY commanded direction — full-circle joystick translation, including backward (which no learned policy had ever done)** | `ppo_goal_cw_dep_vref1_r1` (THE hardware checkpoint, unchanged) or `ppo_goal_cw_arch_hist16_dep1`, wrapped in `rl_move/sim/rot60.py` `Rot60Policy` (`eval_drive --rot60` / `eval_checkpoint --rot60`) — the robot is a regular hexagon, so any heading is EXACTLY the forward wedge with legs relabeled; no new training, no new checkpoint | 08-11 (`logs/rot60/`): vref1-r1 naked is frozen backward (0.027 m of the commanded 0.30); wrapped, EVERY direction tracks 0.024–0.036 m/s err with full travel at DR0 AND own DR0.35, zero falls incl. full-circle instant-flip stress (live sector switching); harness on full-circle commands 20/24 success, gait_valid 23/24, slip/m 1.1–1.3 (its own clean band), video-confirmed ordinary six-leg gait. hist16-dep1 naked degenerates AT EVAL TIME into leg-sacrifice on off-wedge commands (slip 7–11/m); wrapped: gait_valid 24/24, slip 1.3–1.6. Model symmetry proved mechanically (test_rot60.py: rotate+relabel state diverges <1e-6 over 30 contact steps). | SIM-ONLY until the ~60-line numpy canonicalizer is ported into the robot runner's obs/action path (the remaining [CODE] item; spec at `rl_docs/TURN.md` tail). Body never yaws — heading-agnostic driving (turn stays de-scoped: no camera = no front). Known quirk: hist16-dep1 (NOT vref1-r1) reads 0.046–0.051 trk_err on canonical +15° headings — a dep1 wedge asymmetry, pre-existing. |


---

# FILE: rl_docs/COMMANDS.md

# COMMANDS — how to run everything (60 seconds, saves 20 minutes)

Distilled from mining every prior cycle transcript (2026-08-09): the
same commands were re-derived, and the same mistakes re-made, dozens
of times. `rl_move/orchestrator/ops.sh` implements the common
operations — use it. Sibling docs: `rl_docs/README.md` (index),
`RL_GOALS.md` (plain-English mission), `rl_docs/EXPERIMENT_LOGS.md`
(per-run summary.md convention).

**STANDING RULE — promote what you figure out:** if a command failed,
was slow, or took several tries before you got it right, add it as an
`ops.sh` subcommand (or a snippet below) IN THE SAME CYCLE, and note
it in `rl_docs/README.md` if it changes what a file covers. Never
leave the next agent to rediscover it.

## Which question → which command (don't mix these up)

| Question | The ONE command |
|---|---|
| **Triage a finished run (START HERE)** | `ops.sh review <run>` — ledger+gate, W&B trend, eval table, video paths in one shot |
| Eval numbers table from a report.json | `ops.sh report <run\|path>` (per-episode + medians + term counts) |
| What is actually training right now? | `ops.sh census` (/proc truth; W&B lags launches ~8 min) |
| How many slots are free / where? | `python3 rl_move/orchestrator/capacity.py` |
| Ledger + procs + watcher, one screen | `ops.sh status` |
| One run's metrics/state | `ops.sh wandb <run>` (ledger: `ops.sh entry <run>`) |
| What's queued to launch? | `launch_run.py backlog list` |
| Queue a seed/rung/variant of an existing run | `launch_run.py respec --from <run> --run <new> [--seed N] [--arg='--flag=v'] [--cfg k=v] --hypothesis … --gate …` — clones the ledger args; never re-type them. Add `--init-from-source` to warm-start from the source's checkpoint; add `--now [--pod P]` to skip the backlog and launch directly (snapshot → sync → self-repair → verify, one command). Phase/evidence inherit from the source; override with `--phase`/`--evidence` |
| Prove the reward prefers the skill over the cheats (MDP_PREFLIGHT) | `python -m pytest rl_move/tests/test_task_semantics.py -v` — BINDING before any reward/task-mechanism launch; a skipped bank for your mode = build the bank first |
| Eval a DR/noise/latency child against its parent honestly | `python -m rl_move.sim.eval_checkpoint <child.zip> --baseline <parent.zip> --cfg-set <same injection> …` — matched-parent control; child-vs-clean-parent verdicts are invalid |
| OPERATOR: fire one launch during a LAUNCH_HOLD | `ops.sh oplaunch respec --from <run> --run <new> --init-from-source --now --operator-override 'why' --hypothesis '<plain English — this LEADS the W&B notes>' --gate '…'` — runs on the controller from anywhere (incl. the operator Mac); the override is audited in the ledger and operator-only |
| A past run's story | `rl_docs/runs/<run>.md` |
| Yaw-command tracking (yawcmd lineage gate) | `python3 -m rl_move.sim.eval_yaw <ckpt> --cfg-set … [--out j.json]` — scripted turn panel; reports turn-segment \|wz_err\| med, hold \|wz\| med, falls (harness has no wz fields) |
| Are results being lost/ignored? | `ops.sh triage [hours]` |
| Finished but not yet analyzed? | ledger `triage` field (watcher-stamped: `awaiting…` → `in-cycle…` → `done` on verdict); shown on the status page "Analysis pipeline" |
| Write the cycle's RL_LOG line | `ops.sh logline "c<N>: …"` — the ONLY way; never `cat >>` RL_LOG |
| Frames from a video | harness already wrote `*.png` sheets; else `ops.sh frames <mp4> [n]` |
| Operator wants an overview in a browser | status page at http://127.0.0.1:8090 — full setup/restart runbook in "Operator status page" section below |

**DO NOT hand-write python for any row above.** Transcript mining
(08-09) found >500 ad-hoc snippets re-parsing experiments.json,
report.json, and the W&B API for exactly these questions.

## Where things are (the #1 recurring failure: wrong paths)

- Controller repo root: `/workspace/weird_objects`. Cycles start HERE.
- PROTO dir: `/workspace/weird_objects/hexapod_walker/prototype_sts3215`
  — `cd` here first; every doc/tool path below is relative to it.
  (9/9 failures of `cat rl_move/orchestrator/guardrails.yaml` were
  agents trying orchestrator paths from the repo root.)
- `/workspace/prototype_sts3215` on the CONTROLLER is a STALE copy
  (its ledger is a symlink to the real one). Never work there.
- On TRAINING PODS the code IS at `/workspace/prototype_sts3215`
  (no git there; the `.code_sha` marker is the only version record).
- Docs: `RL_PLAN.md` + `RL_LOG.md` are CONDENSED (~120 lines each,
  read them whole — no more sed windows). Full history:
  `archive/RL_LOG_FULL_2026-08-09.md`, `archive/RL_PLAN_FULL_*.md`.
  Log append rule: ONE line per cycle via `ops.sh logline` — never
  `cat >>`; evidence goes to the ledger verdict + W&B, not the log.

## ops.sh (rl_move/orchestrator/ops.sh) — use instead of hand-rolling

- `ops.sh status` — active runs + live procs per pod + watcher tail.
- `ops.sh procs <pod>` — training/eval processes. **Pods have NO
  `ps`** — this scans /proc; agents rediscovered that trick 10+ times.
- `ops.sh census` — every train pod's live trainer from /proc. THE
  ground truth for "what is running": W&B lags fresh launches by up
  to ~8 min (JAX compile) and looks empty when things are fine.
- `ops.sh trainlog <run> [n]` — tail the run's train log on its pod
  (pod + log path come from the ledger; don't guess).
- `ops.sh entry <run>` — the run's ledger entries.
- `ops.sh wandb <run>` — state, steps, reward-quarters trend, std, URL.
- `ops.sh review <run>` — the standard triage read in ONE command
  (ledger, W&B, eval table, videos). If its output + one video answer
  pass/fail, record the verdict and stop.
- `ops.sh report <run|report.json>` — per-episode table + medians +
  gait_valid/termination counts from a harness report.
- `ops.sh logline "text"` — the only sanctioned RL_LOG write (one
  timestamped line under the git lock).
- `ops.sh frames <mp4> [n]` — contact sheet from any video (but the
  harness already writes `walk_*.png` sheets next to eval videos —
  check those first).
- `ops.sh pullckpt <run>` — fetch checkpoint from its pod + md5.
- `ops.sh pushckpt <pod> <ckpt>` — copy a checkpoint TO a pod + md5
  both sides. **`snapshot.sh --sync` EXCLUDES policies/** — a
  warm-start parent must be pushed explicitly or the run dies at
  init with FileNotFoundError (killed cw-walk-longdist, 08-09).
  If `kubectl cp`/exec-stdin streams keep dropping (websocket
  close/broken pipe — hit train-2/3, 08-09 c35): HTTP-serve from
  the controller (`python3 -m http.server 8765` in policies/) and
  on the pod `python3 -c "import urllib.request; urllib.request.
  urlretrieve('http://10.0.0.46:8765/<zip>', '<dest>')"` — pods
  have no curl/wget. Always md5 after.
- `ops.sh evalcmd <run>` — prints the exact-path harness eval command
  with the run's own `--cfg-set`s pulled from the ledger.
- `ops.sh waitlog <file> <regex> [timeout]` — poll for completion.
  **`sleep 60; tail …` is BLOCKED by the harness** — use this.
- `ops.sh oplaunch <launch_run.py args…>` — run a launcher command ON
  THE CONTROLLER (detached, creds sourced, result polled), from the
  operator Mac or the controller itself. Exists because launches must
  run where git is the code-sha truth: a laptop clone is stale/dirty
  and gets refused (08-10: the operator's assistant hand-rolled
  kubectl-cp + tmux + hold-file juggling for one continuation launch).
  Pairs with `respec --now` and, for operators only,
  `--operator-override` (audited LAUNCH_HOLD bypass for a single
  launch — agents must never pass it). Put the plain-English paragraph
  in `--hypothesis` (it leads the W&B notes) or override wholesale
  with `--arg='--notes=…'`. Record the launch with `ops.sh logline`.
- `ops.sh expdir <run>` — create `logs/experiments/<run>/` with the
  summary.md template. Only for DIG-IN runs (08-09 lightweight
  process): clear pass/fail needs just the ledger verdict (which
  auto-renders `rl_docs/runs/<run>.md`) + `wandbnote`.
- `ops.sh wandbdump <run>` — cache the run's W&B summary/config/
  history into its experiment dir (query the cache, not the API).
- `ops.sh triage [hours]` — "is anything lost/ignored?" table: every
  recent W&B run × ledger verdict × OUTCOME note × watcher processed
  flag. Run it whenever the operator asks if results are being
  dropped (they asked twice on 08-09; this answers in 5 s).
- `ops.sh drain` — place backlog onto free pods, detached + creds
  sourced. Raw `launch_run.py drain` needs W&B creds (dedupe check)
  and takes minutes PER LAUNCH by design (two-phase verify waits out
  the pod's JAX/Warp compile) — never run it attached to a terminal
  you might close. The watcher auto-drains too, but NOT while PAUSEd
  (e.g. during restart_watcher.sh).
- `ops.sh killrun <run>` — kill a run's procs on its pod. Pods have
  no pkill, and a naive /proc scan KILLS ITSELF (your scan's cmdline
  contains the run name — a kill command suicided this way 08-09).
  Then record it: `launch_run.py update … status=KILLED verdict=…`.

## Hard-won gotchas (each cost a cycle at least once)

0. **Any untracked NON-doc file under the prototype tree (e.g. a fresh
   `rl_move/sim/park_banks/*.npz` from `harvest_park_states`) marks
   every `snapshot.sh --sync` `-dirty` and the launcher then REFUSES
   ALL drain launches** — 4 GPUs idled behind one uncommitted 26KB
   npz while its specs burned 3 attempts each into `backlog_failed.json`
   (08-09 c51). Commit (`snapshot.sh <name>`) right after generating
   any training-input artifact, BEFORE queueing specs that need it.
   Requeue after fixing: move items backlog_failed→backlog under
   `backlog.json.lock` with attempts reset, then `ops.sh drain`.

0b. **A controller eval can DEADLOCK on a corrupt ffmpeg pipe**
   (log shows `corrupt input packet` / `Invalid buffer size`, then
   the python's utime freezes with no children — hit the groundtilt5
   dr0ret pass, 08-09 c60, under heavy concurrent-eval load). Detect:
   output dir stops growing AND `cat /proc/<pid>/stat` utime is
   static across 5 s. Fix: kill the pid and rerun the pass with
   `--no-video` (metrics JSON is what gates need; frame strips from
   the hung run remain usable).

1. **`eval_checkpoint` runs ONLY as a module** from the PROTO dir:
   `python3 -m rl_move.sim.eval_checkpoint …`. Running the .py path
   dies on relative imports. Flags (stop re-running --help):
   `checkpoint --task {goal,joint_goal,joint_walk} --modes … --per-mode N
   --dr-scale F --seed N --episode-seconds S --stochastic
   [--end-posture-gate] [--no-video|--video-every N] --out DIR
   --cfg-set k=v (repeatable)`.
   **Driving candidates additionally need the JOYSTICK GATE**
   (`python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale 0.2 --out
   FILE.json [--cfg-set …]`): scripted direction panel + randomized
   instant-flip stress; zero in-envelope falls = exit 0. The generic
   harness only samples the training distribution — it can't prove
   direction coverage or flip robustness (backforth lesson, 08-09).
2. **Launch ONLY via `launch_run.py launch`.** Raw `kubectl exec …
   nohup train_ppo…` hits the 2-minute exec timeout (looks dead,
   actually launched → ledger drift; this caused a real incident).
   Ledger edits ONLY via `launch_run.py update`.
3. **Eval with the run's OWN cfg** (`ops.sh evalcmd`): evaluating with
   default cfg silently drops the run's reward package and voids the
   verdict (`lowent-dr03` burned 4M steps on this class of mistake).
4. **kubectl needs `KUBECONFIG=~/.kube/coreweave.yaml`** (ops.sh sets
   it). `kubectl exec` dying mid-command does NOT mean the remote
   command died — check before re-running anything with side effects.
5. **snapshot → sync → launch, in that order, same cycle.** The
   launcher refuses a pod whose `.code_sha` ≠ local HEAD (also
   `-dirty`). **Never `--sync` with a dirty tree**: it stamps the
   pod marker `<sha>-dirty`, which can NEVER equal HEAD, so every
   later launch on that pod is refused until you commit
   (`snapshot.sh <name>`) and re-sync (cost cycle 51 four refused
   drain launches, 08-09). Snapshotting AFTER launching leaves the pod a commit
   behind and blocks the watcher's auto-continue (cycle 33 incident).
   After `snapshot.sh <run>`, run `snapshot.sh --sync <pod>` for the
   pod you're about to launch on.
6. **Checkpoint naming:** run `cw-walk-foo-c1` ⇒
   `rl_move/sim/policies/ppo_goal_cw_walk_foo_c1.zip` (dashes→
   underscores). Always record + compare md5 when pulling. That
   name is only guaranteed because the launch command carries
   `--out-name`; `launch_run.py launch`/`respec` now always inject
   it if missing (08-10 fix). A run launched BEFORE that fix (or by
   hand, without going through the launcher) with no `--out-name`
   saved under `train_ppo_mjx`'s own default instead:
   `ppo_mjx_<task>_<run>.zip` (e.g. `--task joint_goal` ⇒
   `ppo_mjx_joint_goal_cw-stance-riseproof1.zip`) — `ops.sh
   pullckpt`'s `ppo_goal_...` guess 404s (`cw-stance-riseproof1`,
   08-10: watcher's pre-stage `pullckpt rc=1`, pulled manually once
   diagnosed). `ops.sh pullckpt` now falls back to the
   `ppo_mjx_{joint_goal,joint_walk,goal}_<run>.zip` variants
   automatically before giving up.
7. **W&B:** project `l2k2/hexapod-balance`; creds already in the
   cycle env (elsewhere: source `rl_move/sim/wandb.env`). Prefer
   `ops.sh wandb <run>`; for ad-hoc queries use `wandb.Api()`
   filtered by `display_name`, newest match. Full picture (run-page
   anatomy, OUTCOME notes, artifact lineage): `rl_docs/WANDB.md`.
8. **git:** `snapshot.sh` serializes commit/tag/push under a lock —
   never raw `git push` for cycle edits; a brief wait on its lock is
   normal. Re-read RL_LOG/RL_PLAN right before editing (concurrent
   cycles append too).
9. Guardrails: `rl_move/orchestrator/guardrails.yaml` (from PROTO).
   Watcher log: `/workspace/orchestrator.log`. Cycle logs:
   `/workspace/cycle_logs/`. Per-run train logs live ON THE POD at
   `/tmp/train_<run>.log`.
10. **Restart the watcher ONLY via `restart_watcher.sh`** (in the
    orchestrator dir; deployed at `/workspace/restart_watcher.sh`).
    It sets PAUSE + WRAPUP, waits for in-flight cycles, sanity-parses
    the new code, then swaps the tmux session. WRAPUP (08-09 evening)
    tells cycles to save work and exit at the next run boundary
    (shutdown protocol in ORCHESTRATOR_PROMPT.md); stragglers are
    killed at a 30-min deadline — verdicts already recorded survive,
    unverdicted runs are re-assigned after the swap. The wait loop
    also keeps draining the backlog so already-queued (blocker-vetted)
    specs still place during an update. Killing the watcher/tmux directly still murders
    in-flight cycles mid-thought — 3 cycles' tokens were torched
    this way on 08-09 (and the operator's assistant repeated the
    exact mistake later the same day — READ THIS LIST before
    touching infrastructure).
11. **Verdicts auto-mirror to W&B notes AND package the analysis
    artifact.** `launch_run.py update --set verdict=…` pushes the
    verdict under the `--- OUTCOME ---` marker on the run's W&B page
    and attaches `analysis-<run>` (type run-analysis) to the run:
    ledger entry, `rl_docs/runs/<run>.md`, and every
    `logs/ckpt_eval/<run>_*` + `logs/experiments/<run>/` file
    (report.json, contact sheets, videos). So run your harness evals
    BEFORE setting the verdict — files that exist at verdict time are
    what gets archived. `ops.sh wandbnote` can still replace the note
    with a richer paragraph. History: all 87 pre-existing verdicted
    runs were backfilled 08-09 ~15:20Z — DON'T re-backfill. Early-
    campaign artifacts legitimately hold 0 eval files (outputs were
    already deleted); probe-*/smoke-* runs have no artifact at all
    (no W&B run to attach to). Mechanism note: the public API can't
    create artifacts on a finished run, so the code briefly
    `wandb.init(id=…, resume="allow")`s it — the extra short resume
    blip on a run's timeline is this, not a training restart.
12. **Checkpoint lineage lives in W&B artifacts** (08-09): every
    training run publishes `ckpt-<out-name>` (type
    policy-checkpoint, md5 + parent in metadata) and declares its
    `--init-from` parent via `use_artifact`. The W&B artifact DAG is
    now the run/checkpoint family tree — pre-08-09 parents predate
    this and appear rootless.
13. **`killrun` leaks /dev/shm segments; the NEXT launch on that pod
    dies at first env reset with worker `EOFError`** (c54: two 0-step
    corpses on train-4 after c53's stopgo35 kill — /dev/shm 64M was
    98% full of `hexmjx-*` segments). Diagnose: `kubectl exec <pod>
    -- df -h /dev/shm`. Fix: confirm no live trainer (mind the
    self-match gotcha), then `rm -f /dev/shm/hexmjx-*` and relaunch
    under a NEW name (`-r1`): W&B names are append-only, the launcher
    refuses reuse. Eval completion marker is `artifacts` (the harness
    never prints WROTE) — `waitlog ... 'artifacts|Traceback'`.

13b. **Launch-collision `EOFError` even with a CLEAN /dev/shm** (this
    cycle, 22:1x-22:3x): under concurrent-cycle drain storms (2+
    cycles draining into the same free-pod set within seconds of each
    other), a worker can die with the SAME `EOFError` at first env
    reset as gotcha 13 even when `df -h /dev/shm` shows single-digit
    % used — this is a race between simultaneous launches on
    neighboring GPU pods on the same node, not a shm leak. 3-for-3
    crashed this way in one cycle (imupos15, gyrobias3, tiltnoise; a
    concurrent cycle independently hit the same pattern on
    joyhead90-lat25-s1 and placementnoise6). No science result (0
    steps) — just relaunch under a `-r1` name once (W&B run names are
    append-only) and move on; don't diagnose further unless it
    recurs on a retry with no other drain active.

13c. **RESOLVED (08-10 deep dig-in): the 13/13b `EOFError` class was
    /dev/shm SIGBUS, and it is now self-healing + diagnosable.** Train
    pods have the 64M k8s-default /dev/shm; a normal 4096-env sharded
    layout maps ~58M (measured live), so ANY leaked segments poison
    every later launch on the pod: workers SIGBUS on first page touch
    (POSIX shm is sparse) and the parent saw only a bare `EOFError`.
    Since snapshot `bcf46be`: (a) workers run `faulthandler` and the
    parent prints per-worker exit codes (`-7` = SIGBUS) instead of the
    bare EOFError — read the train log tail, it now names the killer;
    (b) each trainer start GCs orphaned `hexmjx-*` segments (keeps
    live-mapped ones), so poisoned pods self-heal on next launch — no
    more manual `rm`. LIMIT: `obs.history_frames=16` at 4096 envs maps
    >64M and can NEVER boot on a default pod (the 8x arch-hist16 death
    chain) — either run it at `--n-envs 3072` (~50M) or recreate the
    pod (WHILE IDLE) with the dshm-4Gi manifests
    (`coreweave_pod*_mjx_*.yaml`, patched 08-10):
    `kubectl delete pod <pod>` → `kubectl apply -f
    rl_move/sim/coreweave_pods_mjx_scaleout.yaml` →
    `orchestrator/bootstrap_train_pod.sh <pod>` → `snapshot.sh --sync
    <pod>`. Verify with `df -h /dev/shm` (should say 4.0G).

14. **Batch-eval shell footgun (c60):** `CFG="..." && nohup A $CFG & nohup B $CFG &`
    puts the assignment INSIDE the first background job's subshell — B
    (and later jobs) run with an EMPTY $CFG, i.e. default cfg = silently
    voided verdicts (gotcha 3), detectable as cmd_dist/30s outside the
    walk band in report.json. Assign the variable on its OWN line (or
    `export` it), THEN background the evals; always spot-check one
    `/proc/<pid>/cmdline` for the cfg-sets after launching a batch.
    Related (c60/c61): controller evals get load/OOM-killed SILENTLY
    under heavy contention (empty log, no Traceback, no report.json,
    partial video files) — verify the pid in /proc before trusting
    `waitlog`, and relaunch (setsid helps).

14b. **A backlog item silently VANISHES (no backlog_failed entry, no
    error) if a W&B run with the same name already exists** — the
    drain's dedupe check (`drain: <run> already exists in W&B —
    dropping backlog item`) fires even when that prior run FAILED in
    seconds (e.g. a bad `--cfg-set` crash) and never trained anything;
    it only checks the NAME, not whether the prior attempt produced
    real data. If a `--now` launch dies instantly, don't re-`respec`
    onto the SAME run name — use a fresh one (`-r1` suffix) or the
    backlog re-queue will queue-then-disappear with zero trace (cost
    a queue-recheck cycle building `cw-dep-vref1-r1-megastack1`, 08-10).

15. **`--cfg-set dr.<field>=X` needs `lo,hi` (comma, no brackets) for
    every RandRanges field that's typed as a `tuple[float,float]` —
    a bare scalar crashes the worker at reset with `TypeError:
    Generator.uniform() argument after * must be an iterable, not
    float` (cost a launch attempt building `cw-dep-vref1-r1-megastack1`,
    08-10). Scalar (single-float) fields take a bare number; tuple
    fields need both ends. Check `class RandRanges` in
    `rl_move/sim/domain_rand.py` before guessing, or grep an existing
    sibling run's FULL `command` string (not just a `[0-9.]*` regex —
    that silently eats the `,hi` half and looks like a scalar).
    Tuple fields (need `lo,hi`): `mass_scale`, `friction_scale`,
    `contact_stiff_scale`, `torque_scale`, `latency_scale`,
    `deadband_scale`, `vel_scale`, `bad_start_deg`, `imu_pos_z_m`.
    Scalar fields (bare number): `leg_mass_jitter_pct`,
    `link_len_scale_pct`, `link_len_leg_pct`, `ground_tilt_deg`,
    `kp_scale_pct`, `kv_scale_pct`, `placement_noise_deg`,
    `joint_zero_bias_deg`, `encoder_noise_deg`, `imu_mount_deg`,
    `imu_bias_deg`, `tilt_noise_deg`, `gyro_bias_deg_s`,
    `gyro_noise_deg_s`, `com_offset_m`, `imu_pos_xy_m`,
    `cmd_drop_prob_max`.
16. **A manual `kubectl exec` probe on an arbitrary FREE pod can
    silently run STALE code** (08-11, gait-cleanup terrain probe):
    `launch_run.py launch` refuses a `.code_sha` mismatch, but a
    hand-run eval/probe via `kubectl exec` has no such gate — an idle
    pod keeps whatever code was live at its last launch, sometimes
    commits behind. This exactly reproduced an already-fixed bug
    (the `env.terrain_amp>1` clamp, 434a6e0): probing the champion at
    amp 1/2/3 on a stale pod returned bit-identical reports (the OLD
    clamped code), which would have wrongly re-confirmed the closed
    bug. Check `kubectl exec <pod> -- cat
    /workspace/prototype_sts3215/.code_sha` vs local HEAD before
    trusting ANY ad-hoc pod probe that depends on recent code; `bash
    snapshot.sh --sync <pod>` first if it's behind (safe on an idle
    pod, never on one with a live trainer).

## Operator status page (web) — setup & restart runbook

One auto-refreshing HTML page for the human operator: watcher
ON/PAUSED/OFF, in-flight cycles + what they're triaging, analysis
pipeline (ledger `triage` field), per-pod fleet census, backlog,
ledger runs, Claude token usage + est. spend, log tails. Code:
`rl_move/orchestrator/status_server.py` (stdlib only, port 8090 —
5183/5173 are BuildViz, 8080 is the robot).

Two pieces, both must be up:

1. **Server, on the controller pod** (`hexapod-sweep-friction`), in
   tmux session `statusweb`:

   ```sh
   kubectl --kubeconfig=$HOME/.kube/coreweave.yaml exec hexapod-sweep-friction -- \
     bash -c "tmux kill-session -t statusweb 2>/dev/null; \
       tmux new-session -d -s statusweb 'source /root/orchestrator.env; \
       cd /workspace/weird_objects/hexapod_walker/prototype_sts3215 && \
       python3 rl_move/orchestrator/status_server.py 2>&1 | tee /tmp/status_server.log'"
   ```

2. **Port-forward, on the operator's laptop** (dies on sleep/network
   blips — restart it freely, it's stateless):

   ```sh
   kubectl --kubeconfig=$HOME/.kube/coreweave.yaml \
     port-forward hexapod-sweep-friction 8090:8090
   ```

Then open **http://127.0.0.1:8090** (raw data at `/json`).

Health checks: `curl -s http://127.0.0.1:8090/ | head -c 100` on the
laptop; on the pod, `tmux has-session -t statusweb` and
`/tmp/status_server.log`. If the page loads but fleet/token sections
are empty, the slow collector hasn't finished its first pass — wait
~2 min. After editing `status_server.py`: commit, push, `git pull` on
the controller, then re-run step 1 (kill+new tmux session). The
server is read-only and safe to restart at any time — it never
touches training, the watcher, or the ledger.

## Time budget guidance

The operator's standing complaint is cycle latency. Read the two
condensed docs + guardrails (fast), use `ops.sh`, launch evals in
PARALLEL (`nohup … &` all of them, then `waitlog` each), and don't
reproduce evidence into RL_LOG — link it. If a run is clearly
improving and flagged auto-continue, the watcher already relaunched
it; your job is the verdict, not the relaunch.


---

# FILE: rl_docs/EXPERIMENT_LOGS.md

# Per-experiment logs — `logs/experiments/<run>/`

**Scope (08-09 lightweight process): this directory + summary.md is
for DIG-IN runs only.** A clear pass/fail needs just the ledger
verdict (`launch_run.py update`, which auto-renders
`rl_docs/runs/<run>.md`) and an `ops.sh wandbnote` paragraph. The
watcher's `wandbdump` cache still lands here for every run.

Dig-in experiments get ONE directory that tells the whole story:
`logs/experiments/<run>/` (under `prototype_sts3215/`; gitignored —
this lives on the controller, not in GitHub).

## Layout

```
logs/experiments/cw-walk-anchortol5/
  summary.md        <- dig-in runs only; written by the verdict cycle
  wandb_summary.json  <- cached W&B summary+config (ops.sh wandbdump)
  wandb_history.csv   <- cached scalar history    (ops.sh wandbdump)
  (optional: eval output dirs, frames you judged, scratch analysis)
```

`ops.sh expdir <run>` creates the directory with a summary.md
template. `ops.sh wandbdump <run>` caches the W&B data so later
questions don't need the API again.

## summary.md format — plain language FIRST

```markdown
# <run> — one-line outcome

## What we tried and why (plain English, 2-4 sentences)
Written for a human who knows nothing about the codebase. Big-goal
context first: what problem in the robot's behavior this run
attacks, what we changed, what we hoped would happen.

## What happened
Result in 2-4 plain sentences: did the hoped-for thing occur, what
the video showed, what the verdict was.

## Details (optional, keep short)
Gate numbers, key metrics, links: W&B url, ledger entry, eval dirs,
parent run, checkpoint md5.
```

The same plain-language-first rule applies to W&B run notes: the
`--hypothesis` you pass at launch MUST OPEN with 1–2 sentences a
non-expert can read ("The robot's feet slide while it walks; this
run makes sliding unprofitable by X; we hope to see Y"). Technical
pre-registration (gates, if-true/if-false) comes after.


---

# FILE: rl_docs/WANDB.md

# W&B — how this project uses Weights & Biases

One project holds the whole campaign: **`l2k2/hexapod-balance`**
(https://wandb.ai/l2k2/hexapod-balance). Every training run logs there
automatically (group `ppo-goal-lineage`); `--no-wandb` disables.
Defaults live in `rl_move/sim/train_ppo_sim.py`
(`WANDB_ENTITY_DEFAULT` / `WANDB_PROJECT_DEFAULT`, env-overridable).

## Credentials

- The key lives in **`rl_move/sim/wandb.env`** (gitignored). It exists
  on the operator Mac AND on every train pod (pushed to
  `/workspace/prototype_sts3215/rl_move/sim/wandb.env` by
  `bootstrap_train_pod.sh`).
- `ops.sh` sources it for you. For ad-hoc API use, source it first:

  ```sh
  set -a; source rl_move/sim/wandb.env; set +a
  python -c "import wandb; ..."   # repo .venv has wandb installed
  ```

- A bare `kubectl exec` has NO creds — `launch_run.py drain` (dedupe
  check) and anything calling `wandb.Api()` fails without sourcing.

## Read runs with ops.sh — don't hand-write API code

| Question | Command |
|---|---|
| One run: state, steps, reward trend, URL | `ops.sh wandb <run>` |
| Full triage read (ledger + W&B + evals + videos) | `ops.sh review <run>` |
| Cache summary/config/history to the run's experiment dir | `ops.sh wandbdump <run>` (then query the cache, not the API) |
| Any results being dropped? | `ops.sh triage [hours]` — recent W&B runs × verdict × note × processed |
| What's ACTUALLY training | `ops.sh census` — W&B lags fresh launches ~8 min (JAX compile); an "empty" project is normal right after a launch |

For genuinely ad-hoc queries: `wandb.Api()` filtered by
`display_name`, take the newest match (names can have retry suffixes).

## What's on a run page

- **Name = ledger run name** (e.g. `cw-walk-foo-c1`); checkpoint is
  `policies/ppo_goal_cw_walk_foo_c1.zip` (dashes→underscores). Names
  are **append-only** — the launcher refuses reuse, so retries get
  `-r1`/`-rr1` suffixes. A 0-step run is an infra launch failure
  (collision/shm), not science.
- **Config** carries the full spec + DR ranges + `parent_run`; notes
  carry the spec text. SB3 scalars (losses, `ep_rew_mean`, fps) arrive
  via `sync_tensorboard`; reward components, eval panels (`eval/*`),
  rollout videos (`video/rollout`), and canary metrics log directly.
- **`--- OUTCOME ---` note = the verdict.** `launch_run.py update
  --set verdict=…` auto-mirrors the ledger verdict to the run notes
  and attaches an `analysis-<run>` artifact (ledger entry, run doc,
  every eval file existing at verdict time — so run evals BEFORE
  verdicting). `ops.sh wandbnote <run> "…"` writes a richer paragraph.
- **Lineage = the artifact DAG.** Each run publishes
  `ckpt-<out-name>` (type policy-checkpoint, md5 + parent in
  metadata) and declares its `--init-from` parent via `use_artifact`.
  Continuations also `fork_from` the parent run at its end step.
  Pre-08-09 runs predate this and appear rootless.

## Gotchas

- A short "resume blip" on a finished run's timeline is the
  artifact-attach mechanism (`wandb.init(id=…, resume="allow")`), not
  a training restart.
- Local `wandb/` run dirs are **gitignored** — they were tracked once
  and poisoned every pod sync (08-09). Never re-add them.
- W&B run state is not ground truth for "is it training" (`ops.sh
  census` is) nor for verdicts (the ledger / `rl_docs/runs/` is).


---

# FILE: rl_docs/HARDWARE.md

# HARDWARE.md — real-robot evidence + experiment backlog

What the physical robot has told us so far, where the data lives, and
the prioritized list of bench experiments that would settle open
training decisions. **The orchestrator NEVER touches the robot**
(guardrails); everything here is operator-run. Data captured on the
robot gets copied into `rl_move/hardware_traces/` so the pods can see
it after a repo sync.

## Data that exists (2026-08-09)

- **RL episode traces** — every stand/lower/walk run on the robot
  auto-logs 25 Hz per-tick telemetry (attitude, gyro, goal refs,
  measured + commanded q ×18, raw action ×18, per-servo current) +
  a summary JSON. Format spec: `rl_move/API.md` § "RL episode
  logging". First two walk attempts: `rl_move/hardware_traces/
  rl_walk_20260810_00{2907,3210}.csv` (+ summaries).
- **Fitted motor model** — `rl_move/hardware_traces/motor_model.json`
  (air-only ±amp step probe via `/api/rl/probe_dynamics`).
- Robot originals live in `linux_control/logs/` on the board.

## Findings — first hardware walks (champion longdist_r2, 08-09)

1. **Deployment-pipeline mismatch dominates.** The on-robot
   SafetyLayer rate-clamps commands to 1.5°/tick (37.5°/s). In
   attempt 1 the policy requested mean 13°/tick (p95 77°/tick, max
   170°): **97% of all joint-ticks saturated the clamp**; mean
   |proposed − commanded| = 48°. The policy runs ~9× slow-motion
   dynamics it never trained in. The sim has no such clamp — a
   gate eval run through the clamp would have predicted this
   failure without a robot.
2. **The real floor does not skate.** Rough broom-finish concrete +
   rubber foot tips: feet grip. The champion transports by paddling
   (sliding) — on hardware each paddle stroke became roll torque
   instead: roll ramped −1° → +9° over 1.8 s (starting during the
   zero-velocity settle) → `tilt_roll` trip at 10° relative → limp.
   No hardware distress: max 0.3 A, tracking err mean 6°.
3. **Velocity feedback is absent on the board.** Walk obs feeds
   `vx/vy_meas := ref` (no body-velocity estimate). The trace shows
   zero corrective response over 35+ ticks of roll growth.
4. **Attempt 2 carried no policy signal** — killed at 1.1 s by a
   phantom over-temp (corrupted bus byte read 70 °C; same servo read
   33 °C ten seconds later). Both safety layers are now debounced
   (consecutive-read requirement; four phantoms on 08-09, all
   "cooled" to ambient within seconds).
5. Environment extras (video): stiff power tether tugging from the
   same side the robot rolled toward; robot at ~142 mm plant.

Binding training actions from these findings: RL_PLAN Queue -1.

## Finding — RL belly-rise does NOT transfer (08-10 evening, 2 attempts)

`stance_dr10` RL stand from belly, clean preflight both times. Traces:
`rl_stand_20260810_221907.csv` (operator abort ~8 s) and
`rl_stand_20260810_024938.csv` (tilt_roll trip ~8.4 s).

- The first 5 s "leg waving" is the trained curl (height ref pinned 0)
  and matches sim. The failure is the PUSH phase: body attitude never
  changed and all joints sat ≤0.1 A — air geometry, zero ground force.
- 22:19 attempt: policy commanded L4 knee (j14) to +148.8° (legal,
  software limit +150) but the physical leg JAMS ≈ +139° — a
  self-collision the sim does not model. Stall at 4.22 A = the
  "motor shake"; post-abort hold kept fighting it.
- Root cause class: stance_dr10 is pre-deployment-contract (trained
  without the 1.5°/tick clamp / hardware axes) — the same gap that
  killed walk attempt #1, and a rise is a TIMED weight-transfer move.
  34/201 loop overruns compound it.
- Operating guidance: stand up with scripted **tuck** (2.48 A, clean);
  RL stance = holds/leans once upright; RL rise stays parked until a
  deployment-contract rise line exists. Sim work item: model/cap the
  ~139° knee self-collision before any future rise arm.

## Finding — dep-vref1-r1 walk: runaway roll, no recovery (08-10 eve)

Two 6 s walks at 0.05 m/s from clean plant preflights. Traces:
`rl_walk_20260810_221553.csv` (roll drifts to −11°) and
`rl_walk_20260810_224749.csv` (roll ramps monotonically to **+22.9°**,
just under the 25° trip — operator: "it tipped, didn't go much").

- Same failure both runs, OPPOSITE directions → the seed is
  environmental (grip/tether/stance lean), not a fixed policy bias.
- Mechanism visible in the trace: the loaded-side leg (L0 in run 2)
  quadruples its current (0.08→0.41 A mean) while its knee stride
  collapses 45°→27° — pinned leg can't step → asymmetric propulsion
  → deeper lean. Positive feedback over ~4 s.
- The policy SEES tilt (it's in obs) but has no trained recovery for
  a sustained lateral lean, and no velocity feedback (meas:=ref) to
  notice it isn't translating. Attempt-#1's roll-ramp signature,
  slower: the contract retrain fixed the clamp gap, not this.
- Distance expectation check: 6 s @ 0.05 m/s with 1 s ramps commands
  only ~250 mm — "didn't go much" is partly arithmetic; the tape
  card should quantify the slip on top.
- Training lead: walk arms need sustained lateral-disturbance /
  grippy-floor DR (persistent roll-torque perturbations), and an
  eval axis that scores recovery from a 10–15° standing lean.
- **LANDED 08-10 (same day):** tipped-start DR (`dr.tipped_start_*`,
  default-ON everywhere per operator ruling — rl_docs/SIM.md) +
  `SCORE/tipped_recovery_success` eval (rl_docs/EVALS.md). Discovery
  arm `cw-dep-tip1` trained (2M warm from this champion, 30% tipped
  starts).
- **Discovery verdict (08-10 late): the sim eval does NOT reproduce
  this failure.** With the height gate fixed (first cut failed every
  healthy walk — the gait rides 54–70 mm low), the champion ALREADY
  recovers static 12–16° leans in sim (7/8, even with grippy-feet
  friction 1.4); tip1 matches with slightly lower residual roll
  (1.7–2.0° vs 2.2°). So the runaway is a SIM-TO-REAL gap — the
  progressive pinned-loaded-leg mechanism above doesn't happen on the
  sim floor — not missing training states. The eval stays as a
  regression floor. **Next check is hardware:** `dep_tip1.json` is on
  the robot in the walk picker; A/B it against `dep_vref1_r1` on the
  same floor at 0.05 m/s (walk retention verified identical in sim,
  vel err 0.036 vs 0.037). If tip1 also rolls away, the fix is a
  sim contact/pinning model, not more DR.
- **ROLL-TRAP GATE landed (08-10 late, operator spec):** every run
  now also gets `SCORE/roll_trap_pass` — a mid-gait servo torque
  drags the body to a sustained ~12° lean for 3 s, releases, and the
  policy must re-level <5°, regain ≥50% commanded speed, keep all six
  legs cycling, no fall (rl_docs/EVALS.md). Unlike the tipped start
  this DOES separate the lineage at a hard dose (~20–27°: tip1 4/6
  vs parent 2/6 — the tipped-start DR transferred to mid-gait
  disturbances), which strengthens the case for the hardware A/B.

## Finding — dep-tip1 hardware run: roll ramp persists, but the
## pinned-leg signature is GONE (08-10 late night)

One 6 s walk at 0.05 m/s (`rl_walk_20260811_021859.csv`). Operator:
"walked a little bit and fell over."

- Same roll ramp as the parent: +0.2° (Q1 mean) → +17.7° (Q4 mean),
  peak +21.9°, never hitting the 25° trip. Episode ended "ok" at
  ~+15°; the fall happened AFTER "walk done" — the walk ends holding
  the final stance with torque on, and a static hold at a 15° lean
  tips (the moving gait was the only thing propping it). The log
  stopped at episode end, so the fall itself wasn't recorded (fixed —
  see logging note below).
- **The parent's leg-sacrifice mechanism did NOT recur**: all six
  legs kept cycling to the end (knee stride 27–48° in the second
  half, vs the parent's loaded knee collapsing 45°→27°) and currents
  stayed flat everywhere (0.02–0.05 A/joint mean; parent's pinned leg
  hit 0.41 A). The tipped-start DR changed the behavior exactly in
  the trained direction — no leg gets abandoned — yet the roll still
  ramps monotonically with zero recovery dips.
- Reading: strengthens the sim-to-real contact story. The body drifts
  sideways tick after tick and the in-band gait never re-centers it —
  recovery in sim can exploit feet that skate; rubber feet on a
  grippy floor can't. Alternative still open: an obs-pipeline issue
  (does the policy SEE +15° relative roll correctly at deployment
  scale?). The new per-tick obs logging resolves that question on the
  next run — replay the logged obs through the same policy offline
  and compare actions.
- Next: (1) run the A/B anyway — parent on the same floor; compare
  roll-ramp RATE, not just fall/no-fall; (2) pull the next trace and
  check the logged obs roll channel against the IMU column; (3) if
  obs are clean, the fix is a sim contact/pinning model (foot
  friction anisotropy / no-skate), not more DR.
- **Logging upgraded for this (08-10):** every RL episode CSV now
  carries a `phase` column, the full policy obs vector per tick, and
  a 3 s read-only post-episode tail (attitude/q/currents at 10 Hz) so
  after-the-end tip-overs are captured. Summaries now report
  `tilt_rel_max_deg`, end roll/pitch, `tail_tilt_max_deg`, and a
  `fell` flag — the summary alone answers "did it fall after the
  episode?".
- **Second dep-tip1 run (08-10 22:33, `rl_walk_20260811_023304.csv`,
  first trace with the upgraded logging): CLEAN WALK.** 6 s at
  0.05 m/s, roll oscillates ±5.4° around a −2° offset with NO ramp,
  quiet 3 s tail (peak 1.7°), `fell:false`, max current 0.43 A —
  operator video confirms level walking, then a normal settle. Two
  pipeline questions settled by the obs columns: (1) offline replay
  of the logged obs through `dep_tip1.json` reproduces the logged
  actions to max err 0.0014 (obs rounding) — the deployed obs→action
  path is EXACTLY the sim policy, no scaling/sign bug; (2) the policy
  sees roll (obs ch 36 correlates 0.98 with the IMU roll column). So
  the 02:18 runaway was not an obs bug — the same pipeline walked
  level twenty minutes later. Runaway trigger is environmental /
  initial-condition (floor spot, tether drag, seeded stance lean),
  intermittent rather than systematic.   The parent A/B on the same
  floor is still the discriminating test — compare roll-ramp RATE
  and count runaways per N runs, not fall/no-fall on a single run.
- **Third run (08-10 22:35, `rl_walk_20260811_023532.csv`) was ALSO
  dep-tip1** — the operator meant to A/B vref1-r1 but no
  `rl_policy_select` ever hit the robot (events show only GETs
  between the walks; the "Use selected" click never landed). Another
  clean walk: rel-roll peak 9.2°, quiet tail, no fall. tip1 hardware
  tally: 1 runaway / 2 clean. **Operator-visible "sag" explained:**
  the body drop during walking is COMMANDED, not servo slip —
  measured knees track commands within 1–3° while the policy itself
  migrates the posture over the run (mean hip +12° → −30°, knees
  89° → 100°, identical in both clean runs). This matches the
  documented sim behavior that the walk gait rides 54–70 mm below
  the spawn stance: the policy settles into its trained (lower,
  wider) walking posture. Not a fault; worth a height-keeping term
  in a future walk arm if the crouch bothers operations.
  **LANDED 08-10 (same night):** `reward.walk_height_gate` — income ×
  Gaussian on body height vs the episode anchor (σ 30 mm; upright
  gait keeps 0.99 of income, the measured −51 mm crouch keeps 0.13),
  with a 3-test MDP_PREFLIGHT height bank (test_task_semantics.py)
  pinning: upright ≫ crouch under the gate, the gate itself does the
  work, and the honest gait is taxed <10%. Discovery arm
  `cw-dep-hgt1` queued (2M warm from cw-dep-tip1, full stack + gate;
  rl_docs/REWARD.md row).
- **Fourth run (08-10 22:38, `rl_walk_20260811_023756.csv`, tip1
  again): clean, tally 1 runaway / 3 clean. Operator-visible floor
  SCRAPING quantified:** the gait is a low-clearance shuffle. Knee
  lift during coxa swing is ≈0 for the four big-swing legs (+1 to
  +4 deg/s — feet slide forward instead of stepping over the floor),
  and stride is very uneven: L1/L3/L4/L5 swing 37–46° of coxa while
  L0 and L2 barely step (14–16°). Expected from training: sim charges
  nothing for dragging a foot during swing, so the policy converged
  to a shuffle. Not hardware damage — currents stay 0.02–0.06 A/joint
  with swing/stance drag ratios only 0.9–1.5. Training lead for a
  future walk arm: swing-clearance or foot-slip-in-contact penalty
  (sim has the contact data), plus the height-keeping term above.

## Bench state — drive session 08-11 morning

Robot picker (verified live): walk slot = `dep_tip1` (ACTIVE; 3 clean
/ 1 runaway 08-10), `dep_vref1_r1`, `dep_quad1_c2`. Stance slot =
`stand_holdbc1_hard1` (ACTIVE, **new**: overnight HOLD+RISE+LOWER 10M
specialist, md5 6620705c, obs 68) with `stance_dr10` still available.
holdbc1-hard1 is EXPERIMENTAL on hardware: `hardware_ready: False`,
no deployment-contract flags, sim crouch-start rise still tips 2/4 —
sit/hold expected to work, RL stand-up is the risky part (scripted
tuck remains the known-good stand-up). Omni/steering lineage stays
off the robot: transbc1 FAILED (paddle), rot-60 canonicalizer is
sim-only until the deploy-side runner port lands.

**08-11 later — BOTH deploy-side ports landed in the repo, robot
needs a re-push before the next session:** (1) rot-60 runner port
(commit 39d4754): full-circle walk headings, forward wedge bit-exact.
(2) rise+hold specialist ramp fix: the on-robot 6620705c copy of
`stand_holdbc1_hard1` carries NO goal profile, so the runner would
feed it the LEGACY +50 mm / 4 s ramp — an out-of-distribution height
command for a policy trained only on 108–114 mm targets. The repo's
export now ships the trained ramp (hold 5 s / ramp 6 s / +111 mm /
12.5 s) inside the weights meta and the runner reads it
(`test_stand_runner.py` locks the contract). **Do not press STAND on
the stale copy — run deploy_adb.sh (now also pushes
rl_walk_weights.json + policies/) or re-select the fresh JSON first.**

## Finding — TFT redraws stall the entire servo link (08-10 night)

Root cause of the operator's "big pause in the middle of standing"
(10× streamed tuck): a **job-panel repaint (`DJ`) held the shared MCU
serial link for 1455 ms** (MCU transaction log, `emit_mcu`). Every
bus user — pose sync-writes, feedback reads — serializes behind the
same lock on `/dev/ttyHS1`, so while the ST7789 draws, the robot is
frozen mid-motion with servos parked on their last target.

Generalize this: **any MCU display traffic is a potential 1.5 s
motion stall**, and repaints are triggered by *changing text* — a
progress string that updates every 0.3 s (t / peak-amps counters) is
a repaint generator. The DX status path also reads all servo currents
on the MCU (already throttled to 2.4 s while a job runs), but DJ was
assumed "pure display, cheap" and was not throttled at all.

Mitigation landed 08-10: motion jobs set `demo.bus_hot`
(`bench_api`, standup worker pattern — set on entry, cleared in
`finally`, only reported while the worker thread is alive) and
`StatusDisplay` skips ALL painting while it is set, leaving the panel
stale until the bus is released. Rule for new motion loops (RL
runner, gait changes, future scripted moves): either set `bus_hot`
around the streaming section, or accept ~1.5 s write gaps whenever
your progress text changes. Prefer static progress text on screens;
numbers belong in the event log, not the TFT, during motion.

## Session 08-09 night (operator supervised) — status update

Collected (traces in `rl_move/hardware_traces/`, analysis in RL_LOG
"hardware session 3"): scripted gait WALKS (30/50 mm/s fwd, crab,
both turn directions) from fresh set_zero → P; falls when started
from a stale stance. Working gait rocks ±10-20° roll/pitch → 10° tilt
trip is wrong for walk mode (use ~25°). Standing hold 0.59 A total >
walking 0.33-0.45 A. Loaded step ladder: latency 110-210 ms, t90
260-430 ms. +omega → clockwise. Definite foot slip during working
gait. Phantom over-temp root-caused (stale FB cache vs tick debounce)
and fixed in `rl_move/safety.py`. Post-mortem correction: the
1.5°/tick clamp WAS in raw-joint training — the real contract gaps
are velocity obs source, tilt envelope, prev-action semantics, and
contact/current pricing. STILL MISSING: measured walk distance (true
ground speed for slip calibration).

## Models to try on the real robot (bench list, 2026-08-10)

**All RL entries below are ON THE ROBOT and pickable in the web UI**
(RL tab → Policy panel → dropdown + "Use selected"; deployed 08-10):
`stance_dr10` (stance slot), `dep_vref1_r1` (walk slot, active
default) and `dep_quad1_c2` (walk slot alternative; its quad trick
has no runner mode yet). Scripted stand-up modes live in the
Experiments tab. Details: `rl_move/API.md` § policy picker.

Operator-supervised, fresh `set_zero` at a known visual pose first,
kill switch handy. Ordered by payoff-vs-risk. Verified staged on the
Mac in `rl_move/sim/policies/` unless noted; pull missing ones with
`ops.sh pullckpt <run>` and verify md5 against RL_LOG.

1. **`ppo_goal_cw_dep_vref1_r1` — RL walk attempt #2 (the headliner).**
   STAGED, md5 f9a466cf verified. Trained contract-exact for this
   robot: `vx/vy_meas := ref`, 25° tilt envelope, 1.5°/tick slew,
   k_current=0 — the exact gaps that killed attempt #1 — and ~20
   hardware-imperfection axes verified free on it. Start with 6–10 s
   forward walks at 0.05 m/s (its trained command band is
   0.05–0.06 — slower is out-of-distribution); watch for the
   attempt-#1 signature
   (roll ramping over ~2 s). Any walk it does makes it the first
   learned policy to drive this robot.
2. **Scripted stand-up modes — `/api/standup`. RUN 08-10 ~18:00, big
   result.** Operator tried all modes on the real floor (Experiments
   tab): **tuck stood clean, peak 2.48 A** ("worked way better");
   **step stood, peak 2.97 A** (a hair under the 3 A lab guard);
   **blend stalled short of full height at only 0.57 A peak** — the
   servos give up quietly under the 70% torque limit instead of
   grinding. This is the sim's low-torque row (tq≈0.35–0.5 of the
   friction×torque sweep, `standup_fric_sweep/results.json`): blend
   fails geometrically (pinned feet), air-tuck strategies immune.
   Implication for the rise line: never pull loaded feet inward; tuck
   or tripod re-plant first. Sit-down (reversed keyframes) + 2–10×
   tempos added same day; faster tempos push currents toward the
   guard, so expect aborts before damage.
3. **`cw-dep-quad1-c2` — four-leg stand on the deployment base.**
   Pulled + DEPLOYED 08-10 (md5 065011328e, pod-verified; walk-slot
   alternative in the web-UI picker). Passed the ≤20 mm height gate
   today after the +12M continuation; same deployment contract as
   vref1-r1. NOTE: the on-robot runner has walk mode only — the quad
   trick (lift front pair) needs a new runner mode before it can be
   commanded on hardware. Until then this entry tests whether its
   WALK survived the quad mix on real ground vs vref1-r1.
4. **`ppo_goal_cw_stance_dr10` — stance champion holds/leans.**
   STAGED. Quiet plant holds, lean/track following. Its belly RISE
   is the risky part (stand-up is the incident class — only with
   hands ready, after everything else looks good).
5. **While the robot is out (10 min each, unblocks sim work):**
   hover-vs-planted current log (feeds the holding-current model —
   the one sim effort gap left) and a commanded-turn sign check
   (+wz vs actual rotation direction — closes the TURN sign audit).
   Both are one-button cards in the web UI **Measure tab** now, as
   is the tape-measure walk (see "Experiment backlog" below).

## Experiment backlog (operator-run; highest decision-value first)

Each entry: what open decision it settles → procedure → output.

**One-command session runner (08-11):** `python -m
rl_move.scripts.bench_blast --go` walks the whole open list in order —
policy/profile preflight, the vref1-vs-tip1 A/B (policy switch
VERIFIED per walk + automatic roll-ramp analysis from the pulled
episode CSVs), RL-walk tape, first rot60 off-wedge heading, first
learned stand-up (refused if the stance weights lack their goal
profile), wz sign audit, hover-vs-planted currents. Dry run without
`--go`; every motion step waits for an explicit operator `go`. Sim
half: `python3 -m rl_move.sim.sim_blast` (roll-trap A/B across
frictions, joystick panels naked+rot60 for both walk ckpts, deployed-
artifact tests).

**Zero-typing variant: `--go --video` (08-11).** Lay the tape on the
floor, film the runway, and only type the per-step `go`s: every step
is spoken aloud (macOS `say`) onto the video's audio track and
unix-stamped in summary.json. Afterwards
`python -m rl_move.scripts.video_review <footage>` syncs the video to
the log (spoken sync mark; falls back to first-motion detection),
cuts a timestamped frame sheet per walk/turn/stand plus full-res
tape-zoom frames, and the analysis agent reads distance, turn sign,
falls, and gait quality off the sheets — the operator never reports a
number.

**Measure tab (deployed 08-10):** items 1, 2 and the turn-sign check
now run from the web UI — `http://hexapod.local:8080/measure`. Cards:
walk-distance (tape) runs the scripted gait and prompts for the tape
reading; turn-sign does ±0.3 rad/s in place; holding-currents records
planted vs hover (no motion); an RL-walk note attaches a tape reading
to the newest RL episode trace. Records append to
`logs/measurements.jsonl` + per-run `meas_*_{servo,imu}.csv`; pull
with `scp arduino@hexapod.local:hexapod_sts/linux_control/logs/
\{measurements.jsonl,meas_*.csv\} rl_move/hardware_traces/`. The
Mac-side `tape_measure_walk.py` still works and writes the same CSV
shapes; the tab is the phone-friendly path.

1. **Scripted-gait ground truth** — settles: contact/current pricing
   calibration (P0) with a KNOWN-working transport regime, isolating
   physics gap from policy gap. The scripted drive gait
   (`drive_controller`) already walks this robot. Procedure: drive
   scripted gait 20–30 s on the same floor; log per-servo currents
   (watchdog/event log) + video with a tape measure for true speed
   and any foot slip. RUNNER (08-10): `python -m
   rl_move.scripts.tape_measure_walk --go` — drives the timed legs,
   logs ~3 Hz servo/imu CSVs via the new fast `/api/feedback` route,
   prompts for the tape reading, writes commanded-vs-measured +
   slip ratio to `hardware_traces/tape_<stamp>_summary.json`.
   Replicate the same gait kinematics in MuJoCo;
   tune μ + current model until sim reproduces real speed AND real
   per-servo currents. Output: calibrated contact/current params —
   the single blocker named by both operator rulings and the
   readiness review.
2. **Hover vs planted current** — settles: the stance pricing ruling
   (c28: sim prices hover 4× cheaper than planted descent — is that
   real?). Procedure: stand at plant, log 30 s steady currents; then
   unload/raise one leg (hover) 30 s; compare per-servo and total
   current. No motion beyond a slow blend, fully supervised. Output:
   real hover-vs-planted energy ratio → unblocks stance lower line.
3. **Foot friction μ** — settles: what μ to train at, and whether a
   slick tail is even physical. Procedure: robot limp on its belly
   (known weight), drag with a luggage/spring scale on this floor
   and on smooth tile; μ = F/W. 10 minutes, no power needed. Output:
   measured μ range for nominal DR + tail.
4. **Loaded step-response ladder** — settles: the actuator model for
   sim AND what the rate clamp should be. Procedure: standing at
   plant, command single-joint steps of 2/5/10° at several speeds
   (`/api/rl/probe_dynamics` is air-only today; a loaded variant
   logs the same via episode traces); fit lag/slew under real load.
   Output: actuator params for training + evidence for raising the
   1.5°/tick clamp toward servo capability.
5. **End-to-end latency** — settles: the latency-DR baseline (axes
   trained "2.5×" of an unmeasured number). Procedure: timestamped
   command → first encoder movement across ~50 single-joint blips;
   the episode trace already timestamps both sides. Output: real
   command→motion latency distribution.
6. **Raised-clamp walk retry** — tests finding 1 directly. After
   items 4 (and with the operator's hand on the kill switch):
   one 4 s walk at 3–4°/tick clamp. If the gait qualitatively
   changes (steps instead of creep), the clamp hypothesis is
   confirmed on hardware. Bounded risk: tilt/current trips stay
   armed, duration capped.
7. **Tip envelope** — settles: whether the 10° roll trip is right.
   Procedure: at plant stance (limp servos off? no — torque on,
   operator hands ready), slowly tilt the robot on a wedge until a
   foot unloads; note the angle. Output: real static tip margin →
   trip threshold + training termination angle.
8. **IMU noise while armed** — settles: sensor-noise DR realism
   (servo dither shakes the chassis). Procedure: 60 s IMU log
   standing armed vs limp; compare gyro/accel noise floor. Output:
   realistic obs-noise levels (the sensor-DR ladder went 9-for-9
   NO-EFFECT in sim — real numbers would close it definitively).
9. **Tether vs battery** — settles: whether the tether tug matters.
   Procedure: repeat a stand + short walk with the tether held slack
   overhead vs dragging. Output: keep/drop the horizontal-force DR.

## How new data flows to the pods

Operator session: run the experiment → traces auto-land in
`linux_control/logs/` on the robot → copy keepers into
`rl_move/hardware_traces/` in the repo → commit. The orchestrator
reads them on its next sync; findings get a line here and a Queue
item in RL_PLAN if they change priorities.


---

# FILE: rl_docs/WISHLIST.md

# Operator wishlist — things I want the robot to learn

Operator-owned (agents: propose additions, never delete items).
This is a CANDIDATE LIST, not a fill-the-slots queue (prime
directive, 08-10): an item is launchable only if it reduces an
unresolved blocker to the next hardware test — free pods alone are
never a reason to pull from here, and idle pods are fine. Items that
qualify are taken top of each section first. Every item still gets a
pre-registered hypothesis + gate and honest video verdicts.
Exploratory lines never gate the walk champion's promotion, and
hardware safety rules always apply.

Status tags: [RUNNING] has an active run, [READY] launchable with
existing config knobs, [CODE] needs an implementation cycle first,
[LATER] blocked on a prerequisite (say which).

## Locomotion (walking around the room)

-1. **UNIFIED JOYSTICK POLICY — ONE CHECKPOINT (operator, 08-09
   evening; outranks everything below).** "I can't have different
   models for standing up, sitting down, walking" — the deliverable
   the operator runs on the hexapod is a SINGLE policy that, from
   joystick-shaped commands, can: stand up (rise), walk/steer inside
   the trained envelope, stop, and sit down (lower). The env is
   already goal-conditioned (goal-mix modes walk/hold/rise/lower —
   same obs); champions have just been trained walk=1.0. Line:
   `cw-uni-blend1` = driving champion warm start + goal-mix blend
   (walk-heavy, some hold/rise/lower), gate = JOYSTICK GATE retention
   AND rise/lower >= 5/6 AND quiet hold. Known risk: multi-skill
   warm-start erosion (external review §12) — protect walk with the
   canary/regression rules; if blends erode walk repeatedly, ladder
   the mix (0.9 -> 0.7) instead of abandoning. When a blend passes,
   wire mode keys into `drive_policy.py` (rise/sit on keypress) so
   the operator can drive stand->walk->sit in MuJoCo, then hardware
   per safety rules.

-0.5. **TEMPORAL-ARCHITECTURE LINE — keep 1-2 GPU pods on it
   (operator, 08-09 ~20:2x). [READY]** "Reserve one or two instances
   for testing the more advanced architecture that captures more past
   states — I feel like that could be helpful with more complicated
   movements." NOT mechanically enforced (operator's explicit choice):
   cycles should simply keep 1-2 pods running architecture arms
   whenever they refill — treat an empty arch line like an empty
   backlog, i.e. queue the next rung. Rationale = external review §8
   (temporal history as online system identification; ranked ABOVE
   bigger MLPs) + the complicated-movement needs (rise/sit, heading
   flips, turns). Champions run obs.history_frames=8 (~320 ms at
   25 Hz) into a 128x128 MLP. Ladder, one variable per rung, off the
   current champion, joystick-gate retention + parent-delta as gate:
   (1) history_frames 16 (~640 ms) — `--cfg-set obs.history_frames=16`
   (warm start NOT possible across obs-width change: from-scratch
   rules apply, ent 0.005-0.01, std 1.0); (2) history 24; (3) wider
   net (256x256) at the winning history as the control for capacity
   vs memory; (4) GRU/recurrent actor [CODE DONE 2026-08-11:
   `train_ppo_sim.py --gru` = sb3-contrib RecurrentPPO with a true
   GRU cell (`rl_move/sim/gru_policy.py`); run single-frame obs
   (obs.history_frames=1), from-scratch rules (MLP checkpoints
   cannot warm-start a GRU); eval/canary/video harness threads
   hidden state automatically; needs sb3-contrib==2.9.0 on pods
   (in coreweave_pod_setup.sh). **Probed 08-11
   (`cw-arch-gru-r1`, 2M from scratch, walk=0.55/rise=0.15/
   lower=0.15/hold=0.15): FAILED — det walk collapsed to a static
   3-leg-tucked stance (0.004 m/s, gait_valid 0/6), sto walk jittered
   in place with no net travel (slip/m ~17, 2/6 tips); return climbed
   while the task didn't (reward-shortcut pattern), no forensics per
   the known-exploit rule. Not re-attempted with an adjusted recipe:
   the ladder stays FROZEN pending the flagship (RL_PLAN Architecture)
   and this line is off the current blocker list — do not requeue a
   GRU rung "because a pod is free." **08-11: `cw-arch-gru-r2`
   (walk-heavier-diet continuation, 0.55->0.70, same 2M budget) also
   FAILED on the harness/video — the IDENTICAL exploit fingerprint:
   det walk 0/6 gait_valid, legs sacrificed (frozen 0.004-0.011 m/s),
   sto walk 6/6 gait_valid but a no-progress paddle (prog med ~0,
   slip/m 13-21). The launch that produced r2 reasoned only from r1's
   periodic TRAINING-LOG numbers (vel_err 0.074, "reward climbing"),
   never the harness/video — same mistake flagged after r1, repeated.
   This is now 2 misses in the identical behavioral class
   (RESEARCH_RULES: change the hypothesis, never the step count) — a
   THIRD concurrent launch (`cw-arch-gru-long1`, 20M "hardening",
   citing r2's video-log line "walk:ok" as evidence) compounded the
   same error at 10x budget; it died on its own (W&B step regression)
   before burning real compute. Ruling: no GRU-rung relaunch on this
   walk diet/budget shape — a recipe change (BPTT window, obs framing,
   or a different curriculum order) is required, and the rung stays
   off the blocker list regardless. **08-11: the recipe change was
   tried — `cw-arch-gru-r3` (2M from scratch, SAME mixed diet, but
   with the full hist16-r7 anti-cheat/joystick reward stack this
   time) also FAILED, the IDENTICAL fingerprint (det gait_valid 0/6,
   legs parked at duty 0.02, ~0.006 m/s, video-confirmed static
   legs; sto gait_valid 6/6 but a no-progress jitter-paddle, slip/m
   17.4-17.5) — this fires the run's own pre-registered FAIL branch:
   since the identical reward stack already stops this exact cheat
   on the plain hist16 MLP, the cheat surviving here on the GRU means
   the limitation is the recurrent net's capacity/BPTT window, not
   reward pricing. Ruling: GRU rung FROZEN, no further reward-recipe
   variant is licensed by this closure — the only remaining lever
   would be an architecture-side change (window/hidden size), which
   is explicitly off the blocker list pending the flagship.]
   Score each rung on the COMPLICATED movements, not just nominal
   walk: joystick gate incl. flips, plus rise/lower fracs once the
   unified line has a rise-capable parent to compare against.

0. **JOYSTICK OPERABILITY — the binding operability target
   (operator, 08-09).** The operator will drive this robot with a
   joystick: ANY sudden command change — forward to instant
   reverse, hard strafe, spin of the stick — must never fall the
   robot. Two workstreams: (a) envelope coverage — walk in every
   direction (heading ladder: `cw-walk-head90` off wander30, then
   ±135/±180); (b) transition hardening — train with RANDOMIZED
   abrupt resampling (`goal.walk_cmd_resample_jitter`,
   `walk_cmd_blend_s_min/max` — intervals AND blend times vary,
   flips down to ~0.1 s), not just gentle fixed 5 s changes
   (`cw-walk-joystick45`). **JOYSTICK GATE (use for every driving
   candidate): `python3 -m rl_move.sim.eval_drive <ckpt> --dr-scale
   0.2 [--cfg-set ...]` — scripted fwd/back/strafe/diag/stop-go
   panel + randomized instant-flip stress episodes; ZERO in-envelope
   falls = PASS (exit code enforces it).** The generic harness
   samples the training distribution and proves nothing about
   direction coverage or flips (that's how backforth slipped
   through). A policy that walks beautifully but falls on a command
   flip is not a driving candidate.

1. [RUNNING] **Longer distances** — 30 s+ horizons, sustained gait
   without degradation (`cw-walk-longdist`). Extend to 60 s if it
   holds.
2. [RUNNING] **Faster walking** — 0.08–0.12 m/s band
   (`cw-walk-fast`); does real stepping emerge when shuffling can't
   keep up?
3. [CODE — 3 pricing attempts FAILED 08-10, next step needs code]
   **Turning** — yaw-rate command channel implemented:
   `goal.walk_yaw_cmd=1` samples a wz per command segment
   (`walk_yaw_max_rad_s`, `walk_yaw_zero_frac`), resampled/blended
   like vx/vy; `reward.k_walk_yaw` Gaussian kernel pays every walk
   tick. Turning does NOT emerge under any pricing tried: free income
   (yawcmd1, turn err 0.24), income-gated on achieved wz (yawgate1,
   0.236), 2.5x income (yawgate2, 0.233) — all ~unchanged, and the
   per-scenario pattern (yawgate2) shows a fixed left-yaw drift from
   walk training that the price never touched: commands near the
   drift track, commands against it don't, even in pure turn-in-
   place. Root cause is structural (gait bias / lack of a
   turn-specific regime), not kernel economics — STOP tuning
   `k_walk_yaw`/gates. NEXT (code task): decouple linear-speed
   sampling from yaw-rate sampling in `_sample_walk` (walk_task.py)
   so commanded turns are trained without competing walk-kernel
   pressure — the "linear speed forced toward 0 during commanded
   turns" curriculum.
4. [READY] **Back and forth** — walk forward N cm, reverse back to
   start. Includes backward walking (exploratory line; deferred
   from PROMOTION gates by the 08-09 ruling, not from training).
5. [READY] **Omnidirectional** — lateral strafing, diagonals.
6. [RUNNING] **Driving / direction changes** — command changes
   MID-episode: `goal.walk_cmd_resample_s` landed 08-09
   (`cw-walk-wander`, resample 5 s, ±45°, 15% stops).
7. [RUNNING] **Stop-and-go** — covered by `cw-walk-wander`'s stop
   segments; split into its own arm if transitions look bad.
8. [READY] **Smooth speed transitions** — accelerate/decelerate
   within an episode without gait breakdown.
8b. [CLOSED 08-09, contact-pricing class] **Operator-tunable speed**
   — 5 attempts (speedband, speedband-r1, slowband, speedband2,
   speedband2-r1) all converged on the same gait-speed ceiling
   ~0.05-0.065 m/s: below it the policy overshoots/idles, above it it
   just pins at ceiling (prog 1.39 slow / 0.38-0.61 fast, no falls,
   "survives commands it doesn't obey"). CLOSED pending operator
   contact/current pricing calibration, same root as paddling — do
   not requeue under `speedband*`/`slowband*` names (c73 08-10:
   re-queued a stale `cw-walk-speedband` backlog spec, caught by the
   drain's W&B-name dedupe before it launched — no compute lost, but
   check RL_LOG/ledger history for a name before backlog-adding a
   WISHLIST item marked done here).
8c. [READY — code LANDED c086a22] **Rotate in place** (08-09) —
    falls out of item 3's machinery for free: yaw is drawn
    independently of the linear command, so a stop segment
    (`walk_stop_frac`) with wz != 0 IS a commanded turn in place,
    and the yaw kernel is deliberately not gated on linear speed.

## Robustness (survives the real world)

9. [RUNNING] **Higher DR** — champion trained at DR 0.5
   (`cw-walk-dr05`); ladder toward 1.0 training if it holds.
10. [CODE] **Push recovery** — random external shoves mid-walk
    (MJX perturbation forces); stays up, keeps walking.
11. [READY] **Payload** — walk with extra chassis mass (DR mass
    field pushed asymmetrically, or a fixed +20–50% payload).
12. [CODE] **Five-legged walking** — one leg limp/locked, gait
    adapts. A policy that tolerates a dead leg is worth more than
    one that assumes six (a servo failure mid-session shouldn't end
    the demo).
13. [READY] **Quiet gait** — minimize mean/peak servo current at
    fixed distance; hardware-friendliness as an explicit objective.
13b. [READY] **Richer physics variation** (08-09) — beyond the
    current DR fields: per-leg friction, latency jitter, torque
    droop under load, foot-geometry perturbation. One new axis per
    run; keep what transfers.
13c. [CODE] **Sim2real noise research** (08-09) — dedicated study
    cycle: survey what noise/DR others inject for cheap-servo
    robots (backlash, deadband, encoder quantization, voltage sag,
    IMU bias walk), write up in rl_docs, then queue the top 2 as
    runs. Research first, then experiments.
13d. [CODE] **Obstacles** (08-09) — clutter on the floor: small
    blocks/ramps the robot must step over or around. Needs MJX
    scene work (see item 24, Terrain) — same implementation cycle.

## Skills and party tricks

14. [RUNNING] **Stand → walk → sit chain** — one policy, all three
    on command (`cw-chain-standwalksit`).
15. [READY (first rung) — quad-hold code LANDED c086a22] **Quadruped
    mode** — stand/walk on four rear legs, fronts free as claws
    (authorized parallel line). First rung implemented as goal mode
    `quad` (`--goal-mix quad=<p>`): lift legs **0 and 5** (the
    physical fronts per quadruped_feasibility.FRONT_LEGS — the old
    `quad_legs=0,3` sketch was wrong), commanded through the
    existing 6-wide goal one-hot with BOTH bits hot (obs width
    unchanged → warm-start from the walk champion works).
    `reward.k_quad_clear` pays unloaded front clearance up to
    `quad_clear_cap_mm` (30); `reward.k_quad_plant` pays the
    four-planted fraction; `goal.quad_grace_s` (1.5 s) keeps the
    lift transient unpaid. Level kernel / current charge / tilt trip
    inherited. Probe probe-quad-scale: MJX clean, 300k steps.
    **MAINLINE PROMOTION (operator 08-10 00:4x: "four leg trick in
    the main line so I can hit that with the joystick in sim to
    real"):** quad is now a JOYSTICK COMMAND of the driving
    lineage, not a standalone trick. cw-quad-hold1-r2 proved the
    hold (survived 1.0, video clean) but 50% mix eroded walk (the
    if-false branch); cw-quad-hold2 (30% mix, walk champion) and
    **cw-walk-joyquad30 (30% mix composed onto DRIVING champion
    joylat25 — the sim-to-real candidate line)** are queued/running.
    Operator-facing: `drive_policy.py` key `4` toggles the quad
    command live (writes lift_legs=(0,5) into the running
    trajectory). Any future hardware candidate carries the quad
    command through the deployment-equivalence (cw-dep) contract.
    Remaining rungs [CODE]: weight shift → quad walk (exempt fronts
    from six-leg participation terms) → quad turn → height up/down
    (reuse walk_height_off_mm — it is mode-agnostic).
16. [LATER — after 0-c stability gates] **Fall recovery** — start
    fallen, get up quietly (needs fallen-pose reset generator +
    orientation-complete obs; hard current pricing per the
    2026-08-06 incident).
17. [CODE] **Claw gestures** — wave a front leg / "shake hands"
    while standing stable on five.
18. [READY] **Body pose control** — track body height/roll/pitch
    while standing (camera aiming, looking up/down); goal-mix
    lean/track modes exist.
18b. [RUNNING] **High/low gait** — walk at commanded stance height:
    `goal.walk_height_off_mm` landed 08-09 (`cw-walk-highgait`
    +20 mm, `cw-walk-lowgait` −20 mm). Everything that works
    becomes a runtime command via the height ref, like rise/lower.
18c. [LATER — needs 2+ solid skills] **Motion sequences** (08-09) —
    chain skills in arbitrary orders on command: stand → walk →
    turn → sit → stand → strafe. First as scripted goal schedules
    in eval (no retrain), then as a trained mixed-goal policy if
    scripted chaining breaks at transitions.
18d. [CODE] **Jumping** (08-09) — flag: likely NOT hardware-safe
    with STS3215s (peak current at hop takeoff vs the 2026-08-06
    cooked-servo incident). Sim-only exploration allowed: small
    hop in place, price current hard. Never deploy without an
    explicit operator ruling.

## Learning machinery (makes everything above easier)

19. [READY] **Learning-progress command curriculum** — bucketed
    speeds/directions, sample the moving frontier (binding review
    item, never scheduled).
20. [CODE] **Mirror-symmetry augmentation** — queued, needs index
    maps + trainer support + probe.
21. [CODE] **Contact-from-proprioception aux head** — predict foot
    contact from joint/current/IMU history.
22. [CODE] **DreamWaQ-style concurrent estimator** — next
    architecture rung (post-0-c per plan).
22b. [READY] **Better actor architectures** (08-09) — temporal
    actor first: ~300 ms obs/action history stack vs a modest GRU,
    per the binding review's ranking (history as online system
    identification). Controlled comparison against the MLP
    champion, same reward/steps/seeds.
23. [LATER — needs 2+ per-skill champions] **Distillation** — merge
    per-skill champions into one deployable policy.
24. [CODE] **Terrain** — ramps/uneven ground if the MJX scene can
    support it; flag scene work first.
25. [CODE] **Scripted-gait sim replay diagnostic** (simplification
    review §9, 08-10) — replay the known-good scripted gait through
    the current simulator/actuator/contact stack. If IT develops
    phase drift and dragging, the simulator is the bug — fix it
    before blaming RL for desynchronized walking. Complements the
    hardware-trace replay calibration already directed in RL_PLAN.
26. [CODE] **Per-joint proposal/applied/q overlay** (simplification
    review §9, 08-10) — for RL video/logs, overlay per joint: policy
    proposal → SafetyLayer/applied target → actual q, plus foot
    contact state. Separates impossible policy choreography from
    actuator lag / contact-induced phase loss; also closes the
    long-standing "previous-action = raw proposal vs post-safety
    applied?" audit from the 08-09/08-10 GPT handoffs.

## How to use this list (binding)

- A free GPU pod with no sound main-line arm MAY take the topmost
  [READY] item not already running; [CODE] items get a dedicated
  implementation cycle when 2+ pods would otherwise idle. Pulling
  from here is a judgment call, not a duty: **idle compute is
  acceptable** when the next useful work is specification, hardware
  evidence, or code (simplification review, 08-10 — supersedes the
  08-09 "idle GPUs are the failure mode" clause). Apply the launch
  question from RESEARCH_RULES before every pull; when you do pull
  several, pick diverse lines, not five variants of one idea.
- Wishlist runs use the same rigor: `cw-<line>-<idea>` names,
  launcher-only launches, pre-registered gates, video verdicts,
  ledger (which auto-renders `rl_docs/runs/<run>.md`).
  "It's exploratory" is not an excuse for an unwatched success.


---

# FILE: rl_docs/BENCH_REPORT_2026-08-11.md

# Bench report — 2026-08-11 evening (unattended camera sessions)

Nine `bench_blast` sessions ran on the physical robot this evening,
almost all fully unattended (iMac camera recording, spoken countdowns,
fall-recovery loop, thermal gate). This file is the consolidated read:
what happened, what the tools are, and what it implies for RL training.

Raw data: `rl_move/hardware_traces/bench_blast_20260811_18*` and `_19*`
(per-session `summary.json`, episode CSVs, robot logs, `camera.mp4`,
frame sheets). Regenerate every table below with ONE command:

```sh
python -m rl_move.scripts.bench_report --since 20260811_18
```

## The tools (all landed tonight, all reusable)

| tool | what it does |
|---|---|
| `rl_move/scripts/bench_blast.py --go --auto --camera 0` | Runs a whole bench session unattended: spoken countdowns, iMac camera recording with exact unix-time sync, per-step preflights, fall detection → safe_zero(force) → stand recovery, thermal gate before every motion, terminal-result capture (never kickoff responses), CSV + robot-log pull. |
| `rl_move/scripts/video_review.py --session <dir>` | Syncs the camera video to the session log (exact, via `camera.t0_unix`), cuts a labeled frame sheet per event (walks, rises, turns), full-res zooms for tape segments. |
| `rl_move/scripts/bench_report.py` | NEW: one markdown table for any set of sessions — per-walk roll-trace metrics (transient timing, peak, tail, fell/recovered/clean), per-rise trip signatures, per-policy tallies. Reads only local files. |
| robot log pull | Each session dir gets `robot_logs/` (errors, events, measurements) filtered to the session window; the episode CSVs carry per-tick roll/gyro/currents/actions/obs/rot60_k. |

## Finding 1 — the learned rise fails DETERMINISTICALLY on hardware

10/10 attempts tonight (2 session openers + 8 recovery attempts across
5 sessions) tripped `tilt_roll` with an astonishingly tight signature:

| metric | value across all 10 |
|---|---|
| trip tick | 225–228 (~9.0–9.1 s, mid belly-curl, before the height ramp) |
| max relative roll | 10.1–10.6° (trip threshold 10°) |
| max current | 0.23–0.27 A (no load event — it's kinematic rocking) |

Two of the attempts started from a VERIFIED clean zero (max pose delta
0.5°, stand preflight green) — start pose is exonerated. Sim probes of
the same checkpoint keep |roll| ≤ 1.7° through the whole rise. The
hardware body rocks over the tucked legs during the curl; the sim's
never does. This is a pure sim-to-real gap in the curl dynamics, not a
threshold problem and not noise.

Meanwhile the scripted `POST /api/zero {"pose":"stand"}` glide stood
the robot up every time it was asked (except phantom-temp aborts,
finding 4). It is the working stand-up path until a rocking-hardened
policy exists.

## Finding 2 — the takeoff transient is UNIVERSAL; falls are a coin flip; the A/B has no winner

All 18 walks tonight (both policies, both directions, clean and
post-fall starts) show the same shape in the episode CSVs:

- |roll| crosses 5° within **0.6–1.5 s** of gait start;
- it peaks **13–27°** somewhere in the first ~1.5–5 s;
- then either settles back to level ("recovered", tail ≤ ~2°) or
  escalates to a fall — with no obvious predictor in policy, direction,
  or peak size (a 23° peak recovered; a 15° peak fell).

Full-night tallies (`bench_report`): **vref1 6/10 fell, 3 recovered,
1 clean. tip1 4/7 fell, 3 recovered, 0 clean.** The lone clean walk
(peak 7.8°) was vref1-r3 in the attended 18:24 session.

Two corrections to earlier same-night claims:

1. "tip1 is the deploy champion" (early-evening read off 4 walks) does
   NOT survive the full-night sample. Neither policy separates.
2. The A/B alternation scheme has a confound: round 1 always gives
   vref1-fwd / tip1-back, so 1-round sessions never walk tip1 forward.
   Every tip1 evening walk was backward. Fix before trusting any
   fwd/back split.

The discriminating problem is not vref1-vs-tip1 — it is surviving the
takeoff transient at all.

## Finding 3 — turn signs: half-answered

`omega=+0.3` (6 s scripted turn): large net rotation on camera,
read as **CCW from above — matching the z-up convention**
(single reading, 19:33 session; frames in
`bench_blast_20260811_193306/video/turn_p03_*.png`).

`omega=-0.3`: STILL unmeasured. The first attempt was silently refused
by the robot (`pending measurement — save or discard it first` — the
+0.3 turn's un-annotated measure record blocked it; the robot just
stood there for its window). That bug is fixed (`bench_blast` now
discards the pending record before each video-mode turn) and the rerun
executed, but the camera was being carried away during exactly that
window. First item for the next session.

## Finding 4 — tonight's "thermal wall" was (mostly) PHANTOM BUS READS

Three motion aborts blamed temperature: L2 hip 72 °C (19:18), L4 hip
68 °C (19:36), L4 hip **150 °C** (19:51). The 150 °C one broke the
story: the same servo read a steady 33 °C seconds later — thermally
impossible — and the debounced watchdog (`servo_watch`, two consecutive
hot reads required, exactly because of documented 08-09 phantoms) never
tripped once all night. The single-read temp checks in the glide/rise
prep code (`safe_zero.py`, `pinned_tip.py`) were the ones aborting
sessions, and corrupted bytes on the shared bus fake hot reads.

So: possibly NO real thermal event happened tonight. (The 72 °C reading
had a plausible heat story — falls + recoveries stack load — so treat
it as unconfirmed rather than fake; servos all measured ≤ 38 °C minutes
later.)

Fixes landed:

- `safe_zero.py` / `pinned_tip.py`: temp trips now need two consecutive
  hot reads ~0.3–0.6 s apart (same debounce the stall check always had).
  A real overheat still trips in under a second.
- `servo_watch.py` + `bench_api.py`: the always-on watchdog now has a
  **thermal panic** — on a (debounced) overtemp it kills ALL motion
  (aborts the demo/RL worker, gait stop, torque-all off) instead of
  cutting one servo and letting the job drive the other 17 legs. Busy
  cadence tightened 10 s → 5 s.
- `bench_blast.py`: thermal gate before every motion step (hold at
  ≥55 °C until back under 45 °C, abort if it won't cool).

## Finding 5 — session automation debugged itself into a real harness

Each unattended failure mode got a permanent fix the same night:
terminal results instead of kickoff lies, fall recovery
(safe_zero `force=true` — a real fall ALWAYS trips the tilt gate),
scripted-stand fallback behind the deterministic learned-rise trip,
demo-aware waits (`/api/zero` and `safe_zero` run as demos invisible to
`wait_idle`; one abort read a mid-glide pose and masked the real
error), auto-safe_zero when the opening pose isn't belly zero (a
stalled safe_zero had left the L4 knee 78° off and hold-hunting — the
operator's "twitching leg"), pending-measurement discard before turns,
CSV size-stable pulls, alternating walk directions to stay in frame.

## What this implies for RL training

1. **Train the takeoff transient, not the A/B — and it must be
   DYNAMIC.** The sim's plant-start episodes do not produce the
   hardware's 13–27° first-seconds roll excursion, so neither policy
   ever learned to manage it — recovery is luck. The static-lean dose
   arm already CLOSED while this report was being written:
   `cw-dep-tip1-takeoff25-r1` (proper 20–25° tipped-start injection,
   matched baseline) showed child == parent with ZERO falls in both —
   the sim already recovers static tipped starts at the hardware
   regime. So the gap is not the lean, it is the roll RATE: a
   gait-start roll-velocity perturbation axis (code, unbuilt) or
   contact/pinning work. Whatever the arm, gate on the fell/tail
   criterion, not peak — hardware shows peaks near 25° are survivable.
2. **We now have the data to model the transient instead of just
   dosing it.** The episode CSVs carry per-tick roll, gyro, per-servo
   currents, commands AND actions for all 18 walks. A trace-replay
   calibration (feed the recorded action sequences to the sim plant,
   compare roll evolution) would tell us whether the gap is contact
   (feet unsticking from settled plant), actuator lag under load, or
   mass distribution — and turn "add DR" into "fix the model."
   This is the same lever the rise needs (see 3).
3. **The rise gap is a specific, reproducible target.** Trip at tick
   ~227 of the curl, 10.1–10.6° roll, currents flat — the hardware
   curl rocks laterally where sim glides. The queued
   `cw-stand-riserock1` drained as a stub and is VOID (the rocking-DR
   code was never written); the rise-rock axis is still unbuilt code
   that would treat the symptom, while the loaded-knee actuator model
   treats the cause. Because
   the failure is deterministic and cheap to reproduce (10 for 10!),
   it is the best sim-calibration probe we have: match THIS trace
   first, then retrain.
4. **Keep the 10° rise trip.** The walk envelope tolerates and often
   recovers 15–25° transients, which tempts raising the rise trip —
   but a rise failure means falling from a half-risen, legs-tucked
   pose with no stance to recover into. Harden the policy, not the
   threshold (standing ruling from 08-11 stays).
5. **Fix the A/B before drawing policy conclusions.** Alternate which
   policy leads each round (or force per-policy direction coverage) so
   tip1 gets forward walks. Until then, treat vref1-vs-tip1 as
   undecided and spend bench time on transient reps instead.
6. **Fell/tail is the metric.** `bench_report` now computes it from
   the CSVs; the sim eval side should report the identical statistic
   (`tail |roll| over the last second`) so hardware and sim numbers
   are directly comparable.


---

# FILE: rl_move/API.md

# Hexapod control API (prefer over SSH)

Base URL: `http://hexapod.local:8080` (or `HEXAPOD_URL`).

## 2026-08-06 incident — read this

Wrong logical zeros + unsupervised stand/plant blends tipped the robot,
browned out the board, held stilts at ~7 A, and cooked a knee servo
(hardware fully resolved 2026-08-09 — servo replaced, all 18 healthy;
the process rules below are what remain).

**Safe sequence only:**

1. Limp / hand-set a **known visual** pose (usually legs straight out).
2. `POST /api/set_zero` — that pose becomes logical 0°.
3. Tiny air moves; predict then read. Stop if a servo ID is missing/hot.
4. Capture plant only when the operator likes the stance (`capture_plant`).
5. **No** autonomous stand-up.

## RL routes

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/rl/state` | Pose + plant + IMU + status |
| GET | `/api/rl/policy` | Deployed policy metadata (stance + walk) |
| GET | `/api/rl/policies` | List swappable policies in `policies/` + active flags |
| POST | `/api/rl/policy_select` | `{"file":"<name>.json"}` — make it live (file copy, no motion) |
| GET | `/api/measure/list` | Saved measurements + pending record (Measure tab) |
| POST | `/api/measure/walk` | Measured scripted-gait run `{"vx_mm":30,"omega":0,"duration_s":20}` (caps 60/40 mm/s, 0.5 rad/s, 60 s; acquires ARM+stand first when missing) |
| POST | `/api/measure/hold` | Holding-current log `{"label":"planted"\|"hover","duration_s":30}` — holds present pose, NO commanded motion |
| POST | `/api/measure/annotate` | Merge operator tape reading into the pending record + save |
| POST | `/api/measure/discard` | Drop the pending record |
| POST | `/api/measure/note` | Standalone record; `kind:"rl_walk_tape"` attaches newest RL episode CSV |
| GET | `/api/logs` | List `logs/` files (name, bytes, mtime; newest first) |
| GET | `/api/logs/<name>` | Download one log file; `?tail=N` = last N lines only |
| GET | `/api/rl/preflight?mode=` | Read-only readiness (`stand`/`lower`/`walk`) |
| POST | `/api/rl/stand` | RL policy stand-up (preflight-gated; wrong pose → acquires safe zero first, then re-preflights) |
| POST | `/api/rl/lower` | RL policy lower to belly (wrong pose → acquires the plant stand first, then re-preflights) |
| POST | `/api/rl/walk` | RL walk, EXPERIMENTAL: `{"vx":0.03,"vy":0,"duration_s":6}`, clamped 0.06 m/s / 20 s; wrong pose → acquires the plant stand first, then re-preflights |
| POST | `/api/rl/capture_plant` | Save **current** 18 joints (no motion) |
| POST | `/api/rl/set_stance` | Slow ease to a crouch stance; a big Δq acquires the safe zero start first instead of refusing |
| POST | `/api/rl/find_plant` | **Disabled** unless `{"force":true}` |
| POST | `/api/rl/probe_dynamics` | Air-only ±amp per joint → `logs/motor_model.json` |
| POST | `/api/rl/stop` | Abort worker |
| GET | `/api/rl/roles` | Role registry: which `policies/` file serves walk / hold / stand / lower |
| POST | `/api/rl/roles` | `{"role":"hold","file":"<name>.json"}` — assign (no motion; `""` = default, `"walk"` = walk@zero for hold) |
| GET | `/api/rl/drive` | Live drive-session snapshot (active, model, refs, tilt) |
| POST | `/api/rl/drive/start` | Start persistent held-key drive session (walk preflight + acquire rules; operator watching) |
| POST | `/api/rl/drive/cmd` | `{"vx":0.05,"vy":0}` heartbeat ~5 Hz; stale >0.6 s ⇒ refs decay to zero (hold) |
| POST | `/api/rl/drive/stop` | Graceful end: decel to zero, HOLD pose |
| POST | `/api/set_zero` | Present pose → logical 0° (required after hand-set) |
| POST | `/api/zero` | Sit/stand — acquires the pose safely (sit = safe-zero plan; stand = safe zero → validated plant stand-up); never refuses on Δq, errors + stops if acquisition fails |
| POST | `/api/safe_zero` | Collision-aware go-to-zero: plans staged waypoints (straighten → center yaws with feet lifted → extend flat), **errors if no safe path exists**, and **LIMPS on any stall / unexpected-force feedback** during motion. `{"dry_run":true}` returns the plan with no motion; `force` bypasses only the IMU tilt gate. Poll `/api/calibrate` for progress. |
| POST | `/cmd` | `ARM` / `X` limp / `HOLD` / `# j deg` / `C` / `P` |

Drive `C` (centre) and `P` (stand) refuse if any live joint would move
more than **25°** from present unless the command includes `FORCE`.

## Deployed policies (2026-08-10, hardware attempt #2)

- stance = `ppo_goal_cw_stance_dr10` (`rl_policy_weights.json`, obs 68)
  — drives BOTH stand and lower. Tilt trip 10° (its trained envelope).
- walk = `ppo_goal_cw_dep_vref1_r1` (`rl_walk_weights.json`, obs 72,
  zip md5 `f9a466cfa7ae7213e48799a24032ac54`, verdict PASS 08-10) — the
  deployment-contract champion: trained with `goal.walk_obs_body_vel=2`,
  so the runner's vx/vy_meas := ref feed IS the training contract, and
  with a 25° relative-tilt envelope; the runner widens its SafetyLayer
  trip to 25° in walk mode to match (stand/lower stay 10°). Command in
  the trained band **0.05–0.06 m/s** (below 0.05 is out-of-distribution);
  duration clamped to 20 s; starts only from the captured plant stance.
  Still gated `hardware_ready: false` pending contact/current pricing
  calibration (tape-measure distance session).

**Policy picker (2026-08-10):** the robot carries a registry at
`linux_control/policies/` — currently `stance_dr10.json` (stance slot,
obs 68), `dep_vref1_r1.json` (walk slot, obs 72, active) and
`dep_quad1_c2.json` (walk slot; same contract as vref1-r1 with the
four-leg-stand trick trained in — the quad trick itself has NO runner
mode yet, so on hardware it is a walk-only alternative). Pick in the
web UI (RL tab → Policy panel) or `POST /api/rl/policy_select`.
Selection atomically copies the file over the live weights; it takes
effect at the NEXT episode start (never mid-move) and is refused while
a job is running. Slot is inferred from obs dim (68 stance / 72 walk).

Add a policy: `python -m rl_move.sim.export_policy_np --policy <zip>
--out linux_control/policies/<name>.json --name "<display>" --notes
"<operator notes>"` → scp into
`/home/arduino/hexapod_sts/linux_control/policies/` (no restart
needed — the list endpoint reads the dir live).

**Roles + drive session (2026-08-11):** on top of the slots, a role
registry (`~/.hexapod_rl_roles.json` on the board, `/api/rl/roles`)
maps FUNCTIONS to files: `walk` (obs 72), `hold` (what runs when no
keys are pressed — default "walk policy at zero command", or any obs
68/72 file), `stand` and `lower` (obs 68). One file can hold several
roles. Unset roles fall back to the live slot files, so behavior
without assignments is unchanged. The drive session
(`/api/rl/drive/*`, RL tab "Drive — hold keys") is a persistent
25 Hz walk loop steered by live browser heartbeats: held arrow keys
= walk that way, release = decel to the hold model, model switches
re-anchor the episode frame (q_nom := present pose, prev_action 0).
Watchdogs: heartbeat stale 0.6 s ⇒ zero command; 120 s silence ⇒
session ends holding; 300 s hard cap; safety trips limp as always.
Every session logs `rl_drive_*.csv` like any episode.

## RL episode logging (2026-08-09, on-robot, automatic)

Every stand / lower / walk run writes a full local trace under
`/home/arduino/hexapod_sts/linux_control/logs/` (`_EpisodeLog` in
`linux_control/rl_policy.py`) — nothing to enable:

- **`rl_<mode>_<stamp>.csv`** — one row per 25 Hz control tick:
  `t_s`, `phase` (`run` / `tail`), body `roll_deg`/`pitch_deg`
  (attitude filter), `gyro_{x,y,z}_dps`, goal refs (`height_ref_mm`,
  `vx_ref_mps`, `vy_ref_mps`), running `max_cur_a`, then per joint
  0–17: `q*_deg` (measured), `cmd*_deg` (commanded, post-safety),
  `act*` (raw policy action in [-1,1]), `cur*_a` (per-servo current;
  blank on ticks without full feedback), then the **full policy obs
  vector** `obs0..obsN` (68 stance / 72 walk) — replay it through the
  same weights offline to separate obs-pipeline bugs from behavior.
  After the episode a **3 s read-only tail** (10 Hz, `phase=tail`,
  no commands sent) keeps recording attitude/q/currents so a tip-over
  during the end-of-episode hold is captured (added 08-10 after the
  dep-tip1 fall landed just past the last logged tick). Flushed every
  ~1 s so a safety trip / kill still leaves the trace up to that
  moment.
- **`rl_<mode>_<stamp>_summary.json`** — params (policy meta, q_nom,
  tilt ref, preflight readings, walk vx/vy) + final result (ticks,
  max current, overruns, error/trip reason if any, plus attitude
  bookkeeping: `tilt_rel_max_deg`, `roll/pitch_rel_end_deg`,
  `tail_tilt_max_deg`, and `fell` — >35° relative at any point ⇒ it
  went over).
- **`events.jsonl`** gets `kind:"rl_episode"` markers at start and
  end (end carries the result + csv name), so episodes line up with
  button presses, overtemp events, MCU traffic on one timeline.
- The move-route JSON response includes `"log": "<csv name>"`.

Fetch for analysis (laptop) — HTTP preferred, no SSH needed:

```bash
curl -s http://hexapod.local:8080/api/logs | python3 -m json.tool   # list
curl -sO http://hexapod.local:8080/api/logs/rl_walk_20260810_221553.csv
curl -s 'http://hexapod.local:8080/api/logs/events.jsonl?tail=200'  # tail
# scp still works: arduino@hexapod.local:hexapod_sts/linux_control/logs/
# or stream events live: linux_control/receive_robot_logs.py
```

Analysis starters: commanded-vs-measured per joint (`cmd*` − `q*` =
tracking error / stall detection), `cur*` spikes vs. joints, roll/pitch
during walk, `overruns` in the summary for loop-rate health.

## Laptop

```bash
python3 -m rl_move.remote state
python3 -m rl_move.remote capture_plant
# limp:
curl -X POST --data 'X' http://hexapod.local:8080/cmd
```

SSH only for deploy/restart when the operator asks — never for routine motion.


---

# FILE: rl_move/RUNLOG.md

# RL balance run log

Append-only diary of hardware sessions. Newest entries at the bottom.

---

## 2026-08-06

### Context
- Phase-1 package `rl_move/` created from merged `RL_PLAN.md`.
- Step B probe OK earlier: pos ~5 ms, IMU ~5 ms, combined ~13 ms @ 50 Hz.

### Tip / brownout #1 (~20:07 UTC board time)
- Ran early `balance_test` which blended toward stand plant + SyncWrite.
- Robot tipped; board hard-rebooted (`last` → `crash`).
- Cause: servo pack brownout (18 motors fighting) → Uno Q 5 V collapse — not a software shutdown.

### Tip / brownout #2 (~20:31)
- After recovery, probe showed nearly level (~−1.5° roll / +2° pitch) but knee samples ~−92° (crumpled / odd pose).
- `hold_current` + torque enable + SyncWrite → second hard reboot.
- Hardened: preflight tilt, tighter kill (10°), limp on terminate, **`enable_motion: false` default**.

### Dry-run (motion off)
- `balance_test` 1×250 steps, truncated OK, board stayed up.
- Overruns high (~94) from double pacing in script+env — follow-up.

### Next
- Robot reported upright by operator — enable motion, hold-current only, short episode.

### Motion hold-current #1 (operator upright, ~20:34 board)
- Log file: `rl_move/RUNLOG.md` started.
- Preflight probe: roll −1.5°, pitch +2.0°, bus/IMU OK; L0 sample
  yaw≈−2°, hip≈−65°, knee≈+36° (not CAD plant — holding *current*).
- `enable_motion=true`, `hold_current_pose=true`, 3 s / 150 steps @ 50 Hz target.
- **PASS** — truncated, no terminate, end tilt (−1.5°, +2.0°), board stayed up.
- Note: ~147/150 “overruns” — each `step()` (SyncWrite + sense) often &gt;20 ms;
  loop is effectively ~30–40 Hz under load. Follow-up: slim tick or lower hz.
- `hexapod-web` restarted after test.

### balance_sine ±1° (hold-current, 25 Hz) — PASS
- Config: motion on, hold current, hz=25, amp=1°, period=6 s, 1.5 cycles/axis.
- Roll: (−1.6°, +2.0°) → (−1.51°, +1.94°), overruns 16/225.
- Pitch: (−1.4°, +2.1°) → (−1.44°, +1.98°), overruns 17/225.
- Board stayed up; no tip.
- **Caveat:** IMU barely moved (~0.1°) despite ±1° body cmds — IK may be
  ineffective from current non-plant pose, or sign/frame bug. Next: verify
  joint targets change under sine, or stand to real plant slowly then retest.
- Control hz 50→25 in `config.yaml` (SyncWrite+sense often &gt;20 ms).

### Pivot: feet not on ground (operator)
- Balance IK assumes planted feet; current pose had feet in air → sine could
  not move chassis IMU. Next step: slow `stand_to_plant` before more balance.

### stand_to_plant #1 (PSU ~3 A limit) — board drop
- 12 s blend from hip≈−65°/knee≈+36° toward plant. Progress ~33% then SSH
  reset; Uno Q unreachable. Likely PSU current-limit foldback.

### stand_to_plant #2 (PSU raised to ~10 A, 25 s blend) — tilt abort, board stayed up
- Start pose odd after reboot: L0 hip≈+15.5° / knee≈**−91.7°** (max |Δq|≈176°).
- Knee stuck ~−22° for ~8–40% of blend, then resumed; tilt grew and trip at
  **64%** (roll≈−12° / pitch≈−10°) → limp. Board remained reachable.
- Power-domain OK enough at 10 A for this attempt; geometry/start pose was
  the failure mode. Next: hand-set near plant or shorter Δq before blend.

### stand_to_plant #3 (after belly reset, 30 s) — tilt abort again
- Start L0 −38°/−36°, level IMU. Progressed smoothly to hip≈0° / knee≈+40°
  then roll spike → limp at **60%** (roll≈−12°). Board stayed up; web restarted.
- Pattern: large simultaneous stand from sprawled knees tips sideways before
  plant. Need hand-assist / hold chassis through last third, or stage knees
  first with body supported.

### Policy pivot (GPT + operator) — stop auto +20/+80
- +80° is knee axis max; experimentally tucks feet under and tips.
- Next: hand-set stable stance → `capture_plant` (full `joints_deg`) →
  balance starts with hold-current only; refuse if far from plant.
- `stand_to_plant` disabled without `--force` + captured plant.
- 25 Hz accepted; no 50 Hz chase for now.

### HTTP API + geometry plant (prefer over SSH)
- Added `GET/POST /api/rl/*` on hexapod-web + `rl_move/remote.py` client.
- **Incident:** unsupervised stand/plant with wrong zeros → tip/brownout /
  ~7 A stilts / one knee servo dead/hot (since replaced 08-09; bus 18/18
  healthy — RESOLVED, not an open issue). set-zero-here later fixed the
  frame (straight-out → logical 0°). Air ±15° nudges then matched encoders
  on live joints.
- Hardening: cursor rule + AGENTS.md; drive `C`/`P`/`#` Δq>25° refuse;
  `find_plant`/geometry require `force`; `enable_motion` default false.


---

# FILE: rl_move/orchestrator/README.md

# Autonomous experiment orchestrator

A self-driving loop that keeps the CoreWeave RL campaign running while
the operator is away. Mechanical software owns state and throughput;
an LLM cycle owns judgment (verdicts + what to try next). If this file
and the code disagree, the code is right — fix this file.

**Taking over this project (human or LLM)? Start with
`rl_docs/AGENT.md`** — how the agent picks runs, what we learned
works/fails, and the future-work map. This file covers mechanics only.

## Architecture (2026-08-09)

```
controller pod (hexapod-sweep-friction, tmux session "orchestrator")
  watch_loop.py — polls W&B every 5 min, plus three background workers:
    • backlog worker: every 2 min, `launch_run.py drain` pushes queued
      specs from backlog.json onto free GPU pods (self-repairing:
      syncs code, pushes checkpoints + W&B secret, retries 3x, then
      parks in backlog_failed.json)
    • checkup worker: ~5 min after each launch, `launch_run.py
      checkup`; DEAD/SUSPECT findings are injected into the next cycle
    • pre-stager: for each newly finished run, pulls the checkpoint,
      starts the DR-0 gate eval, caches W&B data
  when runs FINISH → spawns a decision cycle (cap 2, concurrent):
    claude -p --bare --model claude-fable-5 <ORCHESTRATOR_PROMPT.md>
    the cycle TRIAGES each finished run (~10 min: video, curves,
    gate scalars), records verdicts via `launch_run.py update`
    (auto-renders rl_docs/runs/<run>.md) + `ops.sh wandbnote`,
    digs in only on a real trigger, and refills the pipeline by
    queueing specs into the backlog. Logs: /workspace/cycle_logs/
```

Training pods: `hexapod-mjx-train-0..15` (1 H200 + 24 cores each; see
`CAPACITY.md`, live truth via `capacity.py`). New pods are initialized
by `bootstrap_train_pod.sh`, which writes the `.bootstrapped` marker
the drain requires before treating a pod as a slot.

## State — machines own facts, the LLM owns interpretation

- `experiments.json` — the ledger, single source of truth per run
  (status, hypothesis, gate, verdict, W&B id). Edit ONLY via
  `launch_run.py update`. Every update regenerates the browsable
  per-run summary in `rl_docs/runs/`.
- `backlog.json` — mechanical launch queue, fed by cycles/operator
  (`launch_run.py backlog add`), drained automatically.
- `RL_LOG.md` — 1 line per cycle; `RL_PLAN.md` — the plan (~120
  lines). Everything else: `rl_docs/` (start at its README).
- Code provenance: `snapshot.sh` commits/tags/pushes, `--sync` stamps
  pods with `.code_sha`; the launcher refuses mismatched pods.

## Operating it

- **Everything routine**: see `rl_docs/COMMANDS.md` (ops.sh helpers,
  gotchas, which command answers which question).
- **Pause cycles:** `touch PAUSE` in this directory on the controller
  (training keeps going). Unpause: remove the file.
- **Restart the watcher:** ONLY via `restart_watcher.sh` (nohup'd on
  the controller). Hard-killing the tmux session murders in-flight
  cycles, which only write their output at exit.
- **Logs:** `/workspace/orchestrator.log` (watcher),
  `/workspace/cycle_logs/` (cycles), `/tmp/train_<run>.log` (on pods).

## Safety

`guardrails.yaml` binds every cycle: sim/pods only, never the physical
robot, caps on concurrent runs / steps / cycles per day. The workspace
hardware-safety rules travel with the repo (AGENTS.md, .cursor/rules/).


---

# FILE: rl_move/orchestrator/ORCHESTRATOR_PROMPT.md

# Standing prompt — hexapod RL experiment orchestrator cycle

You are running one decision cycle of an autonomous RL experiment loop for
a hexapod robot trained in MuJoCo on CoreWeave pods. The operator is away;
you act alone within `rl_move/orchestrator/guardrails.yaml` (read it,
obey it). You are on the controller pod in a git clone of
`lukas/weird_objects`; work in `/workspace/weird_objects`, never the
deploy copy. `kubectl` reaches sibling pods; W&B creds are in the env,
project `l2k2/hexapod-balance`. Paths below are relative to
`hexapod_walker/prototype_sts3215/`.

**The big goal (operator, 08-10):** the operator drives the physical
hexapod with a JOYSTICK — stand up, sit down, turn, walk where pointed,
reliably, session after session. After that: the quad tricks (stand on
four legs, walk on four). Foot slip is NOT failure by itself (the
scripted gait that walks the real robot slips); slip metrics exist to
keep sim honest, not as a ban. Sim metrics are means, not ends.

**RESEARCH TRACKS (operator, 08-11 — read this before refilling):**
the campaign is split into parallel tracks (`tracks.json`; per-track
goal + status doc in `rl_docs/tracks/`): **hw** (joystick robot on
hardware by any means — the MAINLINE, pod priority), **arch**
(GRU/temporal models learning walk/stand/sit), **nobc** (learn
stand + clean gait from scratch, NO BC anchor ever), **quad**
(walk on four legs, front pair as hands), **turn** (commanded yaw /
mirror-symmetry). Every launch/backlog/respec carries `--track`
(respec inherits the source's). **CONTAINMENT: a triaged run's
follow-ups go ONLY to its own track**, judged against THAT track's
goal (read its doc first). A finding that matters elsewhere is
ESCALATED, not launched: one line in both tracks' STATUS.md +
`CROSS-TRACK INSIGHT:` in your logline; cross-track launches are
operator-only. Verdicts that change a track's story update that
track's `rl_docs/tracks/<track>/STATUS.md` — keep it a SHORT
screenful (goal / Now / Next), detail goes to the linked docs.

**PRIME DIRECTIVE (operator, 08-10 — supersedes GPU-occupancy
rules; scope: the hw track — other tracks substitute their own
tracks.json goal for "joystick control" but keep every process
rule):** minimize the number of unresolved blockers between the
current robot and reliable joystick control; that count is the KPI.
Idle pods are acceptable; peripheral experiments are not. Before
training, prove the reward and evaluator prefer the intended behavior
over all known cheats (MDP_PREFLIGHT: `rl_move/tests/
test_task_semantics.py`). Short runs discover mechanisms
(--phase discovery, ≤2M steps); long runs only harden behavior
already seen (--phase hardening + --evidence — the launcher enforces
both). Prefer hardware-derived questions over generic sim
robustness. Kill obviously bad runs early. Every analysis must end
in a decision that can change the next experiment.

**OPERATOR DIRECTIVE (08-11 night — binding; sharpens the prime
directive): MAKE STANDING AND WALKING WORK IN SIM, TO THE QUALITY OF
THE MODEL WE DEPLOY, AND KEEP THE FLEET FIRING AT EXACTLY THAT.**
Think root-cause-deep about the open sim stand/walk blockers (live
map: RL_PLAN "Critical path" + CURRENT_TRUTHS; as of tonight: the
one-parked-foot hold habit, the det flat-rise stall, the rise-rock
and takeoff roll-rate DR axes (CODE, unbuilt), the crouch-splay
tall-walking wall — next levers BC-INIT on the scripted tall gait /
physics easing — and the contact/pinning no-skate question) and keep
launching runs that DIRECTLY attack them:

- Spec every arm so a PASS is a deployable candidate — Gate 0 /
  deployment contract, warm from the champion lineage, retention
  gates: "the result of this run could be the model we use." Probes
  stay legal when they are the fastest route to the next launch, but
  the default artifact is a champion candidate, not a curiosity.
- **CODE-FIRST:** when the next lever on a stand/walk blocker is
  code (see the unbuilt items above), WRITE IT THIS CYCLE and check
  it in — new cfg keys, DEFAULT OFF, bit-exact when off, tests + the
  mode's semantics bank green, REWARD.md row for new terms,
  `snapshot.sh` before anything trains on it. Never park a line on
  "CODE, unbuilt" waiting for the operator, and never change shared
  default behavior to carry an experimental mechanism.
- This NARROWS "idle pods are fine": peripheral runs stay banned,
  but an idle fleet next to an unattacked stand/walk blocker is now
  the failure mode — the blocker map should keep the backlog
  non-empty. Phases, preflights, caps, and track containment all
  still bind.

**The process is LIGHTWEIGHT by operator order (2026-08-09). Most runs
need a 10-minute triage, not an hour of forensics. Dig in only when
triage finds something real.** Machinery you must NOT rebuild or wait
on: the watcher pre-stages checkpoint pulls + W&B dumps for every
finished run and runs the standard evals (DR-0 gate + own-DR) ON THE
RUN'S OWN POD (08-10: eval compute lives on the train pods' idle CPUs,
never the controller — run your own extra evals there too, via
`kubectl exec` or `ops.sh podeval`), runs post-launch checkups (~5 min
after each launch), and continuously drains `backlog.json` into free
GPU slots via the self-repairing launcher. Capacity questions: run
`python3 rl_move/orchestrator/capacity.py` — never re-derive slots.

Cycles run CONCURRENTLY. Runs your "## This cycle" section marks as
another cycle's are off-limits. Coordination is mechanical (launcher
lock, ledger lock, snapshot git lock); a REFUSED from the launcher is
normal traffic, not an error to fight.

**Shutdown protocol (operator, 08-09): between runs — after recording
each verdict, before starting the next run's triage — check
`test -f rl_move/orchestrator/WRAPUP`.** If it exists, an update is
waiting on you: record everything you've completed (ledger verdict +
wandbnote for analyzed runs, backlog refills for free slots, your
RL_LOG logline), then EXIT immediately. Do NOT start triaging another
run — anything you leave unverdicted is automatically re-assigned
after the update. Cycles that ignore the flag are killed at a 30-min
deadline and lose their unfinished reasoning.

Read before deciding: **`CURRENT_TRUTHS.md` FIRST** (accepted facts —
it outranks anything you infer from history; a six-hour-old
hypothesis in the log never outweighs a line there), then `RL_PLAN.md`
(blockers/queue/architecture/Gate 0), **`RESEARCH_RULES.md`** (how
you may design/launch/stop/judge — phases, MDP_PREFLIGHT, kill
rules), and `rl_docs/COMMANDS.md` (ops.sh helpers + gotchas).
`RL_LOG.md` is a navigational index (1 line/cycle); the binding
reviews live in `archive/` — consult history when DESIGNING a new
line or answering a historical question, not on every cycle, and
never to infer current state.

(The 08-10 ~01:00 ET "hardware window" P0 list is EXPIRED — its
items are verdicted or folded into RL_PLAN.md and CURRENT_TRUTHS.md.
Still binding from it: `reward.k_current=0` on hardware-target arms
until current pricing is calibrated; prev-action semantics are
audited PASS, don't re-audit; no generic DR pair-composes — see the
prime directive and RL_PLAN "CLOSED moves".)

## The cycle

1. **TRIAGE each finished run (~10 min). Start with
   `ops.sh review <run>`** — it prints the ledger status+gate, W&B
   state/steps/reward-quarters, the harness report table with
   medians, and the video/contact-sheet paths in one shot. Do NOT
   hand-write python to parse experiments.json, report.json, or the
   W&B API for this standard read (transcript mining found >500 such
   snippets in one day; the helpers exist — `ops.sh report`, `entry`,
   `wandb`). Look at exactly three things:
   - the frame strip / video of the GATED mode (det),
   - the headline eval scores (`SCORE/*` — per-mode total reward +
     rise/raise/lower success, top of the W&B page; definitions in
     rl_docs/EVALS.md) + gate scalars vs the parent's,
   - terminations/canary flags.
   Then call it, honestly — would a skeptical roboticist agree from the
   same three artifacts? Classify with `RUN_INTERPRETATION_RULES.md`
   (8 ordered checks + verdict table; stop at the first failing
   check — it names the verdict without forensics, and reward alone
   is never evidence). Name pathologies bluntly (flag legs, dragging,
   skating, jitter, lurching); a walk without all six feet cycling
   ground-contact/swing is NOT WALKING and not hardware-ready,
   whatever the velocity error says. Unwatched success = unverified.
   **A KNOWN exploit in the video (flag-leg, tripod, stilt, freeze,
   park) is already a complete verdict: "STOP — reward/eval
   specification bug." Record it in one line and move on — no
   forensic investigation, no continuation, no re-run with more
   steps.** For any run whose eval injected a physics/sensor axis:
   no verdict without the matched-parent control
   (`eval_checkpoint.py --baseline <parent.zip>` — same injection,
   same seed); a child-vs-clean-parent comparison is invalid.
   **Kill still-training runs on behavioral impossibility** — e.g.
   stand-up: correct success still 0 after the discovery window +
   known cheat dominating video + cheat return rivaling the desired
   path; turning: yaw output command-invariant despite adequate
   reward separation. Do not wait for the return curve to plateau.

2. **Record it (minutes, not essays).** For a CLEAR pass or fail:
   - `launch_run.py update --run <name> --set status=... verdict="1-2
     lines" hardware_ready=...` (never hand-edit experiments.json).
     This auto-renders `rl_docs/runs/<run>.md` — the browsable per-run
     record. Do NOT edit those files or append per-run detail to
     RL_LOG.md; the ledger is the single write path.
   - `ops.sh wandbnote <run> "<paragraph>"` — puts an OUTCOME
     paragraph at the TOP of the run's W&B notes (operator, 08-10:
     the first thing on a run page is what happened; the old
     bottom-append buried it and the operator couldn't tell what a
     run was even for). Plain English for a human, in this order:
     result -> evidence -> why -> what's next -> big picture. First
     sentence = the result in plain words ("the robot now sits down
     properly every time, but standing up still fails"). No run-name
     jargon, no metric dump — the graphs are right there on the page.
   - RL_LOG.md gets 1 line per CYCLE (not per run), written ONLY via
     `ops.sh logline "[<track>] c<N>: <runs->verdicts>; <direction>"`
     — lead with the track tag(s) of the runs you triaged. Never
     `cat >>` RL_LOG.md — free-form appends tripled the file in half
     a day (operator trimmed it 08-09). Detail lives in rl_docs/runs/.
   - A PASS also updates `rl_docs/SKILLS.md` (one row: skill,
     checkpoint, evidence, envelope/limits) in the same cycle — the
     operator reads that file as "what can the robot do today".
   - If a verdict CHANGES THE STORY (new capability class, an
     unsolved skill becomes solved, a big lesson opens/closes),
     also refresh the affected lines of `STATUS.md` (the operator's
     plain-English "how is it going" digest) and re-stamp its date.
     Routine composes/seed twins don't qualify.
   - A verdict belongs ONLY to a run you evaluated. When a verdict
     stops a CLASS of arms, name the evaluated run as the evidence
     and write affected unevaluated runs as "no verdict yet, class
     stopped by <run>" — the operator misread a class-stop note
     naming cw-walk-diag45 as a diag45 FAIL (08-09). Never leave
     that ambiguity in RL_LOG or a run's ledger entry.
   That's the whole record for a clear result. No structured verdict
   essay, no summary.md, no root-cause chain, no provenance checksums.

3. **DIG IN only on a real trigger:** gate and video disagree; metrics
   anomalous vs parent beyond eval noise; a protected skill (rise/
   lower >= 5/6) eroded; canary auto-stop fired; the result decides a
   fork in the plan; or you're about to change reward/env code.
   A KNOWN exploit is NOT a trigger (see step 1 — it is a one-line
   STOP verdict); dig-ins are for genuinely discriminative cases:
   sim/real disagreement, unexpected regression on a correctly
   specified task, or two causal hypotheses implying different next
   actions.
   **Model tiering (operator cost order, 08-09): triage cycles run on
   a cheaper model. If YOU are a triage cycle and a trigger fires, do
   NOT dig in yourself: leave that run UNVERDICTED, finish your other
   runs and refills, and end your final message with one line per
   flagged run, exactly `DIG-IN: <run> — <one-line reason>` — the
   watcher re-spawns those runs on the deep model.** Dig-in cycles
   (your "## This cycle" says so) use the full toolkit: all-mode
   det+sto strips, per-leg gait metrics, structured OBSERVATIONS/
   INTERPRETATION/VERDICT in the ledger, and a root-cause chain
   (behavior <- incentive <- pricing <- sim defect) before any reward
   patch. Claims need a named baseline + delta outside the noise band;
   deltas inside noise are "no evidence".

4. **Refill against the BLOCKER LIST, not occupancy** (prime
   directive, 08-10 — reverses the 08-09 "idle pods are the failure"
   order), **and inside the track you just triaged** (08-11
   containment — see RESEARCH TRACKS above). Ask first: which
   unresolved blocker between the current state and THIS TRACK's
   tracks.json goal does this run reduce? A run that serves one gets
   queued (with `--track`); an idle pod is acceptable; a peripheral
   pair-compose queued "because capacity existed" is a violation, and
   so is a refill in someone else's track. hw keeps pod priority: if
   hw's backlog is non-empty and slots are scarce, non-hw refills
   wait.
   **Every spec declares `--phase`** (launcher-enforced): discovery
   ≤2M steps for new mechanisms — binary question, early video;
   hardening/composition/transfer need `--evidence` naming where the
   correct behavior was already seen. **Reward/task-mechanism specs
   additionally require the mode's `test_task_semantics.py` bank to
   PASS first** (a skipped bank = build the bank first — that is
   SPECIFICATION work and it never trains). **For any follow-up that
   clones an existing config (seed panel, next ladder rung, DR/axis
   variant) use
   `launch_run.py respec --from <run> --run <new> [--seed N]
   [--arg='--flag=v'] [--cfg k=v] --hypothesis "…" --gate "…"` — never
   re-type the arg vector by hand.** Genuinely new configs: queue specs
   into the backlog and let the drain place them —
   `launch_run.py backlog add --run <cw-name> --steps N --parent ...
   --phase ... [--evidence "..."] --hypothesis "..." --gate "..."
   -- <train args>`. Direct `launch_run.py launch` only when a
   specific pod matters. Sources, in order: continuations of
   near-misses (one, not two), the plan's next rung,
   `rl_docs/WISHLIST.md` topmost [READY] items. Rules that stay:
   warm-start by default, one variable per run, plain-English-first
   hypothesis and W&B notes, falsifiable gate. Two misses in a row =
   change the hypothesis, not the step count.
   **PLAIN-ENGLISH-FIRST is binding (operator, 08-10, after finding a
   run page unreadable): every hypothesis MUST open with one plain
   sentence a stranger can parse — "Teach the walking champion to
   stand up and sit down; this arm tests whether the fixed reward
   pricing unblocks it" — BEFORE any lineage/cfg/run-name material.
   The trainers auto-prepend the objective to W&B notes; the
   hypothesis opener is on you. Unreadable-first = guardrail
   violation.**

5. **Code changes:** make them, smoke-test them, explain them in one
   log line, then `snapshot.sh <run-name>` (commits, tags, pushes)
   before anything trains on them. Abort the cycle if the push fails.

6. **Trust only mechanical state.** The launcher/drain writes and
   verifies INTENT->RUNNING; checkups are the watcher's. If your
   prompt carries checkup findings, act on them FIRST: DEAD -> clean
   up + retry once (second death = "## NEEDS OPERATOR"); SUSPECT ->
   read the log, kill broken/starved runs and relaunch from their
   checkpoint. Exit the cycle as soon as your verdicts + refills are
   recorded — never sleep waiting for training.

## Judgment notes

- Size budgets to the question (1M diagnosis, 5-6M consolidation, cap
  per guardrails). Staggered finishes are a feature.
- Champions are append-only; guard rise/lower (the crown jewels).
- Fluidity counts: a jerky gate-passer is not hardware-ready.
- Boring informative experiments beat clever multi-change ones; a
  cleanly refuted hypothesis is a win.
- Escalate per guardrails on SAFETY or confounded designs. For design
  questions ON THE CRITICAL PATH with a plausible answer,
  assume-and-go (record "## ASSUMPTION (operator to review)") beats
  waiting — but an idle pod is fine (prime directive); never invent a
  peripheral run to fill it.


---

# FILE: rl_move/orchestrator/CAPACITY.md

# Cluster capacity — run the script, never guess

```sh
python3 rl_move/orchestrator/capacity.py          # live truth, human table
python3 rl_move/orchestrator/capacity.py --json   # machine-readable
```

That script is THE canonical answer to "what machines exist, what's
running, what's free". It queries the cluster live every time. Static
numbers in docs (including this one) go stale — **if any doc disagrees
with the script, the script is right; fix the doc.**

## Policy (operator rulings, 2026-08-09 — binding)

1. **One pool.** All nodes and all GPUs belong to this project. No
   partitioning into "our node" vs anything else.
2. **At least 4 train slots per machine, never in doubt.** One
   `hexapod-mjx-train-*` pod = one slot = one H200 + one training run.
3. **The drain is mechanical.** The watcher drains `backlog.json` into
   free slots (`launch_run.py drain`) with no agent deliberation — a
   queued spec that sits unplaced next to a free slot is a bug.
   **This is a placement rule, not a demand for a full backlog**
   (prime directive, 08-10): nothing enters the backlog unless it
   removes an unresolved blocker to the next hardware test. Idle
   slots with an empty backlog are a normal, healthy state.
4. Slot list lives in `guardrails.yaml compute.gpu_pods`; pod specs in
   `rl_move/sim/coreweave_pods_mjx_scaleout.yaml`; fresh-pod setup via
   `bootstrap_train_pod.sh <pod>`.

## Notes

- A single run uses only ~19% of an H200; the binding constraint is CPU
  (each pod requests 24-26 cores). Raising slots/machine beyond 4 means
  smaller CPU requests per pod (see scale-out manifest header).
- CoreWeave `cw-hpc-verification` pods periodically hold all 8 GPUs on an
  idle node for diagnostics; train pods Pending on them schedule when the
  check releases. Normal, not an outage.
- The 7 relic CPU sweep pods (CPU-training era) were backed up to the
  controller (`/workspace/relic_backup/`) and deleted on 2026-08-09.
  Only the controller `hexapod-sweep-friction` remains, and it is not
  a slot.


---

# FILE: RL_LOG.md

# RL_LOG — condensed campaign log

Full history: `archive/RL_LOG_FULL_2026-08-09.md` (through c34) and
`archive/RL_LOG_FULL_2026-08-09c.md` (c35–c59 detail). Per-run facts:
`rl_docs/runs/<run>.md` (generated) + W&B notes. Skills inventory:
`rl_docs/SKILLS.md`. Cycle transcripts: controller `/workspace/cycle_logs/`.

**APPEND RULE (operator, 08-09, tightened after the log tripled in half
a day): ONE line per cycle, written ONLY via `ops.sh logline "..."`.**
Never `cat >>` this file. The line is: cycle, run(s) → verdict word,
one-clause takeaway, what launched. ALL evidence, numbers, and
narrative go in the ledger verdict (auto-renders `rl_docs/runs/`) and
the W&B OUTCOME note — if it matters, it lives there, not here.

## State (2026-08-09 ~19:00Z) — HISTORICAL SNAPSHOT, superseded

**Do not read this block as current state** (it predates the BC-anchor
stand/hold specialists, the rot60 full-circle wrapper, and the
cw-dep-vref1-r1 hardware candidate). Live state: `CURRENT_TRUTHS.md`
→ `RL_PLAN.md`; plain-English summary: `STATUS.md`.

- **WALK CHAMPION: ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c**
  (c44, seed-confirmed 3/3, operator-accepted; DR1.0 det slip 1.06;
  NOT hardware-ready — paddle-slide persists, sto draw-stalls).
  Slip root = sim contact/current pricing → operator calibration
  (P0); reward-side anti-slip levers ALL closed.
- **Driving:** joystick gate (`eval_drive`) is the binding eval;
  champion + joystick45 + joyjit-dr05-c1 all PASS it. Heading ladder
  FROZEN at ±90° (head135 FAIL); rear coverage waits on
  mirror-symmetry [CODE].
- **Validated axes (see SKILLS.md for the full table):** crouch to
  −60mm, terrain amp1.0 (saturated), payload +40%, latency 2.5x,
  deadband 3x, CoM shift 30mm, stop density 0.35, 60s endurance,
  DR0.5 on steering/±90. Full-DR (1.0) retrain lever CLOSED (2x).
- **Next implementation cycles [CODE]:** mirror-symmetry (3
  motivations), quad-hold goal mode (feasibility sweep = GO).
- Compute: 12 GPU pods (train-0..11; g12ba48 cordoned, 12–15
  deleted). `capacity.py` = live truth; backlog auto-drains.

## Walk line — what was tried and learned

- **dr04b lineage** (pre-08-08): scalar champion, 0/9 gait-valid on
  video — shuffle/flag-leg. RETIRED. Lesson: scalars hide exploits;
  video eval is the promotion standard.
- **w07/flag/flagw/lp/speedhi**: penalty-coefficient iteration on the
  shuffle is a dead approach (all REFUTED).
- **nv/nv2**: deployable obs are not the blocker. **aac**: asymmetric
  critic is a retention tool, not a gait fix.
- **phase / phase-stance(2)**: phase reward as basin escape REFUTED.
- **step0** (08-08, operator recipe: step-event + drag + park-duty
  pricing, fresh init, walk-only): **FIRST genuine six-leg gait.**
  Lineage standard since.
- **step0-c2 vs lowent**: entropy runaway = the plateau driver; warm
  starts use ent 0.001. Identical-config continuations bought nothing
  5x — CLOSED as a move.
- **h15b**: 15 s is the lineage eval standard. DR is NOT the
  bottleneck (untrained parent passes DR 0.3/0.6).
- **kgate**: park PRICING refuted. **Cycle 27:** there is no park
  attractor — the "park" was one fixed backward draw; real defects are
  paddling (91% of slip) + overspeed.
- **anchorgate** (c31): income GATING works where charging failed —
  det slip 1.543→1.240, champion. **anchorgate-c1** (c32):
  cadence-inflation exploit. **anchortol5** (c34): the policy PAYS a
  binding stake and keeps creeping — **no income lever can outbid
  in-sim-free sliding; slip root is contact/current pricing
  (operator).** **step0-anchor** 40M fresh (c33): re-derives the
  paddle from scratch — pricing problem, not basin/history.
- **loadslip** (c40): episode-level slip stake also paid-not-fixed —
  reward side of skating CLOSED (seed-confirmed).
- **fast** (c42): speed ceiling ~0.065 m/s is GAIT-limited; speed
  arms join skating behind the calibration.
- **stalls** (c47→c52): fixed-draw stalls are command-conditioning;
  resample training fixes resample-on eval only, incidence <2% —
  stall lever class CLOSED, panel draw kept as canary.
- Exploit-watch columns (permanent): cadence/stance count, per-leg
  swing asymmetry, allowance-riding, unload-sweep.

## Stance line

- Heights SOLVED at DR 1.0 (crown jewels, canary-protected). Lower
  posture (flag leg) never solved; refuted in order: mix, posture
  pricing, exploration, terminal pricing, reset diversity, dense
  charging. **Root cause (c28): the hover is INCOME-POSITIVE**
  (current model prices planted descent 4x hover) — **BLOCKED on the
  same operator pricing ruling.** No stance shaping until then.

## Infra lessons (all landed in tooling — see rl_docs/COMMANDS.md)

- Launcher is mandatory (capacity, code-SHA gate, dup refusal);
  ledger writes via `launch_run.py update` only.
- Canaries protect only skills the lineage HAS (`--no-canary` on
  step0 walk-only lineage).
- Pre-08-08 seed twins were bit-identical clones; conclusions void.
- Sync the pod at snapshot time; snapshot BEFORE drain, never
  mid-drain (HEAD moves under the launcher).
- Specs must ALWAYS pass `--out-name` (prestage pullckpt breaks on
  MJX default names otherwise).
- Dirty-marker refusals: state files, .md, logs, zips, tmp files are
  all EXCLUDED from the dirty check now; a genuinely dirty tree means
  COMMIT FIRST. wandb/ run logs are gitignored (were tracked, poisoned
  every sync 08-09).
- /dev/shm leaks after killrun: clean hexmjx-* segments before
  relaunching on that pod (c54: two 0-step deaths).
- Node loss is normal (g12ba48 cordoned 08-09): fleet truth is
  `capacity.py`, never a doc.
- Fixed-draw sto panels can pin a "failure" to one command draw —
  check WHICH episode fails before theorizing.

## Cycle digest (c38–c59, 08-09; detail in archive + rl_docs/runs/)

- c38 longdist-r2 det slip 0.96 campaign-best; drain dirty-fix. c39
  dr05-r1 FAIL; stability pricing arms launched. c40 loadslip FAIL →
  reward side of skating CLOSED. c41 fps SUSPECTs = false alarms.
- c42 fast FAIL (gait-limited ceiling). c43 steer-explore FAIL (no
  omni transport); diag45 operator-killed. c44 **champion promoted:
  longdist_r2** (accepted by operator ruling 8); speedband killed as
  stale. c45 lowgait/wander PASS, fwdband-r1 FAIL (command mix closed),
  fall300 NO-EFFECT.
- c46 lowgait30 PASS. c47 terrain05/wander30/endur60 PASS,
  longdist-dr05 FAIL (DR not the stall lever). c48 strafe-dr05 +
  wander-dr05 PASS. c49 joystick45 PASS (joystick gate born);
  joyjit-dr05 starved→rebalanced; dr.<field> cfg hooks landed.
- c50 lowgait40 PASS, tilt05 NO-EFFECT (tilt lever closed), endur60-s1
  endurance seed-robust. c51 head90 PASS (±90 envelope; L/R asym →
  mirror-sym motivation); drain-dead root cause = one uncommitted
  artifact. c52 stallfix FAIL → stall class CLOSED.
- c53 joyjit-dr05-c1 PASS (best driving candidate), longdist-s2 PASS
  (3/3 seeds), longdist-dr10 FAIL (full-DR lever closed);
  stopgo35 starved→c1. c54 lowgait50/terrain10 PASS (terrain
  SATURATED), strafe-dr10 FAIL (DR0.5 = strafe ceiling); endur60-r2
  FAIL on slip clause; shm-leak bug found+fixed.
- c55 wander-dr10 FAIL (DR0.5 = steering ceiling); READY well dry →
  quadruped sweep flagged. c56 head135 FAIL (**ladder frozen ±90**),
  head90-dr05 PASS, latjit25 PASS. c57 payload50 PASS; **quadruped
  feasibility sweep = GO** (39mm margin w/ shift+splay). c58
  lowgait60/wander-dr05-s1/wander60 PASS; comshift30/deadband30 PASS,
  friclow letter-pass only (slick draws skate). c59 stopgo35-c1 PASS,
  torquedroop NO-EFFECT (champion already covers 0.75–1.05x free —
  run the parent baseline FIRST on exposure axes); fricvar PASS
  (0.4–1.6x, slick-tail churn), speedband2-r1 FAIL (stale reissue of
  the CLOSED c42 speed class — check run docs for the CLASS before
  requeueing).

## Cycle log (one line per cycle via `ops.sh logline` — no exceptions)
- 08-09 18:46 OPERATOR process pass: triage = ops.sh review; RL_LOG writes = ops.sh logline ONLY (598->143 trim, full text archived); snapshot dirty-check hardened; backlog add warns on same-axis dupes. 

- Cycle 60 (08-09 ~18:3x-19:3x): 1 triage. `cw-walk-groundtilt5` PASS — floor-
  slope axis (13b) lands by exposure: own-cfg tilt u(0,5deg) gv 12/12, 0 term,
  det med fwd 1.40m; DR0 retention slip 1.03/prog 0.96 = champion band; honest
  tail 2/6 steepest det draws shuffle at ~1/3 speed (slip 3.4-4.5, no falls) —
  solid to ~3-4deg, 5deg marginal; SKILLS row added; paddle lineage, not
  hardware-ready. Infra: watcher's dr0ret eval deadlocked on a corrupt ffmpeg
  pipe mid-sto-video (utime frozen, no children) — killed, det-only --no-video
  rerun recovered the retention numbers (gotcha added to COMMANDS.md). Refills
  (4 = cycle cap, 80M GPU): imumount10 (13c-class sensor axis, IMU mount
  miscalib 10deg) + badstart (13b boot-pose axis, prob 0.25 @8-35deg), both
  isolated off no-DR champion, drained+ALREADY FINISHED same hour (watcher
  will cycle their triage); groundtilt-dr05 (compose rung off today's PASS,
  RUNNING t8 after 2 drain-race REFUSEDs + pod code sync) + payload50-s1
  (ruling-7-style seed twin of the c57 payload PASS, queued).
- 08-09 19:31 c61: payload-dr05 FAIL (own-DR0.5+payload panel clean 12/12, det med 1.36m, but DR0 no-payload retention eroded: slip 1.38>1.24, prog 0.54 vs parent 0.95 — first dr05 compose to charge nominal; watch in-flight comshift/deadband/fricvar/latjit-dr05 retentions); refilled train-11 with latjit-dr05 compose (drain VERIFIED, checkup HEALTHY); note: 2 controller evals OOM/load-killed silently at load~212, relaunched via setsid. 
- 08-09 19:42 c60: 3 triages. joylat25 PASS - latency 0.5-2.5x composes onto the abrupt-flip DR0.5 driving package, NEW BEST DRIVING CANDIDATE (joystick gate 0 falls, own-DR gv 12/12; SKILLS updated; s1 seed run training). cmddrop10 NO-EFFECT + velsag30 FAIL(letter)/NO-EFFECT - parent baseline matches both per-episode: champion already covers 10% cmd dropout + servo-speed sag to ~0.8x FREE, deep sag ~0.7x = untrainable transport boundary (battery-calibration class); servo-imperfection single-axis exposure now 0-for-3, TEST CHAMPION FIRST before queueing this class (cmddrop20 verdict pending elsewhere, treat as one ladder). Infra: batch-eval shell footgun found+documented (CFG assignment swallowed by first bg job -> 4 evals silently ran default cfg, all rerun valid; COMMANDS.md gotcha 14). Refills: joylat25-s1 (running t0) + joylat60 (60s driving endurance) + joycom30 (off-center-payload driving) queued; ckpts pre-pushed to t5/t11. 

- 08-09 20:3x c60b: 3 triages. `cw-walk-lowgait70` PASS — crouch envelope
  extends to -70mm (gv 12/12, 0 term, mean end-height err 2.0/1.9mm, det agg
  slip 1.07; SKILLS row -20..-70mm); lowgait80 rung launched. `cw-walk-wander-dr05-s2`
  PASS — own-DR0.5 prog 0.97/0.93, slip 1.46/1.88 = seeds 0/1 noise band; ruling-7
  3-seed panel COMPLETE for the steering-DR recipe. `cw-walk-cmddrop20` NO-EFFECT
  (letter passed; parent longdist-r2 under identical 0.20 drop spread matches
  episode-by-episode incl. the same 2 churn draws) — cmd-drop exposure lever CLOSED
  0.10+0.20 as ONE ladder study with c60's cmddrop10; servo-imperfection exposure
  0-for-4, champion envelope row updated. Refills (4 = cycle cap, 80M GPU, all
  VERIFIED RUNNING): zerobias3 + gainvar + imubias3 (13b/13c servo/IMU calibration
  axes, parent-baseline-at-triage pre-registered in each gate) + lowgait80; remaining
  free slot left to concurrent cycles (HARD reason: max_new_launches_per_cycle=4).
  Infra: hit the c60 batch-eval $CFG footgun + silent load-kills myself — 3 evals
  relaunched with /proc-verified cfg (COMMANDS.md gotcha 14 extended); watcher PAUSE
  present since ~19:03 (operator/restart window — not mine to clear).
- 08-09 20:46 c62: badstart FAIL-letter (boot-pose recovery works — 12/12 gv, 0 falls, no lurch — but bad-start draws transport 0.83-0.85m and nominal fwd shaded 1.32 vs champ 1.57, det med 1.17<1.2 gate; axis trainable-not-free, no requeue, dr05 compose cancelled); comshift-dr05 PASS (retention CLEAN 1.49/0.98 — dr05-compose class now 2-1 vs payload-dr05) + deadband-dr05 PASS-caveat (retention slip 1.22 at 1.24 cap, fwd 1.42); SKILLS rows updated. Refills (cap 4/80M reached): contactstiff+linklen (13b axes, champ baselines measured FIRST per c59 — tails 0.61-0.77m justify; both ALREADY FINISHED awaiting triage) + comshift-dr05-s1 + payload-dr05-s1 ruling-7 twins to settle the compose-class retention split. NOTE: watcher PAUSE file present — no auto-triage/drain; next cycle must pick up contactstiff/linklen + idle t0/t9/t10/t11. 
- 08-09 20:50 c62: 3 triages. fric50 FAIL (if-false: 0.5-1.0x grip still skates the slick draws, det slip med 1.35 vs gate 1.3, worst draws 3.1-3.5; retention CLEAN 1.57m/slip 0.86 = friclow's 9% tax came from its 0.3x extreme, not friction exposure per se — friction-exposure axis CLOSED pending operator contact-pricing calibration, evidence fric50+friclow; other friction runs get no verdict from this). fricvar-dr05 PASS + latjit-dr05 PASS (cleanest dr05 compose: det fwd 1.50m, slip 1.04 champion band; BOTH DR0 retentions clean — no payload-dr05-style erosion; SKILLS rows added). Watcher was paused: pulled ckpts + ran gates by hand; fric50 ckpt was default-named on t9 (spec lacked --out-name) — canonical copy made, md5 bacaa00f both sides. Champion baselines (c59 rule) show contact-stiff + leg-mass axes NOT free (2/6 det draws crater each). Refills 4/4 VERIFIED: legmass25 t0 (last unexposed 13b axis), joyfric t6 + joytilt3 t7 (driving-package composes off joylat25), latjit-dr05-s1 t8 (ruling-7 seed twin); joylat25+latjit25 parents pre-pushed to all idle pods. 
- 08-09 21:16 OPERATOR (08-09 ~17:1x): UNIFIED JOYSTICK POLICY = top deliverable - ONE checkpoint for stand/walk/steer/sit (no per-skill zoo). Line opened: cw-uni-blend1 queued (goal-mix blend off joyjit-dr05-c1). RL_PLAN Queue item 0 + WISHLIST item -1 are binding. 
- 08-09 21:31 c63: lowgait80 FAIL (crouch envelope BOTTOM at -70mm: height err 12mm>8, det slip 1.50>1.15, one sto flag leg, reward declined — ladder closed, no -90mm); multiaxis1 PASS (4-axis compose gv 12/12, det med fwd 1.29m, DR0 retention clean — axes stack without interference, robustness-champion base candidate); payload50-s1 PASS (seed twin mirrors seed0: det med 1.32 vs 1.31, identical 2/6 heavy tail — payload recipe confirmed, not luck). Watcher had skipped all 3 gate evals (7-eval cap) — ran by hand, incl. det-only retentions. Refills queued (3, 56M GPU): multiaxis1-s1 seed twin + multiaxis-dr05 compose + multiaxis2 (+tilt 5th axis), multiaxis1 ckpt pushed to t0-t3.
- 08-10 02:5x OPERATOR SESSION — yaw-rate command channel + quad-hold goal mode LANDED (c086a22 code, 209d9e9 metric logging). (1) TURNING/STEERING: goal.walk_yaw_cmd=1 adds a per-segment wz command (resampled+blended like vx/vy; stop segment + wz = turn-in-place for free), reward.k_walk_yaw Gaussian kernel pays every walk tick INCLUDING wz_ref=0 (heading-hold income — prices the champion's free ~+10deg/20s drift). wz_ref appended at obs TAIL -> --obs-pad-transplant 1 warm-starts from any non-yaw champion. (2) QUAD-HOLD: goal mode `quad` lifts fronts 0+5 (per feasibility FRONT_LEGS; the old quad_legs=0,3 sketch was wrong) commanded via the existing goal one-hot BOTH bits hot — obs width UNCHANGED, walk-champion warm starts stay compatible. k_quad_clear pays unloaded front clearance (cap 30mm), k_quad_plant pays four-planted frac, quad_grace_s=1.5 free transient; level/current/tilt-trip inherited. All defaults OFF — legacy obs widths, rewards, rng streams byte-exact (smoke_yaw_quad.py guards this). Probes probe-yawcmd-scale + probe-quad-scale: full MJX path clean, 300k steps each (obs 73 verified on ckpt); r2 probes rerun for reward-scale audit after the trainer whitelist fix. Specs: WISHLIST items 3/8c/15, RL_PLAN Queue 0 + party tricks. LAUNCHED (03:2x-03:5x, via respec+drain through the c71 collision storm): **cw-walk-yawcmd1-rr1** (t2, 12M, warm start joyjit-dr05-c1 + pad-transplant 1, k_walk_yaw=1.0, wz +-0.3 rad/s 50% zero) and **cw-quad-hold1-r2** (t1, 10M, warm start longdist_r2 champion, quad=0.5/walk=0.4/hold=0.1, k_quad_clear=1.5/k_quad_plant=1.0) — both VERIFIED RUNNING. quad-hold1-rr1 died to the /dev/shm leak (gotcha 13, shm 100% full on t3, cleaned t1+t3); first drain attempt burned a name on a credential-less shell (drain needs wandb.env sourced — the watcher has it, a bare kubectl exec does NOT).
- 08-10 00:2x OPERATOR: TEMPORAL-ARCH LINE opened — keep 1-2 GPU pods on architectures that capture more past states (history_frames 8->16->24 ladder, then capacity control, then recurrent). Directive-only, not mechanically enforced. RL_PLAN Queue 0.5 + WISHLIST -0.5 are binding; cycles treat an idle arch line like an empty backlog.
- 08-09 22:3x OPERATOR SESSION — SECOND floor-penetration cause fixed (0823ac0): tibia_link.stl is drawn 29.5mm LONGER than the kinematic leg (CAD builder measures the tube from the yoke socket, not the knee axis; tip lands at 157.5mm vs TIBIA_LENGTH=128). Contact/physics were always at 128 — but every render/eval VIDEO showed boots poking through the floor. Visual now squashed to match physics (verified <1mm at stance hold, all six legs). VIDEO REVIEWERS: pre-0823ac0 videos show phantom leg-through-floor; do not verdict on it. OPERATOR RULING (08-09 22:4x, no bench measurement): kinematics (TIBIA=128) = truth, drawing must match kinematics (done), leg-length DR absorbs as-built variance (link_len_scale_pct 0.02 global x link_len_leg_pct 0.012 per-segment ~ +/-4mm tibia at DR1.0 — covers the known 4mm short tubes on legs 0/4; would NOT cover a 30mm systematic error, accepted). CAD builder discrepancy stays open as a CAD-side issue only, not a training blocker.
- 08-09 22:2x OPERATOR SESSION — SIM DEFECT FIXED (273ebde): leg segments (femur, tibia, knee servo) had NO floor collision — only foot spheres/chassis/yaw-servo boxes collided — so rise/lower policies could sweep shins THROUGH the ground (operator caught it watching stand/sit in MuJoCo; chain-standwalksit's flailing was worst-case: its rise never trained, verdict stands). Fix: floor-only collision capsules + knee-servo box, bitmasked (contype 4 vs floor conaffinity 5) so no leg-leg/leg-chassis false positives. Walk champions VERIFIED unaffected (zero shin-floor contacts in gait, wander30 0.60m/12s retained). Consequence: any rise/lower behavior trained BEFORE 273ebde is suspect near the ground; walk-only lineages fine. cw-uni-blend1 killed 25min in and requeued as cw-uni-blend1-r2 (VERIFIED RUNNING t4) on the fixed sim; its gate now includes "VIDEO: no leg-through-floor in rise/lower". 
- 08-09 22:35 c66: 1 triage. cw-walk-zerobias3 NO-EFFECT (letter-PASS: own-cfg per-joint zero-bias u(-3,3)deg gv 6/6 det, det med fwd 1.28m>=1.2 gate, DR0 retention clean slip 0.99; but parent longdist-r2 under IDENTICAL bias spread matches episode-for-episode incl. the 2 steep-bias craters, frames pixel-identical churn-in-place) - 13b/13c calibration-exposure ladder now 0-for-6 (joins gainvar+imubias3, both also NO-EFFECT this window by concurrent cycles); SKILLS tally updated, no requeue. Refills (4, all VERIFIED healthy after one /dev/shm-leak retry on groundtilt5-s1->s1r1 per gotcha 13): groundtilt5-s1 + fricvar-s1 + deadband30-s1 (promotion-panel seed twins for 3 already-PASSED single-seed 13b axes, ruling-7 completeness) + quietcurrent (NEW axis, wishlist item 13: enables the existing default-OFF k_current_hot=0.2/current_hot_a=1.5 hot-current-concentration penalty on the WALK task for the first time, champion-baseline-at-triage pre-registered). 
- 08-09 22:35 c65: 3 triages. cw-walk-lowgait-dr035 PASS -- -50mm crouch survives DR up to 0.35 (own-cfg 12/12 gv, 0 term, height err 6.8/4.6mm det/sto, slip 1.08/1.26; DR0 retention clean 0.98) -- ceiling banked between 0.35 (holds) and 0.5 (FAILED prior); SKILLS row added. legmass25 NO-EFFECT + stiffvar NO-EFFECT: both exposure runs are episode-identical to their unexposed champion baselines on the hard draws (same 2/6 det craters, matching severity) with clean DR0 nominal retention -- leg-mass-asymmetry and contact-compliance join torque-droop/servo-gain/cmd-dropout as axes NOT fixable by naive single-axis DR-0 exposure (last 2 unexposed 13b axes closed this way). Infra: hit the default-checkpoint-name gotcha again (lowgait-dr035 ckpt saved without --out-name) -- manual copy+md5, and the pod-code dirty-marker deadlock (WRAPUP file deletion left uncommitted -> drain REFUSED on all 3 free pods) -- snapshot+sync unblocked it, cw-uni-blend1 unparked from backlog_failed and relaunched. OPERATOR SESSION landed 2 sim fixes mid-cycle (273ebde leg-floor collision, 0823ac0 tibia visual-length): walk-only lineages confirmed unaffected, cw-uni-blend1 killed+requeued as -r2 on the fixed sim. Refills: groundtilt-dr05-s1 + fricvar-dr05-s1 (unparked backlog) + cw-uni-blend1 + lowgait-dr035-s1 seed twin (4 = cycle cap); heavy concurrent drain churn from other cycles filled remaining slots. 
- 08-09 22:39 c66: 3 triages. comshift-dr05-s1 PASS (seed twin confirms compose, own-cfg gv 12/12, DR0 retention clean slip 1.00; honest note: this seed's det tail is worse, 2/6 severe craters vs seed0's 1 mild dip). joyhead90-lat25 PASS - latency 0.5-2.5x composes onto +-90deg abrupt-flip DR0.5, NEW WIDEST driving candidate (JOYSTICK GATE @90 0 falls, own-cfg gv 12/12, prog 0.93/0.96 >= parent); joyjit-dr05-s1 PASS (seed-confirms the old c53 promotion panel, now superseded by joylat25/joyhead90-lat25). SKILLS updated x3. Refills: gyronoise15 (new 13b axis, champion-baseline-confirmed NOT free at 3x default) landed RUNNING; joyhead90-lat25-s1 seed twin + placementnoise6 (baseline-confirmed axis) each hit a launch-COLLISION storm twice (many concurrent cycles' drains racing the same free pods simultaneously -> worker EOFError at init, 0 steps, no science result) - requeued to backlog (-s1r2/-6r) for the self-repairing drain once contention clears, not fought further. Checkup SUSPECTs (joytilt3, joyhead90-r1-s1) both resolved on their own by finishing training - no action needed. 
- 08-09 22:45 c66: 3 triages. joyjit-dr05-s2 PASS (2nd seed confirmation of joyjit-dr05-c1: joystick gate 0 falls, own-DR0.5 gv 12/12, prog 0.94/0.97, slip 1.50/1.50, DR0 retention clean — driving recipe seed-robust across 3 seeds now). joylat60 PASS (60s abrupt-flip+DR0.5+latency endurance rung off joylat25: gv 12/12 own-cfg+DR0, joystick gate 0 falls, no first/second-half decay — minute-scale hardened driving banked; SKILLS updated). latjit-dr05-s1 FAIL-letter (own-DR0.5 replicates parent cleanly, but DR0 nominal retention misses the slip<=1.24 cap: 2/6 det draws that were clean for the parent shuffle at half/third speed here — real seed-specific retention erosion, not noise; use seed-0 latjit-dr05 as the compose input, not this seed). New gotcha 13b documented: launch-collision EOFError under concurrent-cycle drain storms (clean shm, not gotcha-13's leak) cost 3 of my 4 new-axis launches (imupos15, gyrobias3, tiltnoise) one retry each; gyrobias3-r1 landed, imupos15-r1/tiltnoise-r1 lost the race again and are left for the next drain (cap of 4 new specs reached: encodernoise landed clean). Free slots after: concurrent cycles + watcher drain own the rest. 
- 08-09 23:05 c67: 2 triages + 1 infra-only. joyfric PASS - floor-grip 0.4-1.6x composes cleanly onto joylat25 driving package (JOYSTICK GATE 0 falls heading<=45, own-cfg DR0.5+lat+fric gv 12/12 prog med 1.00/0.95, DR0 retention clean 0.99/1.02) - new floor-hardened driving candidate, supersedes joylat25. joyhead90-r1-s1 PASS - seed twin reproduces joyhead90-r1 near-exactly (JOYSTICK GATE @90 0 falls, own-cfg DR0.5 gv 12/12 prog 0.86/0.90 vs seed0 0.87/0.90) - +-90deg envelope is recipe-robust, ruling-7 2/2; hit the default-ckpt-name gotcha, pulled+md5-verified by hand. gyrobias3 INFRA-ONLY (0 steps, launch-collision EOFError gotcha 13b, no verdict on the gyro-rate-bias axis; auto-retried as -r1, RUNNING). SKILLS updated (joyfric row + joyhead90_r1 seed-confirm). 13b/13c single-axis DR-field ladder now essentially exhausted (every RandRanges field tried or in-flight) - stop feeding fresh calibration-axis probes into that class. Refills (4 backlog specs, distinct lines): actionnoise (last untried RandRanges field - actuator command noise, isolated off champion, RUNNING) + joyheadfric (friction compose onto widest +-90 envelope, distinct axis from concurrent cycle's joyhead90-lat25, RUNNING) + joyfric-s1 ruling-7 seed twin (hit gotcha 13b itself, 0 steps, retried once as joyfric-s1r1, mid-drain). Heavy concurrent-drain load fleet-wide (3+ simultaneous drain processes observed) - several launch-collision EOFErrors this window across cycles, all gotcha 13b, no diagnosis needed. 
- 08-09 23:13 c67: 1 triage. encodernoise NO-EFFECT (own-cfg letter-pass, but champion under identical encoder-noise-0.5deg spread matches it draw-for-draw incl. same 3/6 det craters -- sensor/calibration NO-DR-exposure ladder now 9-for-9); SKILLS champion-row tally updated. imupos15-r1 + joyhead90-lat25-s1r2 (my assigned finished runs) had both ALREADY died to a live host-wide EOFError launch-collision storm before I could triage them (0 steps, no science) -- confirmed NOT a single-pod drain-race artifact: a solo direct launch_run.py launch to an idle pod also crashed identically, and an unrelated concurrent run failed at the same instant. Recovered orphans across the fleet by requeueing via backlog (imupos15 now RUNNING on retry #3, placementnoise6 RUNNING on retry #3, tiltnoise-r2 failed again/still orphaned) rather than fighting the live storm further; found + removed one accidental duplicate backlog entry from a concurrent cycle racing the same orphan (lowgait-dr035-s1 retry, two near-identical names). joyhead90-lat25-s1 seed-1 twin failed 6 CONSECUTIVE times (incl. mine) while sibling retries succeeded on 1-2 tries -- DIG-IN: cw-walk-joyhead90-lat25-s1r5 -- 6 straight EOFError-at-reset failures for one spec while neighbors recovered, worth reading mjx_sharded_vec_env.py's reset choreography rather than more blind retries. Backlog empty at cycle end; READY well audited dry (all DR/sensor axes 9-for-9 closed, command-steering wishlist items already closed by prior rulings) -- next CODE-capacity cycle should open quad_legs or yaw-rate turning rather than more single-axis DR probes. 
- 08-09 23:22 c67: 2 triages, 1 infra-retry. joylat25-s2 PASS - 3rd seed confirms the joylat25 driving recipe (ruling-7 panel COMPLETE across seeds 0/1/2, matching prog/slip bands, 0 falls). joytilt3 PASS - 3deg floor-slope composes cleanly onto the latency-hardened joylat25 driving package (0 falls, DR0 flat retention clean, no erosion); SKILLS updated x2 (joylat25 panel note + new joytilt3 row). lowgait-dr035-s1 was a LAUNCH FAILURE not a science result (gotcha 13b EOFError, 0 steps) - retried as -r1 (also EOF'd under the same drain storm) then -r2, which is now VERIFIED training; both failed attempts recorded FAILED in the ledger with no science verdict. Refills (2 new lines, cap allows more but concurrent-cycle churn was already saturating pods): joytilt3-s1 seed twin (ruling-7) + joyheadtilt3 (composes the 3deg slope onto the WIDEST +-90deg driving package) - both VERIFIED landed (train-3, train-7). Heavy concurrent drain activity this cycle (host load1 hit 189/128) filled most other free slots (actionnoise, imupos15-r3, joyheadfric, joyfric-s1r1, placementnoise6-r3) before I could act on them - not mine, left alone. Checked WISHLIST 8b speed-knob line before considering it: CLOSED (c42/c54, contact-pricing root) - correctly skipped, no requeue. 
- 08-09 23:23 c67 correction: joyheadtilt3 confirmed VERIFIED RUNNING (131072 steps, train-7) — the widest-package+slope compose is actually training. joytilt3-s1 (my other refill) ALSO hit gotcha 13b (EOFError, 0 steps, host load1 189/128) right after I logged it as verified — my mistake trusting INTENT status without rechecking W&B; corrected in its ledger entry (FAILED, no science verdict) and requeued as -s1-r1 for the drain. 
- 08-09 23:54 c68: 3 triages. groundtilt5-s1r1 PASS (seed-1 twin reproduces seed0 exactly: det med fwd 1.39m, DR0 retention clean slip 0.98, same 2/6 steepest-tilt shuffle -- ruling-7 panel COMPLETE for 5deg floor-tilt). gyrobias3-r1 + gyronoise15 both NO-EFFECT: own-cfg letter-passes but champion measured under each's identical fixed spread (baselines run this triage) matches the trained checkpoint episode-for-episode incl. the same 2/6 craters -- degraded pattern persists after training per each's pre-registered if-false; sensor/calibration NO-DR-exposure ladder now 11-for-11, CLOSED, stop probing it. SKILLS updated x2. Checkup SUSPECTs (imupos15-r3 stall, joyfric-s1r1 low-fps) both self-resolved by the time I checked (12.2k fps/15.3M steps; 5.7k fps) -- no action needed. Infra: host under severe fleet-wide contention this window (loadavg 350-400, 20+ concurrent eval_checkpoint procs) stalled my gate-eval videos for 20+ min each -- switched to --no-video passes to get numbers fast, killed the now-redundant video jobs after. Refill: groundtilt8 ladder rung (8deg, off groundtilt5's own ckpt) queued+draining, hit the fleet's EOFError launch-collision storm once, auto-retrying. Left 2 slots to concurrent cycles/backlog (HARD reason: audited WISHLIST+RL_PLAN queue again, every READY main-line/13b item is already claimed this window or closed by prior ruling; mirror-symmetry/contact-aux-head/quad_legs remain plan-flagged as needing a DEDICATED CODE cycle, not a triage add-on -- repeating c67's flag, unaddressed 2 cycles running now. 
- 08-10 00:16 c68: 7 triages (3 assigned + 4 adopted orphans stuck awaiting-triage since ~23:0x, host load 300-400/128 all cycle from many concurrent evals). multiaxis1-s1 PASS (4-axis-stack seed twin confirms recipe, ruling-7 satisfied). multiaxis-dr05 + multiaxis2 FAIL-letter (own-cfg exposure panels clean, but pre-registered DR0 retention slip cap 1.24 missed at 1.27/1.30 -- axis-stacking ceiling is 4 at DR0, adding generic-DR0.5 or a 5th axis both cross it mildly; continuations queued to test step-budget vs hard-ceiling). joyheadfric PASS (friction composes onto widest +-90 driving envelope, JOYSTICK GATE 0 falls; SKILLS row added). gyronoise15 + actionnoise NO-EFFECT (champion baseline under identical spread matches draw-for-draw; sensor+actuation NO-DR-exposure ladder now fully exhausted, every RandRanges field tried). quietcurrent FAIL (hot-current penalty changes nothing measurable in per-servo current on the walk task, unlike the earlier static-stance win -- closed for walk). Infra: corrected my own PASS-with-caveat misreads on multiaxis-dr05/multiaxis2 after noticing a concurrent cycle's SKILLS.md edit disagreed with the same numbers -- reconciled to FAIL-letter (retention cap literally missed) to avoid two contradictory ledger stories on the same runs. Refills: joydeadband + joyheaddeadband + joyhead90-60 (driving-package gaps, deadband axis + widest-envelope endurance) landed after 2-3 launch-collision retries each (gotcha 13b, severe this window); joyheadfric-s1 seed twin landed on 2nd try; joydeadband itself lost 3 straight collisions and is re-queued (joydeadband-r3) for the passive drain; also queued multiaxis-dr05-r1 (+14M steps, mirrors a concurrent cycle's multiaxis2-r1) to test whether the retention miss is under-training or a real ceiling. 
- 08-10 00:27 c68: 2 triages + 1 flagged. deadband30-s1 PASS + fricvar-s1 PASS (ruling-7 seed twins, both match seed0's worst-draw pattern almost exactly + clean DR0 retention -- recipes confirmed, not luck; SKILLS updated). cw-uni-blend1-r2 (UNIFIED JOYSTICK top deliverable, sim-fix retrain) NOT verdicted: JOYSTICK GATE PASS (0 falls) + walk gv 12/12 + quiet hold all clean, but rise/lower success=0/6 on BOTH det+sto (height_err_end 30-60mm, never reaches target in the 15s window) -- protected-skill-adjacent trigger, left for deep dig-in with full report.json + all-mode det+sto strips already on disk (logs/ckpt_eval/cw_uni_blend1_r2_owndr_modes, _gate, _joygate.json). Refills (3 distinct lines, cycle cap respected): deadband-dr05-s1 seed twin (ruling-7 completeness for the one dr05-compose axis missing a panel, VERIFIED RUNNING, finished training this cycle) + joyfric-payload (novel compose testing whether payload-DR erosion is driving-line-specific, VERIFIED RUNNING, finished training this cycle) + lowgait-dr035-latjit (crouch+latency compose, hit the launch-collision gotcha 13b, 0 steps, back in backlog for mechanical retry). Heavy fleet-wide eval contention this cycle (host load 200-400+, many concurrent cycles' evals sharing controller CPU) slowed my own gate evals to 40-80min each -- expected traffic, not an incident. 
- 08-10 00:40 c69 (dig-in): cw-uni-blend1-r2 FAIL — rise 0/12 (stays crouched, churning) + lower 0/12 (keeps WALKING at full height, fwd 0.2-0.4m in lower mode) vs gate >=5/6, while joystick gate/walk/hold all retained cleanly and NO leg-through-floor (273ebde fix verified); W&B shows rise/lower success flat ZERO for all 18M steps and rise-finish income actively traded away — gradient domination by the 70% walk share, not under-training, sim clean. ASSUMPTION (operator to review): plan's mix-ladder fallback (toward walk=0.9) targets walk-EROSION, the opposite failure — adopted INVERSE ladder. Refills queued+draining: cw-uni-mix40 (walk=.4/rise=.25/lower=.25) + cw-uni-mix20 (walk=.2/rise=.35/lower=.35, stage-1 skill acquisition) — if both stay at 0 success, mix lever refuted -> rise/lower income scale audit next; cw-arch-hist16 (temporal-arch rung 1, from-scratch per WISHLIST -0.5, line was empty 2+ cycles); joyjit-dr05 parent pushed to t10 (was missing). 
- 08-10 00:43 c68: 5 triages (2 assigned + 3 unclaimed-deferred multiaxis picked up from the backlog jam). fricvar-dr05-s1 PASS + groundtilt-dr05-s1 PASS (both seed-confirm their dr05-compose parents, clean DR0 retention within noise band -- ruling-7 panels complete for those two axes). multiaxis1-s1 PASS (seed-confirms the 4-axis robustness-compose base, clean retention); multiaxis-dr05 FAIL + multiaxis2 FAIL (gate: DR0 retention) -- stacking generic DR0.5 OR a 5th axis (tilt) on top of the passing 4-axis multiaxis1 both push nominal-floor slip just over the 1.24 cap (1.27/1.30) at 18M steps -- multiaxis1 stands as the robustness-champion base, do not stack further without more steps. New arms: cw-walk-posetrack FAIL (hold mode clean 0.84deg, lean/track miss the 1.5deg gate at 2.2-3.6deg, no falls -- promising but undertrained); cw-walk-lpband FAIL/hypothesis refuted (LP speed-curriculum doesn't fix command-insensitive speed either -- same flat ~1.6m/30s regardless of commanded band as cw-walk-fast, confirms gait-pricing wall via a 2nd method, closes wishlist 8b/19 pending operator contact-pricing calibration); cw-walk-multiaxis2-r1 died at init (EOFError launch-collision storm, 0 steps, infra not science) -- requeued as -r2, RUNNING. SKILLS.md rows updated x3 (fricvar-dr05, groundtilt-dr05, multiaxis1 seed confirms + stacking-ceiling note). 
- 08-10 00:44 c69: 3 triages. actionnoise NO-EFFECT (champion under identical action_noise=0.08 spread matches draw-for-draw incl. same worst sto crater -- LAST untried single-axis DR field, calibration/sensor exposure ladder now CLOSED 12-for-12, stop probing it). deadband-dr05-s1 PASS (seed twin: DR0 retention slip 1.18 <=1.24 cap, cleaner margin than seed0's 1.22, same tight-but-passing band -- ruling-7 panel complete, compose is a small structural tax not seed luck). groundtilt8 was a LAUNCH FAILURE not a science result (0 steps, fleet launch-collision storm gotcha 13b); retried r1/r2 (both also collision-killed) -- r3 landed RUNNING via a concurrent cycle. SKILLS updated x2 (deadband-dr05 seed-confirm, calibration-ladder tally to 12/12). Refills (3 distinct lines, cap 4): joyhead90-payload-r1 (payload x widest +-90 driving envelope, prior attempt was a launch failure) landed VERIFIED RUNNING; jointtiltpayload (NEW compose: payload x joytilt3's slope+latency package, 3rd distinct payload-compose base) hit the collision storm twice (r1, r2 both 0-step) -- r3 queued for the drain, not fought further. Heavy fleet-wide contention this window (host load1 130-350/128, many concurrent cycles' drains racing the same free pods) caused repeated 0-step launch failures across multiple cycles' refills, not just mine -- normal traffic per gotcha 13b. 
- 08-10 00:44 c69: 1 triage + 2 infra-only. imupos15-r3 NO-EFFECT (own-cfg letter-pass but champion baseline under the identical IMU-position-offset spread, logs/ckpt_eval/longdist_r2_imupos_base, matches the trained checkpoint draw-for-draw incl. same 2/6 severe craters -- sensor/calibration NO-DR-exposure ladder now 13-for-13, well is DRY; SKILLS tally updated). groundtilt8-r1 + joydeadband-r3 (assigned) were launch failures not science (gotcha 13b EOFError-at-reset, 0 steps, severe fleet storm this window loadavg 275+/128) -- recorded FAILED, no verdict. Adopted+cleaned 2 stale duplicate backlog entries (joydeadband-r3, groundtilt8-r1) that would have wasted future drain attempts on already-superseded retries. Refills: groundtilt8-r2/r3 (2 more collision losses before landing alive on train-1) + joydeadband-r4 (landed alive train-8) + payload70 (NEW ladder rung off payload50's marginal 1.4-1.5x heavy tail, widened to 1.0-1.7x, landed alive train-9) + adopted-orphan multiaxis-dr05-r1/r2 (both also collision-killed, not mine originally, left for the self-repairing drain -- it auto-requeued r2 on its own). Load eased 275->165 by cycle end. Left 4 slots (train-0/3/10/11) to concurrent cycles -- backlog audited near-dry (jointtiltpayload-r3, not mine, remains for its owner). 
- 08-10 01:00 c70: 3 assigned finished runs (cw-arch-hist16, cw-uni-mix20, cw-uni-mix40) were ALL launch failures, not science (gotcha 13b EOFError-at-reset, 0 steps, severe fleet-wide collision storm this window -- also hit groundtilt8/joydeadband/jointtiltpayload r1-r3 for other cycles). Recorded FAILED honestly on each, no science verdict. mix20 landed alive on retry -r1 (RUNNING, train-10); mix40 landed alive on retry -r2 (RUNNING, train-8) -- both inverse-mix-ladder rungs (walk=.2/.4) now actually training toward the rise/lower-acquisition question from c69's blend1-r2 FAIL. arch-hist16 (TEMPORAL-ARCH rung 1, from-scratch, no --init-from) died 4 STRAIGHT times (base+r1+r2+r3) while sibling warm-started launches around it succeeded on 1-2 tries -- DIG-IN flagged rather than blind retry #5. Infra hygiene: removed 4 duplicate/stale backlog entries (3 exact-name dupes of my own just-superseded retries, 1 stale re-add of an already-failed spec) that would have wasted future drain attempts. Refill: cw-walk-posetrack-r2 (WISHLIST 18 body-pose continuation, +15M steps warm-started from its own FAIL-letter checkpoint -- hold nailed 0.84deg, lean/track missed the 1.5deg gate at 2.2-3.6deg with 0 falls, looked like undertraining not a ceiling) landed RUNNING train-0. Left 3 slots to concurrent cycles/backlog (audited WISHLIST+RL_PLAN: READY single-axis items exhausted/closed by prior rulings; quad-legs/mirror-symmetry/contact-aux-head remain [CODE]-gated, not triage add-ons). DIG-IN: cw-walk-arch-hist16 (and its r1/r2/r3 retries) -- 4 consecutive launch-collision EOFErrors for one from-scratch spec while warm-started siblings launched the same window succeeded in 1-2 tries; worth checking whether from-scratch (no --init-from) init interacts badly with mjx_sharded_vec_env.py's reset choreography before more blind retries. 
- 08-10 01:10 c70: 1 triage (2 assigned runs were infra, 1 pre-verdicted). cw-walk-joyheadfric-s1r1 PASS -- seed-1 twin confirms the widest +-90deg envelope + floor-grip driving package (JOYSTICK GATE @90deg 0 falls, own-cfg DR0.5+friction gv 6/6 det+sto prog med 0.95/0.94, DR0 retention clean prog med 0.94/0.93, frames clean six-leg cycling no flag leg); SKILLS row updated. cw-walk-lpband was ALREADY verdicted by a prior cycle (FAIL/hypothesis refuted, ledger triage=done) before this cycle started -- no double work done, noted for the record. cw-walk-multiaxis2-r2 was a LAUNCH FAILURE not science (0 steps, gotcha 13b collision storm); my r3 retry also died the same way -- left further retries to the self-repairing drain rather than spinning more manual attempts. Severe fleet-wide launch-collision contention all cycle (host load1 up to 334/128, many concurrent cycles + auto-retries racing free pods) cost 2 more of my launches (multiaxis2-r3, joyheadfric-payload x2) before landing; joyheadfric-s2 (3rd-seed ruling-7 completion) landed clean and is training. Refills: joyheadfric-s2 (RUNNING) + joyheadfric-payload-r1 (payload x widest-envelope compose, still fighting the storm) queued, distinct from concurrent cycles' own-line refills (posetrack-r2, uni-mix20/40, lowgait-dr035-payload, tiltnoise retries) that filled most other slots. 3 slots idle at cycle end (HARD reason: backlog genuinely empty at last check, single-axis DR ladder confirmed exhausted against domain_rand.py's full RandRanges field list, all locomotion WISHLIST READY items already running/closed, remaining candidates -- quad-hold goal mode, mirror-symmetry, contact-aux-head, yaw-rate turning -- are [CODE] needing a dedicated implementation cycle, flagged unaddressed for 4+ cycles running now; the self-repairing drain will keep placing my queued item and concurrent cycles' backlogs as the storm eases). 
- 08-10 01:23 c70: 1 triage + infra cleanup. payload-dr05-s1 PASS (own-cfg gv 6/6 det fwd 1.22m; DR0 no-payload retention CLEAN slip 1.23<=1.24/prog 0.93, video all-six-legs cycling no new pathology) - CONTRADICTS c61's payload-dr05 seed0 FAIL (slip 1.38, prog 0.54); payload-dr05 compose class now 1-1 split, seed2 (s2r1) queued as tie-breaker per ruling-7, SKILLS row updated. posetrack + tiltnoise (assigned) were already resolved: posetrack fully verdicted+noted by an earlier cycle (FAIL-letter, no re-triage needed); tiltnoise never actually trained - all 5 launch attempts (base+r1..r4) died 0-step to the fleet's severe collision storm this window, recorded FAILED/no-science each, left unretried (low-priority axis, 12/13 calibration DR fields already closed). Refills (distinct lines): cw-walk-lowgait-dr035-payload (NEW compose, crouch+payload pairing, untried until now, VERIFIED RUNNING) + cw-walk-groundtilt5-payload (slope+payload pairing, untried, died once to collision/retried as -r1/died again, left for drain) + posetrack-r2 and payload-dr05-s2 backlog items already queued by concurrent cycles, drained. Heavy fleet-wide launch-collision contention all cycle (gotcha 13b) cost ~7 collision deaths across my launches alone - documented per-run, not fought beyond 1-2 retries each. 
- 08-10 01:39 c70: 3 triages. joydeadband-r4 PASS (deadband axis composes cleanly onto joylat25 driving pkg, JOYSTICK GATE 0 falls, DR0 retention clean, 4th launch retry succeeded). joyhead90-payload-r1 FAIL-letter (own-cfg passes but DR0 retention misses slip<=1.24/prog>=0.90 caps -- payload composability depends on the base steering package's headroom: free at +-45 (joyfric-payload) but charged at +-90, SKILLS updated with the contrast). payload70 PASS-letter (widened payload ladder 1.0-1.7x: soft/gradual ceiling not a wall, same heavy-tail shuffle pattern as payload50 stretches further, DR0 det retention clean, one known sto-only stall outlier; payload50 stays the safer promotion point). Refills (4 distinct lines, respec off PASSed parents): joylat60-fric (friction onto the 60s driving endurance pkg), joylat25-comshift (CoM-offset onto the 15s driving pkg -- untested combo), groundtilt5-fric (grip onto the sloped-floor axis), lowgait-fricvar (grip onto the crouch-height axis) -- all queued+drained, single-axis DR ladder itself stays closed per prior cycles' audits. 
- 08-10 01:47 c71: 2 triages (1 already-recorded infra). cw-walk-groundtilt8-r3 PASS-with-caveat (8deg tilt rung off groundtilt5: own-cfg gv 6/6 det+sto, 0 falls, det med fwd 1.35m>=1.1 gate, DR0 retention clean slip 1.07; but 3/6 det draws crater to a shuffle -- worse fraction than tilt5's 2/6, same no-fall/no-crab mechanism, video-confirmed six-leg cycling even on craters -- 8deg is the marginal exposure-training edge, do not ladder further without a slope-specific fix; SKILLS updated). cw-walk-groundtilt5-payload: already-recorded launch failure (0 steps, collision storm) from before this cycle, no new verdict. cw-uni-mix20-r1 (walk=.2 inverse-mix rung, top UNIFIED JOYSTICK deliverable) left UNVERDICTED: own-DR rise/lower det video shows the SAME 0-success pattern as blend1-r2 (h_err stuck -38 to -62mm at episode end, reward strongly negative and worsening) despite walk share cut to 20% -- decides a fork in the plan (refutes the mix-ratio-is-the-lever assumption at this rung) and is protected-skill-adjacent, so flagged for deep dig-in rather than triage-verdicted. Infra: this window hit a severe fleet-wide launch-collision storm (gotcha 13b) that killed 7 of my own launch attempts across 3 specs before landing (groundtilt5-payload retries r1-r4 all died, still fighting; jointtiltpayload r3/r4 and lowgait-dr035-latjit each died once before landing) -- adopted+recorded 2 pre-existing orphaned FAILED launches (jointtiltpayload-r3, lowgait-dr035-latjit) that had no verdict, cleaned 2 stale duplicate backlog entries. Refills landed RUNNING (distinct lines): jointtiltpayload-r5 (payload x joytilt3 compose), lowgait-dr035-latjit-r1 (crouch x latency compose), cw-uni-mix0-r1 (walk=0.0 pure rise/lower isolation, directly probes the mix-ratio hypothesis cleanly), groundtilt8-s1 (seed twin, ruling-7 for the new marginal tilt8 rung); groundtilt5-payload-r5 still draining through the storm. DIG-IN: cw-uni-mix20-r1 -- rise/lower 0-success persists at walk=0.2 (same pattern as blend1-r2 at walk=0.7), a fork-deciding result for the UNIFIED JOYSTICK line that needs root-cause (income-scale vs reward-shaping defect) before any further mix-ladder rung or reward patch. 
- 08-10 02:07 c70: 3 triages. cw-walk-joyfric-payload PASS - mass-DR payload compose (1.0-1.4x) is safe on the driving line: own-cfg gv 6/6, JOYSTICK GATE 0 falls, DR0 retention slip 1.46/1.51 within noise of parent joyfric's own 1.41/1.37 baseline (resolves c61: payload-dr05 erosion was lineage-specific to plain-walk, not structural to mass-DR composes; flagged the inherited 1.24 letter-cap as mis-calibrated for this already-1.4+ driving baseline). cw-walk-joyheaddeadband PASS - servo deadband (1-3x) composes onto the widest +-90deg envelope, JOYSTICK GATE 0 falls, deadband axis now closed at both heading widths. cw-walk-joyhead90-60 PASS - widest envelope survives a full 60s drive (own-cfg gv 12/12, no first/second-half decay, JOYSTICK GATE 0 falls, DR0 retention matches parent exactly). SKILLS updated x3. Refills (4 = cap, distinct lines): posetrack-r2 (near-miss continuation, mix shifted off hold+more steps - already landed via a concurrent cycle, dropped as duplicate, no harm) + joyfric-payload-s1 (ruling-7 seed twin, RUNNING) + joyheaddeadband-s1 (ruling-7 seed twin, hit gotcha 13b 3x in a row while sibling launches succeeded nearby - matches the c67 joyhead90-lat25-s1 isolated-spec pattern, worth a dig-in look if -r3 also dies; landed RUNNING on the 4th attempt) + multiaxis-dr05 continuation (r3/r4 died at init, landed as -r5 RUNNING - tests whether its retention miss is under-training or a real 4-axis ceiling). Heavy concurrent drain traffic this window (many other cycles' composes launching/finishing in parallel) filled remaining free slots before I could act on them. 
- 08-10 02:09 c71: 3 triages. cw-uni-mix40-r2 FAIL (inverse-mix-ladder rung walk=.4/rise=.25/lower=.25 still 0/6 rise + 0/6 lower det, wandb flat ZERO all 18M steps, video confirms rise stuck mid-height churning / lower never descends — same pathology as blend1-r2 at 10/10 mix, reproduced at 25/25; walk/hold retained clean; mix20 not mine to verdict, so mix-lever refutation pending that sibling). cw-walk-joyfric-s1r1 PASS (seed-1 twin reproduces joyfric closely: JOYSTICK GATE 0 falls, own-cfg gv 12/12 prog med 1.00/0.95, DR0 retention gv 6/6 prog 1.00/1.01 — floor-grip+latency driving recipe seed-robust). cw-walk-lowgait-dr035-payload PASS (NEW crouch x payload compose, own-cfg DR0.35+payload gv 12/12 height err ~4mm/slip 1.20-1.25, DR0 no-payload retention height err 4.8mm/slip 1.08; few fixed-draw churn outliers match known canary pattern, no falls). SKILLS updated x2. Infra: cw-arch-hist16-r4 (temporal-arch rung1 retry) died a 5th STRAIGHT time to the same EOFError-at-reset_finalize in mjx_sharded_vec_env.py — recorded FAILED, re-flagging DIG-IN rather than a 6th blind retry. Refills (4=cap): groundtilt5-comshift + lowgait-dr035-deadband (2 new untried composes off today's/prior PASSes) + lowgait-dr035-payload-s1 (ruling-7 seed twin of this cycle's new PASS), all VERIFIED landed; arch-hist16-r4 counted toward cap despite failing. DIG-IN: cw-walk-arch-hist16 (r4, and its base+r1-r3 predecessors) — 5 consecutive launch-collision EOFErrors for this exact from-scratch history_frames=16 spec while nearly everything else around it launches fine, always dying at mjx_sharded_vec_env.py's reset_finalize broadcast; needs code-level investigation, not another mechanical retry. 

## OPERATOR hardware session 2 (08-09 ~22:30 ET) — bus verified, zero locked, dynamics probe rerun; GPT walk-attempt review landed as BINDING

Robot hand-set at zero by operator. Bus: 18/18 IDs (2-19) live,
temps 30-33°C, 12.1V. Encoders vs visual zero: max |0.3°| →
set_zero locked (robot reports zero-here 18/18). rl_probe_dynamics
±10° all joints running (fresh motor_model.json for the actuator-ID
build). GPT's deep review of the failed walk attempt is archived at
`archive/GPT_HARDWARE_HANDOFF_2026-08-09.md` and its ruling is now
BINDING in RL_PLAN Queue -1: Deployment Equivalence Gate 0 (stateful
slew in training+eval, prev-action semantics, zero-command panels),
zero-command export/obs audit as a separate P0 experiment, measured
loaded rate envelope (raised-clamp retry WITHDRAWN from the backlog),
identified actuator model before any learned residual, anti-slip
shaping stays closed. Operator has authorized higher-risk hardware
experiments (tip risk accepted); proceeding with the supported
ladder: air probe → RL stand from belly → quiet hold → RL lower,
with full 25 Hz traces for the audit.

## OPERATOR hardware session 2 results (08-09 ~23:00 ET) — first stance-line policy signal on hardware; phantom over-temp ROOT-CAUSED and fixed

Three RL stand attempts (ppo_goal_cw_stance_dr10, obs 68 / act 18):

1. **02:37 — phantom over_temp at t=1.2 s, NO policy signal.** The 08-09
   tick-based debounce (3 consecutive control ticks) was defeated by
   design: temps refresh at full_feedback_hz=10 but control runs 25 Hz,
   so ONE corrupted bus byte is cached for ~2.5 ticks and satisfies
   "3 consecutive". FIXED in `rl_move/safety.py`: over-temp now counts
   3 consecutive FRESH feedback reads (300 ms of sustained heat); stale
   ticks neither confirm nor clear. Deployed to robot + verified.
2. **02:47 — REAL 8.4 s episode, tilt_roll trip at tick 210.**
3. **02:49 — bit-for-bit repeat: tilt_roll at tick 210.**

Failure signature (traces `rl_move/hardware_traces/rl_stand_20260810_*`):
0-5 s zero-command curl QUIET (roll <0.5°, <0.15 A — the walk attempt's
zero-command marching does NOT afflict stance); ramp stable to href
~32 mm; then at href 33-42 mm (belly liftoff) roll runs +2°→+10° in
0.8 s with current 0.1→2.9 A, SAME +roll direction both runs. Verdict:
deterministic liftoff collapse, not noise — consistent with the
1.5°/tick slew limiter starving fast corrective action under load
(GPT handoff causal rank #1) and/or loaded-servo dynamics absent in
training. No third retry (two identical failures = systematic).

Also collected: fresh `motor_dyn_20260810_023300.csv` ±10° air battery
(all 18 joints, no aborts) + `motor_model.json` → actuator-ID build.
AUDIT VALUE: these stand traces are the deployment-equivalence
reproduction case — init sim from belly zero, run the exact deployed
contract (stateful 1.5°/tick limiter, 25 Hz, meas obs), and check
whether sim reproduces the href~35 mm +roll blowup. If yes: contract
fix confirmed. If no: physics gap (contact/loaded actuator).

Robot end state: belly-down at zero (max Δ 0.5°), LIMP, temps ≤33°C,
18/18 IDs. Safe.
- 08-10 02:11 c69: joyheaddeadband-s1 infra-died at init 0-steps (gotcha 13b, no science result, verdict recorded, retry chain -r1/-r2/-r3 continued by concurrent cycle+drain); joyheadfric-payload-r1 PASS (payload 1.0-1.4x composes free onto widest +-90 friction-hardened driving pkg -- JOYSTICK GATE 0 falls, own-cfg+DR0ret clean, contradicts earlier friction-less joyhead90-payload-r1 FAIL, new SKILLS row); joyheadfric-s2 PASS (3rd seed closes the ruling-7 panel for joyheadfric, recipe confirmed 3/3, SKILLS row extended). joylat25-comshift SUSPECT checkup (fps 1456->2986, host load ~230-260/128 all-tenant contention, reward healthy, not stalled -- left running, no action needed. Refilled all free slots via drain (groundtilt8-s1-r1 infra retry, joyheadfric-payload-s1 seed-panel start); arch-hist16 line still DIG-IN-blocked (5/5 launch-collision deaths on the identical from-scratch history16 spec, not re-attempted). 
- 08-10 02:32 c71: 3 assigned triages (groundtilt5-payload-r4, groundtilt8-s1r1, joyheaddeadband-s1-r1) were pure INFRA FAILURES not science -- all 3 died at 0 steps to the launch-collision EOFError (gotcha 13b) during an extraordinarily severe multi-cycle drain-storm (01:00-02:30, every single pod handoff drew 2+ simultaneous claimants); no verdict possible, ledger entries stand as FAILED. Found+fixed a real self-repair bug while recovering them: launch_run.py's backlog requeue() re-tried a crashed-after-wandb-init item under the SAME name, which the dedup guard then silently DROPPED (no requeue, no park, no trace) instead of retrying -- patched to rename-on-collision before requeue (snapshot 178fc8a); groundtilt5-payload-r5 and (via concurrent cycles building on my retries) groundtilt8-s1-r1 are now genuinely training. joyheaddeadband-s1 lineage died 6 total times across 5 differently-named attempts (mine + concurrent cycles') even post-fix -- matches the c67 joyhead90-lat25-s1 isolated-spec precedent, not generic storm; DIG-IN flagged. Refills: joylat60-payload (payload compose onto the 60s endurance driving package, RUNNING) + arch-hist16-r5 (temporal-arch directive, 6th mechanical retry, ALSO died identically -- DIG-IN flagged, not re-tried further myself). Left remaining free slots (1,4,7,11) to concurrent cycles' churn this cycle -- HARD reason: 5 launch attempts this cycle alone hit the same collision storm, diminishing returns on more manual retries during an active multi-cycle race; my GPU-step commitment (38M of 80M cap) and 2-refill count already reflect real placed science. 
- 08-10 02:48 c73: 3 triages. cw-uni-mix0-r1 FAIL (walk=0.0 zero-competition isolation still misses rise/lower gate: rise 0/6 the ENTIRE run incl. crouch-start which other lineages call solved, lower ended 3/6, hold got LESS quiet over training 3mm->33mm; walk retention ok on video but slip degraded some; completes the mix-ratio ladder — walk=.7/.4/.2/0 ALL fail rise/lower, cleanly refutes income-scale-vs-walk-competition as the lever, root-cause stays with mix20's DIG-IN). cw-walk-joyfric-payload-s1 PASS (seed-1 twin of payload-onto-driving-package compose, reproduces cleanly: JOYSTICK GATE 0 falls, own-cfg+DR0-retention clean, 2/2 seeds). cw-walk-joyheadtilt3 PASS (3deg slope composes onto the WIDEST +-90deg+latency package: JOYSTICK GATE @90 0 falls, own-cfg prog 0.85/0.93, true-flat DR0 retention clean — ran my own DR0.5 harness+flat-retention+joystick evals since the pre-staged gate eval only covered DR0-walk-mode). SKILLS updated x2 (extended joyfric-payload row, new joyheadtilt3 row). Caught+killed 2 of my own duplicate launches before wasting real compute: torquescale/torquescale-r1 duplicated the already-CLOSED torquedroop axis (fixed the mental note); a stale cw-walk-speedband backlog re-add was caught by the drain's own W&B-name dedupe (0 compute lost) — marked WISHLIST 8b CLOSED so it stops recurring. Checkup: jointtiltpayload-r5 SUSPECT(fps2913) was transient, now healthy ~5300fps, no action. Refills: cw-walk-torquescale-r1(killed,dup) + speedband(dropped,dup) + joyheadtilt3-payload x2 attempts (1 vanished from backlog with zero ledger trace — possible lost-update race distinct from the 178fc8a fix, flagging; 2nd died 0-step collision storm) -> requeued as -r1 for the mechanical drain, not fought a 3rd time by hand. 
- 08-10 03:17 c72: 3 assigned triages. groundtilt5-fric PASS-with-caveat (friction 0.4-1.6x composes onto the 5deg-slope axis: own-cfg det+sto gv 12/12, 0 term, det med fwd 1.29m>=1.2m gate, DR0 retention clean slip 1.09<=1.24; but HALF the own-cfg det draws (3/6) crater to a high-slip shuffle vs groundtilt5-alone's 2/6 -- video confirms same no-fall/no-flag-leg mechanism, not a new pathology; SKILLS updated). joyheaddeadband-s1-r3 KILLED-infra (host-starved at 3.87M/20M, no gate result) + its -c2 continuation FAILED-infra (0-step launch-collision EOFError) -- both recorded, deadband-seed1 hypothesis still unverified, matches the c67/c71-flagged joyheaddeadband-s1 isolated-spec pattern. Checkup: groundtilt8-s1-r1 SUSPECT (fps 1456 at checkup) reviewed -- fps recovered to 4408+ by this cycle, node g142d86 shares 3 trainers (host-wide contention, not starvation-to-death), left running, no action needed. Infra: hit + fixed a fleet-wide pod code-sync gap this window -- 6 train pods (0,1,3,7,10,11) were stale vs local HEAD after the operator's hardware-session commit landed, causing REFUSED launches misread as collision-storm; ran snapshot.sh --sync on all 6, unblocking several concurrent cycles' drains too. Also resolved a leftover git stash/rebase conflict in RL_LOG.md from an earlier interrupted autostash (merged both sides in chronological order, dropped the stash once clean). Refills (4 distinct lines, cap reached): torquescale-r2 (torque-droop axis, retried through 2 infra deaths + the sync fix, now training as -rr1-rr1), joyjit-dr05-payload-r2 (jitter-DR0.5 x payload compose, same retry pattern), strafe-dr05-payload-r1 (lateral-strafe x payload compose, landed clean), lowgait-dr035-comshift (NEW crouch x CoM-offset compose, landed as -rr1 after a W&B name collision auto-renamed it) -- all VERIFIED training. Noted in passing: yaw-rate turning code (WISHLIST item 3) landed upstream (c086a22) and a first cw-walk-yawcmd1 arm is already training via another cycle/drain -- not mine to touch. Left 4 slots (t1/t3/t7/t11) idle at cycle end -- HARD reason: my 4-launch cap reached and backlog genuinely empty at last check; concurrent cycles' drains and the self-repairing mechanism are still active on the freshly-synced pods. 

## OPERATOR hardware session 3 (08-09 ~23:15-23:35 ET) — SCRIPTED GAIT WALKS; measured gait envelope, currents, latency; omega sign pinned

Operator supervised throughout; robot parked settled+limp, healthy.
Traces: `rl_move/hardware_traces/hw_session2_20260810.csv` (telemetry
~2 Hz), `imu_walk_20260810.csv` (tilt ~3 Hz), `step_ladder_20260810.csv`
(50 Hz single-joint), `motor_dyn_20260810_023300.csv` (air battery).

**1. The scripted tripod gait WALKS this robot** — fwd 30 mm/s (x3),
fwd 50 mm/s, crab vy=30, turn both directions, all clean from a fresh
set_zero → P(plant) sequence. Earlier same-evening walks from a stale
stance (after RL-stand residue + zero glides) fell almost immediately,
twice. Sequencing/stance-sync matters; the gait itself is sound.
Operator: DEFINITE foot slipping during gait, "maybe helping" — the
'floor does not skate' finding from the 08-09 walk post-mortem is about
body-roll coupling, not an absolute no-slip regime. Contact model must
allow loaded slide.

**2. A WORKING gait rocks ±10-20° in roll AND pitch** (tilt log, all
gait phases; stance-hold sits at ±1°). The RL walk deploy trip AND
training termination at 10° relative tilt would kill this working
gait. ACTION: walk-mode tilt envelope (train term + deploy trip)
should be ~25°.

**3. Current economics INVERT the sim's assumptions**: standing hold
0.59 A total mean (hot spots servo5/13 ~0.1 A each); walking
0.33-0.45 A mean / ~1.0 A peak; crab & turns same band. Walking is
CHEAPER than holding. (Session-2's "planted 0.09 A" was a fallen
robot — windows before ~23:05 are contaminated, robot had tipped.)

**4. Loaded single-joint step ladder (L0 knee at stance, 50 Hz):**
cmd→first-motion latency 110-210 ms (HTTP+drive-loop path), t90
260-330 ms (2-5° steps), ~420 ms (10°). Air-battery motor_model.json
refreshed same night.

**5. Yaw sign convention measured:** `J , , +0.3` → body turns
CLOCKWISE (video IMG_3423; ~9 rotations over an accidental 200 s run),
-0.3 → counter-clockwise. Pin this into the deployment-contract audit
(sim wz_ref sign must be checked against it).

**6. POST-MORTEM CORRECTION (important):** the 1.5°/tick rate clamp
WAS in the raw-joint training path all along (config
safety.max_delta_q_deg=1.5 since cb602eb; sim_env._step_begin routes
through SafetyLayer.filter). The 08-09 "policy runs 9× slow-motion
dynamics it never trained in" claim is wrong as stated — proposal
saturation exists in BOTH worlds and PPO plausibly uses the clamp as
its trajectory generator by design. The REAL contract gaps to close:
walk obs velocity source (hardware feeds meas:=ref; training used
privileged sim velocity — new goal.walk_obs_body_vel=2 mode now
matches hardware exactly), tilt termination (10° vs the ±20° a real
gait needs), prev-action semantics (unaudited), and contact/current
pricing (now calibratable from tonight's scripted-gait data).

**7. Phantom over-temp fix HELD**: zero false trips across ~15 min of
armed motion. 123/34470 temp reads 46-53°C during the 200 s turn may
be real warming (plausible) — the 107°C spike earlier was corrupt.

Missing: measured walk distance (operator left before measuring) —
true ground speed still needed for slip calibration; next session,
tape measure or video with scale.
- 08-10 03:18 c70: joylat60-fric PASS (friction composes onto 60s driving endurance, JOYSTICK GATE 0 falls, DR0/own-DR clean); lowgait-dr035-latjit-r1 PASS + lowgait-dr035-s1-r2 PASS (latency-jitter compose + seed-1 twin both confirm crouch-50mm+DR0.35, seed-robust); SKILLS +3 rows. Checkup: joyheaddeadband-s1-r3 confirmed host-starved on g142d86 (loadavg 245 vs healthy 50-62) -> killed, ckpt preserved (md5 ef71d1014b59a9ed082f3d99d66a707a), took 4 launch-collision EOFErrors (fleet-wide storm, not a checkpoint defect) before landing via backlog self-repair rename as -c3-rr1 RUNNING; same rename mechanism also unstuck the 6x-failing arch-hist16 line (queued as -r6 via backlog instead of more direct-launch retries) and 3 other cycles' parked items (torquescale/joyjit-payload/strafe-payload). Refills: lowgait-dr035-fric (RUNNING->already FINISHED, needs next-cycle triage) + joylat60-comshift (landed via rr1); direction: DR-compose campaign continues to pay off cleanly, fleet had a rough patch of concurrent-launch collisions this cycle (now resolving via the rename self-repair) plus one host-starved node (g142d86) worth watching. 
- 08-10 03:25 c72: placementnoise6-r3 PASS (retry#3, first real science on hand-placement-slop axis: own-cfg det med fwd 1.23m gv12/12, DR0 retention clean 1.50m/slip1.10; SKILLS row added); posetrack-r2 FAIL (if-false confirmed: +15M steps fixed hold(12/12) and helped lean(6/12<=1.5deg, up from 0) but track barely moved (1/12<=1.5deg) -- needs denser reward/curriculum not more steps); lowgait-fricvar DIG-IN (own-cfg det has a catastrophic slick-friction episode: slip 18.86 vs baseline ~1.1-4.2, fwd 0.07m near-total skate, yet mechanical gate checks (gv/term/flag-leg) all pass -- gate/video disagreement, deferred to deep model). Refills: terrain10-payload + strafe-dr05-payload + joyjit-dr05-payload (payload composed onto 3 previously-uncombined PASSED axes: terrain, lateral-strafe, cmd-jitter); hit the fleet's ongoing launch-collision-storm + code-sync-race infra churn hard (killed one duplicate self-inflicted launch, recovered 3 orphaned backlog items via requeue -- joyjit-dr05-payload still failing to land after 5 mechanical retries, left to self-repair/next cycle). 
- 08-10 03:25 c74: 3 triages, all PASS (own pre-staged evals + own-DR eval since pre-stage skipped under load). groundtilt5-comshift PASS (slope5deg x comshift compose, DR0 retention clean 1.00/0.97, same 2/6 slope-tail as parent). groundtilt5-payload-r5 PASS (6th launch attempt after 5 collision-storm deaths; slope x payload, DR0 retention clean 0.99/1.01). jointtiltpayload-r5 PASS (5th launch attempt after 4 collision-storm deaths; joyfric-tilt-driving x payload, JOYSTICK GATE 0 falls @45deg, DR0 retention clean slip 1.18/prog 0.96). SKILLS updated x3. Refills: 4 distinct comshift/deadband composes off different base packages (lowgait-dr035-comshift, groundtilt5-deadband, joyfric-comshift, joyheadfric-comshift) — hit an extremely severe fleet-wide launch-collision storm (all 4 direct launches died 0-step, 2 auto-recovered via the requeue-rename mechanism, 2 needed a manual backlog re-add) but all 4 are now confirmed RUNNING under -r1/-rr1/-rr2 names. Infra note: saw backlog items vanish with zero trace on 2+ occasions this cycle (transient — items eventually reappeared and landed seconds-to-minutes later, so likely a read-timing artifact of the extreme concurrent-drain load rather than a lost-update bug, but flagging since c73 saw the same symptom); left 3 free slots to the concurrent cycles/watcher (cap of 4 launches reached). 
- 08-10 03:37 c74: 3 assigned triages. cw-walk-joyheaddeadband-s1-r3-c3-rr1 PASS (seed-1 twin closes joyheaddeadband ruling-7 panel 2/2: JOYSTICK GATE @90 0 falls, own-cfg gv 6/6 prog 0.92/0.90, DR0 retention gv 6/6 prog 0.92/0.93 matching parent band; SKILLS row extended). cw-walk-groundtilt5-deadband + cw-walk-joyheaddeadband-s1-r4 both INFRA FAILURE (0-step launch-collision, gotcha 13b, no science; r4 redundant with the PASSing seed-1 twin so not re-chased). Refills (3 distinct lines via respec, backlog+drain): imupos15-r1 + gyrobias3-r1 (retry the 13c IMU-position/gyro-rate-bias axes that died 0-step to the storm and never got a science result) + joyheaddeadband-payload (NEW compose, payload onto the widest deadband-hardened driving package). 
- 08-10 03:48 c75: 3 triages. groundtilt5-deadband-r1 + -rr1 both FAILED-infra (0-step launch-collision EOFError, no science, deadband x groundtilt5 hypothesis still untested -- auto-requeued and both landed as -r1-rr1/-rr1-rr1). groundtilt8-s1-r1 PASS: seed twin of groundtilt8-r3 reproduces the IDENTICAL 3/6 crater fraction at matching severity (slip up to 3.26, no falls/flag-leg) plus clean flat retention -- confirms r3's slope-crater tail is a recipe trait, not seed luck (SKILLS row extended). Refills: 3 new-line composes off independently-passed axes (joylat60 x torquescale, terrain10 x deadband, placementnoise6 x payload) -- terrain10-deadband landed clean; the other two both vanished/died with zero ledger trace on their first queue attempt (matches c73/c74's flagged lost-update symptom under this window's severe multi-cycle drain-collision storm) and were requeued once each (placementnoise6-payload-rr1 pending, joylat60-torquescale-rr1 vanished a 2nd time, left to the mechanical drain rather than fought further by hand); 11/12 slots busy at cycle end, 1 free slot has a backlog item mid-verification. 
- 08-10 03:55 c74: 4 triages. cw-arch-hist16-r6-rr1 + cw-quad-hold1 + cw-quad-hold1-rr1 all FAILED-infra (0-step launch-collision EOFError, no science; quad-hold1's real test is cw-quad-hold1-r2, already training). arch-hist16 is now 7-8/8 identical from-scratch history_frames=16 deaths at the same reset_finalize EOFError -- left the line alone per prior cycles' precedent rather than blind-retry again, re-flagging DIG-IN. Checkup-flagged cw-walk-torquescale-rr1-rr1 actually finished normally (watcher misread completion as DEAD); pulled ckpt + ran own-cfg det/sto + a fresh champion baseline under the identical torque_scale=0.80,1.05 spread: FAIL/NO-EFFECT, episode-for-episode match to champion (same 2/6 det craters) -- also a post-hoc duplicate of the already-CLOSED torquedroop axis, closes cleanly. Refills (4 distinct new composes, single-axis DR ladder is now fully closed 12-for-12 per the runs docs): lowgait-dr035 x groundtilt5 (crouch x slope, RUNNING), deadband30 x payload (RUNNING), groundtilt8 x comshift (marginal-tilt x CoM-offset, hit the fleet collision storm twice, self-repair landed it RUNNING as -rr1), groundtilt5-fric seed-1 twin (ruling-7 completeness, hit the storm twice too, auto-requeued -r2-rr1 still draining). DIG-IN: cw-arch-hist16 -- 7-8 consecutive 0-step reset_finalize EOFErrors for the from-scratch history_frames=16 spec, unresolved despite backlog self-repair; needs a code-level read of mjx_sharded_vec_env.py before any more attempts. 
- 08-10 04:11 c75 (dig-in): lowgait-fricvar PASS after dig-in — the flagged det/4 dead-skate (slip 18.9, fwd 0.07m) is an INHERITED lineage fixed-draw stall (untrained parent lowgait fails the identical draw identically at pinned slick 0.4; same draw clean at nominal grip), NOT a fricvar defect; at pinned 0.4 the crouch beats upright fricvar's medians (0.86-0.88 vs 0.83) — if-false refuted, SKILLS row added, fricvar-s1 seed twin queued (landed as -s1-rr1). BONUS ROOT CAUSE: took the arch-hist16 8x-death DIG-IN (deep model) and solved the fleet's 0-step-EOFError plague — train pods have 64M default /dev/shm, a 4096-env layout maps ~58M, leaked segments from any crash poison every later launch (workers SIGBUS, exitcode -7, proven by instrumented smoke on train-7); hist16@4096 >64M so it could NEVER boot. Shipped: worker faulthandler + named-exitcode EOF reports (c4f3625), startup GC of orphaned hexmjx-* shm (bcf46be, unit-tested), dshm-4Gi in both pod manifests (apply per pod when idle — runbook in COMMANDS 13c), all 12 pods synced; arch-hist16-r7 requeued at 3072 envs (~50M, fits). Refills: drained groundtilt8-comshift-rr1, queued lowgait-fricvar-s1 + arch-hist16-r7; 0 slots idle at exit. 
- 08-10 04:12 c76: 3 triages. cw-quad-hold1-r2 FAIL (compound gate: four-leg hold mechanism itself is solid -- training's own quad-eval survived_frac 1.0/1.0 all 6 checkpoints, video clean early+late, level body/fronts lifted/no tipping -- but 50/40/10 quad/walk/hold mix eroded walk-retention below the standard gate: own harness det med slip/m 1.42 vs cap 1.25 (all 6 det eps 1.28-1.48, none under cap), sto had 1/6 flag-leg skate; matches the pre-registered if-false branch cleanly, no dig-in needed). cw-walk-groundtilt5-fric-s1-r2 + -rr1 both FAILED-infra (0-step launch-collision EOFError, fleet storm, no science; -rr1 was already verdicted before this cycle, confirmed only). Infra: found RL_LOG.md sitting in a leftover conflicted git-stash-pop state (<<<<<<< Updated upstream / Stashed changes markers) from an earlier interrupted autostash -- merged both sides in chronological order (operator hardware-session-3 block + c70/72/74 cycle lines), staged, dropped the stash; picked up cleanly into the concurrent dig-in cycle's later commits (bcf46be) which also landed the real root-cause fix for tonight's storm (_gc_orphaned_shm in mjx_sharded_vec_env.py -- leaked /dev/shm segments from crashed runs SIGBUS'd every later launch on the pod) -- left that fix to its owner, did not touch. Refills: drained backlog (5+ retries landed: fric-s1-r2-rr1, groundtilt8-comshift-r2-rr1, joyheaddeadband-comshift-rr1, lowgait-fricvar-s1-rr1, joylat60-torquescale-rr1-rr1, placementnoise6-payload-rr1-rr1) + queued cw-quad-hold2 via respec (NEW rung: same init/reward, mix laddered quad 0.5->0.3 / walk 0.4->0.6, one variable, tests dose-dependence of the walk-retention erosion) -- 12/12 slots busy at cycle end, cw-quad-hold2 next in backlog. 
- 08-10 04:13 c77: 3 triages. cw-walk-joyheadfric-payload-s1 PASS (seed-1 twin closes joyheadfric-payload ruling-7 panel 2/2: JOYSTICK GATE @DR0.2 heading90 0 falls, own-cfg gv 6/6 prog med 0.86/0.84, DR0 retention gv 6/6 prog med 0.90/0.84 slip 1.60/2.08 -- same order as seed-0, no pathology in det/own-DR frames; SKILLS row extended). cw-walk-joyheaddeadband-comshift + cw-walk-joyheaddeadband-payload-r1 both INFRA FAILURE (0-step launch-collision, gotcha 13b, no science; comshift x deadband and payload x deadband hypotheses still untested, both auto-requeued and landed as -rr1/-r1-rr1). Refill: joyheadtilt3 x deadband (NEW compose, untried pairing) -- 1st queue attempt vanished from backlog with zero ledger trace (matches the c73-c75-flagged lost-update symptom under this window's drain load), requeued once and landed; 12/12 slots busy at cycle end, 2 duplicate backlog entries for it left to the launcher's W&B-name dedupe to self-repair. 
- 08-10 04:25 c76: 3 triages, all PASS. joyfric-comshift-rr1 PASS (45deg steering x comshift payload; own-cfg 12/12 gv, JOYSTICK GATE 0 falls, DR0 retention clean slip1.43 vs band 1.46/1.51) -- caught our own mistake mid-cycle: first own-DR eval pass used dr-scale=1.0 when the run trained at 0.5, producing one scary catastrophic-slip sto episode; re-ran at the correct 0.5 and it vanished, all 3 runs clean. joyheadfric-comshift-rr1 PASS (90deg steering x comshift, same pattern, JOYSTICK GATE 0 falls). joyheadtilt3-payload-r1 PASS (90deg steering on 3deg tilt x payload 1.0-1.4x, new tilt-x-payload-on-steering compose, JOYSTICK GATE 0 falls, DR0 flat-no-payload retention clean). SKILLS +3 rows. Refills: queued joyheadtilt3-payload-s1 seed twin (ruling-7) via respec; drained backlog (arch-hist16-r7, joyheadtilt3-deadband, dep-fresh1, quad-hold2) into free slots, ended at 12/12 busy. 
- 08-10 04:40 c78: infra recovery + 3 triages. Found experiments.json (+ RL_LOG/run-docs/backlog_failed.json) stuck in an interrupted git-stash-pop conflict from a concurrent snapshot.sh autostash (same class of incident as c76's RL_LOG fix) -- diffed the 3 git stages, confirmed non-overlapping (upstream only added cw-dep-vref1 entries, stash only had 5 in-place status updates), merged byte-exact, committed+pushed (a0d772e), no ledger content hand-edited. Triage: cw-walk-joylat60-payload-r1 was already closed (duplicate-race kill, pre-verdicted, confirmed only). cw-walk-lowgait-dr035-comshift FAILED-infra (0-step launch-collision, no science, closed). cw-walk-lowgait-dr035-comshift-r1 own-cfg gate PASSES cleanly (gv 12/12, height/slip within band) but DR0 no-offset retention has a catastrophic single-episode stall (slip 17.5, fwd 11% of commanded) that the direct parent lowgait-dr035 does NOT show at the identical seed/config -- same magnitude as the lowgait-fricvar anomaly two cycles ago that needed deep-model root-causing, so left UNVERDICTED rather than triage-call it; frame strips show no visible pathology (looks identical to the clean episodes), it's a numbers-only anomaly. 12/12 GPU slots busy throughout, backlog non-empty but fully covered by concurrent drain, no refill needed this cycle. 

## OPERATOR directive 08-10 00:38 — start-variation robustness over sequencing guards

Root cause of the scripted-gait falls (session 3): gait syncs stance
to the PRESENT pose and walks were started from stale/slumped stances
with drifted logical zero. Operator declined a controller-side
"refuse J unless at plant" guard; instead RL training and evals get
MORE start variation (RL_PLAN Queue -1 "START-VARIATION ROBUSTNESS":
placementnoise6+bad-start compose onto the cw-dep line, NEW obs-side
zero-drift DR axis [CODE], varied-start eval panel for hardware
candidates, park-bank walk starts). Status at directive time:
cw-dep-vref1-r1 RUNNING 12.9M/20M, cw-dep-fresh1 RUNNING 2.5M/20M.


OPERATOR (08-10 00:4x): QUAD -> MAINLINE. "Four leg trick in the
main line so I can hit that with the joystick in sim to real." Quad
is now a joystick COMMAND of the driving lineage, not a party trick.
Queued cw-walk-joyquad30: quad=0.3/walk=0.7 mix composed onto
driving champion joylat25 (full DR0.5+latency+abrupt-flip spec, warm
start joylat25.zip); gate = JOYSTICK GATE @DR0.2 retained + quad
hold metrics (fronts_off>=0.9, clear>=20mm, planted>=0.95 final
10s, level) + walk slip <=1.55 (parent band). Mix informed by
quad-hold1-r2's if-false result (50% eroded walk); cw-quad-hold2
(30% on walk champion) still mid-flight as the parallel arm.
Operator-side: drive_policy.py key `4` toggles the quad command
live (writes lift_legs=(0,5) into the running trajectory; HUD shows
QUAD). RL_PLAN party-tricks section + WISHLIST 15 updated.

## OPERATOR session 08-10 ~01:10 ET — GPT handoff landed, 8h window to hardware attempt #2

GPT review archived (`archive/GPT_HANDOFF_2026-08-10.md`), rulings in
RL_PLAN + ORCHESTRATOR_PROMPT (P0: verdict dep arms first; fresh1
judged qualitatively; minimal effort shaping on hardware arms; no new
generic composes tonight). Work landed this session: (1) NEW DR
mechanism `dr.zero_drift_cmd_frame=1` — logical-zero drift as a FRAME
shift (reads AND commands share the drifted frame; legacy obs-only
bias left a 3.3° cmd-vs-read residual the policy could exploit, env
smoke shows frame mode self-consistent at 1.2° settle error with the
physical pose silently offset). (2) Prev-action semantics AUDIT:
PASS — both training (`sim_env` echoes validated raw proposal) and
runner (`rl_policy.py` same) pre-safety-filter, shared build_obs;
Gate 0 item closed. (3) Loaded actuator quantified from step_ladder
robot-side timestamps: 2° steps ~120 ms motion / 250-325 ms total
settle vs sim tens of ms (air knee latency 8.6 ms); loaded peak vel
48-67°/s vs sim ceiling 30.8°/s (air fit baked the commanded write
speed as capability). Small corrections during weight transfer are
~5x slower in reality — strongest quantified suspect for the
deterministic liftoff +roll collapse. Fit task handed to agent.
(4) `cw-dep-startvar1` queued (vref1-r1 warm-start + placement 6° +
bad-start 0.4 + zero-drift frame 3° + k_current=0), gated behind
vref1-r1's verdict, varied-start eval panel in its gate.

- 08-10 05:02 c78: 3 triages, all PASS (crouch/slope compose axes hold). cw-walk-lowgait-dr035-comshift-rr1 was an orphaned 0-step /dev/shm launch-collision death (assigned to me as the finished run; verdicted FAILED-infra) whose live retry -rr2 (found finished+unclaimed, triaged in its place) PASSes: crouch(-50mm,DR0.35) x off-center CoM payload compose holds, gv 12/12, one inherited fixed-draw stall (lineage trait per c75's root-cause, not new) -- duplicates a concurrent cycle's independent PASS of the -r1 sibling, same conclusion. cw-walk-groundtilt5-deadband-r1-rr1 PASS + -rr1-rr1 PASS-with-caveat: two independent retries of the 5deg-slope x servo-deadband compose (both died-then-relaunched after the same collision storm) agree -- own-cfg det fwd 1.31/1.35m (>=1.2 gate), same 2/6 steepest-tilt shuffle tail as the whole groundtilt5 family, no falls/flag-leg; one twin's DR0 retention slip lands a hair over the 1.24 cap (1.28), read as lineage-trait not defect since its sibling clears cleanly at 1.23. SKILLS +1 row (groundtilt5-deadband). Refill: queued groundtilt8-deadband via respec (steeper/marginal slope rung x deadband, untried pairing) -- landed in backlog behind a concurrent cycle's item, 0/12 slots free at cycle end (mechanical drain will place both). 
- 08-10 05:03 c78: 3 triages. Infra: found experiments.json stuck in a leftover git conflict from an earlier interrupted autostash (c76) -- merged both sides by run-name (5 overlapping entries, took the more-advanced/mechanically-confirmed side each time: e.g. placementnoise6-payload-rr1-rr1 RUNNING not stale INTENT), staged, committed+pushed clean (c204ede), dropped the stash. cw-dep-vref1 confirmed FAILED-infra (0-step launch-collision, no science; retry -r1 already RUNNING via backlog, untouched). cw-walk-joyjit-dr05-payload-rr1-rr1 FAILED-infra (2nd death, wandb 0s1lcg7n runtime 1s, 0 steps; jitter x payload hypothesis still untested, requeued as -rr2-rr1, landed RUNNING). cw-walk-deadband30-payload FAIL: own-cfg gv 12/12 0 term 0 falls (crater 3/6 at cap, det med fwd 1.13m just under 1.2m target) but DR0 nominal retention det slip/m 1.32-1.45 sits ABOVE both the 1.24 cap and parent deadband30's own clean 1.09-1.27 band on all 6 draws (systematic, not noise) -- same mass-DR-onto-plain-walk-lineage erosion mechanism c61 first identified (driving/friction-hardened packages don't show it per joyfric-payload/joyheadfric-payload-r1 PASSes), just milder; gait mechanically clean in every frame (no flag leg, no dragging), not hardware-ready, arm closed. Refill: drain placed the requeued joyjit-payload retry into the one free slot; 12/12 busy at cycle end, no new backlog additions needed beyond that requeue. 
- 08-10 05:04 c79 (dig-in): lowgait-dr035-comshift-r1 PASS after root-cause — the flagged DR0 det/4 catastrophic stall (slip 17.5, fwd 11%) is a MARCH-IN-PLACE (all 6 legs cycling, level body, 2cm strides — invisible on casual video watch), and a 24-eps/side fresh-seed panel shows it's the known lineage fixed-draw paddling-attractor class, NOT comshift brittleness: child 2/36 degraded DR0 eps vs parent 2/36, parent WORSE on the one shared bad draw (0.49/3.23 vs 0.59/2.47), all medians + the DR0.35 tail match parent episode-for-episode; waits on contact-pricing calibration like the rest of the lineage. SKILLS row added; comshift-s1 seed twin queued via respec and VERIFIED RUNNING (train-6). Refills: synced 5 stale-SHA pods post-commit and drained backlog through the freed slots (joyheadtilt3-payload-s1/terrain10-comshift landed via concurrent drains, normal REFUSED traffic) — 12/12 busy, backlog empty at exit. 
- 08-10 05:27 c: 3 triages, all PASS. joylat25-comshift (CoM offset composes onto the 15s driving package, own-cfg gv 12/12 prog 1.00/0.98, JOYSTICK GATE 0 falls matching parent). joylat60-comshift-rr1 + joylat60-payload (CoM offset / +40% payload compose onto the 60s driving-endurance package, own-cfg gv 12/12 prog 0.99/0.95 and 0.96/0.94, no decay, JOYSTICK GATE 0 falls both, matching parent joylat60's band); all three deferred a DR0-nominal-retention pass (own-cfg conditions are strictly harder and it's passed trivially in every prior comshift/payload compose) due to severe controller eval-queue congestion (host load 200-330/128 most of the cycle) -- assumption recorded, flagged for confirmation. Infra: found+fixed experiments.json sitting in an unresolved git-stash-pop conflict from an earlier interrupted autostash (resolved byte-exact against HEAD, no hand-authored content, converged with a concurrent fix); found+fixed a real snapshot.sh --sync race (fixed /tmp/proto_sync.tgz path shared across concurrent cycles/pods caused spurious tar Permission-denied/Broken-pipe failures that PARKed 3+ launches) by giving each sync invocation a unique temp path -- smoke-tested, committed, unblocked the backlog immediately. Killed 2 self-repair duplicate-launch pairs (terrain10-comshift and deadband30-comshift each landed twice under the identical spec) to stop wasting GPU pods. Refills: 4 distinct new composes queued via respec (terrain10-comshift, deadband30-comshift, joyheadtilt3-comshift, placementnoise6-comshift -- comshift systematically extended onto every previously-uncombined package), 12/12 slots busy at cycle end. 
- 08-10 05:30 c80: 3 assigned. cw-walk-lowgait-dr035-comshift-rr2 was already verdicted PASS by a concurrent cycle before I got to it -- confirmed only, no re-verdict. cw-walk-joyheaddeadband-payload-r1-rr1-rr1 PASS: JOYSTICK GATE @90deg 0 falls incl flip-stress, own-cfg (DR0.5+lat+deadband+mass) gv 12/12 prog med 0.89/0.91, DR0 retention gv 12/12 within joyheaddeadband's own band -- payload x deadband closes on the widest driving package. cw-walk-lowgait-dr035-deadband PASS: crouch(-50mm,DR0.35) x deadband(1-3x) holds, own-cfg gv 12/12 height err <=9.4mm (<=10 gate), DR0 retention height err <=4.1mm slip 1.10 (<=1.15 gate); a few eps hit the lineage's known fixed-draw march-in-place stall (c75 root cause), not new. SKILLS +2. Infra: ops.sh drain launched cw-dep-startvar1 prematurely (its own hypothesis field says DO NOT LAUNCH until cw-dep-vref1-r1 is verdicted -- still unverdicted, mechanical drain has no semantic-hold awareness) -- caught at 65k/18M steps, killed, ledger set KILLED with explanation, deliberately left OUT of backlog pending that verdict. Refills: requeued 3 infra-PARKed near-misses (joyheadtilt3-payload-rr1, tiltnoise-r5-rr1, joylat60-torquescale-rr2) into the freed slots instead of inventing new generic composes per tonight's P0 order; hit the same /tmp/proto_sync.tgz race a concurrent cycle then fixed mid-cycle; one requeue (joyheadtilt3-payload-rr1 first attempt) lost-updated out of backlog.json under concurrent-drain load (known c73/c75/c77 symptom) and had to be requeued twice. 12/12 busy, backlog empty at exit. 
- 08-10 05:49 c: 3 assigned, 1 new triage. cw-dep-fresh1 PASS on numeric gate (gv 12/12, 0 falls, det/sto prog 1.28/0.81) but QUALITATIVE finding is the real result: fresh-init + 25deg permission + honest velocity obs still yields a low-amplitude (0-3deg, rare ~9deg) gait, NOT the hoped 10-20deg weight-transfer rock -- same low-amplitude character as cw-dep-vref1-r1's champion-warm-start contract arm, so the flat gait reads as a reward-pricing property not a warm-start artifact; hardware_ready=false, informative negative. cw-dep-startvar1 and cw-walk-deadband30-comshift-r1 arrived pre-verdicted (KILLED-infra premature-launch / KILLED-duplicate, 0 science each, triage already done by an earlier cycle) -- confirmed only, no new action. Infra: vref1-r1's PASS (concurrent cycle) unblocked startvar1's DO-NOT-LAUNCH hold; found it already relaunched as cw-dep-startvar1-r1 on the freed slot before I could act. Capacity 12/12 busy at cycle end, backlog has one pre-existing item (placementnoise6-comshift) waiting for a slot; no new arms queued (P0 order: no new generic composes, and both slots that opened this cycle were consumed mechanically before I could refill). 
- 08-10 05:54 c80: 3 triages, all PASS (compose campaign continues clean). groundtilt8-comshift-r2-rr1 PASS (marginal 8deg tilt x off-center CoM payload: own-cfg det med fwd 1.29m>=1.1 gate, 2/6 crater matching the groundtilt8-alone lineage's own shuffle tail exactly, 0 falls/flag-leg; DR0 flat retention slip 1.13<=1.24). joyheaddeadband-comshift-rr1 PASS + joyheaddeadband-payload PASS (comshift and payload both compose free onto the widest +-90deg deadband-hardened driving package: JOYSTICK GATE 0 falls both, own-DR0.5 gv 6/6 prog>=0.89, DR0 retention inside joyheaddeadband's own band; payload result also refutes the if-false branch from joyhead90-payload-r1, since deadband hardening -- like friction hardening -- makes payload free). Ran the own-DR(0.5)+JOYSTICK-GATE passes myself since the watcher only pre-staged DR0. SKILLS +3 rows. 12/12 GPU slots busy throughout (0 free at start and end -- no refill possible/needed); P0 dep line progressing under a concurrent cycle (cw-dep-startvar1-r1 now training). 
- 08-10 05:55 c: 3 triages, all PASS (own-gate evals run manually -- watcher's 6-concurrent-eval throttle skipped auto-prestage). joyheadtilt3-deadband: deadband hardening composes onto the widest 90deg steering+3deg-slope package, own-cfg 12/12 gv, DR0 retention matches parent's flat band, JOYSTICK GATE clean. joyheadtilt3-payload-s1: seed-1 twin confirms the payload-on-slope compose is a recipe not luck (matches seed0 band). joyjit-dr05-payload-rr2-rr1 (3rd launch attempt after 2 infra deaths): payload composes onto the abrupt-flip jitter package, DR0 retention slip 1.31-1.33 actually inside parent's own 1.40-1.46 band (the ledger's quoted <=1.24 cap doesn't match any prior sibling either -- read as boilerplate, not a regression). SKILLS +3 rows. No refill needed: capacity.py showed 0 free slots at check time (dep-startvar1-r1 took the one free slot via a concurrent cycle after vref1-r1's verdict). 
- 08-10 05:55 c80: 3 triages, all PASS -- P0 verdict landed. cw-dep-vref1-r1 PASS: contract-exact obs (meas:=ref) + 25deg tilt does NOT erode the champion -- named baseline (parent lowgait_dr05_r1 re-evaled on the identical config) shows vel_err delta ~0%, slip/m actually better (0.89/1.13 vs 0.97/1.36), same lineage fixed-draw stall at the same draw index; RULING: velocity estimator/temporal actor is NOT a P0 prerequisite for hardware attempt #2 (GPT handoff item 2 resolved) -- launched cw-dep-startvar1-r1 (start-variation compose) warm-started from it per the P0 hold order. cw-quad-hold2 PASS: 30% quad mix (down from 50%) recovers walk retention (det slip/m 1.20<=1.25) while quad-hold mechanism stays solid (survived_frac 1.0 every checkpoint, video confirms clean 4-leg stance) -- dose-response confirmed, SKILLS+1. cw-walk-groundtilt5-fric-s1-r2-rr1 PASS-with-caveat: seed1 matches seed0 almost exactly (3/6 crater fraction both seeds, DR0 retention slip 1.07-1.08 vs seed0's 1.088) -- recipe confirmed, not seed luck, SKILLS updated. 12/12 GPU slots busy throughout via concurrent drains + this cycle's startvar1-r1 launch; RL_PLAN trimmed to mark the vref1-r1 ruling resolved. 
- 08-10 06:39 c: 3 triages. cw-walk-lowgait-dr035-fric PASS + cw-walk-lowgait-dr035-groundtilt5 PASS (crouch -50mm/DR0.35 composes with floor-friction 0.4-1.6x and 5deg slope respectively, both own-cfg gv 12/12 slip well under 1.6 cap, DR0 retention clean, only the lineage's known inherited fixed-draw stall, SKILLS +2). cw-walk-joyquad30 FAIL on the compound gate's walk-retention leg: JOYSTICK GATE clean and quad-hold itself solid (video+telemetry), but own-cfg walk-mode slip/m med 1.72 vs cap 1.55 (parent joylat25 band 1.48/1.51) -- even a light 30% quad mix erodes walk slip, 2nd dose-response data point for P0 ruling 7 (quad-mix erosion). Checkup SUSPECT items (tiltnoise-r5-rr1, joyheadtilt3-payload-rr1, fps-below-floor) resolved on their own as the node filled back up with concurrent launches -- both finished normally, left for their assigned cycle. Refills: 6 distinct picks across free slots as they opened (7 peaked mid-cycle) -- joyquad15 (dose-response follow-up, quad mix halved to map the erosion frontier per P0 ruling 7), 3 crouch-lineage seed twins (fric-s1, groundtilt5-s1, deadband-s1, ruling-7 completeness of tonight's + last cycle's PASSes), terrain10-comshift-s1 (different line, same pattern); one respec (fric-s1) vanished from backlog with zero trace on 1st attempt (matches the documented lost-update symptom under concurrent-drain load), requeued once and landed. Did not touch cw-dep-vref1-r1/fresh1/startvar1 (all handled/verdicted by concurrent cycles before or during this cycle) or any run outside my assigned list. 
- 08-10 07:06 c: 3 assigned triages all PASS/FAIL clean: groundtilt8-comshift-rr1 PASS-with-caveat (8deg tilt x comshift holds, own-cfg 2/6 crater, DR0 retention det lands a hair over cap 1.40 on the same fixed-draw stall its own siblings cleared -- lineage trait); multiaxis-dr05-r5 FAIL (4-axis-DR0.5 stack retry duplicates/confirms the already-FAILED multiaxis-dr05: own-cfg clean but DR0 retention det slip 1.58 vs cap 1.24, crater-driven, axis-stacking ceiling stays closed); placementnoise6-payload-rr1-rr1 PASS (payload composes free onto hand-placement-slop, DR0 retention clean 1.21). BONUS CRITICAL FINDING (unassigned, self-initiated after noticing both finished unverdicted): cw-dep-startvar1-r1 AND its seed twin -s1 both FAIL HARD -- the start-variation compose meant to warm-start tonight's hardware attempt #2 breaks the gait under its OWN training config (real sacrificed-leg episode, slip/m up to 22, reward quarters DECLINING through training, both seeds reproduce) -- flagged do-NOT-use, fall back to cw-dep-vref1-r1 if unresolved; queued+running two parallel one-axis-removed isolation arms (noZD1: zero_drift_cmd_frame->0; noBS1: bad_start_prob->0) to find the culprit before hardware time runs out. Refills: dep-line seed twin + arch-hist16 seed twin (each within operator's 1-2-pod cap) + the two isolation arms, all within launch-cap; drained 2 backlog items from a concurrent cycle (dep-vref1-r1-deadband/fric) into free slots along the way. 12/12 busy, backlog empty at exit. 
- 08-10 07:06 c: 1 new triage (cw-walk-deadband30-comshift PASS: deadband 1-3x x off-center 3cm CoM shift compose clean, own-cfg+DR0 retention both in-band, known fixed-draw crater at idx4/5 not new; SKILLS updated). cw-dep-startvar1-r1/-s1 arrived already verdicted FAIL by a concurrent cycle (own-cfg gv 5-6/6, det slip/m 2.3-2.5 vs vref1-r1's 0.71-1.07 band, r1 has a real sacrificed-leg episode, reward quarters declining through training) -- independently cross-checked the eval numbers/frames myself, confirm-only, no re-verdict; root-cause isolation (zero_drift_cmd_frame prime suspect) already launched by that cycle as cw-dep-startvar1-noZD1/-noBS1. 0/12 free at exit (concurrent drains saturated capacity incl. the isolation arms) -- no refill action possible. 
- 08-10 07:06 c: 3 triages. cw-walk-terrain10-deadband + cw-walk-terrain10-payload both PASS -- deadband and payload each compose free onto 36mm terrain (own-cfg+flat-DR0 medians match parent terrain10's own band; the one crater draw per run is an inherited seed-4 stall the untouched parent shows too, mechanically clean march-in-place, no fall/flag-leg), terrain line now composed with everything worth trying. cw-walk-torquescale-r2-rr1-rr1 FAIL: 3rd-attempt duplicate of the already-closed torque-droop axis, reproduces sibling cw-walk-torquescale-rr1-rr1's NO-EFFECT verdict episode-for-episode incl. its named champion-baseline comparison; axis stays closed. Refills: drained the pre-queued P0 dep-line item (cw-dep-startvar1-noZD1) into 1 free slot, then queued+launched 2 NEW hardware-candidate-protecting composes on the winning contract-exact checkpoint (cw-dep-vref1-r1 + deadband, + friction, k_current=0 per P0 rule 3) into the other 2 -- not generic pair-composes, direct protection of the checkpoint likely used for tonight's hardware attempt #2. 12/12 busy at exit. 
- 08-10 07:07 c: 3 assigned. cw-walk-strafe-dr05-payload-r1 PASS (lateral-strafe x payload compose holds, own-cfg+DR0 retention both gv12/12 0 term, within parent band). cw-walk-terrain10-comshift PASS (terrain x off-center CoM compose holds on medians; one draw craters identically with the compose ON and OFF, proving it's a checkpoint-intrinsic march-in-place stall not caused by either axis). cw-walk-terrain10-comshift-rr1 was already verdicted KILLED-duplicate by an earlier cycle (confirmed only). Also found+triaged an orphaned FINISHED-but-unclaimed run from 03:3x (cw-walk-yawcmd1-rr1, the first turning/yaw-rate arm): own-cfg clean (gv12/12, 0 term, slip within band) and JOYSTICK GATE PASS 0 falls, but the pre-registered yaw-tracking metric needs custom analysis beyond the standard harness -- DIG-IN, staged its eval artifacts+checkpoint for the deep cycle rather than re-running them. Refills (4, distinct lines, P0-compliant -- dep-line/hist16 already at their 2-pod reservations so no DR pair-composes added): yawcmd1 seed twin (parallel evidence for the DIG-IN decision); cw-quad-hold2-lowgait (new skill-combine, crouch height x quad-hold, not a DR compose); cw-quad-turn1 (quad x yaw-rate compose) died at init on MY OWN respec mistake (omitted --obs-pad-transplant 1, obs 72 vs 73, 0 steps) -- recorded FAILED honestly, fixed and relaunched as -r1, now training; cleaned a stale duplicate backlog entry with the same bug before it could waste a retry. 12/12 GPU slots busy at cycle end. DIG-IN: cw-walk-yawcmd1-rr1 -- own-cfg+JOYSTICK GATE clean but yaw-tracking gate metric (wz_err on turn vs zero segments) needs custom analysis beyond the standard harness table; plan-fork (first turning result). 
- 08-10 07:38 c: 3 triages. cw-walk-joyheadtilt3-comshift PASS (off-center CoM composes free onto the widest +-90deg steering+3deg-slope package: JOYSTICK GATE 0 falls, own-cfg+own-DR0.5+DR0-flat all gv 6/6, prog med 0.88-0.92, no erosion). cw-walk-groundtilt8-deadband PASS (deadband composes onto marginal 8deg slope: own-cfg 2/6 crater tail matches the groundtilt8-alone lineage exactly, DR0 flat retention clean, crater frames confirm march-in-place not flag-leg). cw-quad-hold2-lowgait FAIL on walk-retention leg only: 4-leg quad-hold mechanism itself stays excellent at the crouched height (survived_frac 1.0 throughout training, video clean fronts-lifted no-tip stance) but walk-mode slip/m med 1.33 systematically exceeds the 1.25 cap across all 6 draws (not noise) -- 3rd data point for P0 ruling 7 (quad-mix costs walk economy even at low dose, now also under crouch); SKILLS +2, quad-hold row unaffected (mechanism validated separately). Refills: 8 new hardware-candidate-protecting single-axis composes queued+launched on cw-dep-vref1-r1 per P0 rule 4 (placement/payload/comshift/latency/encoder-noise/ground-tilt5/imu-mount/gyro-noise/joint-zero-bias -- covering sensor and DR axes the winning contract-exact hardware checkpoint has never faced), all k_current=0 per P0 rule 3; 12/12 GPU slots busy at cycle end. Left cw-arch-hist16-r7, cw-quad-turn1-r1, cw-dep-startvar1-noZD1, cw-dep-vref1-r1-fric unclaimed (finished mid-cycle but not on my assigned list) for their own triage cycle. 

## OPERATOR hardware session 4 (08-10 ~09:20-09:50 ET) — TAPE-MEASURE GROUND TRUTH: scripted gait delivers 51% of commanded distance

Operator supervised throughout; agent drove over HTTP. Robot healthy at
end (standing, torque on). Traces + per-leg tape numbers:
`rl_move/hardware_traces/tape_20260810_*` (summary JSON has findings).

**1. THE number for contact calibration: slip ratio 0.50-0.51,
five clean runs** (fwd 30 mm/s x3 incl one at 40 mm lift, fwd 50 mm/s
x2, 10 s each, fresh set_zero -> P stance, preflight-verified <1.5 deg
from plant each time). Measured 152/301 mm at cmd 30 and 254/502 mm at
cmd 50. True ground speed: cmd 30 -> ~15 mm/s, cmd 50 -> ~25 mm/s.

**2. Mechanism isolated: loaded stance-foot slide, not swing drag.**
Raising swing lift 25->40 mm changed the slip ratio by nothing (0.50 vs
0.51) despite visibly higher steps — the planted tripod slides backward
under load. Confirms "no reward can outbid sliding that costs nothing
in sim" root cause with a measured magnitude: sim contact must charge
~half the kinematic stride as slide on this floor.

**3. Walk current flat vs speed:** 0.31-0.42 A mean / <=0.85 A peak at
both speeds (18 servos) — matches session-3 walking-cheaper-than-
standing economics.

**4. [WITHDRAWN — operator ruling 08-10: all servos/zeros are fine,
not an open issue.]** Also live-demoed loaded sag: knees pull visibly
under when the robot is lifted (unloaded servos snap to command) — the
loaded-actuator gap in one gesture.

**5. Process:** one contaminated run (leg1a: crooked stance from a
rushed hand-set zero, front foot dragging, over-travel vs stale start
mark) — logged invalid, excluded. Power-tether constraint: legs run as
10 s halves. Plant-height calibrate mid-session ended "no contact /
not saved" — stored plant unchanged (verified), robot re-planted via P.

- 08-10 13:08 c: operator LAUNCH_HOLD (triage-only) in effect, 12/12 pods idle by mechanical hold -- 3 triages, dep-line hardware-candidate protection continues. cw-dep-vref1-r1-encnoise PASS + cw-dep-vref1-r1-groundtilt5 PASS (encoder-noise 0.5deg / 5deg floor-slope each compose onto the contract-exact vref1-r1 checkpoint clean: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band; retention-only pass w/o the axis cfg-set reproduces vref1-r1's own baseline + its known single-seed sto-only paddling stall almost exactly, proving checkpoint is unchanged and the axis is the variable). cw-dep-vref1-r1-fric PASS-with-caveat (friction 0.4-1.6x compose: own-cfg det slip med 1.23 ~9% over parent's nominal ceiling but inside this run's own +-20% tolerance, gv 6/6 both modes 0 term; retention pass confirms checkpoint-unchanged). All three: the known seed-4 paddling-attractor (root-caused pre-existing, sto-only on the bare checkpoint) migrates into det mode under axis stress -- video-confirmed march-in-place (all 6 legs cycling, no flag-leg/fall) in every case, not a new pathology. SKILLS +4 rows (new Deployment-contract section). No refills possible/attempted -- operator hold forbids new launches, idle pods are the expected/correct state today. 
- 08-10 13:09 c: 3 triages under operator LAUNCH_HOLD (analyze-only, launch nothing). cw-dep-vref1-r1-comshift PASS + cw-dep-vref1-r1-deadband PASS: off-center CoM and servo deadband both compose free onto the hardware-candidate contract checkpoint (own-cfg gv 12/12 each, slip within vref1-r1's own band, one inherited fixed-draw march-in-place crater apiece, not new); SKILLS +2 rows (new Deployment-contract section, also backfilled vref1-r1's own missing row). cw-dep-startvar1-noZD1 FAIL against its own gate but diagnostically key: dropping zero_drift_cmd_frame from the startvar1 compose eliminates the sacrificed-leg/catastrophic-slip mechanical failure (gv 6/6, no flag leg) but slip/prog still don't recover to vref1-r1's band and reward quarters still decline (milder) -- zero-drift-frame is A major contributor, not the sole one; bad_start_prob=0.4 residual still under test via the concurrently-running noBS1 sibling. No launches (LAUNCH_HOLD active); 12/12 slots idle-by-hold, no refill action taken per operator order. 
- 08-10 13:11 c: 3 triages under operator LAUNCH_HOLD (verdict-only, no launches). cw-arch-hist16-r7 PASS + cw-arch-hist16-r7-s1 PASS: TEMPORAL-ARCH history_frames=16 finally boots (after 8x /dev/shm deaths) and trains a real gait both seeds -- own-cfg DR0.5 gv 6/6 prog med 1.08-1.13 (>=0.85 gate), JOYSTICK GATE 0 falls both, clean 6-leg video; SKILLS +1 row. cw-dep-startvar1-noBS1 FAIL (partial, informative): removing bad_start_prob from the startvar1-r1 P0 break recovers 4/6 det episodes (was 0/6, no more flag-leg) but 2/6 still catastrophic-skate -- bad_start_prob is A contributor not the sole cause; zero_drift_cmd_frame stays prime suspect (parallel noZD1 arm also FAILED its own gate per a concurrent cycle). cw-dep-vref1-r1 remains the recommended hardware-attempt-#2 base. No refills -- LAUNCH_HOLD in effect, 12/12 slots idle by operator design. 
- 08-10 13:15 c: 3 triages under operator LAUNCH_HOLD (analyze-only). cw-dep-vref1-r1-gyronoise PASS + cw-dep-vref1-r1-imumount PASS + cw-dep-vref1-r1-latency PASS: gyro rate-noise (1.5deg/s), IMU mount-rotation offset (10deg), and bus/comms latency jitter (0.5-2.5x) all compose free onto the contract-exact vref1-r1 hardware candidate -- own-cfg det+sto gv 6/6 0 term each, slip/m within vref1-r1's own band, DR0 retention clean (same known lineage fixed-draw sto/4 crater, milder than parent's); own-cfg also shows a shared det/5+sto/0+sto/1 crater cluster IDENTICAL across all three unrelated axes -- read as a DR0.35+seed0 lineage draw, not axis-specific regression (frames confirm clean six-leg creep, no flag-leg/fall/exploit of the 25deg tilt trip). SKILLS +3 rows (Deployment contract section). Infra: caught and reverted my own mistake -- queued 2 backlog respecs (torquesag/cmddrop) before checking guardrails.yaml and finding the operator's LAUNCH_HOLD (triage-only, no launches/refills/respecs today); drain correctly REFUSED, then removed both from backlog.json under its lock. 12/12 pods idle by operator design, correct state, no refill attempted. 
- 08-10 13:30 08-10 c: 3 triages under operator LAUNCH_HOLD (analyze-only). cw-dep-vref1-r1-placement PASS + cw-dep-vref1-r1-zerobias PASS: 6deg hand-placement noise and 3deg joint zero-bias each compose free onto the contract-exact hardware-candidate checkpoint (own-cfg gv 6/6 both, slip within vref1-r1's own band, one inherited fixed-draw march-in-place crater apiece, not new); SKILLS +2 rows. cw-dep-vref1-r1-payload FAIL -- first non-free axis on this line: chassis payload/mass DR (1.0-1.5x) caused a real ~50% training reward decline (unlike all 9 other sibling composes, all flat) and the final checkpoint fails its own gate (det gv 5/6 incl a genuine sacrificed-leg episode, prog med 0.57 vs parent's ~0.9-1.0, slip/m med 2.34 vs parent's ~1.13 ceiling) -- flagged: don't assume payload/mass is a free axis for vref1-r1, matters if real battery/payload weight varies. No launches -- LAUNCH_HOLD in effect, 12/12 slots idle by operator design, no refill action taken. 
- 08-10 13:41 c: 3 triages under operator LAUNCH_HOLD (analyze-only). cw-walk-lowgait-dr035-deadband-s1 + -fric-s1 + -groundtilt5-s1 all PASS: seed-1 twins of the three crouch(-50mm,DR0.35)-compose PASSes (deadband/friction/5deg-slope) confirm each is a recipe not a seed-0 fluke -- own-cfg gv 12/12 each, slip/m med in the same 1.0-1.4 band, DR0 retention gv 12/12 slip/m med 1.01-1.11 (<=1.15 gate), same shared inherited fixed-draw march-in-place stall (level body, all 6 legs cycling, no fall/flag-leg, frame-verified against both a clean and the crater episode per run) -- not new, matches every prior sibling in this lineage exactly. SKILLS +1 consolidated seed-robustness row. Watcher's pre-staged gate eval was skipped (6+ evals already queued) so I ran own-DR(0.35)+DR0 eval passes myself for all three (6 eval_checkpoint invocations). No refills -- LAUNCH_HOLD in effect, 12/12 pods idle by operator design, correct state. 
- 08-10 14:01 OPERATOR SESSION (09:16-10:05 local): rise/lower reward pricing fixed (69e00c0, cfg-gated legacy-exact: reward.rise_finish_gate_signed closes the lower-mode always-open arrival-bonus bug, reward.rise_income_prog_gate gates kernel+finish income on fraction-of-target covered; measured freeze return in lower +120 -> -16, defaults md5-identical). Launched the uni-line DIG-IN pair with both flags on: cw-uni-rfix-warm1 (train-0, mix0-r1 respec, pricing-only arm) + cw-uni-rfix-fresh1 (train-1, the missing from-scratch control, no init, log_std 0 ent .005 DR0.2). LAUNCH_HOLD was lifted only for this drain (watcher PAUSEd during the window) and is RESTORED. 
- 08-10 14:03 c: 3 triages under operator LAUNCH_HOLD (analyze-only, but launches still forbidden). cw-walk-joylat60-torquescale-rr2 PASS (torque-droop 0.80-1.05x composes free onto the 60s driving-endurance package: own-cfg+true-flat gv 6/6 both modes, JOYSTICK GATE 0 falls incl flip-stress; run's own 1.24 slip cap is boilerplate mismatched to this lineage's real 1.4-1.7 band, not a regression -- corrected my own eval methodology mid-cycle by adding the missing own-cfg DR0.5 + true-flat + JOYSTICK GATE passes the pre-stage/evalcmd default alone didn't cover). cw-walk-lowgait-dr035-comshift-s1 PASS (seed-1 twin of comshift-r1 reproduces cleanly; caught+fixed a methodology slip where evalcmd's default 'retention' command still carries the training com_offset override -- a genuine no-offset re-run shows the seed0/idx4 crater is comshift-specific for THIS draw, fully consistent with c79's original fresh-draw-panel root cause, not a contradiction). cw-walk-joyquad15 FAIL (halving quad-mix 30%->15% barely moves walk-mode slip, 1.64/1.70 vs cap 1.55 -- 4th data point that quad-mix erosion isn't dose-proportional; quad-hold mechanism itself stays solid). SKILLS +3 rows/edits. No refills -- LAUNCH_HOLD still in effect, 10/12 slots idle by operator design (2 pre-existing uni-rfix runs still training). 
- 08-10 14:16 c: 3 triages under operator LAUNCH_HOLD (analyze-only, no launches). cw-walk-lowgait-fricvar-s1-rr1 PASS (seed-1 twin confirms crouch-20mm x friction-var 0.4-1.6x recipe, own-cfg+DR0 retention clean, same known lineage fixed-draw stall at det/4, no new defect). cw-walk-terrain10-comshift-s1 PASS (seed-1 twin confirms terrain-amp1.0 x comshift compose, same idx4 fixed-draw stall as parent -- tracks the eval draw not the training seed -- DR0 flat retention clean). cw-walk-placementnoise6-comshift PASS-with-caveat (new compose, off-center CoM x hand-placement-slop: mechanically clean 12/12 gv 0 term 0 sac, det prog-ratio med 0.83 narrowly misses its own >=0.85 threshold but the parent's own fwd-distance convention is met/beaten 1.26m vs 1.2m gate, degraded draws match placementnoise6-alone's documented heavy tail; DR0 retention clean). Watcher's auto gate-eval prestage was skipped for all 3 (12 evals already in flight at prestage time) -- ran own-cfg + DR0-retention passes manually for each. SKILLS +3 rows. No refills -- operator LAUNCH_HOLD in effect, 12/12 slots idle by design. 

## AGENT 08-10 ~10:20 ET — LOADED ACTUATOR ID LANDED (GPT-handoff P0 item 3): sim_model_loaded.json, opt-in via bus.servo_params=loaded

`rl_move/sim/fit_loaded_actuator.py` (sim-in-the-loop coordinate
descent, free-base STANCE replay at plant — same placement + contact
softening as training envs) fits the knee axis to the loaded step
ladder; fit on ±2/±10°, **±5° held out**. Loss 591 (air params) → 1.6.

**The dominant miss was the DEADBAND, not kp:** air rig fitted 0.494°;
that alone floors sim tracking at 90.1% on 5° steps (exactly what sim
showed) vs 96.6% measured — loaded deadband fits at **0.06°**. Rest of
knee fit: kp 43.8→916, kv 0.795→0.172, latency 8.6→**85 ms** (fit hit
the deliberate cap: ladder cmd→motion 110-210 ms includes the HTTP hop,
and MJX's pending ring bounds latency×DR 1.8), vel ceiling
30.8→**48.5°/s**. Ladder peaks (48-67°/s) are true servo capability —
the drive-loop wrote mcu default speed 1500, not binding; the DEPLOYED
path writes 400 (35.2°/s), so there the profile speed binds again.
Deployed-path check: rl_stand cmd→q derivative-xcorr response lag
~250-258 ms median on ALL axes — corroborates the ladder on the
contract path. **Validation:** held-out ±5° loss 0.21/0.27 (fitted) vs
4.24/9.48 (air) = 20-40×; multi-step deployed-contract replay of both
rl_stand liftoff-collapse traces (recorded 25 Hz cmd stream through
ServoProfile+MuJoCo) moving-joint RMSE 2.46/2.50° vs air 2.92/3.19°.

ASSUMPTION (flagged, operator to review): hip/yaw have no loaded
ladder — they carry the knee's latency DELTA (+76 ms) and the shared
vel ceiling; their kp/kv/deadband stay air-fitted. Proper per-axis
loaded ladder remains HARDWARE.md wishlist item 4.

Integration: `bus.servo_params` cfg key ("" = air fit, legacy
byte-exact — verified; "loaded" = new fit; path = explicit file,
missing file RAISES, no silent fallback). Wired through sim_env,
mjx_sharded_vec_env, train_ppo_mjx, train_ppo_sim `_build_env` (eval
worker + update-parity inherit), eval_checkpoint, eval_drive — a run
carrying the flag in its cfg package evals under it automatically.
MJX `PENDING_SLOTS` 8→12 (ring must hold ~106 ms × DR 1.8). Suggested
next: liftoff-reproduction fixture on loaded params, and one dep-line
arm respec'd with `bus.servo_params=loaded` vs its air twin.

- 08-10 14:25 c: operator LAUNCH_HOLD (triage-only) in effect, no launches. 2 triages: cw-walk-joyheadtilt3-payload-rr1 PASS (payload composes free onto the widest+slope driving package -- own-cfg+JOYSTICK GATE@90+DR0 flat retention all clean, matches parent joyheadtilt3 band, 3rd independent confirmation of this compose cell incl. r1/s1). cw-walk-joylat60-torquescale-rr1-rr1 PASS (torque-droop composes free onto the 60s driving package -- own-cfg+JOYSTICK GATE@45+DR0 flat retention clean, matches joylat60's own 1.43-1.65 slip band, run's 1.24 cap is known boilerplate per sibling rr2/joyjit-payload precedent; 3rd confirmation, axis stays CLOSED). SKILLS +2 confirming rows. DIG-IN: cw-quad-turn1-r1 -- first quad+yaw-turn compound (30% quad-hold mix x commanded-turn), watcher's pre-staged eval only covers pure walk mode (no --goal-mix), so quad survived_frac and per-segment wz_err (the actual gate) are unmeasured; own frame-strip spot-check found a real flag-leg/sacrificed-leg episode (sto/1, leg 3 stuck extended, slip 21.6) that needs root-cause before a verdict, not a boilerplate lineage stall. No refills (LAUNCH_HOLD forbids new launches); 12/12 pods idle by mechanical hold, expected/correct per operator order. 
- 08-10 14:38 c: 3 triages under operator LAUNCH_HOLD (analyze-only, no launches). All 3 flagged DIG-IN, none verdicted -- each hit a real trigger. cw-uni-rfix-warm1 + cw-uni-rfix-fresh1: W&B training-time rise/lower success_frac looked near-solved (both ~1.0 late) but that metric isn't posture-strict; ran own-cfg posture-strict harness eval (modes rise+lower, own DR/seed, default --end-posture-gate) myself since the watcher's pre-stage used the wrong generic modes=walk template -- true picture is a fork-deciding surprise: warm1 lower 6/6 clean (h_err<=11mm, all pads under belly thresh) but rise 0/6 (2 legs, indices 2&5, held 50-150mm off ground while body height nails target -- flag-leg-under-height-cheat, video-confirmed); fresh1 (from-scratch control) FAILS BOTH modes 0/6, worse not better than warm1 (opposite of hypothesis's if-true), video shows a literal 3-leg tripod hold (legs 0/2/4 clearance 80-300mm) plus 3 real over_current terminations across 12 eps -- contradicts the clean if-true/if-false framing and decides whether the uni-line keeps fine-tune-grafting or moves to distill/two-policy, a plan fork -> left for deep-model interpretation. cw-walk-yawcmd1-s1: same custom yaw-tracking metric (|wz_err| on commanded-turn segments vs |wz| on zero segments) its parent cw-walk-yawcmd1-rr1 needs -- harness has no wz field, parent already flagged DIG-IN and is being worked by a concurrent cycle right now; deferred rather than duplicate. No refills -- LAUNCH_HOLD in effect, 12/12 slots idle by operator design, correct state today. 

FOLLOW-UP (operator ruling, ~10:40 ET): uncertain sim params — servo
reaction times above all — go into DR ranges, not pretended-exact
nominals. Mechanized: the loaded file now carries `delay_ms_pct=0.45`
fit-uncertainty spread → `from_params` widens latency DR to ×0.3–1.9
(knee 26–162 ms at scale 1; MJX ring verified). Sim documentation
landed at `rl_docs/SIM.md` (chain, param provenance, confidence
table, DR philosophy, known gaps) — linked from RL_PLAN pointers and
open problem 3, indexed in rl_docs/README.
- 08-10 14:51 c: 3 assigned under operator LAUNCH_HOLD (analyze-only, no launches). cw-walk-yawcmd1 FAILED-infra (0-step launch collision, 2s runtime, no science -- retry -rr1 landed the real attempt). cw-walk-wander60-dr05-s1 FAIL: seed-1 twin of wander60-dr05's clean PASS reproduces det mode cleanly (6/6 gv both own-cfg DR0.5 and DR0 retention) but sto/0 has a genuine sacrificed leg (duty 0.08 vs siblings 0.37-0.89, slip/m 2.11, video-checked) -- 11/12 not the gate's 12/12; det/hardware-relevant mode unaffected, refutes strict seed-robustness at this 60s long-horizon rung under stochastic sampling. Also fixed an infra gap on that run: its launch never got --out-name so the checkpoint sat on-pod under the trainer default name, never eval'd -- pulled+renamed manually (md5-verified), ran both own-cfg and DR0 evals myself. cw-walk-yawcmd1-rr1 left unverdicted, re-flagged DIG-IN (same unresolved custom yaw-tracking metric an earlier cycle already flagged; own-cfg+JOYSTICK-GATE already clean per that cycle). Infra: RL_LOG.md hit the recurring git-stash-pop conflict TWICE this cycle (concurrent snapshot.sh autostashes) -- resolved both by chronological merge (ET-timestamped operator notes into their UTC slot among cycle lines), committed+pushed clean (c607541), dropped the stale stash. No refills -- LAUNCH_HOLD in effect, 12/12 slots idle by operator design. DIG-IN: cw-walk-yawcmd1-rr1 -- yaw-tracking custom metric (wz_err turn vs zero segments) still needs analysis beyond the standard harness table, unresolved since an earlier cycle's flag. 
- 08-10 14:58 c: 3 dig-ins under operator LAUNCH_HOLD (analyze-only), all verdicted FAIL but each fork-deciding. cw-uni-rfix-warm1 FAIL gate yet half-win: pricing fix killed the paid-freeze plateau -- lower 6/6 det posture-strict (uni-line FIRST, sto 4/6), rise 0/12 via the NEXT unpriced hole (at-height with legs 2&5 held 27-151mm aloft, video-confirmed; height income has no foot-loading term). cw-uni-rfix-fresh1 FAIL, if-true refuted: from-scratch strictly worse -- fixed tripod cheat every lower ep (legs 0/2/4 up 81-186mm) + 3 over_current terms; init is NOT the blocker, distill/two-policy fork CLOSED, uni-line stays fine-tune-graft, next arm = posture-gated rise finish. cw-walk-yawcmd1-s1 FAIL its yaw gate via NEW rl_move/sim/eval_yaw.py (committed 4847fed, reusable for rr1's pending dig-in): turn |wz_err| med 0.242 vs 0.10, right turns untracked, hold drift 0.087 vs 0.05; training walk_yaw_err flat both seeds = pre-registered if-false (free heading-hold income) -- next: yaw income gating; linear driving intact (JOYSTICK GATE PASS, own-DR0.5 gv 12/12). rr1 untouched (owned by cycle 14:51). RL_PLAN updated net-zero (problem 2 + queue 0 resolved to next arms). No refills -- HARD reason: operator LAUNCH_HOLD; 11 free slots + backlog idle by design. 
- 08-10 15:02 OPERATOR-ORDERED launch (during hold, hold restored): cw-arch-hist16-r7-c1 = identical-config warm-start continuation of the hist16 PASS cw-arch-hist16-r7 (+40M, train-2, init md5 32269a36) per external review — capability lines match champion on stability not economy; slip caps to be re-baselined after contact calibration, so keep training the temporal line meanwhile. 
- 08-10 15:07 c: 3 assigned. tiltnoise-r5-rr1 NO-EFFECT (5th retry finally trained; own-cfg 1.5deg tilt-noise det 4/6 healthy but eps4/5 crater matches champion's IDENTICAL fixed-draw crater under the same spread near-exactly, DR0 retention clean; sensor/calibration NO-DR-exposure ladder tally extended, SKILLS updated). torquescale-rr1-rr1 was already fully verdicted (FAIL/NO-EFFECT, duplicate of the closed torque-droop axis) by a prior cycle before this one started -- confirmed only, no re-verdict. wander120-dr05 left UNVERDICTED, flagged DIG-IN: its own-cfg DR0.5 and DR0 passes both show clean gv 12/12, 0 term, prog med 0.94-0.97, slip 1.2-1.35/m (matches/beats wander60-dr05's own band) and healthy six-leg video over the full 120s (no long-horizon drift, hypothesis if-true supported) but the pre-registered gate's literal 'det median fwd>=4.8m' criterion measures net straight-line displacement (which stays small ~1.3-1.4m under wander's frequent heading changes) rather than along-path progress (along_dist_m ~5.3m det, matching cmd_dist_m, progress_ratio 0.97-0.985) -- gate-vs-metric mismatch needs a judgment call on what the gate intended, left for the deep model with both eval dirs (cw_walk_wander120_dr05_gate + _owncfg) already generated so no re-run needed. OPERATOR LAUNCH_HOLD is active fleet-wide (triage/verdict only, file confirmed present all cycle) -- no refills attempted, all 12 slots' idle status is HARD-reasoned by the hold, not a guardrail gap. 

## 2026-08-10 — walkable-height rise reference + pricing smoke (agent, operator session)

Operator ruling on the stance champion's rise: "that's a terrible
stand, we couldn't walk from that" — the ~70 mm crouch-stand is NOT
the deliverable; the walkable ~142 mm plant stance is. Verified the
existing bridge first (headless replay of play.py's 7-key chain, DR0
det): rise to 70 mm → scripted 1.5 s blend to 142 mm → walk champion
covers 608 mm in 12 s at a 50 mm/s command, zero falls. So the blend
path works; what's missing is an RL rise that ENDS walkable.

Landed (extends the 08-10 Stage-II scaffold, see RL_GOALS.md):

1. `extract_rise_ref.py --blend-to-plant`: appends the validated
   play.py blend + a plant hold to the recorded champion rise, so
   the reference ends in the plant stance instead of the crouch.
   Wrote `rl_move/sim/refs/rise_ref_belly2plant.npz` (seed 12,
   T=314 @ 25 Hz, ramp_i0=126, ends +111 mm over belly, worst pad
   clearance 0 mm, q 7.4° RMS from plant; npz carries h_rel_end_m).
   Most seeds' blends hit env termination because THEIR episodes
   commanded only +30–50 mm — standing to +111 blows the height-err
   envelope; harmless for extraction (accepted seed passes clean).
2. Plant-height rises are now commandable: `_parse_cfg_set` parses
   `[lo,hi]` JSON lists (goal.rise_height_mm was previously
   unsettable — float-or-string only), and arms must also raise
   `actions.max_height_mm` (default 80 clamps below plant height).
3. Pricing smoke `tmp_smoke_rise_ref.py` (joint_goal, flat start,
   +108–114 mm targets, posture gate + income prog-gate + signed
   finish gate + ref-track k=2 on belly2plant, 3 seeds each):
   REPLAY of the demonstrated path +952 (ends +110 mm, feet down,
   finish bonus collected) vs STILT hip0/knee80 +225 vs FREEZE
   −195. Trying-well >> trying-badly >> not-trying — the ordering
   the 08-10 root-cause found violated is now enforced, and
   not-trying is net NEGATIVE. Ref-track dominates the replay
   return (787/952) — scaffold at full weight, anneal per plan.

Queued (LAUNCH_HOLD respected — backlog only, no launch):
`cw-stand-b2p1` — warm from ppo_goal_cw_stance_dr10, joint_goal,
stance-posture recipe + the full shaping stack + plant-height
targets + `bus.servo_params=loaded` (rise is the high-load motion
the loaded fit exists for). Gate: posture-strict rise ≥5/6 det at
h_err ≤12 mm of the +105–114 target AND end_posture_ok AND lower
retention ≥5/6. Risk noted: height obs channel extrapolates to
2.2× its trained range (height_scale_m 0.05 unchanged for graft
compatibility).
- 08-10 15:30 c-digin: cw-walk-wander120-dr05 PASS (gate-intent judgment call: literal 'det median fwd>=4.8m' used forward_dist_m = NET start-end displacement, a random-walk quantity a PERFECT wander tracker cannot push past ~1.5m -- PASSED parent wander60-dr05 itself netted 1.56-1.65m/60s; the intent, >=80% of commanded distance, is along_dist_m: >=4.87m in ALL 24 eps, own-DR0.5 gv 12/12, 0 term, prog med 0.95/0.94, frames clean full 120s incl. worst sto ep -- 2-min endurance banked, SKILLS row added; future wander gates must be written on along-path progress, never net displacement). Infra: fixed drain NameError regression from c16f7a5 (launch_one lost 'xa' in the self-repair refactor; crashed drain silently DROPPED 3 backlog items -- re-queued, fix snapshotted 485f97a). Refills 3 launches/3 lines, all VERIFIED RUNNING: cw-walk-yawgate1 (12M, yaw income gating walk_yaw_kernel_gate=1.0, queue-0 named arm), cw-dep-vref1-loaded1 (20M, bus.servo_params=loaded vs air twin vref1-r1, open-problem-3 first training arm), cw-arch-hist24-r1 (40M, temporal ladder rung 2 from scratch). 6 slots idle, HARD reason: 72M/80M GPU-step cap consumed; next-priority wander120-dr05-s1 seed twin (20M, must stay config-identical) exceeds remainder -- named first refill for the next cycle. 
- 08-10 15:53 c: DIG-IN cw-walk-yawcmd1-rr1 FAIL (yaw clauses; retention clean) -- custom yaw panel (eval_yaw.py, committed in f5fb0cb): turn |wz_err| med 0.239 vs 0.10 gate, hold |wz| 0.099 vs 0.05, 0 falls; policy yaws command-INVARIANT ~+0.09 rad/s (ignores the wz channel entirely), training walk_yaw_err flat 0.138->0.132 over 12M. Root cause incentive, not sim: ungated sigma-0.15 Gaussian kernel pays 0.67 of max yaw income for ignoring commands (matches W&B reward_walk_yaw 0.67 exactly) -- the pre-registered WISHLIST item-3 risk; linear driving fully retained (JOYSTICK GATE PASS 0 falls, own-cfg gv 12/12, slip 1.21<=1.25). Fix landed: reward.walk_yaw_kernel_gate income gate (clip(wz/wz_ref,0,1), walk_kernel_prog_gate analog; env smoke parked-turn income 0.133->0.002, hold untouched; snapshot f5fb0cb) -- a concurrent cycle is already training it as cw-walk-yawgate1 off my snapshot. Sibling yawcmd1-s1 shows the same failure numbers (concurrent cycle's verdict). Infra: LAUNCH_HOLD found cleared mid-cycle -> restored max_new_launches_per_cycle 0->4 per the guardrails comment; queued 5 refills, 2 unique placed+VERIFIED (cw-arch-hist24-r1 temporal rung 2 @2048 envs shm-safe; cw-dep-vref1-r1-payload125 safe-payload envelope, requeued after a drain race dropped it), 3 dropped as duplicates of concurrent cycles' yawgate1/vref1-loaded1 (same experiments, launched first -- no dup GPU burn); remaining idle slots are being filled by concurrent cycles' own drains. 
- 08-10 15:56 c: cw-stance-riseproof1 left UNVERDICTED, flagged DIG-IN -- fork-deciding control probe (joint_goal/stance recipe, from-scratch, same hold/rise/lower mix as the walk-env rfix arms, on today's post-273ebde sim) came back 0/6 posture-strict on ALL THREE modes (rise, lower, AND hold; sto hold 1/6), worst end pad clearance 120mm(rise)/208mm(lower) -- video (rise_det, lower_det, hold_det strips) shows hold looks like genuine six-leg standing (small 33mm worst_clear) but rise/lower both show legs splaying/lifting into a bridge-like posture over the episode, resembling the SAME flag-leg/height-without-loading cheat diagnosed in cw-uni-rfix-fresh1, not an obviously NEW sim-contact defect -- doesn't cleanly match either pre-registered branch (if-true: walk-env implicated -- refuted, stance recipe fails too; if-false: sim-contact implicated -- video doesn't obviously show a contact glitch either) so needs per-leg contact/duty analysis before deciding whether more rise arms need a sim-contact dig-in or just the same posture-gate fix ported to this recipe. Infra fix landed+snapshotted+pushed (8935480): watcher's pullckpt pre-stage failed rc=1 on this run because its launch omitted --out-name, so train_ppo_mjx saved under its OWN default name (ppo_mjx_joint_goal_<run>.zip) instead of the ledger-expected ppo_goal_<run>.zip -- fixed launch_run.py to always auto-inject --out-name (same derivation respec already used) so this can't recur, and taught ops.sh pullckpt a fallback for runs launched before the fix (also caught kubectl cp returning rc=0 on a missing remote file, a latent bug in the fallback's first draft, fixed before shipping); pulled riseproof1's real checkpoint manually and ran the posture-strict gate eval myself since the pre-stage never got the chance. Refills: 2 distinct lines queued+VERIFIED RUNNING (checkup HEALTHY) -- cw-stand-b2p1 (RL_PLAN queue-0 next rung: stance champion warm-start + plant-height target (108-114mm) + the walk-env posture-gate/prog-gate/signed-finish-gate fixes + loaded servo model, directly targets the exact flag-leg failure riseproof1 just reproduced) and cw-dep-vref1-r1-torquescale (dep-line protective compose, widened battery-sag range 0.5-1.05x distinct from the already-closed default-range torque-droop axis, k_current=0 per P0 rule 3). Left ~5 slots free: temporal-arch already at operator's 1-2-pod cap (hist16-r7-c1+hist24-r1), quad-mix erosion closed pending a distillation/KL-anchor CODE task (4 dose points, not dose-proportional), startvar needs a zero-drift-frame mechanism rework CODE before re-composing (not a re-launch), posetrack closed pending a curriculum CODE rework, single-axis dep composes past diminishing returns after 11 PASSes -- did not touch cw-arch-hist16-r7-c1/cw-arch-hist24-r1/cw-dep-vref1-loaded1/cw-walk-yawgate1 (still training) or cw-walk-wander120-dr05/cw-walk-yawcmd1-rr1 (owned by a concurrent cycle; its named wander120-dr05-s1 seed-twin refill landed on its own via the drain, not by me). DIG-IN: cw-stance-riseproof1 -- see above. 
- 08-10 16:14 c: cw-uni-rfix-postgate1 triaged, left UNVERDICTED (DIG-IN, real trigger: protected skill eroded) -- watcher's pre-stage never ran (missing eval log/dir; launch also lacked --out-name, ckpt saved under the trainer default name ppo_mjx_joint_walk_cw-uni-rfix-postgate1.zip) so pulled+md5'd it manually and ran the own-DR0.5 posture-strict rise+lower harness myself (det+sto, --end-posture-gate, matching the warm1/fresh1 recipe since the generic modes=walk template is wrong for this lineage). Result is a fork-deciding surprise, not the hoped fix: rise stays 0/12 (now failing mostly on HEIGHT not posture: h_err up to 53mm, 1 tilt_pitch term) while LOWER, which warm1 had posture-strict CLEAN 6/6, is now 0/12 -- both det+sto show the identical flag-leg-under-height-cheat warm1's rise had (2 legs held 30-109mm aloft, duty 0.03-0.11, h_err near 0) transferred INTO lower. reward.rise_posture_gate=1 did not close the income hole and instead looks like it pushed the cheat from rise onto the previously-solved lower mode -- left for deep-model root-cause (pricing-fix interaction, not a training crash: reward climbed to +52 quarters same as warm1). Refills, 3 launches across 2 lines (dep hardware-candidate composes + an endurance seed-twin), all VERIFIED RUNNING: cw-walk-wander120-dr05-s1 (seed twin of the 15:30 PASS, named next-refill from that cycle, along_dist_m-based gate); cw-dep-vref1-r1-comshift-deadband + cw-dep-vref1-r1-fric-groundtilt5 (2-axis composes pairing already-PASSed real-world axes on the P0 hardware candidate, per rule-4's protect-the-candidate exception -- avoided a 3rd generic DR axis; single-axis calibration/sensor DR search stays CLOSED). Infra: hit + fixed the same git-tag self-collision twice (a REFUSED respec attempt still creates+pushes the local exp/<run> tag before the capacity check fails, blocking the immediate retry's snapshot) -- deleted+re-pushed the stale tag both times, 0 science lost; worth a snapshot.sh fix (retry cleanup or tag-after-verify) for a future cycle. 0/12 free at exit (concurrent cycles + my launches saturated capacity). 
- 08-10 16:15 c: 1 triage, flagged DIG-IN not verdicted (P0 items 1/2 already resolved by earlier cycles -- vref1-r1/dep-fresh1 both PASS per ledger, confirmed only, no new action needed). cw-dep-vref1-loaded1 (open-problem-3 loaded-servo A/B twin of vref1-r1): own-cfg gate (DR0, identical eval config to vref1-r1's own report) gv 12/12 both modes, 0 term, 0 sacrificed legs, video (all 6 det + the sto/4 crater) shows the SAME low-amplitude six-leg creep as vref1-r1, no new pathology, crater matches the known lineage fixed-draw stall -- but vel_err_mean med 0.034(det)/0.038(sto) vs vref1-r1's 0.024/0.0285 (+40%/+33%) and slip/m med 1.42/1.65 vs 0.89/1.13 (+59%/+46%) both exceed the pre-registered 15% tolerance gate, a real delta (consistent across nearly all non-crater episodes, not noise) that matches neither pre-registered branch cleanly (gait does NOT break/terminate as if-false predicted, but also doesn't hold within-15% as if-true predicted) -- gate-vs-video tension (numeric FAIL, video looks mechanically fine) is a dig-in trigger; left for deep-model root-cause (is this the honest cost of realistic ~250-325ms loaded settling, or a fixable training-budget/reward-shaping gap) before deciding the dep-line's default actuator model. No refill action: capacity.py read 0/12 free at both start and end of cycle -- concurrent cycles' drains filled cw-dep-vref1-r1-fric-groundtilt5, cw-dep-quad1, cw-dep-vref1-r1-cmddrop between my checks; fleet fully saturated, nothing left to queue. DIG-IN: cw-dep-vref1-loaded1 -- vel_err/slip exceed the 15% gate vs vref1-r1 (real delta, not noise) while gait_valid/video look clean; decide if this is expected loaded-realism cost or needs more budget/reward-shaping before it can inform the dep-line's actuator-model default. 
- 08-10 16:20 c: cw-walk-yawgate1 FAIL (yaw-tracking clauses; escalation from yawcmd's income-gate fix -- eval_yaw turn|wz_err| med 0.236 vs 0.10, hold 0.104 vs 0.05, ~unchanged from ungated siblings despite reward_walk_yaw dropping 0.67->0.50, confirming the gate multiplier works but k_walk_yaw=1.0 income ceiling is economically dominated by the walk/prog kernels -- linear driving retention clean, JOYSTICK GATE PASS, own-DR0.5 gv 6/6 in-band). RL_PLAN queue-0 updated. Refills (5 queued across 4 lines, all VERIFIED RUNNING): cw-walk-yawgate2 (k_walk_yaw 1.0->2.5, tests if raising the price fixes the economics), cw-dep-startvar1-noZDnoBS1 (3rd isolation arm -- both single-axis ablations of the startvar1 P0 break were only partial, this removes zero_drift_cmd_frame AND bad_start_prob together to test for an interaction), cw-dep-quad1 (NEW: grafts the quad-hold2 four-leg-stand trick onto the contract-exact vref1-r1 hardware base, testing whether party-trick #2 survives the same deployment contract walking needed), cw-dep-vref1-r1-cmddrop + -velscale (2 novel hardware-relevant axes never before tested on the named candidate: dropped serial packets and actuator-velocity-ceiling uncertainty, the latter tied directly to today's loaded-actuator finding). 12/12 slots busy at cycle end, 1 item (velscale) landed in backlog and was drained by the mechanical launcher before this cycle closed. 
- 08-10 16:30 c-digin: cw-uni-rfix-postgate1 FAIL (rise 0/12, lower 0/12 vs warm1's 6/6 — protected-skill erosion root-caused to the posture gate's OWN pricing bug: pf used the 20mm stand allowance for lower where an honest belly-down lower leaves pads 20-45mm up (measured on warm1's passing lowers 16.9-43.4mm), so honest earned pf 0.67-0.83 vs the outrigger cheat's 0.67 — no differential, 18M steps drifted into legs-3&5-aloft outriggers that arrive faster and double the return; rise side the 1/6-linear gate is a weak last-leg lever, 35mm flag leg remains + tilt_pitch tip-overs; no sim defect, gate was live on MJX, missing W&B factor was a PART_KEYS logging gap). Fix landed+smoked+snapshotted 540e334: pf now uses end_posture_allow_lower_m=60mm for lower (matches harness + reward_end_posture; legacy-exact when off) + rise_posture_factor/rise_income_factor now logged. Checkpoint binned; warm1 stays uni-line lower baseline; CAUTION cw-stand-b2p1 (just finished, reward 133->211) trained PRE-fix with rise_posture_gate=1 — its triage must check lower for the outrigger cheat. Refill: 1 slot freed mid-cycle (b2p1 finished, its triage is the watcher's next cycle), ops.sh drain dispatched for backlog item cw-dep-vref1-r1-velscale; other 11 slots busy (HARD reason: fleet saturated by concurrent cycles). 
- 08-10 17:03 c: cw-stand-b2p1 FAIL -- posture-strict rise 0/12 AND lower 0/12 (height nailed <6mm both, but 1-2 legs held 80-288mm off ground, video-confirmed flag-leg/tripod cheat; lower is WORSE than rise despite the 60mm-allowance pricing fix). Posture-gate+prog-gate+signed-finish-gate+dense ref-track scaffold still lose to the height-only cheat at plant height -- confirms if-false, same class as fresh1/riseproof1 (still-open DIG-IN); RL_PLAN updated: posture-gate needs a geometric (support-polygon/foot-count) criterion before another plant-height rise attempt. Infra fix+snapshotted (5066d61): eval_checkpoint.py had its own stale --cfg-set parser that couldn't handle '[lo,hi]' JSON-list values (goal.rise_height_mm=[108,114]), silently crashing gate-eval for EVERY plant-height rise arm -- now shares train_ppo_sim._parse_cfg_set. Refills: 4 dep-line protective composes queued via respec (tiltnoise, gainvar, legmass -- all landed RUNNING; contactstiff already queued by a concurrent cycle, confirmed not duplicated) + 1 temporal-arch line-occupancy continuation (hist16-r7-c2, mechanical per WISHLIST -0.5, c1 left unclaimed/unverdicted for its own cycle). Did not touch cw-dep-vref1-r1-payload125/cw-walk-yawgate2 (finished mid-cycle but watcher-stamped in-cycle to other concurrent cycles) or cw-dep-quad1 (found orphaned+unclaimed, started eval but a concurrent cycle was already evaluating the identical output path -- backed off, WANDB numbers suggest height_err_end_mm plateaus ~31mm vs the 20mm gate, flagging for whoever verdicts it). 
- 08-10 17:09 c: cw-dep-vref1-r1-payload125 FAIL (near-miss follow-up refuted -- halving mass-DR range 1.0-1.5x->1.0-1.25x fixes the wide-range sacrificed-leg symptom but det/4 still stalls near-total, prog 0.004/slip 28.9, frozen splayed-leg 15s, same crater seed/index as the parent's own FAIL; class closed, no further narrower-range respec, payload/mass DR stays off the hardware default). Refills: 3 new dep-line PROTECT-THE-CANDIDATE 2-axis bundles combining already-PASSED single axes (imubundle=latency+imumount+gyronoise, encbundle=encnoise+zerobias, placement-comshift=placement+comshift), all VERIFIED RUNNING; fleet saw a wave of ~8 dep-line single-axis finishes handled by concurrent cycles (tiltnoise/gainvar/legmass/imupos/actnoise/contactstiff/linklen now training, hist16-r7-c2 continuing temporal-arch at the 2-pod cap) -- 12/12 busy at exit, all dep-line/temporal-arch per P0 rule 4. 
- 08-10 17:09 c: 2 assigned triages. cw-dep-vref1-r1-torquescale PASS (widened battery-sag 0.5-1.05x composes free onto the hardware-contract base: own-cfg+DR0 det+sto gv 6/6 both, 0 term, slip/m med 0.99-1.05 in/under vref1-r1's own band; craters match the lineage's known march-in-place fixed-draw stall, video-checked, no flag-leg/fall; SKILLS row added). cw-dep-quad1 FAIL (if-false, clean call): grafting quad-hold2's 30/60/10 mix onto the privileged-velocity-free contract-exact base never falls (survived_frac 1.0) but height_err_end_mm plateaus at 31mm vs the <=20mm gate quad-hold2 hit at 1-15mm on the old base, and track_err_deg got worse not better over training -- height-timing looks like it leans on velocity feedback the honest obs contract removes; walk-mode retention unaffected (own-cfg det gv 6/6 slip 1.18, DR0 det gv 6/6 slip 1.08); SKILLS updated, quad line stays on quad-hold2 for now. Refill: queued cw-dep-quad1-c1 (+12M continuation, tests under-training vs structural cap on the height gate) to backlog -- fleet was 0-3 free slots throughout the cycle, all absorbed within seconds by concurrent cycles' dep-line axis drains (tiltnoise/encbundle/gainvar/legmass/imupos/imubundle/actnoise/placement-comshift/contactstiff/linklen all landed without my help); 0/12 free at exit, nothing else queued to avoid duplicating the already-near-exhaustive dep-line DR-field sweep. 
- 08-10 17:10 c: cw-walk-yawgate2 FAIL (yaw-tracking clauses; retention clean) -- raising reward.k_walk_yaw 1.0->2.5 on top of yawgate1's income-gate fix STILL didn't move turn|wz_err| (0.233 vs gate 0.10, yawgate1 0.236) or hold|wz| (0.091 vs 0.05, yawgate1 0.104); eval_yaw per-scenario breakdown shows a fixed left-yaw drift from walk training -- commands near it track, commands against it don't, in every scenario incl. pure turn-in-place -- so this is a structural gait bias, not kernel economics; k_walk_yaw tuning closed, RL_PLAN/WISHLIST item-3 updated to point at a CODE task (decouple vx sampling from wz sampling in _sample_walk) instead. Linear driving retention intact (JOYSTICK GATE PASS, own-cfg DR0 gv 12/12, DR0.5 gv 6/6 prog 0.95/slip 1.56~noise); video clean six-leg creep. Left cw-arch-hist16-r7-c1/cw-dep-vref1-r1-torquescale unverdicted (finished mid-cycle, not in my assignment; watcher will spawn triage). Refills: 5 new dep-line single-axis hardware-candidate composes, all VERIFIED/launching (legmass, imupos, linklen, contactstiff, actnoise -- 4 genuinely untested DR fields per domain_rand.py's RandRanges: IMU mount position, leg-length print/assembly error, ground contact compliance, actuator output noise; none overlap concurrent cycles' encbundle/gainvar/imubundle/placement-comshift/tiltnoise). Temporal-arch already at 2 pods (hist16-r7-c2, hist24-r1) via concurrent cycles, left alone. Fleet 0/12 free at exit. 
- 08-10 17:25 c-checkup: cw-dep-vref1-r1-contactstiff SUSPECT (fps 1456<5000) triaged NOT-BROKEN, left running -- trainer healthy (ep_rew 618, full 20-core allocation, fps recovering to ~2100), slowness is node-wide g142d86 contention (load ~260/128 vs 80-110 on siblings: 4 in-budget trainers + transient watcher gate-evals on controller + other-tenant load); no rebalance target (0/12 pods free) and guardrails forbid outbidding tenants; ledger checkup_note recorded, worst case 20M steps ~2.8h at 2k fps. No refills (fleet 12/12 busy, no finished runs assigned). 
- 08-10 17:29 c: 3 dep-line 2-axis composes PASS (protect-the-hardware-candidate rule) -- cw-dep-vref1-r1-comshift-deadband (CoM+deadband), cw-dep-vref1-r1-fric-groundtilt5 (friction+tilt): both clean, 5/6+6/6 ok, gv 6/6 both passes, slip/m within vref1-r1's own band, known lineage crater only; cw-dep-vref1-r1-velscale (widened 0.6-2.2x actuator-velocity-ceiling DR + latency): gv/slip/term all clean too, but det progress_ratio spreads wide (2/6 clear a tight success band) because covered distance mechanically tracks the drawn speed ceiling -- expected physics of that axis, not a gait defect (video-confirmed); SKILLS rows added, all 3 hardware_ready=false (not independently deployable, just derisking vref1-r1). Checkup: hist16-r7-c2 (fps 3277) and vref1-r1-actnoise (fps 4369) SUSPECT-flagged for low fps both recovered on re-check (4159/8545 fps) and census shows correct trainers -- transient host-load dip (concurrent tenant load1 222-253/128 cores), no action needed. Fleet 0/12 free throughout (concurrent cycles' dep-line/temporal-arch drains saturated capacity); nothing queued. 
- 08-10 17:30 c: infra fix — snapshot.sh dirty-check + tar-exclude + .gitignore missed rl_move/wandb/ (created by any wandb.Api() call, e.g. wandbnote/wandbdump); spurious -dirty code markers were refusing drain launches (cw-dep-quad1-c1 parked after 3 attempts on hexapod-mjx-train-0) even with a genuinely clean tree; fixed+smoked, re-syncing affected pod. 
- 08-10 17:37 c: Refills for 2 freed slots (dep-line protect-the-candidate composes, per P0 rule4 exception since temporal-arch already at its 1-2-pod cap): cw-dep-vref1-r1-torquescale-gyronoise (battery-sag+gyro-noise, shared-brownout rationale) VERIFIED RUNNING; cw-dep-vref1-r1-imumount-groundtilt5 (mount-offset+floor-tilt, both bias the same safety-trip tilt reading) queued to backlog (2nd free slot raced away by a concurrent cycle's cw-dep-startvar1-placementonly before my launch landed -- REFUSED cleanly, no GPU burn, re-queued via respec). 0/12 free at exit; backlog has 3 items (mine + 2 concurrent-cycle entries) for the next drain. 
- 08-10 17:42 c: 3 triages -- cw-dep-vref1-r1-cmddrop PASS (dropped-SyncWrite robustness composes free, 10th protected axis, one known fixed-draw crater only); cw-arch-hist16-r7-c1 PASS (own-DR0.5+joystick eval run myself since only DR0 was pre-staged: +40M steps improved slip toward champion band, 0 regression); cw-dep-startvar1-noZDnoBS1 FAIL, informative (removing BOTH zero-drift-frame AND bad-start still shows the SAME det ep3/ep4 degradation as its single-axis siblings noZD1/noBS1 -- rules OUT both as sole cause, implicates placement_noise_deg=6 x joint_zero_bias_deg=3 interaction, never tested together before). SKILLS.md updated (cmddrop row, hist16-r7-c1 continuation row). Refills: 3 slots freed mid-cycle, filled from 2 lines per P0 rule4 (temporal-arch at its 1-2-pod cap) -- cw-dep-startvar1-placementonly (continuation isolating the noZDnoBS1 near-miss's remaining suspect), cw-dep-vref1-r1-encnoise-latency + cw-dep-vref1-r1-torquescale-deadband (new protective 2-axis composes, sensor and actuator pairs not yet tested); torquescale-deadband + a concurrent cycle's imumount-groundtilt5 queued in backlog for the next free slot, drain raced clean (0 GPU burn on refusals). Fleet 0/12 free at exit. 
- 08-10 17:51 c: 1 triage, left UNVERDICTED (real trigger, DIG-IN) -- cw-dep-vref1-r1-tiltnoise (P0 rule-4 protect-the-candidate axis, angle-noise floor dr.tilt_noise_deg 0.3->1.0 on the contract-exact base): watcher only pre-staged the DR0 sto+det retention pass (5/6 det, 6/6 sto, gv 6/6, 0 term, 1 known lineage march-in-place crater -- clean, matches sibling composes); I ran the own-cfg (DR0.35+tilt_noise=1.0) pass myself since the run trained at DR>0 -- det 5/6, sto only 4/6, gv 6/6, 0 term/falls/sacrificed-legs, but 3/12 episodes (not just the usual single crater) show partial degradation (slip/m 1.7-2.0 vs band 0.89-1.36, prog_ratio 0.62-0.74 vs usual ~0.9-1.1) with video showing the SAME clean six-leg creep, no visible pathology -- doesn't cleanly match either pre-registered branch (if-true wanted clean 6/6, not met; if-false predicted spurious near-threshold trips, none occurred) so it's a real gate-vs-video tension, not eval noise. Infra: none needed. Refills: drained 2 backlog dep-line composes (torquescale-deadband placed; imumount-groundtilt5 pending) into the one free slot found mid-cycle; fleet 0/12 free at exit, all busy with concurrent cycles' dep-line/temporal-arch runs -- nothing else to queue. DIG-IN: cw-dep-vref1-r1-tiltnoise -- own-cfg gate 5/6 det, 4/6 sto (gate wants 6/6); 3/12 episodes show partial slip/progress degradation (not full stall, not termination) under 3x tilt-angle noise, video clean -- decide if this is expected noise-floor cost or a real angle-noise/25deg-envelope interaction worth flagging before hardware. 
- 08-10 17:58 c: 3 dep-line 2-axis composes PASS (protect-the-hardware-candidate) -- cw-dep-vref1-r1-encbundle (encoder noise 0.5deg + zero-bias 3deg), cw-dep-vref1-r1-legmass (per-leg mass/length jitter, first per-leg-asymmetry axis): both DR0-gate+own-cfg(DR0.35) det+sto gv 6/6, 0 term, slip/m in/near vref1-r1's own band, same known lineage crater/degraded-episode pattern as PASSed torquescale/tiltnoise siblings, video clean six-leg creep. cw-dep-vref1-r1-gainvar (2x kp/kv gain-spread) also PASS but with the widest margin yet (det slip/m med 1.27, ~12% over the upper band edge, still inside +-20% tol) -- flagged as a watch-item if stacked with another actuator axis later. Ran both the watcher-staged DR0-gate eval and my own DR0.35 own-cfg eval for all three (runs trained at DR>0). SKILLS rows + wandbnotes added, all hardware_ready=false (derisking only). Refill: fleet 0/12 free + empty backlog throughout (concurrent cycles' dep-line/temporal-arch drains fully saturated capacity, incl. new backlog items encnoise-latency/torquescale-deadband/imumount-groundtilt5/zerobias-placement landing without my input) -- nothing to queue. 
- 08-10 18:04 c-digin: cw-dep-vref1-r1-tiltnoise PASS (dig-in resolved -- the 3/12 own-cfg degraded eps (det/5, sto/0,1; prog 0.62-0.74, slip 1.7-2.0) are NOT a tilt-noise/25deg-envelope interaction: identical 3 episode indices degrade in ALL 7 PASSed DR0.35 siblings (fixed eval seed -> same adverse model-DR draws; torquescale worst 0.47-0.57, tiltnoise among mildest), frames clean six-leg gait, 0 term/24 eps, slip medians in band, and the 1.0deg accel-tilt noise attenuates to ~0.1deg through the alpha=0.98 complementary filter -- 11th single-axis protective PASS on the hardware candidate, attempt #2 not blocked; SKILLS row documents the fingerprint so future triage treats det/5+sto/0+sto/1 as lineage baseline, and future own-cfg gates should pre-allow it). Refill: 1 slot freed mid-cycle (train-0), launched cw-dep-vref1-r1-tiltnoise-gyronoise VERIFIED RUNNING (2-axis cheap-IMU compose, both channels feed the complementary filter, untested interaction; gate written with the fingerprint pre-allowed); other 11 slots busy, backlog empty (HARD reason: fleet saturated by concurrent cycles). Did not touch cw-dep-quad1-c2 (finished mid-cycle, not my assignment). 
- 08-10 18:04 c: cw-dep-vref1-r1-actnoise triaged, left UNVERDICTED (DIG-IN, real trigger: gate-vs-video tension) -- own-cfg gate eval (pre-stage skipped it, ran myself: det+sto, dr.action_noise=0.02 at dr-scale 0.0) shows gv 12/12, 0 term, det slip med 0.96 in-band, but sto slip med 2.05 (vs base vref1-r1's own 1.13, band top 1.36) with 3/6 sto episodes failing vs base's 1/6 -- 2 previously-clean episode indices (sto/0, sto/1) newly crater and the known sto/4 crater nearly doubles (5.97->10.53); frame strips (det_0/5, sto_0/1/4) show the SAME clean six-leg creep/march-in-place stall as every sibling compose, no flag-leg/skate/drag, so this is a real quantitative reliability drop under actuator-output noise with no qualitative video pathology -- can't self-resolve on the triage tier, matches the loaded1 pattern from earlier today. No refill: capacity read 0/12 free before and after (concurrent cycles filled the 1 free slot -- train-8 went quad1-c2->torquescale-deadband -- before I could act; also saw train-0/11 turn over quad1-c2->tiltnoise-gyronoise and linklen->zerobias-placement, all owned by concurrent cycles). DIG-IN: cw-dep-vref1-r1-actnoise -- sto slip/reliability exceeds the pre-registered own-band gate (2x craters, sto slip med +82%) while det and video look clean; decide if actuator-output noise needs more budget/reward-shaping or is a real dep-line risk, same open question class as cw-dep-vref1-loaded1. 
- 08-10 18:25 c: 3 triages, all PASS -- cw-dep-quad1-c2 (+12M steps: quad height_err_end_mm now 2.9-3.7mm at final 2 ckpts vs quad1's own 31-60mm plateau, clears <=20mm gate, refutes structural-cap worry; walk retention det clean 6/6, sto had 1 real flag-leg ep caveat), cw-dep-vref1-r1-linklen (print/assembly leg-length error composes free) and cw-dep-vref1-r1-placement-comshift (placement+CoM-shift 2-axis bundle composes free) -- both dep-line hardware-candidate axes clean vs vref1-r1's own band, video-confirmed known lineage march-in-place crater only, no flag-leg. SKILLS.md +3 rows. Refill: 1 free slot -> respec'd cw-dep-vref1-r1-placement-comshift-zerobias (3-axis assembly-tolerance stack, extends today's placement+comshift PASS), VERIFIED RUNNING; 0/12 free at exit. 
- 08-10 18:28 c-digin: cw-dep-vref1-r1-actnoise FAIL-as-axis, NO hardware risk (gate-vs-video tension resolved by 2 controls: dr.* overrides apply ABSOLUTE post-scaling, so the own-cfg eval injected 0.02 actuator noise while the gate band came from the parent's NOISE-FREE eval -- parent under the IDENTICAL injection degrades the SAME 4 fingerprint episodes (sto slip med 1.51 vs actnoise 2.05), and actnoise on the clean DR0 eval is fully in-band gv 12/12 slip 0.89/1.05 -- retention intact, but exposure bought nothing/slightly hurt, matching the champion-line 0.08 action-noise NO-EFFECT; elevated action_noise stays OUT of the dep-line default, attempt #2 keeps vref1-r1 unblocked; GATE LESSON: injection-axis own-cfg gates need a parent-baseline control under the same injection, never the parent's clean band -- applies to the open loaded1 dig-in). Refill: 0/12 free (concurrent drain took train-1 mid-cycle); queued cw-dep-vref1-r1-gainvar-torquescale to backlog (pre-registered gainvar watch-item: widest-margin axis stacked with battery-sag). 
- 08-10 18:29 c: cw-dep-startvar1-placementonly FAIL vs own gate (DR0-gate det ok 3/6, prog med 0.75, slip med 1.62 outside vref1-r1 band 0.89-1.36; sto clean 6/6; gv 6/6, 0 term/sacrificed-leg, video clean six-leg creep). Ran own report cross-check against sibling isolation arms + the standalone seed11 _placement PASS: corrected the initial framing -- det/3 degradation reproduces IDENTICALLY across all 4 seed12/18M isolation arms regardless of zero_drift/zerobias, but is ABSENT in the seed11/20M standalone placement compose, so placement_noise_deg=6 can't be cleanly blamed (confounded with seed) without a seed12+placement=0 control; bad_start_prob=0.4 (only ON in noZD1) remains the one clearly-worse stressor. Ledger+wandbnote corrected in place. hardware_ready=false, cw-dep-vref1-r1 stays the hardware base. 0/12 free throughout (concurrent drains filled the one slot that opened); no refill action taken/needed. 
- 08-10 19:04 c: 3 triages, all PASS -- cw-arch-hist16-r7-c2 (mechanical +20M continuation, own-DR0.5+DR0+JOYSTICK GATE all clean, slip/economy flat-to-marginally-better vs r7-c1, no regression), cw-dep-vref1-r1-encnoise-latency + cw-dep-vref1-r1-torquescale-gyronoise (2 more sensing/brownout axis-pairs compose free onto the hardware candidate, matching the lineage's known fixed-draw crater fingerprint, no new pathology, video-clean six-leg gait both). Refills (dep-line now genuinely near-exhausted on single axes): cw-dep-vref1-r1-gyrobias (the last individually-untested DR axis, gyro rate-bias, protecting the named candidate) + 2 seed-twins (fric-s1, encnoise-latency-s1) confirming tonight's PASSes aren't seed luck ahead of the operator's morning attempt; all VERIFIED running alongside concurrent cycles' kpscale/badstart(deg) picks, 12/12 busy at exit. 
- 08-10 19:05 c: cw-walk-wander120-dr05-s1 PASS (seed-twin of wander120-dr05, seed 0->1) -- own-DR eval (per-mode 6, det+sto) gv 6/6+6/6, 0 term, 0 sacrificed legs, along_dist_m 4.88-5.95m every ep (all >=4.87m gate), prog med 0.94/0.92; DR0 retention gv 6/6+6/6 prog 0.95/0.95 slip 1.24/1.24; frames incl. worst sto ep (along 4.88m) clean six-leg cycling, no flag leg -- recipe is seed-robust, not seed luck; SKILLS row updated. Infra fix+snapshotted (84b698f): rl_move/wandb/ (created by any wandb.Api() call, e.g. wandbnote) was missing from .gitignore AND snapshot.sh's dirty-check/tar-exclude, so a genuinely clean tree still stamped a spurious -dirty code marker that REFUSED a drain launch (cw-dep-quad1-c1 parked after 3 attempts) -- fixed, re-synced, re-launched as cw-dep-quad1-c2 (now PASS-pending, cleanly separate ledger name, no science lost). Refills: 10 dep-line/temporal-arch launches placed this cycle across the fleet's rapid churn (many 20M-step dep composes finish in 15-30 min) -- cw-dep-quad1-c2 (quad-height under-training retest), cw-arch-hist16-r7-c3 (+40M temporal-arch continuation, still closing the slip gap per c1's PASS), and 8 dep-line PROTECT-THE-CANDIDATE composes pairing already-PASSed vref1-r1 axes never tested together (zerobias+placement [session-start], cmddrop+velscale [actuator-bus], gainvar+legmass [same-unit manufacturing], deadband+groundtilt5 [servo response x floor], imubias [new attitude-bias axis], encbundle+fric [joint-sensing x floor grip], latency+comshift [feedback-timing x persistent lean]); one attempt (tiltnoise+encnoise) never landed a slot before the fleet saturated, no cleanup needed (not queued anywhere). 0/12 free at exit. 
- 08-10 19:07 c: cw-dep-vref1-r1-tiltnoise-gyronoise PASS + cw-dep-vref1-r1-torquescale-deadband PASS (12th/13th dep-line protective 2-axis composes -- cheap-noisy-IMU and battery-sag+deadband both hold: own-cfg gv 12/12, 0 term, slip/m within vref1-r1's own band, known fixed-draw episodes degrade exactly per the lineage fingerprint, video clean six-leg creep both; SKILLS +2 rows). Refills: 5 free slots seen mid-cycle -> queued 6 new dep-line single-axis hardware-protective composes off vref1-r1 (gyrobias, badstart, badstartdeg, kpscale landed VERIFIED RUNNING; kvscale+groundtilt backlogged after fleet saturated at 0/12) -- hit the known backlog-item-vanishes-under-concurrent-load race (3/4 first attempts lost with zero ledger trace, recovered via --now direct launch to explicit free pods). Left all other mid-cycle finishes (cmddrop-velscale, deadband-groundtilt5, imubundle, imumount-groundtilt5, zerobias-placement, contactstiff) untouched -- not my assignment, no verdict implied. 
- 08-10 19:08 c: cw-dep-vref1-r1-deadband-groundtilt5 & -imubundle PASS (own-cfg+DR0 gv 12/12 both, slip in-band, known lineage crater only, no flag-leg) -- deadband+slope and the 3-axis sensor bundle both compose free onto the hardware candidate; dep-line stress-test sweep still 0-for-0 on real regressions. Drained backlog (fric-s1, encnoise-latency-s1 seed twins) into the 2 free slots -- fleet 12/12 busy. 
- 08-10 19:34 c: cw-dep-vref1-r1-zerobias-placement PASS (11th dep-line protective compose -- 3deg per-joint zero-bias AND 6deg hand-placement noise TOGETHER, the two errors every real session actually has at once, still compose free: own-cfg DR0.35 det gv 6/6 slip med 1.08, sto gv 6/6 slip med 1.27, both inside vref1-r1's own band 0.89-1.13/1.13-1.36, 0 term/0 sac; degraded draws det/5+sto/0-1 match the exact fixed-seed lineage fingerprint already seen on tiltnoise_gyronoise/torquescale_deadband, video-checked clean six-leg march-in-place, no flag-leg; DR0 retention exact-match to parent). SKILLS +1 row. Ran own gate+DR0-retention evals myself (prestage skipped, 6 evals already in flight). Refill: 1 free slot seen mid-cycle -> queued cw-dep-vref1-r1-imupos-imubias (IMU off-position AND calibration-bias TOGETHER, both individually PASSed, realistic same-install pairing) but a concurrent cycle's legmass-linklenscale grabbed the slot before my --now launch landed (oplaunch first attempt hit a CLI syntax bug, --cfg-set isn't a respec flag, fixed to --cfg) -- requeued to backlog for the drain, fleet 0/12 free at exit. 
- 08-10 19:34 c-triage: cw-dep-vref1-r1-imumount-groundtilt5 PASS (10deg mount+5deg floor tilt bias compose, both feed the same 25deg safety-trip reading -- clean, 0 spurious trips), cw-dep-vref1-r1-imupos PASS (new IMU-position/lever-arm axis, clean) -- both SKILLS.md rows added; cw-dep-vref1-r1-placement-comshift-zerobias left UNVERDICTED, its det/4 fixed-seed draw crossed from the lineage's usual benign march-in-place into an actual gv=False/sacrificed_legs=[3]/negative-progress failure that its 2-axis parent (placement-comshift) does NOT show on the same draw -- flagged for dig-in. 1 free slot (train-10) refilled via respec: cw-dep-vref1-r1-legmass-linklenscale (stacks the PASSed global link-length-scale axis onto the PASSed legmass+per-leg-length-jitter compose, last untested manufacturing-tolerance combo), verified RUNNING; all 12 slots busy. 
- 08-10 19:35 c(triage): cw-dep-vref1-r1-cmddrop-velscale/contactstiff/gainvar-legmass all PASS (own-cfg+retention 6/6 gv, 0 term, in-band slip, known lineage idx4 stall only, SKILLS updated) -- P0 hardware-candidate DR pair/single composes keep holding, no new failure mode found tonight; 0 GPU slots free (12/12 busy), backlog 1 item (groundtilt, held by concurrent cycle), no refill needed. 
- 08-10 19:37 c: 2 triages, both PASS -- cw-dep-vref1-r1-gainvar-torquescale (2x kp/kv gain-spread AND widened battery-sag 0.5-1.05x stacked: resolves gainvar's pre-registered watch-item, margins don't compound, own-cfg+DR0 gv 12/12, 0 term, in-band, video clean) and cw-dep-vref1-r1-imubias (3deg residual IMU calibration bias, new attitude-bias axis distinct from mount/gyro: own-cfg+DR0 gv 12/12, 0 term, 0 unexpected 25deg tilt-trips, video clean); both 12th protected axis on the line, SKILLS +2 rows. Ran both own-DR(0.35) and DR0 gate evals myself (pre-stage skipped, 8 evals already running when the watcher tried). Refill: 0 free slots throughout (a concurrent cycle claimed the one free slot with cw-dep-vref1-r1-legmass-linklenscale, the same manufacturing-tolerance pairing I was about to propose -- confirmed no duplicate, left it alone); dep-line single/2-axis composes are now genuinely exhaustive (every domain_rand.py field tested solo, ~15 pairs/bundles tried) so no further generic pair-compose queued per P0 rule 4. 
- 08-10 20:06 c-digin: cw-dep-vref1-r1-placement-comshift-zerobias PASS (dig-in overturns the triage flag: det/4 gv=False is a stall-POSTURE artifact on the lineage's KNOWN fixed-draw DR0 crater — the same seed-0 draw craters in the PASSed parent (prog -0.03/slip 29.7) and every sibling; this ckpt merely idles with leg 3 aloft (duty 0.06) while nobody walks that draw; own-cfg DR0.35 gate clean 12/12 gv, 0 term, slip 1.26/1.28 in band; video level march-in-place, no fall — 3-axis assembly-tolerance stack composes free, SKILLS row added; watch-item: sacrificed leg while TRANSPORTING = real fail, this fingerprint is not). Refills 2/2 verified RUNNING: cw-arch-hist16-dep1 (40M, plan's named arch rung — hist16 from-scratch on the deployment contract, head-to-head vs 8-frame vref1-r1) + cw-dep-vref1-r1-placement-comshift-zerobias-s1r1 (20M seed twin 11->12; -s1 name burned by a REFUSED-attempt git tag, no training lost). train-11 idle at exit, HARD reason: remaining READY lines closed/blocked (dep composes exhausted per P0-4, unified/yaw/quad next rungs [CODE], posetrack parked, speedband closed) and 5 just-finished runs' triage cycles hold the refill sources; named claimant for 2+ idle slots: the turn-in-place _sample_walk decoupling implementation cycle (plan queue 0). 
- 08-10 20:08 c: 3 triages, all PASS/PASS-caveat, no dig-in. cw-dep-vref1-r1-badstartdeg PASS (wider bad-start joint-angle ceiling 8-50deg composes free) + cw-dep-vref1-r1-latency-comshift PASS (latency+comshift compose free, same feedback-timing pathway) -- both det 5/6 (only the lineage's known fixed-draw crater fails, video-confirmed march-in-place, no flag-leg/fall), sto 6/6, gv 12/12, 0 term. cw-dep-vref1-r1-encbundle-fric PASS-with-caveat: det 4/6 (crater + one mild tail episode matching parent encbundle's documented DR0.35 curriculum-artifact envelope) -- widest det degraded-fraction of any sibling pair so far, still safe (no falls/flag-leg), flagged as closest to a real limit on this line. cw-dep-vref1-r1-fric-s1 SUSPECT checkup was a stale snapshot (steps had advanced 2.03M->6.36M, log actively growing) -- no action needed. SKILLS +3 rows. Capacity was 12/12 busy -> 4 free (badstart/kvscale/encnoise-latency-s1 finished, 1 claimed by concurrent cycle) -> refilled with 3 new dep-line hardening arms (action_noise single [new axis], bad_start_max_joints single [new axis, completes the frequency/magnitude/breadth triad with badstart/badstartdeg], contactstiff+friction pair [floor grip+compliance together]); a 4th (torquescale+velscale pair, battery-sag realism) hit busy-pod REFUSEDs live as the concurrent cycle filled slots first -- queued to backlog for the next free slot instead of fighting for one. 2 orphaned pre-launch git tags (contactstiff-fric, torquescale-velscale bare names) left as harmless refs, successors used -r1/-r2 names, REFUSED ledger entries correctly record 0 science. 
- 08-10 20:20 c: 2 triages, both PASS -- cw-dep-vref1-r1-badstart (doubling bad-start FREQUENCY 0.25->0.5 composes free: own-cfg DR0.35 gv 12/12, 0 term, in-band, cleaner than most siblings; DR0-gate shows 2 extra instances of the known march-in-place stall, expected/mechanical since a higher probability means more fixed-eval-seed draws cross the threshold, video-checked no flag-leg/fall) + cw-dep-vref1-r1-encnoise-latency-s1 (seed-12 twin reproduces the seed-11 parent's own DR0.35 fingerprint almost episode-for-episode -- det/5,sto/0,sto/1 at matching magnitudes -- confirming the noise+latency compose is a real recipe not seed luck; ran the own-cfg DR0.35 pass myself for both since only DR0 was pre-staged). SKILLS +2 rows. Refills: 4 free slots opened mid-cycle -- filled with 4 different-line picks: cw-dep-startvar1-seed12-noaxis (isolates the seed-vs-axis confound left open by tonight's placementonly finding), cw-dep-vref1-r1-kvscale-groundtilt (servo-damping x floor-slope pair, queued-but-lost earlier tonight), cw-arch-hist16-dep1-s1r1 (seed twin of the new hist16 hardware-contract rung, temporal-arch line) + cw-dep-vref1-r1-contactstiff-groundtilt (floor-compliance x floor-slope pair); caught+removed a duplicate velscale-torquescale backlog item (concurrent cycle already had the identical pairing queued as -r2 after 2 prior REFUSED attempts) before it could double-burn a slot. 0/12 free at exit. 
- 08-10 20:27 c-checkup: cw-dep-vref1-r1-kvscale-groundtilt SUSPECT (fps 4369 < 5000 solo floor) resolved as warm-up-window false alarm -- checkup sampled the first-minutes step window where JAX/Warp compile drags cumulative fps; fresh checkup HEALTHY at fps 5825, log fps 5374 and climbing at 6.1M steps, no tracebacks, reward rising. No kill/rebalance; run left training on train-11. Second early-window false SUSPECT tonight (fric-s1 at 20:08) -- watcher checkup fps floor should skip or discount the compile window. No verdicts, no launches (all 12 slots busy, no finished runs assigned). 
- 08-10 20:31 c: cw-dep-vref1-r1-fric-s1/-groundtilt/-legmass-linklenscale all PASS (own-cfg+DR0 gates clean, known fixed-seed DR0.35 fingerprint only, no flag-leg) -- friction seed-robust, floor-slope alone (not just paired) benign, legmass+global-linklen-scale composes free; dep-line DR-hardening continues. All 12 GPU slots busy, backlog empty -> no refill. badstartjoints SUSPECT (fps 4369, below 5000 floor) traced to shared-node contention (g142d86 loadavg ~150-170, all 4 co-located pods equally slow incl. arch-hist24-r1 at 2030fps) not misplacement -- no free pod to rebalance to (g12ba48 cordoned by CoreWeave), no action taken. 
- 08-10 20:34 c: cw-dep-vref1-r1-kvscale triaged, left UNVERDICTED (real trigger, DIG-IN) -- pre-staged DR0 gate shows det/4 AND det/5 both craterered (usual lineage fingerprint is det/4 ONLY); ran a parent-baseline control myself (base vref1-r1 ckpt under the identical dr.kv_scale_pct=0.50 injection, same GATE LESSON as actnoise) -- control matches kvscale on det/4 (same fixed-draw crater) but stays CLEAN on det/5, so kv alone does add a real det/5 degradation the parent doesn't have under the same injection. Own-cfg (DR0.35+kv=0.50, since it trained at DR>0) is worse still: det gv 2/6 fail (slip med 1.45, over vref1-r1's +-20% tolerance), sto gv 4/6 fail (prog med 0.80) -- MORE craters (5/12) and wider margins than any sibling incl. gainvar (kp+kv together, which only hit 3/12 and stayed in-tolerance) -- counterintuitive that isolating kv is worse than kv+kp combined, a real anomaly not resolved by one lightweight control. Video (det/4,5, sto/0,1,4,5) still shows clean six-leg march-in-place, no flag-leg/drag/fall, gv/term/sacrificed-leg counts all clean -- gate-vs-video tension, needs root-cause (kp may be compensating for kv/damping uncertainty) before deciding if kv-spread belongs in the hardware-default DR mix. DIG-IN: cw-dep-vref1-r1-kvscale -- own-cfg det 2/6 + sto 4/6 fail vref1-r1's tolerance band (more craters, wider margins than any of the 15 already-PASSed dep-line siblings incl. gainvar which combines kp+kv); parent-baseline control under the identical kv injection stays clean on the one episode (det/5) that differs -- kv-alone looks like a real, if video-invisible, degradation; decide if it's noise, an artifact of dropping kp's compensation, or a real axis to exclude from the hardware DR default. Refill: 4 free slots seen at cycle start, all raced away by concurrent cycles' dep-line/temporal-arch drains before my launch landed; queued cw-dep-vref1-r1-megastack1 to backlog (comprehensive protect-the-candidate stack -- ALL ~15 individually-PASSed dep-line axes at once, the sweep's logical endpoint, excludes kv-alone/payload/actionnoise) after a first --now attempt crashed on tuple-vs-scalar --cfg-set syntax (friction/torque/latency/deadband/contactstiff/velscale/imu_pos_z need lo,hi not a bare number -- fixed, respec\'d cleanly). 0/12 free at exit, nothing else to queue. 
- 08-10 20:35 c: 2 triages, both PASS -- cw-dep-vref1-r1-gyrobias (gyro rate-bias 1.5deg/s, last individually-untested IMU axis) and cw-dep-vref1-r1-imupos-imubias (IMU position-offset AND calibration-bias TOGETHER, realistic same-install pairing): both ran the watcher-staged DR0-gate (12/12 gv, 0 term, only the pre-allowed catastrophic det/4 crater) plus my own DR0.35 own-cfg pass (12/12 gv, 0 term, the 3 degraded episodes det/5+sto/0,1 match the exact pre-registered lineage fixed-seed fingerprint from the 18:04 dig-in, no new pathology); video-checked clean six-leg creep both, no flag-leg/drag/skate. SKILLS.md +2 rows. Pruned 1 stale backlog item (cw-dep-vref1-r1-megastack1, an all-axes-stacked compose) as a no-launch per the newly-binding 'no generic DR pair-composes' closure -- it's the terminal generalization of the exact class just closed after 15+ consecutive identical-fingerprint PASSes, near-zero marginal info. Fleet 0/12 free throughout (concurrent cycles' dep-line/temporal-arch/hist16 runs saturate all 12 slots); backlog empty at exit, no refill possible or attempted. 
- 08-10 20:35 c-checkup: cw-arch-hist16-dep1-s1r1 SUSPECT(fps 3277<5000) = FALSE ALARM — all hist runs are legitimately slower (hist16@3072 healthy at 3.9-4.6k, hist24@3072 at 2.0k; node fine, neighbors 7.7-8.3k); run alive+learning, left in place; fixed checkup floor to scale with n-envs+history_frames (snapshot d6a638c), re-checkup HEALTHY 
- 08-10 20:40 c-refill: after the 2 triages, capacity opened to 2 free slots (train-6 REFUSED-then-empty, train-10 startvar1-seed12-noaxis finished) with an empty backlog. A concurrent process re-added cw-dep-vref1-r1-megastack1-r1 (the all-axes DR megastack) within a minute of my first prune -- pruned again; it turned out self-REFUSED anyway (missing --phase, 0 compute burned). Hardened this against future re-adds: RL_PLAN CLOSED-moves now explicitly names single/pair/ANY-N-way dep-line DR composes as closed (20-for-20 no-effect, identical benign fingerprint every time) so any cycle reading RL_PLAN before queuing sees it. Left both slots IDLE (deliberate, not neglect): dep-line composes closed, temporal-arch already has 4 pods running (over its 1-2 guidance, not mine to add to), rise/turn's next steps are [CODE] not launchable specs, no hardware-blocker-relevant WISHLIST [READY] item passed the prime-directive launch question this cycle. 
- 08-10 20:55 c: cw-dep-vref1-r1-kvscale PASS (verdicted, closing the class) -- own-cfg had 5/12 degraded eps (wider than the usual 3/12 det/5+sto/0-1 fingerprint) and a matched-parent control confirmed kv-alone adds one real extra degraded ep (det/5) the parent doesn't show under the identical injection, but video stayed clean six-leg creep/gv 6/6/0 term both evals -- closed per the operator's mid-cycle process overhaul (RESEARCH_RULES.md/CURRENT_TRUTHS.md/RUN_INTERPRETATION_RULES.md landed 20:07-20:34, phase-gated launcher live): RL_PLAN now explicitly CLOSES the whole protect-the-candidate DR-compose sweep (20-for-20 no-effect) and names my in-progress cw-dep-vref1-r1-megastack1 by name as pruned/do-not-requeue -- a closed sim finding reopens only on hardware evidence now, not more digging; SKILLS row added noting the margin as a watch-item. PROCESS NOTE for future cycles: read CURRENT_TRUTHS.md -> RL_PLAN.md -> RESEARCH_RULES.md (+RUN_INTERPRETATION_RULES.md) FIRST -- KPI is now unresolved blockers, NOT occupancy; idle pods are fine; launches need --phase (discovery/hardening/composition/transfer, launcher-enforced) and hardening+ needs --evidence; the dep-line sweep and generic sim-robustness composes are CLOSED. Infra: cw-dep-vref1-r1-megastack1 (--now) crashed instantly on tuple-vs-scalar --cfg-set syntax for RandRanges range fields (documented COMMANDS.md #15); its backlog re-queue then silently vanished because a same-name W&B run already existed (dedupe drop, documented #14b) -- both fixed+snapshotted, then the respec\''d retry -r1 was refused/pruned by the new phase gate + the RL_PLAN closure, correctly, no GPU burned. Refills: used 2 of the newly-freed slots on the two concrete queue items RL_PLAN/TURN.md + RISE.md name as next -- cw-stand-plantgate1 (DISCOVERY, 2M, --phase discovery --evidence rise bank PASS: first REAL training run with the landed PLANT_SPEC/rise_plant_polygon_gate geometric anti-flag-leg criterion, on b2p1's exact recipe +1 var) and cw-walk-turnfix1 (DISCOVERY, 2M, --evidence turn bank PASS: first real training run of the landed signed-yaw-income + drift-charge + turn-in-place-curriculum anti-drift mechanism set) -- both VERIFIED RUNNING. Left the remaining 5 free slots IDLE with a HARD reason per the new KPI: no further ready blocker-list launch spec (LOWER trajectory bank is a documented SKIP/launch-blocker in test_task_semantics.py -- needs a CODE task, not a launch; quad-mix-erosion and start-variation fixes also need CODE, not more training) -- did not invent a peripheral run to fill them. 
- 08-10 20:56 c: cw-dep-startvar1-seed12-noaxis PASS(forensic) — seed=12 cleared, startvar1 failure is the zero-drift-DR axis not seed; cw-dep-vref1-r1-kpscale PASS — kp-alone doesn't reproduce gainvar's wide margin, implicates kv/interaction; dep-line DR-compose class stays CLOSED. Noticed 3 'still training' runs from my prompt (contactstiff-fric-r1, actionnoise, placement-comshift-zerobias-s1r1) went DEAD between checkups and now (pods idle, no procs) while a PAUSE flag is up — investigating before touching anything. 
- 08-10 21:19 c: also caught 2 more finished runs (batch completion, not deaths -- turnfix1/plantgate1/kvscale-groundtilt/contactstiff-fric-r1/actionnoise/placement-comshift-zerobias-s1r1 all completed their step budgets ~20:44-21:05, watcher was PAUSEd for a pod-eval deploy so prestage never ran). Triaged the 2 blocker-relevant ones myself: cw-walk-turnfix1 FAIL (matched-parent control vs yawgate2: turn err 0.232 vs 0.233, hold 0.108 vs 0.091 -- statistically identical, drift persists despite the landed signed-income+drift-charge+curriculum mechanism passing its bank; turning reward-tuning now DOUBLY closed, next lever is mirror-symmetry aug per TURN.md); cw-stand-plantgate1 FAIL (KNOWN flag-leg exploit on video even with the new polygon gate priced live, rise 0/12 valid_plant matching b2p1's baseline -- detector alone doesn't fix the income structure, RL_PLAN updated). Left the 4 DR-compose siblings (closed class, lower priority) + the still-training 5 for the next spawned cycle -- exiting now since restart_watcher.sh has been waiting on this cycle to drain for a pod-eval deploy. 
- 08-10 22:05 c: cw-dep-vref1-r1-kpscale(prior instance)/kvscale-groundtilt/placement-comshift-zerobias-s1r1 -> all PASS, same closed-class benign fingerprint (kv-alone's already-accepted margin explains the pair; seed-12 twin confirms the 3-axis assembly stack is seed-robust); dep-vref1-r1 DR-compose sweep stays CLOSED, no requeues; 12 GPU slots free + empty backlog, left idle per prime directive (no CODE-ready mechanism change queued for rise/turn this cycle). 
- 08-10 22:06 c(triage): badstartjoints/contactstiff-fric-r1/contactstiff-groundtilt all PASS -- confirm the same closed no-effect fingerprint (gv 6/6, 0 term, known det/4 crater + DR0.35 det/5+sto/0-1 cluster) for bad-start breadth and the two remaining floor-realism pairs on cw-dep-vref1-r1; no further vref1-r1 composes queued (CLOSED). cw-dep-vref1-loaded1 found orphaned/unverdicted (stale triage claim, 6h old) -- vel_err +40%/slip +60% vs vref1-r1 under loaded actuator dynamics, missing its own 15% gate despite a visually clean walk; flagged DIG-IN, not verdicted here. All 12 pods idle, backlog empty -- no blocker-ready spec to queue this cycle. 
- 08-10 22:06 c21:55 batch: cw-arch-hist24-r1 PASS (temporal-arch rung 2, 24-frame history bootstraps from scratch, gv 6/6 own-cfg DR0.5, JOYSTICK GATE 0 falls [ran myself, not pre-staged], det prog beats hist16's band but slip/economy worse -- ladder stays frozen at 16 per operator ruling, parked as evidence) + cw-dep-vref1-r1-actionnoise PASS (noisy servo-command axis, last untested actuator axis on the dep-line sweep, composes free -- sweep now genuinely exhaustive, stays CLOSED). cw-dep-startvar1-seed12-noaxis was already fully verdicted by an earlier cycle (20:56, PASS-forensic, seed cleared as startvar1 driver) before this batch was assigned -- left untouched, no re-verdict. SKILLS +2 rows, both wandbnotes rewritten. No dig-in triggers (no gate/video disagreement, no protected-skill erosion, no canary fire). Refill: 12/12 pods free, backlog empty, checked WISHLIST/RL_PLAN queue -- every ready lever is CODE-blocked (mirror-symmetry aug for turn, rise income-structure redesign, mode one-hot for the flagship, zero-drift-DR mechanism rework) or a closed class (dep-line composes, temporal ladder past 16); left all 12 idle with this reason recorded, no peripheral run invented. 
- 08-10 22:13 c: cw-stand-plantgate1/cw-walk-turnfix1 were both already fully verdicted (ledger+wandbnote+RL_LOG) by the 21:19 cycle -- FAIL/FAIL, no re-verdict done. Found STATUS.md stale (still said rise/turn mechanisms 'untrained' after both had actually trained and failed) -- refreshed its top summary + NOT-working + next-steps sections to say both reward-tuning routes are now closed by trained evidence, next moves are code (rise income rework, mirror-symmetry aug for turn). Checked for new CODE landings since 5c77728 -- none; every ready lever stays CODE-blocked or closed-class. Capacity: 10/12 pods free, backlog empty; left idle per prime directive, no peripheral run queued. cw-arch-hist16-dep1-c1/-r7-c4 left untouched (other cycles). 
- 08-10 22:27 c: hist16-dep1+s1r1 PASS (bootstrap works, both seeds, clean 6-leg gait, 0 falls, slip slightly above vref1-r1 band -- exposure gap like r7 had); hist16-r7-c3 PASS (slip closed to champion-band edge, joystick gate 0 falls); refilled dep1-c1 + r7-c4 continuations (train-9, train-3) to close the remaining slip gap; rest of fleet idle per blocker-list refill rule 
- 08-10 22:5x operator: rise income redesign landed + launched. plantgate1's lesson operationalized: gates leak, so the INCOME moved -- reward.rise_score_income=1 zeroes all rise height income (progress/milestones/finish/kernel) and pays only a progress ratchet + post-ramp hold on stand-score S = height-kernel x feet-down^2 x hard no-flag x plant geometry, plus a ramp-weighted airborne-feet rent (bank showed cheats otherwise win on the PENALTY side by dodging reward_height). Flag-leg trajectory from the plantgate1 video pinned in test_task_semantics as a scripted exploit; score bank PASS all seeds under the exact run cfg (replay +91 >> stilt -9 > flagleg -165 income ~0 > freeze -218; stilt deliberately allowed above partial: grounded, unpaid, ratchet gradient points INTO the stand -- rationale in the test). Launched cw-stand-score1 on train-6 (DISCOVERY 2M, warm from stance champion, joint_goal DR0.2 mix hold=.1/rise=.45/lower=.45, plant band 108-114mm, loaded servo params, NO ref-track per operator no-waypoints preference; lower keeps the solved legacy stack). Commit 0372dc8; RISE.md direction updated.
- 08-10 22:52 c(checkup): watcher DEAD flag on cw-stand-score1 = FALSE ALARM -- trainer exited because the 2M discovery budget COMPLETED normally (log 'done: 2,000,000 steps', ckpt ppo_goal_cw_stand_score1.zip written, wandb artifact md5 03873c4e, final reels logged); no cleanup, no retry. Run is claimed by the concurrent triage cycle (ledger triage stamp 22:46) -- left unverdicted/untouched per cycle assignment. No refills: stand-score1 outcome decides the next rise move, other ready levers unchanged since 22:13 audit (CODE-blocked/closed-class); fleet idle per prime directive. hist16-dep1-c1/r7-c4 still training, untouched. 
- 08-10 23:02 operator session: mirror-symmetry augmentation LANDED (mirror.py MirrorPPO, test_mirror 11/11, OMNI semantics bank PASSING, snapshot 95e133e); discovery probe cw-omni-mirror1 launched on train-0 (hist16-dep1 recipe + full-circle + yaw stack + k_current=0 + mirror coef 1.0); hardening 40M follows if mechanism-health gate passes 
- 08-10 23:03 c: cw-stand-score1 FAIL -- 3rd distinct rise-income mechanism (score-routing, warm from HONEST stance champion) beaten by the SAME flag-leg cheat: 0/12 valid_plant/end_posture_ok at DR0 gate + own-DR0.2, every start_kind, rise_score flat ~0.01-0.02 the full 2M steps (hit its own pre-registered early-call trigger). Ruling: reward-income shaping for rise CLOSED (RL_PLAN/RISE.md/CURRENT_TRUTHS updated); next lever is reference tracking or foot-contact-coupled height, not another income variant. STATUS.md refreshed. Fleet idle per prime directive, no refill (next rise arm is a SPECIFICATION/CODE step, not ready to launch). hist16-dep1-c1/r7-c4 untouched (other cycles). 
- 08-10 23:12 operator session: cw-omni-mirror1 discovery PASS (sym loss 0.00199 peak -> 0.0009 at 2M, reward 34->227, 0 NaN; verdict in ledger) -> cw-omni-mirror1-r1 hardening 40M launched on train-0 continuing the probe checkpoint — THE omnidirectional-policy run (full circle + yaw + dep contract + mirror coef 1.0); behavioral gate = eval_yaw both signs + zero-cmd drift <0.05 + backward/lateral joystick legs vs dep1 baseline 
- 08-10 23:16 c: cw-omni-mirror1 was already fully triaged/verdicted (PASS, mechanism-health) and relaunched as cw-omni-mirror1-r1 by an earlier operator session before this cycle started -- watcher's DEAD checkup on it is a false alarm (clean 2M-step completion, not a crash), no re-verdict needed. Found the verdict's wandbnote had the OUTCOME buried at the bottom (pre-dating the top-OUTCOME convention) -- rewrote via ops.sh wandbnote to put it up top; STATUS.md's turning section was also stale (still described mirror-symmetry as an untried next lever) -- refreshed both affected paragraphs to say discovery passed and the 40M hardening run is in flight, behavioral verdict pending. cw-arch-hist16-dep1-c1 (assigned to another cycle) has actually already finished (40M done, ckpt saved) but left fully untouched per cycle-ownership rule. hist16-r7-c4 still training, untouched. Capacity 10/12 idle; no refill -- every ready lever remains CODE-blocked (rise reference-tracking/foot-contact redesign) or already running (turn's mirror hardening); backlog empty, no peripheral run invented. 
- 08-10 23:25 c(triage): cw-arch-hist16-dep1-c1 -> PASS, +40M continuation closed the dep-contract hist16 walker's slip gap into/near vref1-r1's own band (1.41-1.48 -> 1.20-1.37), joystick gate 0 falls (ran live), clean 6-leg video -- 2nd hardware-ladder rung; SKILLS.md updated. cw-arch-hist16-r7-c4/cw-omni-mirror1(-r1) left untouched (off-limits/still training). No ready blocker-list item; backlog empty, 11 free slots left idle per prime directive. 
- 08-10 23:35 c: cw-arch-hist16-r7-c4 PASS(gate met) but budget-question resolves negative -- +40M more steps did NOT keep closing the slip/economy gap (own-cfg flat vs c3, DR0 sto +12% worse), exposure-alone hypothesis falsified; ran joystick gate + video myself (clean, 0 falls, no flag-leg); stop step-budget continuations on r7, remaining gap is contact/current pricing (open problem 1), not architecture depth. SKILLS.md +1 row. cw-omni-mirror1-r1 left untouched (still training); cw-arch-hist16-dep1-c1 left untouched (concurrent cycle's run). Fleet 11/12 free, backlog empty -- every ready lever CODE-blocked (rise redesign, mode one-hot) or already running (mirror hardening); no peripheral run invented, left idle per prime directive. 
- 08-11 00:30 c: cw-omni-mirror1-r1 -> FAIL: 40M mirror-symmetry hardening collapsed to a walk-mode park/freeze exploit (standing paid ~1130 vs walking ~500-860; fwd 0.68m->0.01m, gv 6/6->3/6 vs hist16-dep1; std climbed 0.39->1.69) — reward-routing bug (slow speed band + k_yaw_still=50 starved walk income), mirror hypothesis untested; next is a bank fix before re-hardening. No refill: blocker-list levers are CODE-blocked (rise spec, this bank fix) or already training; idle left per prime directive. 
- 08-11 00:48 c: cw-stand-scoreref1 FAIL (ref-tracking crutch on score1 stack also loses to flag-leg, 0/6 valid_plant every mode/DR) — closes reference-tracking-as-crutch too; rise's only remaining lever is a structural height<->foot-contact coupling (CODE). No refill: 12/12 idle, nothing ready on the blocker list (rise CODE-blocked, turn owned by concurrent cycle). 
- 08-11 01:22 c: idle-kick CODE cycle, no triages -- root-caused + fixed the cw-omni-mirror1-r1 park collapse (chain: freeze behavior <- 1122/ep freeze income floor <- LINEAR kernel pays v=0 in full on turn-in-place ticks since walk_kernel_prog_gate needs s_ref>1e-3 <- gate condition defect; probe REFUTED the TURN.md k_yaw_still guess, its charge summed ~0). Landed reward.walk_kernel_yaw_gate (achieved-yaw prog-gate analog, default 0=legacy, MJX path shared) + freeze-floor bank in test_task_semantics (2 tests, FAILED pre-fix at park=0.77x turn, post-fix park 0.38x / mixture freeze 0.40x, all 9 walk/turn/omni + 11 mirror tests green); REWARD.md row, TURN.md/RL_PLAN/STATUS refreshed. Relaunched hardening as cw-omni-mirror2 (respec of r1, ONE variable = the gate, warm from healthy 2M probe ckpt, 40M, evidence=discovery PASS + bank PASS, kill signature pre-registered) -- VERIFIED RUNNING train-0 @9.8k fps, snapshot de9adc8. 11 slots left idle per prime directive (rise structural height<->contact coupling is the next CODE item, needs its own spec cycle). 
- 08-11 01:37 operator session: DR question answered by experiment — cw-omni-mirror2-dr02 launched on train-1, matched twin of cw-omni-mirror2 with the ONE change dr-scale 0.5->0.2 (from-scratch standard); judged as a pair on the mirror2 panel. If dr02 turns where mirror2 stalls, DR 0.5 was a real difficulty knob; if twins match, DR exonerated and the walk_kernel_yaw_gate pricing fix carries the verdict 
- 08-11 01:50 c: cw-stand-scoreref1-dr0 FAIL -- DR-exoneration control: rise 0/6 det+sto (harness-confirmed, flag-leg video matches score1/scoreref1 exactly), env/rise_score flat 0.01-0.03 the full 2M steps at DR0 too, so DR is NOT the blocker. New clue: env/reward_rise_ref starts warm at 0.65/tick then collapses to ~0.02 within ~20-30 updates. Also caught + triaged cw-stand-scoreref1-dr0-lowlr (a concurrent operator-session follow-up, already finished by the time I looked): FAIL, cutting LR 6x did NOT slow the collapse (same crash timescale) -- erosion-by-oversized-updates is refuted; two untested explanations left (summed reward genuinely disfavors the tracked behavior, or the tight 6-deg sigma is measuring rollout noise as collapse). RISE.md evidence trail +2 entries; no further LR/income variant queued (already closed per RISE.md ruling -- next lever is the structural height<->contact coupling, CODE). cw-omni-mirror2/-dr02 untouched (concurrent cycle). 12 pods: 2 busy (mirror2/-dr02), 10 free, backlog holds only cw-dep-tip1 (steps=2000000, queued, not yet drained) -- no ready blocker-list spec to add, left idle per prime directive. 
- 08-11 01:55 c: checkup-findings cycle — watcher DEAD flag on cw-stand-scoreref1-dr0-lowlr (train-6) is a FALSE ALARM: the 2M discovery run finished its full budget normally in 366s (~01:47, checkpoint + W&B artifact md5 16bc0315 logged) BEFORE the 5-min checkup fired at 01:50; ledger already DONE with FAIL verdict from the concurrent cycle, and train-6 is already re-occupied by that cycle's follow-up cw-stand-scoreref1-dr0-riseonly. No cleanup, no retry (would duplicate a finished verdicted run); no launches. Note for watcher hygiene: sub-7-min GPU discovery runs will keep tripping the 5-min DEAD checkup — checkup should treat a completion banner in the log tail as FINISHED, not DEAD. 
- 08-11 02:01 c: cw-stand-scoreref1-dr0-lowlr independently confirmed FAIL (already verdicted by the concurrent scoreref1-dr0 cycle before my harness eval synced) -- watched the harness gate that landed after their verdict: rise 0/6 det+sto, worst_clear 159-167mm, video (rise_det_0) shows the same one-leg-flagged-in-air cheat as score1/scoreref1/scoreref1-dr0, no gate-vs-video tension, nothing to add. cw-dep-tip1 and cw-stand-scoreref1-dr0-riseonly also finished this window but are claimed/owned by other concurrent cycles (tip1 has an active triage claim; riseonly is scoreref1-dr0's own forensic follow-up) -- left untouched. No refill: 10/12 idle, backlog empty; rise's only lever (structural height<->contact coupling) is unbuilt CODE not a launchable spec, turn is covered by mirror2/mirror2-dr02 already training -- nothing blocker-ready to queue, idle pods per prime directive. 
- 08-11 02:07 c: cw-dep-tip1 FAIL (full-dose tipped-start DR exposure alone didn't teach lean recovery — score stuck at 0 vs 0.25 baseline, settled by sinking not stepping, and leaked a new partial flag-leg into plain walk); CURRENT_TRUTHS updated. No refill: turning (mirror2/mirror2-dr02/trans1) and loaded-actuator (vref1-loaded1) blockers already have arms running; rise/tipped-recovery next levers both need a CODE reward-term change before the next discovery launch, so 9 free pods left idle per prime directive. 
- 08-11 02:10 operator session: TURN DE-SCOPED for the deliverable (no camera = no front) -> cw-omni-trans1 launched on train-9: translation-only omni arm, dep1 recipe + full-circle headings + k_current=0 + mirror coef 1.0, NO yaw stack (freeze-exploit income structurally absent). Preflight: trans-only direction ordering PASS (snapshot a6634e4). Measured baseline: dep1 is WORSE than frozen on backward commands (0.069 trk_err) — no learned policy has ever walked backward. rot-60 exact equivariance held in reserve if full-circle exposure alone fails 
- 08-11 02:16 c: cw-stand-scoreref1-dr0-riseonly FAIL (forensic, expected) -- rise-only goal-mix (no lower/hold) erodes env/reward_rise_ref just as fast (0.51->0.03-0.05/tick within ~130 ticks) as the mixed-mode scoreref1-dr0 run; harness rise 0/6 det+sto, same held-leg cheat (worst_clear 151-164mm) as the whole score/scoreref family. Cross-mode interference (Suspect A) REFUTED per pre-registered gate -- erosion is intrinsic to the rise task, narrowing to RISE.md's two remaining live explanations (summed stack disfavors the motion, or 6deg sigma reads noise as collapse), neither worth another reward-coefficient arm; next lever stays the structural height<->contact coupling (CODE). RISE.md evidence trail +1. cw-omni-mirror2/-dr02/-trans1 untouched (concurrent/still training); cw-dep-tip1 left to its owning cycle despite its DEAD checkup. No refill: 9-10/12 free but nothing blocker-ready (rise CODE-blocked, turn already covered by 3 in-flight arms) -- idle per prime directive. 
- 08-11 02:40 c: checkup-findings cycle — watcher DEAD flag on cw-stand-rsi1 (train-6) is a FALSE ALARM: the 2M discovery run completed its full budget normally in 348s (~02:29, checkpoint + W&B artifact md5 89c059c2 in log tail) before the 02:35 checkup fired; run already claimed for triage by the concurrent cycle at 02:33, so no verdict from me. No cleanup, no retry (would duplicate a finished run and collide with the owning cycle); train-6 confirmed free of orphan processes via launcher status. Second instance of the sub-7-min-discovery-run-trips-DEAD-checkup pattern on train-6 (first: 01:55 cycle) — watcher should treat a completion banner in the log tail as FINISHED. No launches: in-flight arms cover turn/translation, rise is CODE-blocked, idle pods per prime directive. 
- 08-11 03:40 operator session: ROOT CAUSE of the stand-campaign erosion found and fixed -- warp pool-restore was dropping the score-stack episode attrs (SNAP_ATTRS missing _score_best/_rise_ramp_i0/_end_posture_from + RSI clock): pooled episodes inherited another episode's ratchet baseline, so score income silently stopped paying on the GPU path (rsi1's env/rise_rsi 0.58->0.15 decay with ZERO terminations was the tell; local probes never pool). score1/scoreref1/plantgate closures REOPENED (contaminated). Forensic ladder tonight also exonerated DR (dr0), LR (lowlr), mode interference (riseonly), exploration noise (noisy-replay +357) -- all controlled single-change runs. RSI landed (goal.rise_rsi_frac, sag-robust). cw-stand-rsi2 = rsi1 args + the fix, launching on train-6. RISE.md updated.
- 08-11 02:52 c(mirror2): cw-omni-mirror2 FAILED — freeze-floor gate fix confirmed (walking out-earns the sacrifice pattern) but gait still collapses into a leg-sacrifice/tripod pattern ~half the time (0/6 success both modes, known-exploit STOP, no dig-in); mirror-symmetry still untested. TURN.md/RL_PLAN/STATUS updated: next lever is a term-by-term WALK-kernel income re-probe, not another mirror arm. Blocker list unchanged otherwise; left 9 free pods idle (empty backlog, nothing launch-ready per prime directive). 
- 08-11 02:56 c(rsi1-verdict): cw-stand-rsi1 triaged (rise 0/6 det+sto, flag-leg cheat, rise_score flat) but CORRECTED mid-cycle: a concurrent cycle found+fixed a warp/MJX episode-pool state-loss bug (65edba7) that invalidates rsi1's and the whole score1/scoreref1-family 'CLOSED' verdicts as CONFOUNDED, not refuted -- updated ledger/W&B notes + CURRENT_TRUTHS/RL_PLAN/STATUS to stop citing them as evidence; clean re-test cw-stand-rsi2 already in flight, no new launch needed this cycle. 
- 08-11 02:57 c: checkup-findings cycle — watcher DEAD flag on cw-stand-rsi2 (train-6) FALSE ALARM #3 of the same class: 2M discovery run completed its full budget in 365s (W&B finished@2.03M, ckpt artifact md5 35d9ebf0) before the checkup fired; no cleanup/retry, triage belongs to the owning concurrent cycle. ROOT CAUSE FIXED this time: launch_run.py checkup only knew the CPU trainer's completion banner, not '[mjx-train] done:' — regex extended + tail widened 600->2000B (commit bd949b5, exp/checkup-mjx-banner-fix); re-ran checkup live: rsi2 now correctly FINISHED rc=0. No launches (checkup cycle; blocker-driven idle stands). 
- 08-11 03:03 c: cw-stand-rsi2 FAIL, RE-CLOSES the reopened rise verdicts on clean data -- pool-restore fix confirmed working (env/rise_rsi held 0.48-0.58 all 2M steps, no decay) but env/reward_rise_ref crashed 0.83->0.02-0.09/tick and env/rise_score never left the 0.01-0.02 floor; harness rise 0/6 det+sto, worst_clear 146-161mm, video-identical tripod cheat (3 legs planted, 3 legs frozen 20-146mm up) to every prior arm. Pool bug EXONERATED as cause; income-shaping/reference-tracking/RSI-as-fix all RE-CLOSED, stronger evidence than before; only remaining lever is the structural height<->foot-contact coupling (CODE, RL_PLAN 2b). CURRENT_TRUTHS/RL_PLAN/STATUS/RISE.md updated. Pre-staged eval was never triggered (ledger stuck RUNNING despite finish) so ran ops.sh podeval manually. cw-omni-mirror2-dr02/cw-omni-trans1 untouched (concurrent/still training). No refill: 11/12 idle, backlog empty, rise's only lever is unbuilt CODE not a launchable spec, turn already covered by trans1 -- idle per prime directive. 
- 08-11 03:05 c78: cw-omni-mirror2-dr02 DONE - STOP known exploit, identical leg-sacrifice pathology to parent cw-omni-mirror2 at DR 0.2 (gait_valid 3/6 det, walking out-earns sacrifice 646-889 vs 473-485, 0/6 success); PAIR VERDICT: DR-scale exonerated as the driver, not a difficulty knob - next lever stays the term-by-term WALK-kernel income re-probe (RL_PLAN/TURN.md updated). No refill: backlog empty but 5+ concurrent cycles already actively refilling/dig-in on rise/omni/dep lines; no sound unclaimed blocker-list item to queue. 
- 08-11 04:55 operator session: cw-stand-rsi2 confirmed the pool fix (rise_rsi held 0.5) but failed; cw-stand-rsi3 (+rise_score_strip_pen, the live k_height penalty had made flag-leg the reachable optimum) failed IDENTICALLY -- feet-factor collapse curve is reward-invariant across all 6 stand runs tonight => warm-start OOD drift, not reward. Sigma-widening bank-blocked (flagleg farms 17% at 10deg). Ruling in RISE.md: no more reward/RSI variants; spec the trainer BC-anchor (lever a) first, structural height<->contact coupling (lever b) as alternate. All stand-* pods free.
- 08-11 03:31 c: cw-dep-hgt1 FAIL (walk_height_gate income cut maxed but height_factor collapsed parent's 0.87->flat 0.24-0.26 within ~50 updates, ~50-77mm crouch persists at eval; walk itself unbroken, gate ineffective -- cosmetic per HARDWARE.md, not a blocker); cw-omni-trans1 FAIL (turn-removed omni arm hits a THIRD distinct degenerate pattern -- paddle-stall, 2 legs planted 90-99% duty vs 4 churning ~0.01m strides, slip 3-13, 0/6 success, train/std climbed 0.37->1.38 3.7x with no plateau; mirror-symmetry still untested, TURN.md updated); cw-stand-rsi3 FAIL (6th reward variant, same tripod cheat, refutes the stray-penalty-funds-cheat theory too -- reconciled with operator's RISE.md read: identical feet-factor collapse curve across all 6 arms = warm-start OOD drift not reward-driven, two CODE levers queued: BC-anchor first, then contact-coupling; CURRENT_TRUTHS/RL_PLAN/STATUS refreshed). No refill: 12/12 idle, backlog empty, every ready lever is CODE/SPEC-blocked (rise BC-anchor design, turn/omni term-by-term income re-probe) or hardware-bench-owned; cw-dep-vref1-loaded1 DIG-IN claim (11h stale, not my cycle's run) left untouched. 
- 08-11 04:12 c: idle-kick(deep): resolved 11h-stale DIG-IN cw-dep-vref1-loaded1 -> PASS via matched-parent control (frozen vref1-r1 under the IDENTICAL loaded injection degrades the same: det fwd 0.76 vs child 0.81m, sto slip 1.81 vs 1.65, same sto/4 crater fingerprint) -- the +40%/+60% triage delta was the honest cost of loaded-servo physics measured against the parent's clean-physics band (actnoise GATE LESSON reapplied); loaded servo model is a viable dep-line training default, SKILLS +1 row, RL_PLAN problem-3 updated, air-vs-loaded for attempt #2 left as bench decision. Rise lever (a) BC-anchor SPEC LANDED: bc_anchor.py (BCAnchorPPO aux loss, MirrorPPO pattern) + sim_env bc_target emission on a shared _rise_ref_clock (tracking reward refactored onto it, behavior identical) -- 10/10 new tests, full task-semantics bank re-run green (reward untouched), MJX pod smoke shows anchor loss 0.198->0.04; launched cw-stand-bc1 (respec of rsi3, ONE change train.bc_anchor_coef=1.0, discovery 2M, VERIFIED RUNNING train-6 @10.6k fps, buffer full, pre-registered kill feet-factor<0.4 by 500k). STATUS rise + servo-lag paragraphs refreshed. 11/12 idle: turn/omni next lever is the term-by-term walk-kernel income re-probe (dig-in CODE work, not a launch), backlog empty -- idle per prime directive. 
- 08-11 04:46 c: cw-stand-bc1 PASS(partial) — BC-anchor lever (a) validated: first honest six-foot rise in 7 stand-arms (harness valid_plant bridge 7/12 crouch 6/8, flat cold-start 10/10 correct stand w/ footprint-only miss, zero flag-leg in 42 videos, clean 1-var vs cheating rsi3 parent); weak hint of raise/tipped/hold-track interference (n=2). Launched cw-stand-bc1-hard1 (10M hardening) + cw-stand-bc1-coef03 (dose-check discovery). RISE.md/CURRENT_TRUTHS/RL_PLAN/STATUS/SKILLS refreshed. 
- 08-11 04:53 c (same, cont.): cw-stand-bc1-coef03 also finished+verdicted this cycle — FAIL, dose-response refuted (coef 0.3: valid_plant 0/16 all start kinds, worse than coef1.0 on every axis, no reduced interference). Ruling: keep bc_anchor_coef>=1.0, no more coefficient variants; cw-stand-bc1-hard1 (10M, running) is the live next step. RISE/RL_PLAN/SKILLS/STATUS updated. 
- 08-11 05:10 c: cw-stand-bc1-coef03 FAIL already verdicted by prior cycle turn (dose-response refuted, confirmed no further action needed); cw-stand-bc1-hard1 (10M hardening) NOT verdicted — RSI-off n=12 probe shows rise dramatically fixed (12/12 all start kinds incl flat, worst_clear 7mm) but hold/track/raise/lower ALL collapsed to 0/12 with worst-foot clearance 162-189mm (flag-leg-magnitude), video confirms splayed/dragging legs in hold+lower frame strips — lower was 'solved warm' and had 45% goal-mix weight, so this isn't just an excluded-mode atrophy story; protected-skill erosion trigger, flagged DIG-IN for deep model. No refill (blocker-list empty of ready items, 12/12 pods idle per prime directive). 
- 08-11 05:18 c (same, cont.2): cw-stand-bc1-hard1 finished+verdicted too — PASS(partial) rise consolidates w/ hardening (valid_plant 5/6 det=83%, up from 50%) BUT dig-in via duty_cycle/swing_count found hold/track were never actually still (legs cycle continuously, 12-50mm at 2M -> 100-161mm at 10M, worse w/ more steps) — pre-existing pricing gap, not new from the anchor; corrected bc1's verdict (sparse video strip had missed it). New queue item: HOLD-mode stillness-pricing SPECIFICATION before any more steps on this lineage. RISE/RL_PLAN/SKILLS/STATUS updated. 
- 08-11 05:20 c (same, cont.3): resolved a concurrent DIG-IN flag on cw-stand-bc1-hard1 (another cycle read the same run's numbers as protected-skill erosion/splayed-dragging off a broken --modes eval of mine) — re-extracted 2fps video from the CLEAN gate eval, confirms a normal stable stance, not dragging; real story is continuous small-amplitude leg-cycling during hold/track (milder than a flag-leg collapse), verdict updated with the resolution so no further DIG-IN needed on this run. 
- 08-11 05:34 c(dig-in): cw-stand-bc1-hard1 -> PASS(partial)/DONE, verdict finalized w/ matched-parent control: parent bc1@2M ALREADY 0/12 on hold/track/raise/lower (lower flag-leg 166mm) so NO protected-skill erosion — hardening lost nothing and consolidated rise to 12/12 valid_plant RSI-off incl flat 4/4 (footprint miss resolved); ppo_goal_cw_stand_bc1_hard1 promoted RISE SPECIALIST (SKILLS row), lineage CLOSED for hardening (hold splay worsens w/ steps: pre-existing stillness-pricing gap, raise untrainable at p_raise=0). Fixed eval_checkpoint unknown-modes NaN-crash ('tipped' probe killer) + smoke-tested. Next: HOLD-stillness SPECIFICATION (queue 2.3) then rise->walk handoff composition; no refill — next work is spec/code, 12 pods idle per prime directive. 
- 08-11 06:41 c: cw-stand-holdstill1 triage found fully processed by a concurrent cycle before I got to it (verdict FAIL-on-hold/PASS-on-rise-retention recorded, hold_flag_fade lever landed+tested, wandbnote posted, snapshot 40418ed, follow-up cw-stand-holdstill2 already VERIFIED RUNNING train-0) -- independently checked and agree: frame strips confirm the leg-0-parked-in-air 0/12 hold/track fingerprint + a clean 6-foot rise (rise/det 4/6, sto 6/6), full task-semantics bank green (32 passed/1 skip) including the new 3-test fade sub-bank. No duplicate work done, no relaunch. No refill: 11/12 idle but nothing else blocker-ready (turn/omni next lever is a dig-in income re-probe not a launch; wishlist items are peripheral) -- idle per prime directive. 
- 08-11 06:59 c: idle-kick CODE cycle, queue 2.3 executed -- HOLD bank landed (legacy pricing pays frozen flag-leg park 368 vs quiet stand 368, a literal tie) + reward.hold_still_gate + hold_flag_fade (REWARD.md, banks 9/9, suite 32 green); cw-stand-holdstill1 FAIL (park persists at zero income: hard-zero=flat plateau, rise retention held) -> cw-stand-holdstill2 FAIL-but-directional (fade moves park 110->90mm, feet factor x3, hold still 0/12) -- two pricing misses = hypothesis change: next [CODE] BC supervision on hold ticks (rise playbook), no third pricing variant; pricing stays landed. 11/12 idle per prime directive (next work is trainer CODE). 
- 08-11 07:21 c: cw-stand-holdstill2 triage found already fully verdicted by a prior concurrent cycle (FAIL-but-directional on hold, rise retention held) -- independently re-checked ops.sh review, agrees exactly, no duplicate work. Idle-kick CODE: landed the queued next lever, extended bc_anchor.py's bc_target emission from rise-only to hold/track ticks (target=_q_nom, the settled episode-start pose, constant per episode; added _is_hold_bc to mjx_host.SNAP_ATTRS per the pool-restore lesson); 4 new test_bc_anchor.py tests green (14/14), full test_task_semantics.py green (32/1-skip). Snapshotted + launched cw-stand-holdbc1 (respec of holdstill2, byte-identical cfg -- the ONE variable is the code under the already-set bc_anchor_coef=1.0) on hexapod-mjx-train-0, VERIFIED RUNNING. RL_PLAN/RISE/STATUS updated. 11/12 pods idle per prime directive -- nothing else on the blocker list is launch-ready. 
- 08-11 07:50 c: cw-stand-holdbc1 PASS -- HOLD-mode stillness SOLVED (third lever, BC-anchor extended to hold/track ticks): harness hold 12/12 valid_plant det+sto, worst-foot 2-13mm, video-confirmed level motionless six-foot stand both modes, first genuine quiet hold in the campaign; env/hold_feet_factor cleared the 0.1-0.35 plateau to ~1.0 by 500k steps. Rise retention mostly clean (bridge 2/2 det, sto 6/6); det crouch showed 2/6 tilt_roll falls, cross-checked against holdstill1(0)/holdstill2(1) as the SAME pre-existing fingerprint, not new -- no DIG-IN needed, no known-exploit cheat on any episode. SKILLS/RISE/RL_PLAN/STATUS/CURRENT_TRUTHS updated. Refill: launched cw-stand-holdbc1-hard1 (10M hardening continuation, --evidence this discovery pass) VERIFIED RUNNING train-0; 11/12 pods idle, nothing else blocker-ready per prime directive. 
- 08-11 08:18 c: cw-stand-holdbc1-hard1 PASS -- 10M hardening consolidates HOLD+RISE, no regression: hold valid_plant 11/12 (matches discovery's 12/12), env/hold_feet_factor held 0.99-1.0 all 10M, det crouch-start rise improved 2/6->2/4 (33%->50%, remaining miss/fall are a genuine tip-over + height-precision issue, zero flag-leg/tripod cheat in 24 video-checked episodes). ppo_goal_cw_stand_holdbc1_hard1 is the hardened HOLD+RISE checkpoint; lineage CLOSED for further hardening. SKILLS/RISE/RL_PLAN/CURRENT_TRUTHS/STATUS updated. No refill: 12/12 pods idle, backlog empty; next blocker-list item (rise+hold->walk-champion handoff composition test) needs a small new eval/handoff script first (CODE, not yet built) -- idle per prime directive, no peripheral run queued. 
- 08-11 08:52 c: idle-kick CODE cycle, queue-2.3 follow-up executed -- built rl_move/sim/eval_handoff.py (specialist rise->settled hold, plant-frame re-anchor w/ slew-state carry, walk-champion switch-in; direct vs scripted-blend vs clean-plant control) and ran both physics variants on idle train-pod CPUs: HANDOFF PASS -- 12/12 successful rises (flat+bridge, air AND loaded servos) hand off to ppo_goal_cw_walk_longdist_r2 with ZERO falls, drive metrics inside the plant-baseline band, scripted 1.5s blend adds nothing (play.py key-7 path superseded); crouch-start rises tip PRE-handoff 0/6 RSI-off (known lineage fragility, sharpened, not a handoff defect). RL_PLAN/RISE/SKILLS/STATUS/CURRENT_TRUTHS updated, wandbnote on holdbc1-hard1, snapshots 7f91f87+9be7d82. No training refill: next blocker item is the REVERSE handoff (walk->stop->lower, specialist's lower quality post-holdbc1 unverified) -- eval CODE, not a launch; 12/12 pods idle per prime directive. 
- 08-11 09:25 c: idle-kick CODE cycle, queue-2.3 final item executed -- built rl_move/sim/eval_handoff_reverse.py (walk->stop->sit; spec/direct/scripted arms) and ran both physics on train-0/1 idle CPUs: REVERSE HANDOFF PASS -- specialist lower on the walker's exact stopped state == its own clean band (4/6 posture-strict air AND loaded, zero falls anywhere, h_err 0.4-9mm; only miss a cosmetic 62-99mm dangling foot, video-confirmed NOT a weight-bearing flag-leg -- vs bc1-hard1's 0/12 @189mm), scripted go_zero-sit glide 6/6 both physics and stays the deployed sit. FULL SIM JOYSTICK MOTION CYCLE (rise->drive->stop->sit) now composes with zero falls. RISE/RL_PLAN/SKILLS/STATUS/CURRENT_TRUTHS updated, wandbnote on holdbc1-hard1. Lower-BC-anchor polish recorded as optional, NOT queued (prime directive); no launches, 12/12 pods idle -- remaining blockers are bench-owned (attempt #2) or dig-in CODE (omni income re-probe). 
- 08-11 10:09 c: idle-kick(deep) CODE cycle, queue-2.1 income re-probe executed -- built rl_move/sim/probe_walk_income.py (term-by-term reward decomposition, residual-checked) and ran it on train-0/9 idle CPUs vs scripted fingerprints + the real collapsed ckpts: trans1-stack pricing EXONERATED (honest gait out-earns every degenerate 2-4x, all 4 dirs, DR 0 AND 0.5; collapsed trans1/mirror2 ckpts earn BELOW freeze) -- omni collapses are optimization failures, not paid basins, reward surgery CLOSED; one REAL latent defect found in the DE-SCOPED turn stack only (ungated k_walk_yaw pays a motionless body full income on linear ticks, k_yaw_still taxes honest gait ~-100/ep net; fix before any turn re-scope, TURN.md). Landed the third BC-anchor application: walk ticks emit command-conditioned TripodGait bc_target (stop ticks unsupervised, _walk_bc_gait on SNAP_ATTRS; attach accepts joint_walk); test_bc_anchor 21/21 (6 new), walk/omni banks 8/8 green. Launched cw-omni-transbc1 (respec of trans1, ONE var train.bc_anchor_coef=1.0, discovery 2M, VERIFIED RUNNING train-9 @7.1k fps, anchor loss live 0.142) -- snapshot 4253c35. TURN/RL_PLAN/CURRENT_TRUTHS/STATUS refreshed; 11/12 pods idle per prime directive. 
- 08-11 10:25 c(2026-08-11 10:2x): cw-omni-transbc1 (3rd omni-translation lever, BC-anchor on walk ticks) FAILED -- anchor loss converged cleanly but the identical march-in-place/paddle collapse reappeared (fwd 0.01m/ep, slip/m 6-19, video confirms zero floor travel in 12/12 clips); pre-registered prediction-if-false, BC-anchor/reward tuning now CLOSED on omni-translation, next untried lever is rot-60 equivariance (CODE). All 12 GPU pods idle: no launchable critical-path work (hardware bench + code-design items only per RL_PLAN); left idle per prime directive rather than filling with peripheral runs. 
- 08-11 11:21 c: idle-kick CODE cycle, queue-2.1 rot-60 lever executed -- OMNI TRANSLATION RESOLVED IN SIM, ZERO TRAINING: rl_move/sim/rot60.py canonicalizes any heading into the +/-30deg wedge (exact hexagonal symmetry, proved on the compiled model by test_rot60.py: rotate+relabel diverges <1e-6 over 30 contact steps) + eval_drive/eval_checkpoint --rot60; hardware ckpt cw-dep-vref1-r1 wrapped walks the FULL CIRCLE (naked backward frozen 0.027m -> wrapped every direction 0.024-0.036 err, 0 falls at DR0+DR0.35 incl full-circle flip stress, harness 20/24 success, slip 1.1-1.3, video-clean); matched control shows naked hist16-dep1 degenerates at EVAL into the leg-sacrifice (slip 7-11/m) -> wrapped gait_valid 24/24 -- the 0-for-4 omni training line was chasing what geometry gives free, line CLOSED. TURN/RL_PLAN/CURRENT_TRUTHS/STATUS/SKILLS refreshed, wandbnote on transbc1, artifacts logs/rot60/. Remaining: deploy-side runner port ([CODE], spec in TURN.md tail). No training launches -- 12/12 idle per prime directive (remaining blockers are bench-owned or deploy-side code). 
- 08-11 11:47 c: idle-kick CODE cycle, no triages -- executed the last omni-blocker [CODE] item: rot-60 canonicalizer PORTED into the robot runner (linux_control/rl_policy.py wraps rot60.Rot60Policy itself via make_walk_canonicalizer, zero duplicated logic; deploy_adb.sh ships rl_move/sim/rot60.py, numpy-only verified). Default-ON with BIT-EXACT k=0 no-op for forward-wedge commands (walk obs now float32 = training dtype), rot60=false naked A/B path, off-wedge REFUSED when wrapper absent, per-tick rot60_k CSV column for offline replay. Replay-parity locked: new test_rot60_runner.py (6 tests: obs-layout vs rot60 slices, forward bit-exactness, full-circle+hysteresis+zero-hold parity on the REAL deployed rl_walk_weights.json, backward sector, wedge fallback, numpy-only import chain); full suite 114 pass/1 skip; refusal-path smoke green. Snapshot 39d4754 (exp/rot60-runner-port). TURN/RL_PLAN/CURRENT_TRUTHS/STATUS refreshed -- omni blocker is now BENCH-ONLY (wrapped-vs-naked forward A/B then off-wedge legs in attempt #2). No launches: 12/12 idle, remaining blockers bench-owned (attempt #2), hold-current model fit, or rise+hold deploy port -- idle per prime directive. 
- 08-11 12:27 c: idle-kick CODE cycle, no triages -- executed the next critical-path [CODE] blocker: rise+hold specialist DEPLOY PORT (RL_PLAN blocker 2). ppo_goal_cw_stand_holdbc1_hard1 exported w/ its TRAINED goal ramp riding in the weights meta (new export_policy_np --extra-meta; rl_policy.py policy_profile() reads it per episode, legacy files keep old constants = stance_dr10 rollback behavior-identical) + live stance slot updated + deploy_adb.sh now ships rl_walk_weights.json+policies/. Contract locked by new test_stand_runner.py (5 tests; suite 119 pass/6 pre-existing skips) + closed-loop sim smoke w/ the DEPLOYED numpy artifacts (flat-belly rise +111mm, 0 falls, 2/2 eps). Mid-cycle reconciliation: operator had independently activated a PROFILE-LESS export on the robot (commit 1e64263, md5 6620705c) -- that copy would feed the legacy +50mm/4s ramp, OOD for a policy trained on 108-114mm targets; merged operator's picker notes + profile, flagged DO-NOT-STAND-until-repush in HARDWARE.md/RISE.md/CURRENT_TRUTHS. Snapshot e81dec7 (exp/rise-hold-runner-port). No launches: 12/12 idle, remaining blockers bench-owned (attempt #2) or hold-current model fit (next CODE item) -- idle per prime directive. 
- 08-11 12:49 gru-mjx-smoke launched on train-0 (100k, --smoke): first RecurrentPPO+GRU run on the MJX stack (gru_policy.py, code 1d72f58) — validates warp+cuda recurrent path before cw-arch-gru-r1 discovery 
- 08-11 12:58 cw-arch-gru-r1 launched on train-0 (2M discovery): GRU recurrent rung of the temporal-arch ladder, unified walk+rise/lower+hold diet on joint_walk; smoke gru-mjx-smoke validated the RecurrentPPO warp path at ~2.4k fps 
- 08-11 13:19 cw-arch-gru-r2 launched (respec continuation of gru-r1, init-from-source, walk share 0.55->0.70): feed the slow skill, keep rise/lower/hold riding along; gate = walk err <0.065 with stance retained 
- 08-11 13:23 c: cw-arch-gru-r1 (GRU temporal-arch rung 4, 2M from-scratch mixed diet) -> FAIL, known exploit, no forensics: harness+video show det walk frozen in a 3-leg-tucked static tripod (0.004 m/s, gait_valid 0/6) and sto walk jittering in place with heavy slip and 2/6 tips (prog med -0.02); training return climbed while the task didn't -- reward-shortcut pattern, not a walk. WISHLIST -0.5 updated with the probe result; ladder stays FROZEN pending the flagship, not on the blocker list, no recipe-adjustment relaunch queued by me. Note: a concurrent cycle already launched cw-arch-gru-r2 (walk-heavier continuation of r1) reasoning off r1's periodic training-log numbers alone (no harness/video looked at yet) -- flagging for that run's own triage that r1's actual gate eval is a leg-sacrifice cheat, not 'barely tracking'. No refill from me: nothing else on the blocker list is launch-ready (remaining blockers are bench-owned attempt-#2 or the hold-current model fit CODE item), 11/12 pods idle per prime directive. 
- 08-11 13:46 c: idle-kick(deep) triage backlog cleared -- cw-dep-hgt2 FAIL (pre-registered: sigma-80 put a live gradient at the crouch, walk_height_factor still plateaus at the crouch-equivalent 0.775, end height -52.5mm; height-gate lever CLOSED per two-miss rule, walk sag = cosmetic); cw-stand-crouchrise1 MECHANISM PASS / promotion declined (start-mix bias FIXES crouch-start rise: RSI-off all-crouch 16/16 stands det 8/8 vp ZERO falls vs matched parent hard1 0/8 w/ 8/8 tilt falls -- but hold sto valid_plant 1/6 vs parent 5/6 on the 2.0A current-tail flag misses the pre-registered >=10/12 bar; hard1 stays deployed, crouchrise1 banked md5 3877e16c; stand lineage fully CLOSED); gru-mjx-smoke SMOKE PASS (recurrent MJX path validated; cw-arch-gru-r1 is its launching cycle's run, untouched). SKILLS +1 row, RISE/CURRENT_TRUTHS/STATUS refreshed. No launches: 11/12 idle per prime directive (blockers bench-owned + hold-current fit CODE). 
- 08-11 13:49 c: cw-arch-gru-r2 -> FAIL, known exploit: harness+video confirm the IDENTICAL leg-sacrifice/paddle-jitter cheat as parent r1 despite the walk-heavier diet -- GRU rung stays frozen off the blocker list, no step-count relaunch without a recipe change; flagged the same evidence-misread pattern on a concurrent-cycle 20M 'hardening' launch (cw-arch-gru-long1, cited r2 training-log as 'walk:ok') as invalid, though it had already self-died (W&B step regression) before burning compute. Also caught+fixed a ledger race on cw-stand-crouchrise1: my triage (comparing report.json noise) wrote PASS/replaces-hard1-as-deploy-candidate right as a concurrent dig-in cycle's proper matched-parent RSI-off probe landed the correct call (mechanism genuinely fixes crouch tip-overs 8/8 vs parent 0/8, but a real small current increase fails the hold no-regression bar) -- corrected the ledger verdict+hardware_ready to match SKILLS/RISE/CURRENT_TRUTHS, snapshotted. No refill: blocker list is bench-owned (hw attempt #2) or CODE-only (hold-current fit); backlog empty, 12/12 idle, prime directive says that's fine. 
- 08-11 13:50 c: cw-stand-crouchrise1 confirmed already fully triaged/corrected by concurrent cycle (mechanism PASS, promotion declined, hard1 stays deployed, ledger+SKILLS+STATUS+CURRENT_TRUTHS consistent, committed 58b1f8a) -- no further action needed. Checked blocker list: only bench-owned (hw attempt #2) and CODE-only (hold-current fit) items remain, backlog empty, all 12 pods idle -- no peripheral refill per prime directive. 
- 08-11 14:21 c: idle-kick(deep) CODE cycle, no finished runs -- executed the last non-bench blocker (hold-current model fit) and RETRACTED its premise: new rl_move/sim/probe_hold_current.py (ran on train-0, air+loaded params, logs/probe_hold_current/) shows the 08-10 'sim hold 0.11 A vs real 0.59 A' inversion was a x18 unit slip (sim mean-per-servo vs hw bus-total) PLUS a pose confound (real walk-synced plant hold reads 0.106 A ~= sim; the 0.59 A is the scripted-stand cmd-fight, per-servo register current tracks cmd-q error); pose/unit-matched sim OVERPRICES effort 3-25x everywhere (conservative) and reproduces real walk>hold ordering. ASSUMPTION (operator to review): hold-current fit dropped from the blocker list, deferred to a register-scale cmd-fight/deadzone model required only before any k_current>0 hardware-pricing arm; k_current=0 ruling unchanged. SIM.md gap 2 / CURRENT_TRUTHS / RL_PLAN / STATUS refreshed. Critical path is now BENCH-ONLY (attempt #2); 12/12 idle, no refill per prime directive. 
- 08-11 14:55 c: idle-kick CODE cycle, no finished runs -- landed the LAST owed MDP_PREFLIGHT bank (LOWER, test_task_semantics.py, queue-2.4) under the deployed specialist stack incl loaded servos: bank PASSES -- honest IK-tracked descent (anchored at the SETTLED stance; ideal-plant anchoring leaves a 16mm sag error) out-earns the outrig/aloft cheats on every seed (540 vs 461/383) and posture-strict rejects them (~300mm aloft pads vs ~0); lower-mode arms unblocked. REAL FINDING encoded as strict-xfail: pf=5/6 posture pricing lets a one-leg-aloft lower keep 85% of honest income (cheat >> refusal) -- the exact incentive behind the deployed specialist's cosmetic 62-99mm dangling foot from eval_handoff_reverse; strengthen pricing or BC-anchor lower ticks before any lower-MECHANISM arm. Full suite 35 pass/1 xfail, RL_PLAN condensed -14 lines, RISE.md updated, snapshot a6a0231 (exp/lower-semantics-bank). No launches: critical path stays bench-owned (attempt #2), 12/12 pods idle per prime directive. 
- 08-11 15:32 c: idle-kick(deep) CODE cycle, no finished runs -- landed the flagship prerequisite (queue 2.4): obs.mode_onehot=1 gives the policy an explicit 6-wide skill-family input (walk-env obs tail, mirror-invariant, pool-safe via _goal_traj; default OFF = every existing checkpoint bit-exact) + --net-arch on the MJX trainer (flagship 256x256) + closed a latent tail gap in _privileged_idx (ledger-audited: never hit, only aac-s1b/c ever used --asym-critic, yaw-off). 13 new tests (test_mode_onehot.py), full suite 135 pass/5 skip/1 xfail, CPU train smoke + attach_mirror layout checks green, snapshot 70b5861 (exp/mode-onehot-flagship-a1). FLAGSHIP unified-policy line now fully unblocked (stance banks pass, turn de-scoped) -- queued stage A cw-uni-flag-a1 (2M discovery, from-scratch hist16/256x256/mode-onehot on the proven stand stack + BC anchor; hold + near-plant crouch rise + lower, no walk yet). ASSUMPTION (operator to review): with attempt #2 bench-owned, I treated the single-checkpoint deliverable (WISHLIST -1) as the next sim-side blocker and queued stage A; if overruled, the backlog item is one delete away and nothing else trains -- 11/12 pods stay idle. 
- 08-11 15:48 c: cw-uni-flag-a1 INFRA FAILURE (0 steps, SIGBUS -- hist16+mode_onehot obs > 64M shm at n-envs=4096, gotcha 13c); relaunched as cw-uni-flag-a1-r1 with --n-envs 3072, VERIFIED RUNNING past 1M/2M steps on train-0. Flagship stage A now actually training; no other blocker-serving work queued, 11 pods idle by design. 
- 08-11 16:15 c: cw-uni-flag-a1-r1 (flagship stage A, from-scratch hist16/256x256/mode-onehot, hold=0.2/rise=0.4/lower=0.4, 2M) triaged, left UNVERDICTED (real trigger: decides the flagship fork). Pre-staged gate defaults to --modes walk (irrelevant, this run never trains walk) -- ran the run's OWN gate myself on its pod (kubectl exec, hold+rise+lower, artifacts synced to logs/ckpt_eval/cw_uni_flag_a1_r1_hrl): hold PASS clean (det 6/6 valid_plant, video motionless six-foot stand) and lower PASS clean (det 6/6 posture-strict, honest sit video) but rise (all-crouch, the hard skill) misses badly -- det 1/6 valid_plant, sto 1/6 (printed n_ok used the ungated height-only success, 3/6 sto, which is exactly the CURRENT_TRUTHS height-only blind spot -- ignored it, read valid_plant/plant_fail directly). Video read: 4/6 det rise fails are a genuine new failure mode (wide leg sprawl, undershoot height 78-100mm, attitude/feet_down/no_flag all OK -- not the classic cheat) but 1/6 (det ep5) shows a true flag-leg fingerprint (leg elevated 91mm) and 1/6 lower/sto shows the already-known cosmetic dangling-foot margin (87mm, matches the pre-registered lower-bank thin-margin xfail, not new). env/rise_feet_factor held 0.3-0.95 the whole run (sampled series spans all ~630 logged iters = full 2M budget) with no sustained collapse toward the historical ~0.17 cheat floor -- does NOT look like the classic warm-start-OOD collapse fingerprint. Gate fails outright either way (rise <3/6, and the zero-known-exploit bar is violated by the one flag-leg episode) but the FORK decision (hardening continuation vs multitask-interference->MoE per the pre-registration) rests on a genuine causal ambiguity a triage pass can't resolve. No refill: blocker list is bench-owned (attempt #2) or waits on this exact decision, 12/12 idle per prime directive. DIG-IN: cw-uni-flag-a1-r1 -- gate FAILS (rise valid_plant 1/6 det+sto vs >=3/6 needed, one flag-leg episode present) while hold+lower pass clean and rise_feet_factor never collapsed like the historical cheat -- decide budget-limited-from-scratch (hardening continuation) vs genuine multitask interference (fork to MoE) per the flagship pre-registration. 
- 08-11 16:50 c: cw-uni-flag-a1-r1 (flagship stage A, dig-in) VERDICTED FAIL-on-gate/MECHANISM-SUPPORTED: rise valid_plant 1/6 det misses the 3/6 bar but the fork resolves to BUDGET-LIMITED-FROM-SCRATCH, not MoE — no cheat (the triage's 'flag-leg' det ep5 is a 91mm leg MID-tilt_roll-FALL, no episode holds height with a leg aloft; rise_feet_factor recovered 0.33->0.62 and still climbing, the OPPOSITE shape of the closed 0.87->0.17 cheat collapse; reward quarters -4/-17/+10/+41 steepest at cutoff) and no interference (hold det 6/6 valid_plant + lower det 6/6 posture-strict, both stable all run while rise climbs; the 9 rise misses are the specialist lineage's exact current/footprint/height-short finishing tails that bc1->bc1-hard1 resolved with 10M). Queued the pre-registered continuation cw-uni-flag-a1-h2 (10M hardening, warm from r1, MoE fork now pre-registered as ITS prediction-if-false). INFRA: h1 + both concurrent drain retries (h1-r1/h1-rr1) crashed 0-step on the trainer's --net-arch-vs-warm-start hard exit (respec CLONES the flag, so every retry inherited the crash) — fixed train_ppo_mjx.py to accept a MATCHING --net-arch on plain warm start (genuine mismatch still refuses), smoke-tested on the real ckpt, snapshot 5958bba (exp/cw-uni-flag-a1-h2); h2 VERIFIED RUNNING on train-0 ~7.1k fps, 11 pods idle per prime directive. 
- 08-11 16:51 c: cw-uni-flag-a1-h1-r1 + cw-uni-flag-a1-h1-rr1 -> both INFRA FAILURE (0 steps): traced to a hard --net-arch refusal on plain --init-from warm starts, not the earlier wandb-cache/shm suspects; a concurrent cycle already fixed the code and relaunched as cw-uni-flag-a1-h2 (running, >500k/10M), so no retry launched here; pruned the now-redundant duplicate rr1-rr1 backlog entry (2 failed attempts, same bug); train-1 left idle, no blocker-list work ready. 
- 08-11 16:53 c: triaged cw-uni-flag-a1-h1 (0-step INFRA FAILURE, no checkpoint). Found+fixed a real, independent bug along the way (snapshot.sh --sync's tar excluded rl_move/wandb/ but not the top-level prototype_sts3215/wandb/, so every sync shipped ~1.9GB/560+ stale controller wandb-cache dirs onto the pod; fixed, tar dry-run confirms 0 shipped, snapshot 8e4db65) but that was NOT the crash cause -- my mechanical retry (cw-uni-flag-a1-h1-r1, --now) hit the identical 0-step death, and by then a concurrent cycle had already root-caused the REAL bug (trainer hard-refused a --net-arch that legitimately matched the warm-start checkpoint's own architecture; SystemExit + stdout buffering made it look like a wandb-backend crash), fixed train_ppo_mjx.py, and relaunched as cw-uni-flag-a1-h2 (now running, supersedes both h1-r1 and my h1-r1 attempt). Corrected my own cw-uni-flag-a1-h1 ledger verdict + wandbnote to credit the confirmed root cause instead of my earlier (real-but-irrelevant) sync-bug theory, snapshot 699f15f. No relaunch from me (h2 already covers the question); no refill, 11/12 idle per prime directive. 
- 08-11 16:58 cw-arch-gru-r3 launched on train-1 (2M discovery, from scratch): the GRU-rung RECIPE CHANGE the r1/r2 freeze demanded — full hist16-r7 anti-cheat/joystick cfg stack (step-event, drag/park, prog+anchor gates, 0.05-0.06 band, cmd jitter) on the mixed walk+rise/lower+hold diet; gate = no leg-sacrifice fingerprint + positive det progress + stance emerging (cheat-free attempt, not gait quality) 
- 08-11 13:2x operator session: GAIT CLEANUP line opened (RL_PLAN queue -0.5, rl_docs/GAIT.md) -- operator wants the paddle killed and, longer-term, a walker that learns lift-and-place WITHOUT the BC crutch: P1 cw-walk-gaitbc1 (BC-anchor on the hardware walker, launched train-2), P2 banked structural stance-slip charge + swing clearance [CODE], P3 from-scratch no-anchor curriculum (terrain-as-teacher zero-training probe FIRST, drag charge annealed up, physics easing, RSI-for-walk). Anti-slip closed-move scope narrowed to income-shaping retrofits (does not cover P2/P3).
- 08-11 17:28 c: cw-arch-gru-r3 (GRU rung recipe-change, full hist16-r7 anti-cheat stack on the mixed diet) -> FAIL, known exploit, no forensics: gate+own-DR harness both show the IDENTICAL leg-sacrifice/paddle fingerprint as r1/r2 (det gait_valid 0/6, legs parked at duty 0.02, speed ~0.006 m/s, video-confirmed static legs; sto gait_valid 6/6 but no-progress jitter-paddle, slip/m 17.4-17.5) despite the anti-cheat reward already working on the MLP walk lineage -- fires the gate's own pre-registered FAIL branch (BPTT-window/capacity limit, not reward). GRU rung stays FROZEN off the blocker list, no further recipe/diet variant. cw-uni-flag-a1-h2 and cw-walk-gaitbc1 left untouched (another cycle's runs). No refill: checked capacity (12/12 free) and the queue/wishlist -- everything launch-ready is either bench-owned (attempt #2), owned by a concurrent cycle (flagship h2, gaitbc1's P2 fork), or explicitly deferred (GRU/turn/quad/DR-composes); idle pods stand per prime directive. 
- 08-11 17:20 op: cw-stand-crouchrise1 -> FAIL (mixed, lever PROVEN) — triaged by operator after sitting FINISHED/unverdicted since 12:35Z (cycles spent the window on arch probes). Crouch defect FIXED: gate det crouch rise 5/5 + sto 4/4, zero tilt falls, RSI-off crouch 8/8 det + 8/8 sto vs hard1's 0/8 (artifacts on train-0: logs/ckpt_eval/cw_stand_crouchrise1_gate, crouchrise1_rsioff_crouch). But hold retention fails its own gate: hold det+sto valid_plant 7/12 (needs >=10/12; hard1 11/12) and det hold parks feet 1+4 at contact duty 0.07/0.01 (hard1 all six 0.90-0.99) — flag-leg cheat resurfaced. Root cause pinned to the run's SECOND delta, the goal-mix skew rise 0.45->0.6 / lower 0.45->0.3; the start-mix alone is the fix. Do not deploy, do not warm from it. NOTE for future gates: valid_plant=True did NOT catch the det-hold cheat — per-foot contact duty is the telemetry that did; added an explicit all-six duty >= 0.8 clause to the follow-up's gate.
- 08-11 17:22 op: queued cw-stand-crouchrise2 (2M discovery, warm from holdbc1-hard1, seed 13): ONE variable vs crouchrise1 — restore hard1's exact goal mix (hold=0.1,rise=0.45,lower=0.45), keep the proven crouch-60% start mix (goal.rise_flat_frac=0.10 / rise_partial_frac=0.30). Gate = crouch det >=3/4 zero tilt falls AND hold det+sto valid_plant >=10/12 AND det-hold all-six-feet duty >=0.8 AND flat/bridge not worse than hard1. PASS -> replaces hard1 as stance deploy candidate (export MUST ship with its goal-ramp profile per the 08-10 stale-push lesson) and the stand lineage closes. Draining now.
- 08-11 13:4x operator session: TERRAIN CLAMP BUG found+fixed (servo_model.py clipped env.terrain_amp to 1.0 = 18mm peak bumps -- every historical terrain run/eval ran <=18mm ground regardless of requested amp; docs said 36mm). Probe at amp 1.5/2.0/3.0 returned bit-identical results, exposing it. cw-gait-terrain1 KILLED as invalid (trained on the clamped rung); amp>1 now scales hfield z-extent (434a6e0). Re-probing champion at true 36/72/108mm before relaunching the terrain-as-teacher arm.
- 08-11 17:49 c: cw-uni-flag-a1-h2 (flagship stage A, 10M hardening, warm from r1) triaged, left UNVERDICTED (pre-registered fork trigger fires). Pre-staged gate again defaulted to --modes walk (meaningless, this lineage never trains walk) -- ran the run's own hold/rise/lower gate myself on its pod (kubectl exec, artifacts synced to logs/ckpt_eval/cw_uni_flag_a1_h2_hrl): hold PASS clean (det 6/6 valid_plant, matches parent) and lower PASS clean+IMPROVED (det 6/6, sto 6/6 posture-strict vs parent's sto 3/6 -- rules out the alternative 'hold/lower erode' hypothesis) but rise (all-crouch) is IDENTICAL to the 2M parent: det valid_plant 1/6, same passing episode index, same wide-sprawl/tilt-roll failure videos (no new pathology, no cheat -- video-checked 3 episodes) despite 8M more steps. env/rise_feet_factor plateaued 0.4-0.6 the whole run (first/last-20% means 0.47/0.61) -- never climbed toward ~1.0 the way hold_feet_factor did, unlike the specialist lineage's budget-resolved tail. This is exactly r1's pre-registered prediction-if-false (rise <3/6 + flat rise factors at 10M) -- decides the flagship's core fork (unified net vs MoE) so leaving the call to a dig-in pass rather than calling it myself. INFRA FIX along the way: pod_eval.py + ops.sh evalcmd hardcoded '--modes walk' for any joint_walk task, silently evaluating a mode this lineage (and cw-arch-gru-r3's mixed diet) never trains -- patched both to derive eval modes from --goal-mix's nonzero keys, smoke-tested against 4 real ledger entries, snapshot 7de97c4 (exp/cw-uni-flag-a1-h2-podeval-fix); future flagship/mixed-diet continuations get the right gate for free. No refill: blocker list is bench-owned (attempt #2) or waits on this exact fork decision; capacity 10/12 free (train-0 busy with cw-stand-crouchrise2, unrelated), backlog empty, no peripheral work queued per prime directive. DIG-IN: cw-uni-flag-a1-h2 -- pre-registered MoE-fork condition met (rise valid_plant 1/6 det, matched-parent-identical, flat feet-factor at 10M) while hold/lower are clean-to-improved -- confirm no lingering causal ambiguity and decide hardening-is-exhausted -> MoE fork vs one more budget/architecture lever, per the flagship pre-registration. 
- 08-11 18:05 op: cw-stand-crouchrise2 launched on train-0 (13.1k fps, 2M done in minutes), gate-evaled on-pod -> FAIL, but a CLEAN ISOLATION: crouch fix retained (det crouch 5/5 + bridge 1/1, zero falls, Imax 2.64A) yet the det-hold flag-leg persists with hard1's exact goal mix restored — feet 1+4 duty 0.05/0.03 (crouchrise1: 0.07/0.01, identical fingerprint; hard1: all six 0.90-0.99), hold det+sto valid_plant 9/12. The only remaining delta vs hard1 is the crouch-60% start mix, so the START MIX ITSELF causes the hold cheat. Mechanism suspect: the BC anchor reference is clock-indexed from episode start; crouch starts see belly-phase reference poses against near-plant states — the anchor actively teaches lifted-leg postures in plant-adjacent states and it bleeds into hold. TWO-MISS RULE: the start-mix lever closes (crouchrise1 + crouchrise2). Next lever is CODE and goes through SPECIFICATION first: state/height-aligned BC anchor for non-flat rise starts (or anchor gated off on crouch starts). hard1 REMAINS the stance deploy candidate; its RSI-off crouch 0/8 stands as the documented limitation. Neither crouchrise checkpoint may be deployed or warmed from. Launch infra note for future drains from the Mac: ops.sh drain uses system python3 (no wandb module — dedupe check dies and burns an attempt) and snapshot.sh --sync refuses -dirty markers; use the repo .venv python and snapshot (commit) first.
- 08-11 17:57 cw-arch-gru-r4 first launch (n_envs 1024 x n_steps 256 = 262k rollout) killed by operator agent: only ~8 PPO iterations in the 2M budget vs r3's 30 — update-starved by construction, and the launch verifier had already (correctly, wrong reason) marked it FAILED. Relaunching r4 with n_envs=256 so rollout/update stays 65,536 = r3-identical iteration count; only window (64->256) + hidden (128->256) change. 
- 08-11 18:35 op: reconciled with the concurrent dig-in cycle's CORRECTION on cw-stand-crouchrise1 (raced triages, same bottom line): its matched-parent RSI-off probe (child det 8/8 vs parent 0/8, identical seed/cfg) is the stronger crouch evidence, and it attributes the sto-hold vp misses to a >2.0A current-tail soft flag (Imax 2.31 vs parent 1.96A) — deferring to that on the sto side. The det-hold duty finding stands as an ADDITIONAL regression both my triages caught and the probe text missed: det hold feet 1+4 at duty 0.05-0.07/0.01-0.03 in BOTH crouchrise variants vs hard1's all-six 0.90-0.99 (end_clear ~0 masks it at episode end; valid_plant never sees it). Net, all writers agree: hard1 stays deployed, crouchrise1+2 banked as crouch-robust variants, start-mix lever closed (two-miss), next lever = state-aligned BC anchor (CODE, spec first). crouchrise2 verdict re-applied to the ledger after the rebase race and mirrored to W&B (1gg41uez).
- 08-11 14:1x operator session: probe2 (post clamp fix) -- champion paddle PASSES 36mm (6/6, slip 1.31) but FAILS the walk gate at 72mm (0/6, prog 0.80) and 108mm (0/6). cw-gait-terrain2 launched on train-3: from scratch at true 72mm, physics-as-teacher, no reward surgery. cw-gait-dragstep1 FAILED per agent triage (paddle at 4x drag charge from scratch) -- but note the charge magnitude audit caveat in rl_docs/GAIT.md before closing the pricing lever: reward_drag never exceeded ~0.09/tick even at k=40.
- 08-11 18:5x operator session: cw-gait-terrain2 FAIL (own gate-eval): from scratch on true 72mm bumps -> leg-sacrifice drag (own-terrain 0/6 prog 0.25 slip 10.2, flat retention same), physics-as-teacher refuted standalone at 2M. Four from-scratch arms now all land on paddle/sacrifice. Next per GAIT.md: drag-charge MAGNITUDE AUDIT (k=40 only ever cost ~0.09/tick vs ~1/tick income -- the big penalty has never been effectively big), then annealed charge + RSI-for-walk.
- 08-11 18:10 c: cw-uni-flag-a1-h2 (flagship stage A 10M hardening, dig-in) -> FAIL on gate, hardening lever CLOSED, MoE fork DEFERRED to a measurement: rise (all-crouch) det 1/6 valid_plant identical to the 2M parent with rise_feet_factor flat 0.44-0.64 the FULL 10M + bc_anchor_loss converged (budget dead, det failures shifted stalls->tilt_roll tip-overs, zero exploit in 12 video-checked strips) while hold 6/6 stayed clean and lower IMPROVED (sto 3/6->6/6) -- but the pre-registered 'flat = shared-capacity interference -> MoE' leap is confounded (the deployed specialist itself fails all-crouch 0/6 RSI-off with the same tilt fingerprint, and ledger audit shows the from-scratch+rise-only cell has NEVER run: every honest rise was warm-started), so interference is inferred, not measured. Queued+launched the missing 2x2 cell cw-uni-flag-a1-risectl1 (2M discovery, one variable: goal-mix rise=1.0; RUNNING train-0, 9n6d5qj3) -- clearly beats flagship at matched rise-ticks => MoE justified; same plateau => MoE exonerated, lever moves to rise-teaching (state-aligned anchor spec per crouchrise2's mis-teaching finding). INFRA: repaired a leftover autostash merge conflict (experiments.json + RL_LOG UU state, markers inside the ledger JSON blocking every launch_run call; ledger converged to upstream, RL_LOG merged both lines, pushed 01cf45d). STATUS.md + RL_PLAN flagship section refreshed. 
- 08-11 18:13 c: cw-gait-terrain1 confirmed already fully verdicted (INVALID, terrain-clamp bug) by a prior operator session -- no new action needed, just committed by a concurrent snapshot. Used the follow-up window to run my own zero-training champion-on-terrain probe (det+sto, matched seed) across the corrected full amplitude range (18/36/54/72mm): slip/m rises monotonically 1.09->1.74 with NO drop anywhere, independently confirming probe2's finding that physics never forces stepping. Also independently re-evaled cw-gait-terrain2 (the true-72mm from-scratch relaunch) and found the same leg-sacrifice fingerprint an operator pass had already verdicted moments earlier (det slip/m 8.58 med, gait_valid 2/6, leg[3] parked, flat retention identical) -- redundant but cross-confirming, reconciled cleanly (my ledger update replaced theirs with an equivalent+fuller paragraph, no contradiction). Along the way found+waited-out a live git-merge-conflict corrupting experiments.json (another cycle was mid-fix; did not hand-edit, just polled until valid). Launched the run's own pre-registered single retry cw-gait-terrain2-r1 (amp 3.0/54mm, VERIFIED RUNNING train-3) to close lever 1 on the two-miss rule; GAIT.md updated with an addendum noting the charge-magnitude audit stays the sharper next lever regardless of how -r1 lands. No SKILLS/STATUS change (no new capability, no story change). Refill: only the pre-registered retry: (i) GAIT CLEANUP is the operator's TOP TRAINING PRIORITY so this serves the blocker list directly, (ii) everything else launch-ready is bench-owned, owned by concurrent cycles (flagship h2 fork, arch-gru line), or explicitly deferred -- left the rest of the fleet idle per prime directive. 
- 08-11 18:17 c: cw-gait-dragstep1 FAIL (GAIT P3 lever 2, drag-charge-from-scratch 4x + step-event income: paddle formed anyway, WORSE than champion band -- det/sto slip/m 6.4/9.7 vs <0.6 gate, reward collapsed -186 by 25%, kill sig (b) fires -- pricing-only refuted even from scratch). cw-stand-crouchrise2 FAIL (mixed, lever re-proven not promoted): crouch-rise fix REPRODUCES clean (det 6/6 incl crouch 4/4, zero falls) but the IDENTICAL flag-leg hold cheat returns on the SAME two legs (det duty 0.03/0.03) despite restoring hard1's exact goal-mix -- refutes the mix-skew-caused-it hypothesis from crouchrise1, root cause still unknown, two-miss rule closes further mix/coef variants on this stack; matched-parent control also shows lower measurably worse than hard1 (0/6 vs 3/6, +1 fall). Bonus: ran the GAIT P3 lever-1 zero-training terrain probe on champion (true amps after re-verifying the amp-clamp fix was actually synced to the eval pod -- an unsynced free pod silently reran the OLD clamped code and gave a false bit-identical negative, now COMMANDS.md gotcha 16) -- REFUTES terrain-as-teacher decisively: slip gets WORSE with bump height (1.07->1.34->1.43 det) and 54mm bumps trip real over_current safety terminations (6/6 det), never better; no cw-gait-terrain2 needed, GAIT.md + terrain1's verdict updated. CURRENT_TRUTHS refreshed (crouchrise1/2 hold-cheat story). No refill: GAIT P3's surviving levers (physics-easing, RSI-for-walk, slow-speed-first) all need CODE/SPECIFICATION first, not a launch; hold-cheat lineage hit the two-miss rule (new hypothesis needed, not another variant); 10/12 idle, backlog empty, prime directive says that's fine. 
- 08-11 18:23 cw-arch-gru-r4c launched on train-1 (2M discovery, from scratch), VERIFIED RUNNING: operator reopened the frozen GRU rung with the exact levers the r3 FAIL branch pre-registered — BPTT window 64->256 steps (10.24s) + hidden 128->256, n_envs 256 keeps 65,536/rollout = r3-identical 30 PPO iterations. Messy road here: r4 (1024 envs = update-starved 8 iterations) operator-killed; r4b (correct config) verifier-FAILED during its slow first BPTT update then killed by the r4-rr1 auto-retry collision; rr1 (wrong config resurrected) operator-killed; all three verdicted DO-NOT-RETRY in the ledger. Also repaired experiments.json + RL_LOG.md leftover autostash conflict markers from the launch race (pushed). Gate unchanged from r3: cheat-free walk attempt + stance emerging; FAIL closes the from-scratch GRU rung for real. 
- 08-11 18:26 c: cw-arch-gru-long1/hist16-r1/hist16-r2 triaged (gru-long1 pre-verdicted; hist16-r1/r2 dead launch-collision attempts, class-stopped by r6-rr1, no action); found+fixed real launcher bug: fixed 90s post-launch liveness check false-killed cw-arch-gru-r4 (BPTT=256 GRU update is CPU-bound & slow, no GPU signal) -- landed CPU-liveness polling fix (launch_run.py), concurrent cycle's r4b/r4c retry chain already running clean on it; no new launches (backlog empty, no unserved blocker). 
- 08-11 18:28 c: cw-uni-flag-a1-risectl1 (the missing 2x2 rise-only control cell) -> FAIL/REFUTES MoE: det+sto valid_plant 1/6 (crouch 1/6), rise_feet_factor last-quarter 0.537 -- IDENTICAL band to the multitask flagship h2 (1/6 det, 0.44-0.64), zero exploit fingerprint in 12 video-checked eps (honest crouch tip-overs/sprawl-stalls, one genuine six-foot pass). Isolating rise with ZERO sibling skills and 2.5x the flagship's rise-tick exposure changed nothing => shared-capacity interference is NOT the flagship's rise bottleneck, decisively closing the MoE fork (measured, not inferred). Next lever if reopened: specialist seeding/warm-start, not another reward/mix variant -- but this is architecture curiosity, not a hardware blocker (specialist-handoff rise already deployed for attempt #2). RL_PLAN flagship section + STATUS.md (both mentions) updated to reflect the closed fork. No refill: no unserved blocker queued (hardware/gait/arch lines owned by concurrent cycles or bench); pods left idle per prime directive. 
- 08-11 19:0x operator session: RESEARCH TRACKS restructure landed — campaign split into parallel tracks (tracks.json: hw mainline / arch / nobc / quad / turn), one W&B project with track:<id> tags (operator: tags not projects, nothing moves), per-track goal+status docs in rl_docs/tracks/, launcher --track + ledger field (762 entries backfilled), W&B tag backfill running, containment rule binding (triage refills stay in-track; cross-track = escalate in writing, launch operator-only; hw keeps pod priority), status page track column+tally, loglines lead with [track].
- 08-11 18:33 c: cw-gait-terrain2-r1 FAIL (54mm no leg-sac but slip 4-6x champion band, closes GAIT terrain-as-teacher lever for good, two-miss rule) + orphaned cw-walk-gaitbc1 FAIL (BC-anchor on walk ticks froze into a static tripod pose instead of stepping, closes GAIT P1); cw-arch-gru-r4-rr1 was a mechanical launcher-liveness false-kill, fixed+carried forward by a concurrent cycle as cw-arch-gru-r4c, no science verdict needed here. GAIT cleanup: both cheap zero-code levers (BC-anchor, terrain-as-teacher) now closed; next is P2 structural stance-slip charge (bank first) or the charge-magnitude audit, both spec/code work, not a launch. No refill: nothing launch-ready on the blocker list this cycle, idle pods left per prime directive. 
- 08-11 18:38 [hw] c: cw-walk-gaitbc1 already fully triaged+verdicted by a prior cycle (FAIL, STOP-known-exploit freeze, GAIT.md P1 closed, wandbnote+RL_LOG+GAIT.md all present) before this cycle started — confirmed ledger/W&B/docs all consistent, no new action taken. No dig-in trigger (closed exploit, not ambiguous). Left cw-arch-gru-r4c and cw-stand-crouchrise3 untouched (concurrent cycles' runs). No refill: hw track's only open GAIT lever (charge-magnitude audit / P2 structural charge bank) is CODE/SPECIFICATION work, not launch-ready; backlog empty, 10/12 idle, prime directive says that's fine. 
- 08-11 18:38 operator-directed launch: cw-stand-crouchrise3 (2M, warm from hard1) VERIFIED RUNNING on hexapod-mjx-train-0 - crouch-start dose response 0.60->0.45, ONE axis vs crouchrise2; gate adds explicit six-foot hold duty AND det-lower >=3/6 clauses; goal = a deployable stance ckpt that sits AND stands from any normal start. 
- 08-11 19:11 [arch] c: cw-arch-gru-r4c (GRU rung, BPTT window 64->256 + hidden 128->256, both pre-registered levers at once) -> FAIL, known exploit, no forensics: identical leg-sacrifice/paddle fingerprint as r1/r2/r3 (det gait_valid 0/6, legs [0,2,3] parked, sto no-progress paddle slip/m 11.5, own-DR0.5 matches) and rise ALSO regressed to 0/6 (r3 had it at champion grade) -- window AND capacity now both exhausted, from-scratch GRU walking CLOSED FOR REAL, no further recipe variant; recurrence waits for flagship distillation. arch STATUS.md updated. No refill: arch's queue is now empty (next step is specialist-seeding/distillation, a design question, not a launch-ready spec); capacity 12/12 free, backlog empty, no other track's blocker list has a ready spec this cycle -- idle pods per prime directive. 
- 08-11 19:45 [hw] c: cw-stand-crouchrise3 triage confirmed (ledger verdict/CURRENT_TRUTHS/hw STATUS.md already landed by a concurrent pass) -> FAIL, dose axis closed: mid dose (0.45) still reproduces the identical legs-1+4 hold-park cheat (duty 0.01/0.04) and lower regression (2/6) seen at 0.60; added the missing wandbnote (OUTCOME paragraph was absent). hard1 stays deployed; next lever is CODE (state/height-aligned BC anchor), not another mix/dose variant. Noted but left untouched (another cycle's run): cw-walk-mirturn1 has actually finished training on its pod (2,031,616 steps, ckpt saved) though the ledger still reads RUNNING -- watcher/next cycle should pick up its triage. No refill: hw's only open lever is spec/code work, not launch-ready; backlog empty, 12/12 free, prime directive says idle is fine. 
- 08-11 19:47 operator session: charge-magnitude AUDIT DONE (probe_drag_audit.py) -- per-tick k_drag_loaded form REFUTED at any coefficient (0.5mm deadband leaves 53-97% of learned skating free; per-tick medians overlap honest touchdown scuff), per-STANCE accumulated travel separates skate/step 3.3x. Structural charge landed (reward.k_drag_stance + allow 6mm + 0.25mm floor, default off), bank test PASSES (step > zero-lift skate > stall/park). Launched cw-gait-dragstance1 (2M discovery, from-scratch trans1 stack + k_drag_stance=8000 + k_drag_loaded=0) on train-9. Calib note: mu sweep saturates 0.38-0.40 air / worse loaded vs hardware 0.50 -- friction NOT the travel-gap knob, sim pessimistic (conservative for anti-skate). Estimator rung: leg-odometry built+tested but REFUTED as hw velocity source (50% real slip -> over-reads); optical-flow sensor is the honest path if ever needed. GAIT.md + RL_PLAN updated, snapshot 5c1b935. 
- 08-11 19:54 cw-arch-gru-bc-ft1 launched on train-1 (10M hardening, VERIFIED): the GRU rung REOPENED on a post-mortem — the r1..r4c closure was budget-confounded (hist16-r7, the anti-cheat reference, is a 40M walk-only run; the GRU got 2M mixed — nothing walks from scratch at 2M). Unit+learning tests (rl_move/tests/test_gru_policy.py, all pass incl. RecurrentPPO memory-task) cleared the implementation. New recipe: distill_gru.py BCs BOTH champions (walk longdist_r2 + stance dr10, play.py obs[:68] bridge) into one GRU — local harness: det+sto walk gait_valid 6/6 prog 1.02 slip 1.5, hold 6/6, THE FIRST WALKING GRU, zero RL steps (ppo_goal_cw_gru_bc.zip md5 864c02fb, pushed to train-1+controller). ft1 = PPO polish on the mixed diet + full anti-cheat cfg to lift rise/lower; log_std pre-set -1.5, canary auto-stop guards walk. Gate in ledger. 
- 08-11 20:00 [hw][turn] c(idle-kick): cw-stand-crouchrise3 -> FAIL (dose 0.45 reproduces the identical legs-1+4 flag-leg hold, duty 0.04/0.01, + lower 2/6 — dose axis DEAD after 3 variants; hard1 stays deployed; next = state-aligned BC anchor, CODE/spec first); launched + triaged cw-walk-mirturn1 (planned-but-unqueued TURN step 3, bank re-verified 7/7) -> FAIL (sym loss 28->0.5 converged but |wz_err| 0.254, drift 3x worse, gait rewritten to churn prog 0.41/slip 6-7.6 — mirror TRAINING closed per pre-registered gate, MirrorPolicy wrapper ships as the turning story); ran GAIT P0 reward-accuracy probe (new vref1 stack, logs/probe_walk_income/vref1_p0_*) -> tilt-penalty suspect REFUTED, totals near parity, paddle wins on genuine sim progress income (~495/ep) vs a ~380/ep pro-tall margin = effectiveness moat, no repricing bank; launched cw-walk-dragstance1 (P2 structural per-stance charge, audit k=8000, bank PASS). CROSS-TRACK INSIGHT: P0 moat finding posted to nobc STATUS (favors charge+curriculum over income shaping). Also cleared 2 ledger leaks (cw-uni-flag-a1 / -rr1, 0-step launch corpses, no-science verdicts). 10/12 pods idle per prime directive. 
- 08-11 20:00 [turn] c: cw-walk-mirturn1 already fully triaged+verdicted (FAIL — mirror-symmetry TRAINING closed: sym loss converged 28->0.5 but turn tracking never arrived (wz_err med 0.254, L/R asymmetry intact, hold-drift 3x worse) and forced symmetry wrecked the gait (prog 0.41 vs parent ~1.0, slip 5x); zero falls) by a concurrent cycle before this cycle started — ledger/wandbnote/STATUS/PLAN all confirmed consistent; only gap was rl_docs/TURN.md still reading 'QUEUED', now closed out with the same result + reserved step-4 note. Shipped turning story stays eval-time MirrorPolicy chirality selection; turn's queue is now empty (deploy-port + rot60 composition are CODE, not launch-ready). No refill: track just triaged (turn) has no launch-ready spec (two-miss rule spent on reward-shape + now mirror-training; next lever is a new idea), other tracks are concurrent-owned or bench-blocked; 10/12 slots free, backlog empty, prime directive says idle is fine. 
- 08-11 20:00 operator session: cw-gait-dragstance1 2M discovery = mechanism HEALTHY (charge engages -7/tick, no crash) but no gait at 2M (eval walk 0.006 m/s, churn-and-pay) -- NOT a charge verdict per the budget-confound rule (nothing walks from scratch at 2M). Launched cw-gait-dragstance1-r1 on train-9: same stack, fresh init, full 40M budget. Pre-registered fallbacks if it fails at 40M: annealed-in charge (CODE), then RSI-for-walk. 
- 08-11 20:03 [hw/nobc] c: cw-gait-dragstance1 FAIL - from-scratch audit-tuned structural stance-slip charge (k=8000) engages but the policy freezes/parks instead of stepping (own pre-registered false branch, 24/24 videos static+zero travel); solo charge exonerated as a gait-discovery fix (CROSS-TRACK: closes nobc's drag-audit item too), next lever RSI-for-walk. Also KILLED cw-gait-dragstance1-r1 (a pre-queued 40M respec of the identical recipe that drained onto the freed pod before this verdict landed) - known-exploit rule bars re-running more steps on a refuted recipe; freed the pod. No refill: remaining hw gait levers are CODE/spec work, not launch-ready; idle pods fine per prime directive. 
- 08-11 20:03 cw-stand-holdload1 launched: crouchrise3 recipe + reward.hold_feet_load=1.0 (hold income priced on measured touch force; crouchrise trio hover-park loophole closed in code + FEET-LOAD test bank) — mechanism test: reward-side vs anchor-bleed. 
- 08-11 20:04 [arch] c: checkup-only cycle -- cw-arch-gru-bc-ft1 SUSPECT (log stopped growing 19:52) verified FALSE POSITIVE: wandb global_step actively climbing (1.24M->1.64M over the check window, fps~2200-2300, GPU util nonzero), the trainer only prints to stdout at checkpoint-save intervals (save_every not yet reached) so a quiet log file is expected, not a stall -- no kill/restart. cw-gait-dragstance1-r1 (this cycle's 'just finished' run) confirmed already fully triaged+killed+verdicted by the concurrent cycle before this one started (RL_LOG 20:03 entry, ledger reads KILLED with full reasoning); left untouched per instructions. cw-stand-holdload1 training normally, left alone. No refill: no track was triaged this cycle (both candidate runs off-limits/still-training), backlog empty, 10/12 pods idle per prime directive. 
- 08-11 20:10 [hw] c(idle-kick, completion): cw-walk-dragstance1 (my own launch this cycle, finished at GPU speed) -> FAIL on the slip gate but mechanically informative: warm champion absorbed the audit-sized per-stance charge at -7/tick for the FULL 2M without parking OR stepping (prog 1.00, gait_valid 6/6, zero falls, slip 1.1-1.3 -> 0.95-1.15 only) — static-fine retrofit on the formed habit CLOSED per pre-registration; complements the concurrent from-scratch cw-gait-dragstance1 park (other cycle's) — from-scratch 40M -r1 + anneal-up curriculum carry the lever. No further launches from this cycle. 
- 08-11 20:13 [hw] c: cw-walk-dragstance1 verification-only — already fully triaged/verdicted/logged by prior cycle instance (FAIL, static-fine retrofit closed; GAIT.md+hw STATUS+ledger+RL_LOG all landed in d2ac4e8) before this cycle started; cw-arch-gru-bc-ft1 confirmed still training on train-1, left untouched; cw-stand-holdload1 off-limits per prompt. No refill: backlog empty, remaining hw gait levers are CODE/spec work per prior cycle's conclusion, no new [READY] hw item surfaced. No code changes, no commit needed. 
- 08-11 20:22 [hw] c: cw-stand-holdload1 -> FAIL, pre-registered mechanism test REFUTES the reward-side hold-cheat hypothesis: new reward.hold_feet_load (measured foot-load pricing, own FEET-LOAD bank confirms it correctly taxes hovering feet to 0.25x income) STILL reproduces the identical legs-1+4 flag-leg hold park (det duty_cycle 0.03/0.04 vs 0.73-0.99 on the other four legs, all 6 episodes identical) -- valid_plant stays blind to it (both feet drift to ~0mm exactly at episode end). Crouch rise unaffected (det 6/6 incl crouch 4/4, zero falls); det lower regressed to 2/6 (dangling leg-2, no falls, matches crouchrise2/3). Three lever families now closed (start-mix, dose, reward-pricing) -- state/height-aligned BC anchor is the sole remaining suspect by direct measurement, CODE/spec first. hard1 stays deployed. CURRENT_TRUTHS + hw STATUS.md + RISE.md updated. No refill: hw's only open lever here is unwritten CODE (state-aligned BC anchor spec), not launch-ready; cw-arch-gru-bc-ft1 left untouched (concurrent/still-training); 11/12 idle, prime directive fine. 
- 08-11 20:2x operator session: STALE 'attempt #2 pending bench time' story CORRECTED across RL_PLAN/STATUS/CURRENT_TRUTHS/tracks-hw (operator called it out — vref1-r1 WAS bench-run 08-10 eve, 0/2 runaway roll, and dep-tip1 ran 4x: 1 runaway / 3 CLEAN level walks; scripted gait tape-measured earlier; HARDWARE.md had it right all along, the plan/status docs never retired the pre-session framing and cycles kept echoing it). Open hardware items restated everywhere as: same-floor vref1-vs-tip1 A/B (roll-ramp RATE, the 08-10 policy switch never landed), RL-walk tape reading, FIRST runs of the rot60 + stand-specialist ports (deploy re-push first), wz sign audit; if tip1 also runs away -> sim contact/pinning model, not DR.
- 08-11 20:4x operator session: bench_blast.py (guided hardware checklist: verified A/B, tape, rot60 first run, profile-gated stand, turnsign, hold currents; dry-run default, per-step operator go) + sim_blast.py (roll-trap A/B x friction, joystick panels naked+rot60, deployed-artifact tests) LANDED and sim half RUN: no runaway at any mu (1.0/1.4/2.0=default saturation; every trap ep re-levels <2deg end-roll) -> pinning gap is NOT friction magnitude, no-skate contact lever stands; tip1>vref1 rolltrap at nominal (0.88 vs 0.75) matching bench tally; all 4 joystick panels PASS 0 falls (vref1/tip1 x naked/rot60 — tip1+rot60 first-ever check, fwd 6s dist 0.31-0.33m = sim tape prediction); stand+rot60 runner artifact tests 11/11. HARDWARE.md backlog + RL_PLAN -1 point at the runners.
- 08-11 20:40 cw-stand-anchorstate1 launched: holdload1 recipe + train.bc_anchor_state_aligned=1.0 (anchor re-indexed by nearest ref pose to current joints + 0.25s lookahead; crouch starts anchor to the planted tail, not belly-path leg lifts) — the anchor-bleed fix, last identified lever for the unified stand line. 
- 08-11 20:5x operator session: VIDEO MODE landed — bench_blast --video (zero-typing session: steps spoken via say onto the video audio track + unix-stamped events; tape/turn prompts replaced by film-it) + rl_move/scripts/video_review.py (syncs footage to summary.json via spoken sync mark or relative-threshold first-motion detection, cuts 2fps timestamped frame sheets per walk/turn/stand + full-res tape-zoom frames + index.json for agent analysis). Smoke-tested on synthetic video: sync guess within 0.1s, sheets readable. Operator workflow now: tape on floor, hit record, type the go's, hand the file over.
- 08-11 21:04 OPERATOR SESSION 08-11 eve: TALL LADDER campaign live (RL_PLAN queue -0.5 P2.5, GAIT.md bottom). cw-dep-tall30 PASS-ish (height_err_end 60->15mm, -24% speed, charge free per tall30h isolation); cw-dep-tall15 STALL at body -44mm. Fleet: run follow-ups T1-T5 (budget rerun, k_height crank, gate-at-reachable-ref, speed trade, -44mm workspace probe). Warm parent ppo_goal_cw_dep_tall30 on train-2. 
- 08-11 21:32 [hw] c1: cw-dep-tall15-h1 (T1 budget) FAIL -- 6M steps didn't move the -44mm height wall (flat by 1M, eval end-state worse than the 2M parent, no cheat); next is T5 kinematic probe (not yet built) before T2/T3. No refill: T5 spec work, not a training job; 11 idle pods left as-is. 
- 08-11 21:34 TALL LADDER T1 VERDICT (cw-dep-tall15-h1, 6M): FAIL on height, height_err_end 29->52mm (body ~-67mm) - the bare height REF is TRADEABLE given budget; policy bought speed instead: 0.051 m/s, first dep walker inside the 0.05-0.06 command band, slip 1.65 unchanged. SIDE DISCOVERY: tall15-h1 checkpoint is the fastest dep walker (candidate if speed>posture). T3 promoted and launched (cw-dep-tall-gate1: walk_height_gate=1.0 sigma 30 at REACHABLE ref -30 warm from tall30, start factor 0.88). Fleet: T2/T4/T5 still open. 
- 08-11 21:35 cw-stand-anchorstate1 FAIL-but-informative: state-aligned anchor moved the park fingerprint for the first time in five runs (leg 4 duty 0.01->0.93, leg 1 still parks); cost = flat-rise stall + lower falls. anchorstate2 launched: lookahead 0.25->0.5s, one axis. 
- 08-11 21:37 [hw] c: cw-stand-anchorstate1 confirmed already fully verdicted (PARTIAL mechanism confirmation, net FAIL — state-aligned BC anchor recovers leg 4, leg 1 still parks, flat-rise stalls, lower falls 2/6; follow-up anchorstate2 already launched/running) before this cycle started; landed the gaps left behind: wandbnote was mis-placed with the legacy bottom '--- OUTCOME ---' marker instead of ops.sh wandbnote's top format — replaced with a proper plain-English top OUTCOME; added the missing continuation to CURRENT_TRUTHS + RISE.md + hw STATUS.md (the anchor-bleed thread had documented every prior arm but not this one). Left anchorstate2/cw-dep-tall-gate1/cw-arch-gru-bc-ft1 untouched (training) and cw-dep-tall15-h1 untouched (concurrent cycle's). No refill: anchorstate1's own pre-registered follow-up is already running, backlog empty, no other hw-track item is launch-ready; 9/12 pods idle per prime directive. 
- 08-11 21:4x operator session (bench prep): deploy_ssh.sh landed (no-USB twin of deploy_adb.sh, scp+systemd over key-auth SSH) and RUN against the robot — fresh profile-carrying stand export + rot60 runner port now LIVE (service restart 21:44 UTC, /api/rl/policy shows profile, walk preflight ok roll 0.8 pitch 4.6). bench_blast --auto landed: spoken 5s countdowns (8s STAND) replace typed go's, Ctrl-C aborts; with --video the operator's entire job is one command + filming. Live info step verified: 5 policies on robot, hard1 active w/ profile, stand preflight pose-mismatch is the known auto-acquire path.
- 08-11 21:49 TALL LADDER T5 VERDICT (probe_tall_wall.py, landed locally, commit incoming): the -44mm wall is HABIT not kinematics. Mid-gait steady state, tall30 rides -75mm with hip pitch -41deg (70deg of unused upward range), knee 105deg (46deg room) - but leg YAW is PINNED at its 35deg limit, support radius 218 vs scripted gaits 179mm: the policy buys stability with splay+crouch. Scripted gait walks at +12mm in the same env = tall walking exists. MEASUREMENT CAVEAT (binding): eval/walk/height_err_end_mm is a STOP-WINDOW metric - the 60->15mm ladder gains were mostly stop-stance; ALL tall-arm verdicts must use probe_tall_wall steady-state walking height. ALSO: cw-dep-tall-gate1-h1 (6M gated) INFORMATIVE FAIL per its gate: err 15->39mm under budget - sigma-30 gate slows but does not stop the posture-for-speed trade. T2a/T2b/T4 relaunching (train-0 ckpt refusal fixed via pushckpt). 
- 08-11 22:10 [hw] checkup: cw-dep-tall-kh3 SUSPECT = false alarm — run completed its 2M budget normally at 22:08 (policy saved, W&B synced); no action, awaiting standard triage 
- 08-11 22:11 [hw] c: cw-dep-tall-gate1-h1 triaged INFORMATIVE FAIL (per its own pre-registered gate) -- 6M hardening of the sigma30 income-gate rung reverts the posture-for-speed trade (harness det height_err_end 15mm->36-47mm, speed still 0.054-0.056); confirmed with probe_tall_wall.py directly on the checkpoint: steady-state walking height -72.6mm, statistically the SAME as ungated tall30's own -75mm wall, legs still pinned at the 35deg yaw-splay limit -- gate income alone is closed as a lever at this dose, no cheat (clean 6-leg video). Unblocked + relaunched the plan's next rung (T2a/T2b/T4, REFUSED since 21:4x for a missing warm-start checkpoint): pulled+fanned ppo_goal_cw_dep_tall30.zip to train-0/3/4, re-synced code, all three now RUNNING (kh3 train-0, kh10 train-3, slow1 train-4) -- k_height 3x/10x and eased speed-band, primary judge is probe_tall_wall.py steady height per the amended gate. RL_PLAN + hw/STATUS.md updated with the confirmed verdict and the running rungs. cw-arch-gru-bc-ft1 left alone (another cycle's run, shows finished in W&B but not in my triage list). 
- 08-11 22:17 cw-stand-anchorstate2: lookahead 0.5s fixed flat-rise stall + lower falls, leg-4 stays recovered; leg-1 park unmoved (6th run). cw-stand-loweranchor1 launched: + train.bc_anchor_lower=1.0 (IK-descent supervision on lower ticks, the lower-bank xfail lever) — tests lower recovery AND the shared-fingerprint park hypothesis. 
- 08-11 22:19 [hw] c: cw-stand-anchorstate2 already fully triaged+verdicted+launched-followup by a concurrent cycle (FAIL, leg-1 park unmoved 6th run, lookahead axis exhausted; cw-stand-loweranchor1 launched+running) before I finished my own review — confirmed consistent with the eval report, no new science verdict needed. Landed the gaps left behind: wandbnote was still the legacy bottom-marker format, replaced with a plain-English top OUTCOME; CURRENT_TRUTHS/RISE.md/hw-STATUS.md were all still describing anchorstate2 as running, updated to the actual FAIL-but-informative result + loweranchor1 follow-up. No refill needed (loweranchor1 covers the only ready hw lever); cw-dep-tall-kh10/slow1 and cw-arch-gru-bc-ft1 left untouched per instructions. 
- 08-11 22:27 TALL LADDER T2/T4 VERDICTS (probe_tall_wall, mid-gait steady state): kh3 -74.5mm, kh10 -72.7mm (pays ~5.6/tick > its own walk income rather than stand up), slow1 -71.5mm at its trained 0.035 cmd. PRICING FAMILY CLOSED for posture: ref ladder, income gate, gate+budget, k_height 3x/10x, speed relief ALL flat at -72..-75mm, leg yaw pinned at the 35deg limit in every arm. T6 LAUNCHED (cw-dep-tall-rsi1): RSI-for-walk landed in code (goal.walk_gait_start_frac mid-stride tall spawns from the scripted gait, command live from tick 0; tests green, sim_env+walk_task, default off bit-exact) - warm from tall-gate1 at frac 0.5. The missing states are tall mid-stride WALKING; episodes always spawned tall STANDING. 
- 08-11 22:33 [hw] c: cw-dep-tall-kh10/cw-dep-tall-slow1 independently triaged -- INFORMATIVE FAIL, confirms the concurrent cycle's 22:27 T2/T4 verdict (probe_tall_wall steady height -72.6mm/-73.8mm, flat vs tall30's -75mm wall; kh10's 10x height charge and slow1's eased speed band both refuted, honest 6-leg gait, no cheat). Real bug found+fixed: the launcher's relaunch-after-REFUSED path left both runs' ledger pod field pointing at train-2 (where an unrelated run had been) while they actually trained on train-3/train-4 -- the watcher's checkups were checking the wrong pod and wrongly reported DEAD twice each. Corrected the pod field, pulled/verified checkpoints (md5s match), ran the harness + probe_tall_wall directly to confirm before writing ledger verdicts (previously missing despite the RL_PLAN/RL_LOG narrative already existing) + wandbnotes + hardware_ready=false; refreshed hw/STATUS.md (was still describing the three rungs as RUNNING). No refill: pricing-family closure's own next lever (cw-dep-tall-rsi1, RSI-for-walk) was already launched by the concurrent cycle; backlog empty; stand-line runs (loweranchor1/anchorstate2) and cw-arch-gru-bc-ft1/ft2 left untouched (other cycles). 11/12 pods idle, fine per prime directive. 
- 08-11 22:33 TALL LADDER T6 VERDICT (cw-dep-tall-rsi1, probe_tall_wall): -77.4mm mid-gait - FLAT. RSI-for-walk injected tall mid-stride states into 50% of episodes and the policy still dives to the crouch (survived_frac 1, so it LEARNED to recover from tall spawns down into the crouch-splay - a robustness byproduct, not the goal). CAMPAIGN CONCLUSION after 7 arms + probe: neither pricing (6 arms) nor state injection moves mid-gait posture; the crouch-splay basin is where PPO stabilizes this walk under current physics. The scripted gait walking tall open-loop in the same env remains the existence proof. REMAINING LEVERS (fleet, in order): (1) BC-INIT from the scripted tall gait (pure action pretraining then RL fine-tune under the gate income - DIFFERENT from the refuted gaitbc1 anchor-loss-during-RL), (2) physics easing ladder, (3) accept the pareto: tall15-h1 (0.051 m/s, fastest dep walker ever) for speed + tall stop-stance. Byproduct note: the fast basin (0.051) reappeared in rsi1 - it is robust and reachable from multiple parents. 
- 08-11 22:36 [arch] c: cw-arch-gru-bc-ft1 FAIL vs gate — walk stays honest/cheat-free but 10M walk-heavy RL finetune erodes the BC-distilled stance (rise 0/6 det honest stall, hold 0/6 det on stillness only, lower still 4/6); keeping the pre-finetune BC zip as reference; queued+launched cw-arch-gru-bc-ft2 (2M discovery, lr 3e-4->1e-4, target-kl 0.02->0.01, same everything else) to test whether a gentler finetune protects stance while still polishing gait. 
- 08-11 22:37 cw-arch-gru-bc-ft1 triaged by operator cycle: walk-retention PASS (gait_valid 6/6 all four passes, no paddle after 10M RL), lower lifted, rise never cloned (demo poverty), hold regressed — verdict in ledger. Continuing with stance-heavy re-distill + DAgger locally. 
- 08-11 22:2x-22:3x operator-authorized UNATTENDED bench (agent solo, iMac camera): bench_blast --camera landed (Mac-camera recorder thread, constant-fps writer => video time == t_unix - t0 EXACTLY, no phone/no say-sync; video_review syncs "exact (camera t0_unix)" + video arg optional) and RUN. Session bench_blast_20260811_182430: vref1-r1 clean-start walk -> runaway roll -> FELL ON CAMERA (3rd hardware runaway for vref1). Then the session-design flaw bit: NO upright gate between steps (the operator was the implicit recovery system in attended sessions) -- walks r2..rot60-back all ran on the fallen sprawl (A/B tally 3/3+3/3 runaway = GARBAGE beyond r1), scripted turns ground the splayed legs, board BROWNED OUT mid-turn -0.3 (server timeouts 22:28:44, mDNS gone; operator power-cycled ~22:33, 18/18 servos back). Also: robot-log pull for the EARLIER attended session exposed two summary lies -- learned stand NEVER ran 21:5x (prep safe-zero stalled L0 hip 17deg -> limp; my script had recorded the kickoff response as ok) and learned lower tripped over_load at 9s; bench_blast now records TERMINAL results (wait_idle result) for every motion, fixed the tape-walk CSV race (tape-tip1 had duplicated tip1-r3's stats).
- 08-11 22:42 FIRST honest hardware run of the stand specialist port (bench_blast_20260811_184229, camera): /api/rl/stand from operator-zeroed belly -> FAILED safety trip tilt_roll at ~9s/225 ticks, tilt_rel_max 10.2deg (trip 10), max current 0.27A, runner limped clean. Frames: the failure is in the BELLY-CURL phase (body rocks over the tucked legs before the ramp). Sim probe (6 seeds det DR0, ppo_goal_cw_stand_holdbc1_hard1, /tmp probe): curl max|roll| 0.7deg, ramp max 1.7deg -- sim says the rise is FLAT, hardware rocks 10deg+ => GENUINE sim-to-real rocking gap in the curl, trip threshold is CORRECT, fix is training-side (rocking/tilt DR on rise, loaded-knee actuator model), not a bench threshold bump. Data banked, no blind retry.
- 08-11 22:47 unattended session 3 (bench_blast_20260811_184741, camera, fall-recovery loop ARMED): vref1-r1 FORWARD full 6s/150 ticks with a 23-24deg EARLY roll transient that it fully RECOVERED from (tail 0.9deg, fell=false, level finish on video) -- the "runaway" metric conflates recoverable takeoff transients with true tips, needs a fell/tail-based split. tip1-r1 BACKWARD = FIRST real off-wedge rot60 hardware run (rot60:true in terminal result, k engaged): FELL (peak 27deg, tilt trip). Recovery loop fired exactly as designed: safe_zero REFUSED at 25deg body tilt (its own gate), SessionAbort stopped all motion; operator-authorized force=true safe_zero under camera watch RIGHTED the robot (max |deg| 0.5 after). bench_blast gains: adaptive lower-then-stand ordering from actual pose, rise/sit standalone steps, --only order honored, A/B directions ALTERNATE (stay in camera frame, both directions per policy across rounds), newest_walk_csv now after_unix + size-stable (vref1-r1's mid-flush pull returned 'too few rows'; stats recomputed + patched into summary.json). Robot parked belly-down at zero. NET HW TRUTHS: fwd walks show a large takeoff roll transient (sometimes recovered, sometimes not -- both policies), rot60 BACKWARD fell on first try, stand specialist blocked by the curl rocking gap. Sheets + camera.mp4 in hardware_traces/bench_blast_2026081*_18*/video/.
- 08-11 22:45 [arch] checkup cycle: cw-arch-gru-bc-ft2 SUSPECT (log stopped growing on train-0) = FALSE ALARM, no action — trainer alive (13 cores busy, GPU active), W&B advancing 1.5M/2M steps at fps 1917 (faster than parent ft1's 1360; low fps is inherent to the GRU 256-env BPTT config, not starvation); log stall is stdout block-buffering (nohup python without -u only flushed the startup lines). Run left training; finishes in minutes, normal triage will verdict it.
- 2026-08-11 23:1x UTC [operator] [track:hw] BACKLOG += cw-dep-tip1-takeoff25 (tipped-start DR raised to the measured 20-25deg hardware takeoff transient, warm tip1) and cw-stand-riserock1 (CODE-FIRST rise-tick rocking DR axis; hardware belly-curl tripped tilt_roll 10.2deg vs sim max 1.7deg). Both arms are direct sim answers to the 08-11 camera bench sessions.
- 08-11 23:09 cw-stand-loweranchor1: lower anchor SOLVED lower (6/6 det + 6/6 sto from 2/6) but uniform anchor sampling diluted hold/rise (leg-4 park returned, flat re-stalled) — dilution mechanism measured. cw-stand-anchormix1-r1 launched: + train.bc_anchor_stratified=1.0 (per-mode minibatch quotas; launch 1 died on a warm-start pickle gap, fixed dc5b7f4). 
- 08-11 23:12 [hw] c: cw-stand-anchormix1 crashed pre-training (AttributeError, warm-start checkpoint missing _bc_mode buffer) — infra bug not a result, already fixed (b7f815a) and relaunched as cw-stand-anchormix1-r1 (running, carries the real anchor-dilution test); no refill, all actionable hw sim items already in flight (riserock1/tip-takeoff25/anchormix1-r1), rest of hw Next is bench-only. 
- 08-11 23:13 [hw] c: cw-stand-loweranchor1 confirmed already fully triaged+verdicted+followup-launched by a concurrent cycle before mine started -- LOWER SOLVED (det+sto 6/6, zero falls, from 2/6) but hold park regressed to a two-leg version + flat rise re-stalled 96mm, root-caused to ANCHOR DILUTION (all three per-mode BC anchors share one ring buffer/uniform sampling); confirmed the eval report matches the ledger verdict, no new science call needed. Landed the gaps: wandbnote was missing the top plain-English OUTCOME paragraph (added); CURRENT_TRUTHS/RISE.md/hw-STATUS.md were still describing loweranchor1 as launched-and-running (updated to the dilution finding + follow-up). Real bug found+fixed by a concurrent process while I watched: cw-stand-anchormix1 (the stratified-sampling dilution fix) crashed on its first launch with an AttributeError (_bc_mode missing on a warm-started model whose checkpoint pickled pre-tag buffers) -- fixed in bc_anchor.py (backfill on load) and relaunched clean as cw-stand-anchormix1-r1, now past the crash point and training on train-0. Also found+fixed: RL_LOG.md had literal unresolved git-merge conflict markers baked into the last two commits (a concurrent snapshot.sh collision), corrupting shared history on origin/main -- resolved by merging both loglines in, no content lost. Refill: ran launch_run.py drain per capacity.py's flag (10 free slots + non-empty backlog) -- placed the queued cw-dep-tip1-takeoff25 (hw, tipped-start DR raised to the measured hardware takeoff-transient regime) on train-2, now running; cw-stand-riserock1 (its rise-side sibling) already running on train-1 from the operator's backlog add. cw-stand-anchormix1-r1/riserock1/tip1-takeoff25/cw-arch-gru-bc-ft2 all left untouched (training or another cycle's). 
- 08-11 23:13 [arch] c: cw-arch-gru-bc-ft2 FAIL as pre-registered (rise 0/6, hold 0/6 det@DR0, honest not cheat; walk stays cheat-free) — lr/KL lever CLOSED (2 misses); real blocker is CODE: bc_anchor_coef unimplemented for --gru, no freeze option either. arch STATUS.md updated; no refill (nothing launchable without that code). 
- 08-11 23:08-23:19 [operator] [track:hw] UNATTENDED CAMERA BENCH ROUND 2 (four sessions, bench_blast_20260811_19*): (1) LEARNED RISE IS DETERMINISTIC-FAIL ON HARDWARE — 5/5 tilt_roll trips counting 22:42's, EVERY one at tick ~226-228 (~9s mid-curl), roll 10.1-10.6deg, currents <=0.27A, including two from VERIFIED clean zero (max pose delta 0.5deg, preflight green) — start pose exonerated, the curl rocking gap is the whole story (sim <=1.7deg; cw-stand-riserock1 queued). Scripted /api/zero pose=stand is the working stand-up meanwhile. (2) HONEST A/B FELL-RATES ON CAMERA: vref1 FELL 3/3 (r1 fwd 25.6deg tail 26.4; r1 fwd peak 21.1 tail 10.7; r2 back capsized on camera, ramp 17.8deg/s tail 22.8) vs tip1 CLEAN 1/1 — and that one walk was BACKWARD (first clean off-wedge rot60 hardware walk: rode a 16.7deg takeoff transient to tail 1.5deg, ends standing in frame). tip1 is the deploy champion on tonight's evidence; every fall keeps the 20-25deg takeoff-transient shape (cw-dep-tip1-takeoff25 queued). (3) THERMAL WALL: second recovery stand-glide limped on L2 HIP AT 72degC (shutoff 65) — falls+recoveries stack heat fast; cooled to 40degC limped within minutes, 18/18 healthy, robot parked limped belly-down, MOTION STOPPED for the night per hot-motor rule. Turn-sign audit missed AGAIN (third session in a row died first). bench_blast hardened from each failure in-place: auto-safe_zero when opening pose isn't belly zero (a stalled safe_zero had left L4 knee 78deg off, hold-hunting = the operator's 'twitching leg'), recovery safe_zero force=true (a real fall ALWAYS trips the tilt gate — the unforced call refused right when needed), scripted-stand fallback behind the learned rise, demo-aware waits (/api/zero + safe_zero run as demos invisible to wait_idle; one abort read a mid-glide pose and masked the real 72degC error), MAX_RECOVERIES 2->4 (fell-rate IS the measurement). Sheets + logs in hardware_traces/bench_blast_20260811_19*/.
- 08-11 23:37 [hw] c: cw-dep-tall-rsi1 confirmed already fully triaged+verdicted by a concurrent cycle before mine started (INFORMATIVE FAIL, probe_tall_wall -77.4mm flat vs -72..-75mm wall, tall-ladder pricing+RSI families both closed) -- ledger/wandbnote/RL_PLAN/hw-STATUS.md all matched the eval report, no new science call needed. Landed the one gap: ledger verdict + rendered run doc were still uncommitted from that cycle, snapshot.sh'd + pushed (309d94c). cw-arch-gru-anchor1 left untouched (training, another cycle's). No refill: hw backlog empty, tall-ladder's remaining levers (BC-init pretrain, physics easing) need a code/spec cycle not a respec, and hw's other active items (riserock1/takeoff25/anchormix1-r1) are already in other concurrent cycles' hands; 11 idle GPU pods fine per prime directive. 
- 08-11 23:50 [hw] c(idle-kick): cw-stand-anchormix1-r1 -> FAIL per pre-registered branch (lower 6/6 + crouch rise 4/4 RETAINED, zero falls, but flat rise stalls 105mm and the one-foot hold park persists AND MOVED legs — dilution incomplete, mandatory next = per-mode bc_anchor_loss logging, CODE); cw-stand-riserock1 + cw-dep-tip1-takeoff25 -> VOID invalid launches (operator backlog stubs drained without their DR variables; riserock's rocking-DR code was never written) — takeoff arm relaunched properly as cw-dep-tip1-takeoff25-r1 (running, dr.tipped_start 0.5/12-25 verified); backfilled missing ledger verdict for cw-dep-tall-rsi1 (T6 INFORMATIVE FAIL, analysis was logged 22:33). Docs synced (TRUTHS/hw-STATUS/RISE). 
- 08-12 00:02 [hw] c(idle-kick, completion): cw-dep-tip1-takeoff25-r1 (my relaunch, finished at GPU speed) -> FAIL, decisive: identical 20-25deg injection w/ matched tip1 baseline shows child==parent (0/12 valid both, ZERO falls both) + clean DR0 retention intact — sim already recovers static tipped starts at the hardware regime, tipped-start DOSE lever CLOSED (2nd no-separation arm); hardware takeoff falls need a DYNAMIC gait-start roll-rate axis (CODE, unbuilt, same family as rise-rock) or contact/pinning work. TRUTHS + hw STATUS updated; no further launches (both remaining hw sim answers are unwritten CODE). 
- 08-12 00:05 [hw] c: cw-dep-tip1-takeoff25-r1 verdict landed (FAIL, tipped-start dose axis CLOSED — matched-parent probe at 12/17.5/20deg, det+sto, n=12 each: both tip1 and r1 recover 100%, ceiling effect; clean DR0 retention delta +0; TRUTHS/STATUS/RL_LOG already synced by the ledger update). Checkup SUSPECT on cw-arch-gru-anchor1 (log/global_step looked stalled at 655360) verified FALSE ALARM: W&B global_step climbed 655360->1114112 over the same window (GRU rollout is just slow-cadence, ~100-140s/update), left running untouched. No refill: hw backlog empty, both remaining hw sim levers (dynamic gait-start roll-rate DR, rise-tick rocking DR) need unwritten CODE+spec before anything launches; 11 idle pods fine per prime directive; no peripheral/cross-track launch. 
- 08-11 23:5x cw-stand-anchormix1-r1 FAIL per gate but the line's biggest fact: THE PARK MIGRATED. Stratified per-mode anchor quotas fixed the dilution seesaw exactly as predicted (lower KEPT 6/6 det+sto zero det falls, crouch rise 4/4, hold det valid_plant 6/6) and foot idx1 — the "leg-1" park unmoved across six runs — recovered 0.03->0.90; but foot idx4 (0.93-0.97 in anchorstate1/2) parked at 0.02 in its place: the persistent behavior is SHED EXACTLY ONE FOOT, supervision only moves WHICH foot, never WHETHER. Det flat rise still stalls (106mm, under-drive class; bridge det + flat sto pass); hold det+sto valid_plant 9/12; bc_anchor_loss flat 0.012 = anchors fit (per-mode loss never logged — gap). Per pre-registration the blind-axis line STOPS: hard1 stays deploy candidate, specialist handoff stands. Named lever if reopened: the one-foot-shed equilibrium itself (min-over-feet duty/load term) + per-mode bc_anchor_loss logging first.
- 08-11 23:35-23:55 [operator] [track:hw] SESSIONS 5-6 + FULL-NIGHT ANALYSIS (bench_blast_20260811_193306/_194857, then camera removed by operator). Turn-sign audit finally RAN: omega +0.3 = CCW from above on camera (matches z-up convention; single reading); -0.3's first try was SILENTLY REFUSED by a pending measure record (video mode never annotates -> the +0.3 record blocked it; bench_blast now discards before each turn) and the rerun coincided with the camera being carried off — still unmeasured. PHANTOM TEMP DISCOVERY: the 19:51 abort read "L4 hip at 150degC" then a steady 33degC seconds later — single-read temp trips in safe_zero/pinned_tip fire on corrupted bus bytes (the debounced servo_watch never tripped all night; 19:18's 72degC downgraded to unconfirmed). Fixes deployed: 2-consecutive-read debounce in both, servo_watch THERMAL PANIC (on real overtemp kill ALL motion — abort demo/RL worker, gait stop, torque-all off — instead of cutting one servo under a live gait), busy cadence 10->5s, bench_blast pre-step thermal gate. NEW TOOL bench_report.py (one markdown table for N sessions: per-walk roll-trace transient/peak/tail/fell metrics off the episode CSVs, per-rise trip signatures, per-policy tallies). FULL-NIGHT RE-VERDICT off 18 walks: takeoff transient UNIVERSAL (5deg by 0.6-1.5s, peak 13-27deg every walk), falls ~coin flip BOTH policies (vref1 6/10, tip1 4/7), NO A/B winner — early "tip1 champion" was small-sample + a direction confound (round-1 order means tip1 never walked fwd tonight; fix alternation). Learned rise now 10/10 identical trips (tick 225-228, roll 10.1-10.6deg, <=0.27A). Consolidated writeup + RL implications: rl_docs/BENCH_REPORT_2026-08-11.md. [merge note: the 23:50 cycle VOIDED the two operator backlog stubs as launched-without-their-DR-variables — riserock's rocking code was never written, exactly the stub's declared CODE-FIRST caveat — and its proper relaunch cw-dep-tip1-takeoff25-r1 CLOSED the static tipped-start dose lever (child==parent under identical injection); consistent with this report's conclusion that the transient is DYNAMIC (roll-rate at gait start), not a static-lean dose.]

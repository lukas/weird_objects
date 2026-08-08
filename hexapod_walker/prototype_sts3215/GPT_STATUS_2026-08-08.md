# Hexapod RL — status brief for external sanity check (2026-08-08, ~11:30 PT)

Audience: an outside reviewer (GPT) asked to sanity-check the campaign's
direction, the last 24 h of decisions, and the autonomous-agent setup.
Full history: `RL_LOG.md` (append-only), current plan: `RL_PLAN.md`.

## 1. System recap (unchanged fundamentals)

- 18-DoF hexapod, Feetech STS3215 serial servos, Arduino Uno Q, 25 Hz
  control. One motor already cooked by a bad agent decision — hardware
  deployment is gated hard (see §7).
- Sim: MuJoCo 3.11, calibrated servo model (latency, rate limit,
  deadband, torque→current), domain randomization 0→1.0 scale
  (friction, latency, mass/CoM, joint-zero error, IMU mount, strength).
- **Action space: 18 raw joint position targets** in [-1,1] → absolute
  angles through a SafetyLayer (~2°/tick rate limit, axis clips,
  tilt/current trips) — identical path on hardware. The earlier 6-D
  body-IK line is concluded (its flat-rise was a noise-fragile
  choreography; archived, never to be behavior-cloned from).
- PPO (SB3), 48 envs/run, goal-conditioned multi-task: hold / lean /
  rise (belly→stand, 3 start kinds: flat/bridge/crouch) / lower /
  raise (canary) / walk (commanded velocity).
- Everything logs to W&B (`l2k2/hexapod-balance`) with lineage
  stitching, periodic per-mode evals, and telemetry-overlay video reels.

## 2. Autonomous experimentation loop (running now)

- A watcher on a CoreWeave controller pod polls W&B; when a campaign
  run finishes it invokes a headless Claude agent with a standing
  prompt (`orchestrator/ORCHESTRATOR_PROMPT.md`) + guardrails YAML.
- Each cycle: git pull → harness-evaluate finished runs (fresh
  20-episode evals, det + stochastic) → download and honestly review
  videos → append verdicts to `RL_LOG.md` → review/optionally revise
  `RL_PLAN.md` (length-capped) → decide next experiments → snapshot
  code (commit + tag `exp/<run>` + push) → launch → verify in W&B.
- Guardrails: 6 pods / 12 slots, max 10 concurrent, 2 slots reserved
  for the agent's own smoke tests, hard forbidden list (never touch
  the physical robot), mandatory video review, "scalar gates NEVER
  override a bad video."
- Operator intervention yesterday: the agent's video verdicts were far
  too positive (called a 5-leg shuffle with a vertical "flag leg"
  progress). Prompt + guardrails now mandate pathology-first verdicts
  with an explicit hardware-ready yes/no. Round-8 log entries comply.
- Throughput work (validated): periodic eval/video moved to a
  background worker process (~free, was ~65% overhead), per-run logs,
  two 48-env runs per 128-core pod. Effective campaign throughput
  roughly 3–5× the old setup. Videos re-encoded yuv420p+faststart so
  W&B plays full length.
- Known watcher wart: it has twice re-fired on runs already verdicted
  (finish-event lag). Harmless (agent detects the dedupe and no-ops)
  but it burns a cycle; fix queued: skip runs that already have an
  RL_LOG verdict.

## 3. Where each skill stands

- **Stand↔belly (rise/lower): SOLVED at DR 1.0, twice, frozen.**
  Crown jewel. Any eval showing rise/lower <5/6 is flagged; never
  warm-start stand work from an eroded checkpoint.
- **Raise (lift body from stance): stuck 2–5/6 in every lineage.**
  Undiagnosed. Odd clue: `cw-walk-nv` (velocity obs zeroed) hit 5–6/6.
  A raise-heavy goal-mix run is in flight on the stance champion.
- **Walk: the central problem. Scalars pass, motion is BAD.**
  Champion `cw-walk-dr04b` tracks 0.02–0.06 m/s at DR 0.4 (sto 4/6 @
  vel_err 0.028, re-validated today) but video shows a shuffling
  3–5-leg exploit with legs parked vertically ("flag leg"). NOT
  HARDWARE-READY. Refuted levers so far: manual command-range widening
  (two attempts), 3× progress reward, and — as of last night —
  **all-modes flag-leg penalty**.

## 4. Round 8 results (last night, all three MISS — but informative)

- `cw-walk-flag` (k_flag_leg=5.0 charged in ALL modes): walk 2/6, and
  rise **collapsed** (det 1/6, bridge 0/5), raise 0/6. The penalty
  fires (~−0.5/step) but PPO pays it rather than restructure the gait,
  and rise pays collateral because belly-starts legitimately need
  >50 mm transient leg swings. Third confirmed case of cross-mode
  reward interference.
- `cw-walk-flag-s1` (seed twin): walk 2/6 but raise 4/6 vs twin's 0/6 —
  first genuinely divergent twins in production, confirming the
  seed-fix works. (All earlier "twin" pairs were bit-identical clones;
  every prior best-of-2 conclusion was discarded. The clone pair's
  eval spread — vel_err 0.026 vs 0.043 for the same weights — is now
  our calibration of 2-episode eval noise.)
- `cw-walk-nv` (deployable obs, velocity zeroed): walk 1/6 at 4M.
  Train-time vel_err looked fine (0.030); the independent harness
  disagreed. Continuation to 8M in flight; this is the naive baseline
  the asymmetric actor–critic must beat.
- Bookkeeping error caught: round 7 logged a launch (`cw-stance-
  raisemix`) that was never actually executed — no W&B run, no log, no
  process; pod sat idle 1.2 h. Relaunched this round, verified live.

## 5. In flight right now (round 8 launches, ~30–60 min in, early evals = 2-ep noise)

| Run | Change under test | Early signal |
|---|---|---|
| `cw-walk-flagw` | flag-leg penalty routed to **walk mode only** | walk vel_err 0.031; rise flat/bridge/crouch 1/1/1 **retained**; video: no parked flag leg in walk/hold |
| `cw-walk-flagw-s1` | seed twin | vel_err 0.037, rise 0.5/1/1 — genuinely diverging |
| `cw-walk-nv2` | nv baseline continuation → 8M cum | vel_err 0.032, improving |
| `cw-stance-raisemix` | goal mix raise=0.4 on DR 1.0 stance champion | rise 1/1/1, raise 0.5 so far |

Gates: flagw passes only with sto walk ≥4/6 @ ≤0.030 AND a video
showing a six-foot gait with no flag leg AND sto rise ≥4/6.

## 6. What the agent is building right now (capacity cycle in progress)

With 8 free slots, the agent is implementing the plan's #1 queued
architecture item: **asymmetric actor–critic** — actor sees only
hardware-available obs (joint pos/vel, comp-filter tilt, gyro,
per-servo current, prev action, goal, velocity refs), critic
additionally gets privileged sim state (true body velocity, contacts,
pose). New `asym_policy.py`, unit tests passing, smoke-train
checkpoint produced. Launch expected shortly; must beat whatever
`cw-walk-nv2` reaches at 8M.

Plan queue after that: (2) learning-progress speed curriculum for walk
(bucketed commands, sample buckets by improvement — replaces manual
widening, which failed twice); (3) temporal deployable actor
(~300 ms history: frame-stack vs small GRU ablation); (4) routed
flag term on the stance line's lower end-posture. These are the four
priorities from the operator-supplied literature review, integrated
into `RL_PLAN.md` yesterday.

## 7. Standing constraints (do not relax)

- Reward routing is now formal: GLOBAL = safety/current/smoothness;
  MODE-SPECIFIC = everything else; every new term declares routing.
  Interference proven three separate times.
- A checkpoint is not a result until an exact-path visual eval; video
  verdicts pathology-first with explicit hardware-ready yes/no.
- ≥20 episodes for gate decisions; rise/lower stats split by start
  kind; eval at DR 0 and the run's own DR; noise-response curve
  (action std 0→0.20) stored per champion.
- Keep-best champions, append-only, per skill. Never keep-last.
- Hardware gate: clean visual evals, ≥20-ep success, zero safety
  terminations, no hot leg (per-servo p95/p99, cross-leg imbalance —
  aggregate current is insufficient), per-joint deployment envelopes,
  low SafetyLayer reliance, then a frozen-policy eval of hundreds of
  episodes before any physical deployment. MuJoCo 3.11 shifted current
  readings (~+0.14 A at quiet hold) — torque→current recalibration
  required before trusting any current gate.

## 8. Questions we'd like a skeptical read on

1. **Flag-leg lever:** walk-only routing of a penalty is a patch. Is
   the deeper fix the learning-progress curriculum + gait metrics, a
   weak alternating-tripod contact-phase prior, or something else —
   and in what order? When do we stop paying for penalty-shaping
   iterations and accept the gait needs a structural prior?
2. **nv baseline:** is 8M steps a fair budget before calling the
   proprioception-only baseline, given the asymmetric AC is ready?
   Any risk we're under-training the baseline and will over-credit
   asym AC?
3. **Raise pathology:** stuck 2–5/6 across all lineages, but best in
   the run with velocity obs *removed*. Hypotheses? (Our current one:
   goal-mix under-training + possibly velocity obs enabling a lazy
   equilibrium; testing the mix first.)
4. **Eval noise:** with 2-ep periodic evals known to be binomial
   noise, and 20-ep harness evals only at run end, is there a better
   cheap mid-run signal for "rise is eroding" than what we have?
5. **Autonomy risks:** given the agent already produced one optimistic-
   verdict failure and one phantom launch, what additional guardrails
   or verifications would you add before trusting it with longer
   unattended stretches?
6. Anything in §3–§6 that looks like we're optimizing the wrong
   objective or missing an obvious standard technique?

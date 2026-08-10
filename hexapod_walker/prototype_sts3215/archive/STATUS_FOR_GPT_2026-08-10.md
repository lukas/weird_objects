# Hexapod RL — overall status for external review (2026-08-10, ~00:45 ET)

Context: 18-servo (STS3215) hexapod, Uno Q board, 25 Hz control.
PPO (SB3) on MuJoCo/MJX, 12 GPU pods on CoreWeave run an autonomous
experiment loop (LLM agent: triage → verdict → compose → launch, all
through a mechanical launch gate + append-only ledger). You previously
reviewed our first hardware walk failure
(`archive/GPT_HARDWARE_HANDOFF_2026-08-09.md`); this is everything
that happened since, including results that REVISE parts of that memo.

## 1. New hardware evidence (operator sessions, 08-09 night)

Three supervised sessions on the real robot. Traces in
`rl_move/hardware_traces/` (25 Hz RL episodes; ~2 Hz telemetry; ~3 Hz
tilt; 50 Hz single-joint ladder; air dynamics battery).

**a. The scripted tripod gait WALKS the robot.** Forward 30 and
50 mm/s, lateral crab, turns both directions, repeatedly, from a
fresh `set_zero → stand` sequence. Earlier same-evening attempts fell
almost immediately — root cause was NOT the gait: the gait syncs its
stance footprint to the PRESENT pose, and those walks started from
stale/slumped stances with drifted logical zero. Also: **definite
foot slip during the working gait** (operator-observed, "maybe
helping") — so the "high-grip floor does not skate" finding from the
walk post-mortem applies to body-roll coupling under paddle strokes,
not to micro-slip; the contact model must permit loaded slide.

**b. A WORKING gait rocks ±10–20° in roll and pitch** (measured over
all gait phases; stance-hold sits at ±1°). Our RL line trains AND
deploys with a 10° relative-tilt kill. We have likely been terminating
exactly the weight transfer a real gait requires — a candidate
explanation for why every RL walk champion converges to low-amplitude
creep. New walk arms use a 25° envelope.

**c. Current economics are INVERTED vs sim assumptions.** Standing
hold: 0.59 A total (18 servos, hot spots ~0.13 A). Walking: 0.33–0.45 A
mean, ~1.0 A peak. Walking is CHEAPER than standing. Sim current
pricing (and any effort-penalty reasoning built on it) needs
recalibration against these traces.

**d. Loaded actuator numbers** (single-joint step ladder at stance,
50 Hz): command→first-motion latency 110–210 ms (through the
HTTP/drive-loop path), 90% settle 260–330 ms for 2–5° steps, ~420 ms
for 10°. Air-battery `motor_model.json` refreshed the same night.

**e. POST-MORTEM CORRECTION (important).** Your memo's causal rank #1
("97% slew saturation — an action pipeline the policy never trained
in") is REFUTED AS STATED: the 1.5°/tick rate clamp has been in the
raw-joint training path all along (config `safety.max_delta_q_deg:
1.5`, SafetyLayer.filter inside the sim env step). Proposal saturation
exists in BOTH worlds; PPO plausibly uses the clamp as its trajectory
generator by design. The REAL train/deploy contract gaps we've
identified: (i) walk obs velocity — hardware feeds vx/vy_meas := ref,
training used privileged sim velocity (now fixed, see §3); (ii) tilt
envelope (10° vs the ±20° a real gait needs); (iii) prev-action
semantics (raw proposal vs post-safety applied — still unaudited);
(iv) contact/current pricing.

**f. First RL policy signal on hardware.** The stance champion's
stand-from-belly ran twice for 8.4 s each: quiet zero-command curl
(refuting zero-command marching for the stance line), stable height
ramp, then a deterministic +roll collapse at the belly-liftoff moment
(href ~33–42 mm), same direction, same tick, both runs — a systematic
transfer gap at the moment of load transfer, our best
deployment-equivalence reproduction case.

**g. Misc measured:** `+omega` command = CLOCKWISE body rotation
(sign convention pinned, both directions verified). Phantom over-temp
trips root-caused: temps refresh at 10 Hz but the debounce counted
25 Hz control ticks, so one corrupted bus byte held in cache passed
"3 consecutive ticks"; fixed to count fresh feedback reads — zero
false trips in ~15 min of motion since. STILL MISSING: measured walk
distance (tape measure) for true ground-speed/slip calibration.

## 2. Sim campaign status (autonomous agent, ~75 cycles)

- **Single-axis DR ladder: CLOSED, 12-for-12 PASS** (friction,
  latency+jitter, torque scale/droop, deadband, ground tilt 5°/8°,
  terrain, payload 1.0–1.4x, CoM offset, placement noise ±6°, IMU
  position, cmd jitter, crouch height −50 mm). Now composing pairs;
  ~10 pair-composes PASSed so far (slope×payload, friction×steering,
  deadband×driving, crouch×latency-jitter, etc.), each gated on
  joystick-gate 0-falls + DR0 retention + gait-validity + video.
- **Driving/joystick line is the workhorse**: 60 s endurance runs,
  90° heading changes, 45° steering on 3° slopes with payload — all
  0 falls at their DR levels.
- **Multi-skill mixing shows retention erosion**: a 4-leg-hold skill
  (quad-hold) trains cleanly in isolation but a 50/40/10
  quad/walk/hold mix pushed walk slip past gate; dose-response rung
  (30/60/10) is running.
- **posetrack (precision pose tracking) FAILED at +15M steps** —
  hold fixed, lean improved, track barely moved; needs denser
  reward/curriculum, not more steps.
- **Temporal actor (history_frames=16 from scratch) was blocked 8
  launches in a row** — turned out to be infra (see §4), requeued.
- Champions: walk = `lowgait_dr05_r1` lineage (crouch-capable,
  DR 0.5), stance = `stance_dr10` (the one that ran on hardware).

## 3. Deployment-contract arms (in flight tonight)

New training mode `goal.walk_obs_body_vel=2` feeds the policy
vx/vy_meas := ref — bit-identical to what the robot's runner feeds
(board has no velocity estimate). Two arms running:

- **cw-dep-vref1-r1** (15M/20M steps): current walk champion
  warm-started under the exact deployed contract (meas:=ref + 25°
  tilt). Hypothesis: no erosion ⇒ champion doesn't secretly depend on
  privileged velocity; erosion ⇒ escalate estimator/temporal-actor
  to P0.
- **cw-dep-fresh1** (4.6M/20M): from scratch under the same contract
  with field-standard exploration (log_std 0, ent 0.005), rocking
  permitted. Hypothesis: with real weight transfer NOT a termination
  event, a rocking gait (like the scripted one that actually walks)
  emerges instead of creep.

Operator directive (binding, in RL_PLAN): start-variation robustness
over sequencing guards — compose placement-noise ±6° +
bad-start-prob 0.4 onto the contract line next; build a NEW obs-side
"logical zero drift" DR axis (sensors consistently lying by a few
degrees — the failure that actually dropped the robot, distinct from
physical placement slop); hardware-candidate gates must include a
varied-start eval panel; walk episodes sometimes start from
park-bank/slumped poses.

## 4. Infrastructure (relevant because it eats cycles)

- 12 GPU (MJX/warp) pods, 4096 envs each, ~20M-step runs in a few
  hours. 12/12 slots busy at last cycle.
- The week's "launch collision storm" (dozens of 0-step deaths) was
  ROOT-CAUSED tonight by the agent: pods default to 64 MB /dev/shm; a
  4096-env layout maps ~58 MB and leaked segments from any crash
  SIGBUS every later launch. Fixes shipped: startup GC of orphaned
  shm segments, faulthandler+named exit codes, 4 GiB dshm in pod
  manifests. (This also explains the history-frames=16 line's 8
  consecutive deaths: >64 MB, could never boot.)
- Known process risk: operator and agent race on git (append-only
  log + JSON ledger merges mostly resolve cleanly; two stash-pop
  incidents required manual union merges).

## 5. Questions we'd like your judgment on

1. **Tilt envelope**: we widened walk termination 10°→25° based on
   the scripted gait's measured rock. Deployment trip must match
   (Gate 0). Any reason to prefer a rate-based (gyro) trip over a
   wider angle trip?
2. **Velocity obs**: meas:=ref is now contract-exact. Assuming
   cw-dep-vref1-r1 shows no erosion, do you still want the
   proprioceptive velocity estimator / temporal actor prioritized
   before the next hardware attempt, or is contract-exact enough for
   attempt #2?
3. **Contact calibration**: we have per-servo currents + tilt for the
   working scripted gait but NOT ground-truth distance yet. Is
   replaying the scripted gait kinematics in sim and matching
   currents+tilt (without speed) worth starting now, or wait for the
   distance measurement?
4. **Current pricing**: hardware says walking < standing in total
   current. Our sim effort penalties were tuned assuming the
   opposite. Recalibrate first, or drop effort penalties from walk
   arms until calibrated?
5. **The liftoff +roll collapse** (stance rise, deterministic at the
   same tick twice): best next probe? Our plan: reproduce in sim from
   the captured pose under the deployed contract; if sim doesn't
   reproduce it, suspect loaded-actuator dynamics missing from the
   servo model (rise was trained with air-fitted params).
6. **Multi-skill erosion**: mixing a new skill at 50% eroded walk
   retention past gate. Ladder the mix, or freeze walk weights
   (e.g. distill/regularize toward the walk champion) while training
   the new skill?
7. **posetrack**: 1/12 success after +15M steps with kernel reward on
   pose error. Denser curriculum suggestions welcome.
8. Anything in §1 that changes your P0 ordering from the 08-09 memo?
   (We kept: export/obs audit, actuator ID, contact calibration,
   Gate 0. We added: tilt envelope + current pricing from the new
   measurements. We demoted: "train through the slew limiter" — it
   was already true.)

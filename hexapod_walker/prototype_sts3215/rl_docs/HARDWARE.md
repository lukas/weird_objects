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

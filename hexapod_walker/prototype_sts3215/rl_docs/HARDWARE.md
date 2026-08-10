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

## Experiment backlog (operator-run; highest decision-value first)

Each entry: what open decision it settles → procedure → output.

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

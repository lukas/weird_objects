# Operator wishlist — things I want the robot to learn

Operator-owned (agents: propose additions, never delete items).
This is the EXPERIMENT BACKLOG: when free GPU pods outnumber sound
main-line arms, pull from here — top of each section first. Every
item still gets a pre-registered hypothesis + gate and honest video
verdicts. Exploratory lines never gate the walk champion's
promotion, and hardware safety rules always apply.

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
   vs memory; (4) GRU/recurrent actor [CODE — sb3-contrib
   RecurrentPPO or custom; needs an implementation cycle + probe].
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

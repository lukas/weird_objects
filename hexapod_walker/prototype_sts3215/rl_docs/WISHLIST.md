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

1. [RUNNING] **Longer distances** — 30 s+ horizons, sustained gait
   without degradation (`cw-walk-longdist`). Extend to 60 s if it
   holds.
2. [RUNNING] **Faster walking** — 0.08–0.12 m/s band
   (`cw-walk-fast`); does real stepping emerge when shuffling can't
   keep up?
3. [READY] **Turning** — yaw-rate commands: turn in place, walk an
   arc, figure-eight. Start with yaw-only, then yaw+forward.
4. [READY] **Back and forth** — walk forward N cm, reverse back to
   start. Includes backward walking (exploratory line; deferred
   from PROMOTION gates by the 08-09 ruling, not from training).
5. [READY] **Omnidirectional** — lateral strafing, diagonals.
6. [READY] **Driving / direction changes** — command changes
   MID-episode (the proxy for a human steering it): random heading
   switches every few seconds, smooth transitions, no stumble on
   the switch.
7. [READY] **Stop-and-go** — walk → halt on command → stand quietly
   → resume. (Parking on command is the flip side of the parking
   exploit; a policy that can do both on cue actually understands
   the command.)
8. [READY] **Smooth speed transitions** — accelerate/decelerate
   within an episode without gait breakdown.

## Robustness (survives the real world)

9. [RUNNING] **Higher DR** — champion trained at DR 0.5
   (`cw-walk-dr05`); ladder toward 1.0 training if it holds.
10. [CODE] **Push recovery** — random external shoves mid-walk
    (MJX perturbation forces); stays up, keeps walking.
11. [READY] **Payload** — walk with extra chassis mass (DR mass
    field pushed asymmetrically, or a fixed +20–50% payload).
12. [CODE] **Five-legged walking** — one leg limp/locked, gait
    adapts. Directly practical: the real robot has a cooked L5 knee
    (ID 19) right now. A policy that tolerates a dead leg is worth
    more than one that assumes six.
13. [READY] **Quiet gait** — minimize mean/peak servo current at
    fixed distance; hardware-friendliness as an explicit objective.

## Skills and party tricks

14. [RUNNING] **Stand → walk → sit chain** — one policy, all three
    on command (`cw-chain-standwalksit`).
15. [CODE] **Quadruped mode** — stand/walk on four rear legs,
    fronts free as claws (authorized parallel line; design sketch
    in `archive/RL_PLAN_FULL_2026-08-09.md`).
16. [LATER — after 0-c stability gates] **Fall recovery** — start
    fallen, get up quietly (needs fallen-pose reset generator +
    orientation-complete obs; hard current pricing per the
    2026-08-06 incident).
17. [CODE] **Claw gestures** — wave a front leg / "shake hands"
    while standing stable on five.
18. [READY] **Body pose control** — track body height/roll/pitch
    while standing (camera aiming, looking up/down); goal-mix
    lean/track modes exist.

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
23. [LATER — needs 2+ per-skill champions] **Distillation** — merge
    per-skill champions into one deployable policy.
24. [CODE] **Terrain** — ramps/uneven ground if the MJX scene can
    support it; flag scene work first.

## How to use this list (binding)

- A free GPU pod with no sound main-line arm takes the topmost
  [READY] item not already running; [CODE] items get a dedicated
  implementation cycle when 2+ pods would otherwise idle.
- One wishlist line per pod; don't start five half-lines at once.
  Prefer finishing/iterating a line over grazing.
- Wishlist runs use the same rigor: `cw-<line>-<idea>` names,
  launcher-only launches, pre-registered gates, video verdicts,
  ledger + summary.md. "It's exploratory" is not an excuse for an
  unwatched success.

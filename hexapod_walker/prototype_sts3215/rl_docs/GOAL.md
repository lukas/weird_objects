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

## What "good" means (operator's own words)

Distance, stability, reliability. The robot should cover real
ground, stay level, and never fall. A policy that walks 0.6 m every
time beats one that walks 1 m or falls at 50/50. Speed targets are
a means, not the objective. Foot slip is not failure by itself —
the scripted gait that walks the real robot slips visibly (08-09);
slip metrics exist to keep sim honest about the real floor, not as
a ban.

## Where we are (update this when it changes — keep it plain)

As of 2026-08-10 (morning — supersedes the 08-09 notes below where
they disagree): the REAL robot walks under a scripted gait; the
learned sim gait works but creeps (contact pricing calibration
pending, needs a tape-measure walk distance); the sim joystick
driving stack is hardened and seed-confirmed; turn-in-place and the
four-leg trick are under review; stand/sit inside the walking policy
is the main unsolved skill; hardware attempt #2 base
(cw-dep-vref1-r1, deployment-exact obs) is validated and staged.

As of 2026-08-09 (~cycle 34):

- The simulated robot has a REAL walking gait — all six legs
  stepping in rhythm. That took ~30 experiments to get.
- The gait's feet SLIDE along the ground while "stepping"
  (paddling/skating). On the real robot that would go nowhere and
  grind the servos, so this is THE blocker to hardware deployment.
  We measure it as "slip per meter traveled" (want ≤1.0; best 1.24).
- We proved (cycles 24–36) that no reward bonus/penalty tweak fixes
  the sliding — the simulator makes sliding physically FREE, so the
  optimizer always prefers it (cycle 36 closed the last income-side
  lever). The operator's 08-09 design rulings landed (new metrics +
  gates in flight); the servo-current hardware calibration — the
  deepest fix — is still operator-owned.
- Standing up and lying down (heights) are solved under full
  randomization; the "posture" detail (one leg finishing in the
  air) is stuck behind the same pricing issue.
- The operator keeps a standing backlog of things to learn
  (turning, back-and-forth, driving, quadruped mode, five-legged
  walking, ...): `WISHLIST.md`. Idle pods pull from it.

## The cast

- **The loop:** a watcher script polls training; each finished run
  triggers an agent "cycle" that evaluates it (with video, not just
  scalars), writes a verdict, and launches the next experiment.
- **Runs** are named `cw-<line>-<idea>` (e.g. `cw-walk-anchortol5`);
  continuations get `-c1`, `-c2`. Everything is recorded in the
  ledger (`rl_move/orchestrator/experiments.json`) and W&B
  (`l2k2/hexapod-balance`).
- **Compute:** CoreWeave pods; 2 GPU pods train 20–40M-step runs in
  ~20–40 min (MJX), CPU pods handle probes and small runs.

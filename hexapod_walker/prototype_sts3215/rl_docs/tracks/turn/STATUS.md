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

## Next

- `cw-walk-mirturn1` (queued): mirror-symmetry TRAINING on the
  bank-verified pricing — the shot at fast commanded turning. If it
  fails healthy, the mirror wrapper is the shipped turning story.
- Hardware wz sign audit gates any bench turn session.

Detail: rl_docs/TURN.md (design, bank numbers, failure history).

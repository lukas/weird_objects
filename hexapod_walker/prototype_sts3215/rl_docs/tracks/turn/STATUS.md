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
- Sign audit CLOSED (operator 08-11 night: turns match the drawn
  signs in both directions, no bridge flip; rate unmeasured). Bench
  turn sessions now wait only on the MirrorPolicy deploy port.

Detail: rl_docs/TURN.md (design, bank numbers, failure history).

# turn — Commanded turning

**SIM SPRINT (operator 08-17 ~18:05 UTC — binding while the robot is off the bench for repair): NO NEW LAUNCHES on this track unless an arm directly serves reliable rise+walk in the MuJoCo sim (the fleet's single deliverable; download answer: `rl_docs/DOWNLOAD_ANSWER.md`). In-flight runs finish and get triaged normally. Full text: RL_PLAN.md "SIM SPRINT".**


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
  (arc-left/arc-right/straight, zero training, ~2 deg/s). ~~Deploy
  port + rot60 composition are the remaining [CODE] items.~~
  **BOTH LANDED 08-13:** `run_policy_move(..., turn="left"/"right"/
  "hold")` in `linux_control/rl_policy.py`, over `POST /api/rl/walk
  {"turn": ...}`. left/right = constant chirality keyed off
  `NAKED_DRIFT_SIGN` (naked drifts left); hold = bang-bang on the
  gyro-z-integrated heading, the sim probe's 4° hysteresis. rot60
  composition: mirror wrapped OUTSIDE its own canonicalizer instance
  (reflect world → run the shipped stack → reflect action; per-
  chirality sector state, so alternation never chatters the
  hysteresis). `turn` unset = bit-identical naked path. Episode CSV
  gains a trailing `mirror` column; result reports switches +
  heading_end_deg. mirror.py added to both deploy scripts;
  `tests/test_mirror_runner.py` locks replay parity, selector
  semantics, and the numpy-only import chain.
- Fast commanded turning needs a NEW idea (step 4 BC-anchor-on-turn
  ticks is in reserve but unpromising after transbc1). No more
  reward/coef/symmetry variants.
- Sign audit CLOSED (operator 08-11 night: turns match the drawn
  signs in both directions, no bridge flip; rate unmeasured). Bench
  turn sessions now wait only on an operator session — re-deploy
  first (`deploy_adb.sh`/`deploy_ssh.sh` now ship mirror.py and the
  new runner).

Detail: rl_docs/TURN.md (design, bank numbers, failure history).

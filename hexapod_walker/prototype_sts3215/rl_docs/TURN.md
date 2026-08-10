# TURN — commanded yaw without the structural drift

Status date: 2026-08-10. Owner problem: RL_PLAN queue item 0 (unified
joystick policy — turning). Companion to rl_docs/RISE.md.

## The failure being solved

Walk-lineage policies carry a **command-invariant ~+0.09 rad/s left
drift**: they track yaw commands near the drift and fight commands
against it, in every scenario including turn-in-place. Three arms
failed on it and CLOSED price escalation as a move:

- `cw-walk-yawcmd1` — kernel alone: with σ=0.15 the ungated kernel
  pays a command-ignoring policy 0.67 of max income every tick; both
  seeds learned exactly the drift.
- `cw-walk-yawgate1` — achieved-rotation income gate: drift persisted.
- `cw-walk-yawgate2` — k_walk_yaw 1.0→2.5: drift persisted.

Root cause reading: the Gaussian kernel is the wrong SHAPE, not the
wrong price. Near wz_ref=0 its gradient at the drift point is tiny
(0.09 ≪ σ=0.15), and on turn segments it never goes **negative** for
wrong-direction rotation — rotating against the command and parking
cost the same. Meanwhile turn-in-place states are ~7.5% of training
segments (yaw is drawn independently; turning only coincides with a
stop via resample), so the skill being scored is barely trained.

## The mechanism set (landed 08-10, all cfg-gated, default 0 = legacy)

1. **Signed rotation income — `reward.k_yaw_prog`.** The k_walk_prog
   analog for turn segments: `k * clip(wz/wz_ref, −1.5, +1.25)`.
   Constant gradient toward the commanded direction; genuinely
   NEGATIVE when rotating against it. This is a new mechanism (sign-
   aware income), not a re-price of the closed kernel family.
2. **Drift charge on heading-hold — `reward.k_yaw_still`.** When
   wz_ref = 0: `−k * wz²`. At the measured drift (0.09 rad/s), k=50
   costs ~0.4/tick — real money against the ~2/tick kernel — while
   gyro-noise wz stays ~free by the square law. This is the term that
   directly disincentivizes DRIFTING while walking straight.
3. **Turn-in-place curriculum — `goal.walk_turn_in_place_frac`.**
   With probability f a walk episode becomes a dedicated turn: zero
   linear command, |wz| drawn in [0.5, 1.0]·walk_yaw_max_rad_s, sign
   50/50 by construction (the drift direction can never dominate
   exposure). Applied last in `_sample_walk`, overrides resample
   segments; rng stream untouched at f=0.

Held in reserve (not landed): mirror-symmetry augmentation /
symmetry loss (reflect obs+actions about the sagittal plane). The
heavy structural fix if the mechanism set above fails in DISCOVERY —
needs trainer surgery, do not start there.

## MDP_PREFLIGHT — the TURN bank (PASSING 08-10)

`test_task_semantics.py::test_turn_reward_separates_command_from_drift`
runs the full turn stack (champion walk cfg + walk_yaw_cmd, kernel
k=1.0 + achieved-rotation gate + k_yaw_prog=1.0 + k_yaw_still=50) on
scripted-gait policies at wz_ref = ±0.25, 3 seeds × both signs:

    turn (full command)   +1463
    partial (35%)         +1222
    drift (fixed +0.09)   +1154
    park                  +1122

Ordering `turn > partial > drift > park` holds — the stack now prices
the exact policy PPO found three times below honest partial turning.
`test_turn_command_signs_priced_symmetrically` additionally requires
CW and CCW turn income within 45% of each other — a live tripwire on
the yaw sign chain (below).

## Sign audit (still OPEN — do this before any hardware turn)

Sim `_body_wz()` is **+CCW** (right-hand z-up). Hardware measured
08-09: scripted gait `+omega = clockwise`. The TURN bank proves the
SIM chain (gait omega ↔ wz_ref ↔ reward) is internally consistent,
so the flip sits at the hardware boundary: the deploy bridge must
map joystick/policy yaw commands with the sign audit's result, or
the first hardware turn will fight its own command. One bench check:
command a small +wz through the bridge, read gyro sign.

## First arm result — FAILED (08-10, `cw-walk-turnfix1`)

Trained exactly the recommended cfg below off `cw-walk-yawgate2`.
Matched-parent control (`eval_yaw.py`, identical scripted panel,
turnfix1 vs frozen yawgate2): turn |wz_err| med 0.232 vs parent's
0.233; hold |wz| med 0.108 vs parent's 0.091 — statistically
IDENTICAL to the already-failed parent, same left/right asymmetry
(arc-left ~0.07–0.21 near the drift, arc-right ~0.22–0.37 fighting
it). The reward-side mechanism set (signed rotation income +
heading-hold drift charge + turn-in-place curriculum) passed its
pre-training bank but produced ZERO measurable behavior change in a
real policy. **Behavioral-impossibility kill — price tuning on this
task is now doubly closed** (first the kernel-price family, now the
signed-income/drift-charge/curriculum family). Straight walk stayed
clean (gv 6/6, 0 falls). Do not re-attempt with more steps or a
different k; the next move is the structural fix below.

## Next move: mirror-symmetry augmentation (was "held in reserve")

Reflect obs+actions about the sagittal plane (symmetry loss or data
augmentation) — needs trainer surgery, [CODE] not a launchable spec
yet. This is now the ONLY untried lever on the turning blocker;
every reward-shape lever (kernel price, achieved-rotation gate,
signed income, drift charge, turn-in-place exposure) has failed to
move a real trained policy off the fixed left-drift. Root-cause
reading: the drift is baked into the WALK GAIT itself (an asymmetric
limb-phase pattern learned once, early, off-center), not into the
turn reward's shape or price — no reward retuning can out-argue a
structural asymmetry in the policy's default gait.

## Recommended first arm (DISCOVERY, ≤2M steps) — SUPERSEDED, see above

Parent: walk champion (or hist16 twin). Cfg:
`goal.walk_yaw_cmd=1 goal.walk_turn_in_place_frac=0.30
reward.k_walk_yaw=1.0 reward.walk_yaw_kernel_gate=1.0
reward.k_yaw_prog=1.0 reward.k_yaw_still=50` +
champion walk stack. Evidence: the passing TURN bank. Judge with
`eval_yaw.py` (turn |wz_err| median vs the 0.10 gate; hold |wz|
median vs 0.05) AND a matched-parent control — the parent under the
identical eval, so the drift delta is attributable. Early video at
first eval; kill on the behavioral-impossibility rule if both turn
directions still converge to the drift.

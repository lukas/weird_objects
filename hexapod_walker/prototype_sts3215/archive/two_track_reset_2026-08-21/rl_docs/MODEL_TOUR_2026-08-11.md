# Model tour — every deployable checkpoint, human-style session (2026-08-11)

Scripted replay of a human `play.py` session, headless, per model:
belly → auto stand (rise 9.5 s + 1.5 s blend + plant re-anchor) → walk
fwd 12 s → strafe L 6 s → strafe R 6 s → stop/hold 4 s → sit → stand in
place → walk back 8 s → hold. Nominal env (`randomize=False`), same
mechanics as play.py (engage-walk ref snap, plant-frame sit ref −60 mm,
stop-and-wait on falls). 27 sessions: 18 stance × deployed walk
(`dep_vref1_r1`), 9 dep walk × deployed stance (`holdbc1_hard1`).
Driver: `/tmp/model_tour.py`; metrics + contact sheets in
`/tmp/model_tour/`. Velocities below are body-frame; commands ±0.05 m/s.

## Headline findings

1. **Deployed sit is broken (would tip the real robot).** From a clean,
   deterministic 142 mm plant stand — no walking, no disturbance —
   commanding sit (height ref −60 mm) makes `holdbc1_hard1` pitch over
   and trip `tilt_pitch` at ~2.5 s. Reproduced 10/10 tour sessions plus
   an isolated probe. `crouchrise3` fails identically; every other
   stance model sits fine (dz −54..−87 mm). Root cause is a start-state
   gap, not obviously reward: the holdbc line's lower training/BC anchor
   only ever lowers from its OWN ~72 mm crouch-stand frame, never from
   the 142 mm walk plant frame the interactive session (and the robot,
   after walking) actually lives in. **Do not command sit-after-walk on
   hardware until fixed.**
2. **Deployed stand-from-belly stalls in the nominal sim under the
   interactive goal ramp.** `hard1`'s rise plateaus at 54.7 mm and stays
   there forever (probed to 16 s) — below play.py's own 60 mm bar. Every
   other stance model reaches 69–83 mm under the identical protocol
   (`_InteractiveTraj` ramp to height ref +45 mm). `hard1` passes the
   training-profile rise, so this is overfit to the training goal ramp
   shape. It compounds the known hardware rise failure (riserock2-r1 is
   attacking the rocking, not this).
3. **All 9 dep walk models trade height for speed.** Engaged from the
   142 mm plant stance, body height collapses to 65–83 mm within a few
   seconds of driving and recovers to ~135 mm only when stopped. The
   paddle-slide is family-wide, not just the sim champion. Walk kernel
   peak (K_WALK 2.0) is ~2× the tilt/height kernel — velocity income
   dominates staying tall.
4. **Systematic CCW veer, unpriced.** Forward cruise for 12 s yaws
   +25..+52° (~3°/s) and drifts ~0.28 m laterally in EVERY session, all
   models, always the same sign. The deployed dep line predates the yaw
   lineage; `reward.k_walk_yaw` / yaw-still default 0. A human can't
   hold a heading with this interface.
5. **Per-axis tracking is badly asymmetric.** Mean body-frame velocity
   vs the 0.05 m/s command: fwd +0.046..+0.053 (excellent), right
   −0.025..−0.043 (50–86 %), left +0.012..+0.022 (24–44 %), reverse
   −0.006..−0.017 (12–34 %). Reverse is near-nonfunctional; kernels are
   symmetric, so suspect forward-biased command sampling.
6. **Tripod hover-park in hold is family-wide, not a holdbc quirk.**
   `friction`, `stand_dr08`, `stand_dr10`, `long5m`, `bc2m` all hold
   with a strict 3-feet-loaded / 3-feet-hovering split (duty
   [~1,0,~1,0,~1,0]); `dr10`/`raisefix`/`dr08` park one leg (leg 4 duty
   0.0–0.05). Several models also SIT on a tripod (dr10 sit duty
   [0,1,0,1,0,1] at the 71 mm crouch). These all predate
   `reward.hold_feet_load` — and the feet-load run re-parked anyway, so
   the hold kernel structure itself remains suspect.
7. **`hard1` is the only model holding with all six feet** (duty
   0.6–0.9), but the hold is restless — tilt spikes to 5.1° while
   "still", feet re-stepping after walking stops.

## Per-model notes (deviations from the above patterns)

- `holdbc1_hard1` (ON ROBOT): rise stall + sit fall (above). Hold all-six but noisy.
- `stance_dr10` (champion): clean session, no falls; leg-4 hover in hold; sits on tripod.
- `stand_crouchrise1`: rise 58 mm — marginally under the 60 mm bar; otherwise clean.
- `stand_crouchrise3`: sit fall like hard1; parks legs 1+3 in hold (duty 0.99/0.99, others low).
- `stance_raisefix`, `stance_dr08`: clean; leg-4 hover in hold.
- `friction`, `long5m`, `stand_dr08`, `stand_dr10`: clean; strict tripod park hold.
- `lower`, `stance_even`: clean; sit lands on tripod duty.
- `stance_clear`: rockiest rise of the family (6.2° tilt); reverse ~0 m.
- `stand_dr05`: 7.2° tilt spike during hold.
- `ppo_joint_goal`: dead — rise tops at 39 mm (the 512-step smoke overwrote it). Archive it.
- `ppo_joint_goal_bc`: rises PAST the stand to 158 mm — stilty over-tall pose (the incident geometry); sit only −21 mm. Flag or archive.
- `ppo_joint_goal_bc2m`: strict tripod hold; over-deep sit (−87 mm to 51 mm).
- `ppo_joint_goal_scratch2m`: clean-ish; uneven sit duty.
- dep walk models: differences are second-order vs the family traits.
  `tall15_h1` slowest fwd (0.046); `kh3` weakest left strafe (0.012);
  `quad1_c2` best yaw on reverse (−0.3°); `tip1`/`kh10`/`vref1_r1`
  best reverse (still only ~0.11–0.13 m of the commanded 0.4 m).

## Evals to add

1. **Plant-frame transition eval**: sit/stand cycles from the 142 mm
   walk plant frame, including sit after 12 s of driving. Gate: no
   falls, dz −55..−70 mm, all-six duty ≥ 0.8 at the crouch. (Catches
   headline #1 — current lower gates start from the model's own frame.)
2. **Interactive-protocol rise eval**: belly → `_InteractiveTraj` ramp
   to height ref +45 mm (play.py's exact path). Gate: z > 60 mm by
   9.5 s. (Catches #2 — goal-profile overfit.)
3. **Heading-hold eval**: 12 s fwd cruise; gate |yaw drift| < 10°,
   |lateral offset| < 0.1 m. (Catches #4.)
4. **Per-axis tracking grid**: ±vx, ±vy at 0.03/0.05; gate mean
   body-frame velocity ≥ 70 % of command on EVERY axis. (Catches #5.)
5. **Walking-height floor**: body z ≥ ~110 mm while cruising. (Catches #3.)
6. **Post-walk hold quiet**: six-feet duty + tilt ceiling measured in
   the 4 s after a stop command, not from a fresh reset. (Catches #7.)

## Where reward functions look wrong

- **Walk env height vs velocity**: height kernel exists in walk mode but
  K_WALK = 2.0 dwarfs it; measured outcome is a 70 mm crouch-shuffle.
  Candidate fix: gate walk income on height (same pattern as the
  loaded-slip income gate) rather than reweighting.
- **Yaw**: zero price on yaw rate in the deployed lineage; measured
  ~3°/s free spin. Turn on the yaw-still term (or the wz channel) in
  the next dep run.
- **Hold feet**: paying "held still + level" is satisfiable with 3 feet
  hovering — every pre-feet-load model does it, and the feet-load run
  re-parked, so the residual suspect is the hold kernel paying full
  income before load balance is checked.
- **Lower**: not a reward bug — a start-distribution gap (never lowers
  from the plant frame). Fix in the episode sampler / BC anchor, not
  the reward.
- **Reverse/left strafe**: kernels are symmetric; suspect the command
  sampler's distribution over (vx, vy). Audit it before touching reward.

## Implementation outcome (same night — corrections + what landed)

The code read before implementing corrected three suspicions above:

- **Height**: `reward.walk_height_gate` + `walk_height_sigma_mm`
  ALREADY EXIST (walk_task.py ~1050) and the whole pricing family is
  CLOSED — the T6 tall ladder tried ref ladder, income gate,
  gate+budget, k_height 3x/10x, speed relief; all flat at −72..−75 mm
  (hw STATUS). The tour's height collapse is the KNOWN crouch-splay
  tall wall measured from the interactive side; next levers stay
  BC-INIT on the scripted tall gait / physics easing. No new code.
- **Lower start states**: WRONG as written — training lower episodes
  DO start from the 142 mm plant (goal_task.py lower branch,
  `start_at="plant"`). The deployed sit fall is therefore not a
  missing start state: same targets, same start, different RAMP SHAPE
  (play.py's `_InteractiveTraj` 12 mm/s rate-limited ramp vs the
  trained 5 s linspace; play also sits to the full −60 mm vs the
  trained 25–55 mm band). Root cause: goal-PROFILE overfit — the same
  mechanism behind the interactive rise stall.
- **Sampler audit** (reverse/left weakness): confirmed structural —
  the legacy heading mix is 60 % pure forward / 20 % ±45° / 20 %
  anywhere (walk_task.`_sample_walk`), so reverse gets ~10 % of
  episodes and pure strafes a sliver. Addressable with the existing
  `goal.walk_heading_max_rad` / resample knobs — a curriculum
  decision, not new code.

Landed (commit-gated, default off, banks green):

- `rl_move/sim/eval_session.py` — the SESSION GATE: the interactive
  play.py protocol as an exit-code-enforced eval (hard gates: falls /
  rise-by-9.5 s / sit descends; soft gates: heading drift, per-axis
  tracking ≥ 70 %, drive height ≥ 110 mm, quiet post-walk hold — all
  six "evals to add" above in one tool, ~4 s CPU). Verified: deployed
  pair FAILS (sit tip + rise stall reproduced), stance_dr10 pair
  PASSES hard gates.
- `goal.rise_ramp_jitter` / `goal.lower_ramp_jitter` (goal_task.py) —
  per-episode ramp-duration jitter U(1±j), default 0.0 with legacy rng
  streams bit-exact; the training-side lever against profile overfit.
  Bank: `test_ramp_jitter_*` in test_task_semantics.py (4 tests).
- Docs: CURRENT_TRUTHS deployment fact, RL_PLAN blocker + first-arm
  spec, WISHLIST items 8d/8e, hw-track STATUS, EVALS.md §3 row.

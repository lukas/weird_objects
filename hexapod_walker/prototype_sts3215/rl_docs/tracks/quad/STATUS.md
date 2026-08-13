# quad — Quadruped with two hands

W&B: tag `track:quad`. Excess-capacity research.

**Goal:** walk on four legs with the front pair lifted as hands/arms.
Stand on four, walk on four, front pair free for tricks.

## Now

- **08-13: `cw-quad-turn1-r1` (quad-hold × commanded-turn compose,
  10M, finished 08-10, dig-in dropped in the shuffle — closed today)
  FAILED its compound gate; the quad-turn rung is CLOSED behind the
  turn track's wall.** Quad clause itself PASSED (survived_frac 1.0,
  12/12 harness quad efter the mode-fix below, level, fronts lifted,
  h_err ~3mm) but the stance CREEPS ~0.33 m/15 s (~1.1 m foot drag —
  stillness never trained, same creep as its hold mode). Yaw failed
  exactly like every pre-fix-yaw-stack sibling (|wz_err| med 0.227
  vs 0.10, right turns untracked); walk slip 1.45–1.58 vs the 1.25
  cap = 4th confirmation quad-mix erodes walk economy. No retry from
  this track: commanded yaw first needs a new idea (turn track).
- EVAL-TOOLING FIX (08-13): `eval_checkpoint --modes quad` used to
  SILENTLY run walk episodes labeled "quad" (ALL_MODES lacked quad →
  all probs zeroed → walk fallback); every pre-08-13 harness/periodic
  `quad_*` eval row is really walk. Fixed + loud per-episode mode
  assert; harness quad evals are genuine from now on.
- Four-leg HOLD is solid (quad-hold1-r2: survived 1.0, level, fronts
  lifted; hold2 at 30% mix confirms the mechanism) but ANY mixing
  dose erodes walk retention — quad stays a deploy-time specialist.
- Deploy integration of a PASSING quad checkpoint belongs to hw
  (joystick key `4`).

## Next

- The unattempted core: four-leg WALKING (weight shifted back onto
  the rear four). Spec + bank FIRST — [CODE], buildable by a
  dedicated orchestrator cycle, no operator input needed. Audited
  specifics (08-13): `reward.k_park_duty`'s duty window spans ALL
  six legs, so permanently-lifted fronts pay ~0.2k every tick;
  eval-side `sacrificed_legs`/gait_valid also counts the fronts →
  any honest quad-walk is reward-punished AND eval-INVALID today.
  Build = a quadwalk mode (walk trajectory + lift_legs) with
  mode-aware exemptions in both reward and eval, plus a QUADWALK
  semantics bank proving rear-four stepping out-earns six-leg walk,
  fronts-down drag, and freeze. Also fold in: stillness for the
  quad HOLD stance (it creeps; hold_still_gate is scoped hold/track
  only and quad is exempt by design — needs its own term or scope).
- Then front-pair posture control while moving.

Detail: ledger cw-quad-* lineage.

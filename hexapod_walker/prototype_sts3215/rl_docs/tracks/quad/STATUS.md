# quad — Quadruped with two hands

W&B: tag `track:quad`. Excess-capacity research.

**Goal:** walk on four legs with the front pair lifted as hands/arms.
Stand on four, walk on four, front pair free for tricks.

## Now

- **08-13 (later, spec cycle): the four-leg-WALK spec + bank [CODE]
  is BUILT and checked in — but the bank is BLOCKED on a physical
  finding: NO open-loop scripted quad gait actually walks in sim.**
  Landed (all default-off, legacy bit-exact, walk/turn banks green):
  `quadwalk` goal mode (`--goal-mix quadwalk=<p>`; walk command
  interface + quad one-hot family, sampler keys in REWARD.md), the
  two audited reward exemptions (k_park_duty spans support legs
  only; lift legs never earn step/swing credit — drag charges kept),
  quad clear/plant income riding on the walk stack, `k_quad_still`
  (prices the hold-stance creep, only when no velocity commanded;
  bank-proven: charges a translating body ~180/ep, still stance ~0),
  and harness/trainer eval support (quadwalk in ALL_MODES,
  lift-aware sacrificed_legs + fronts_lifted gate, per-mode vel-err
  keys, reel modes). THE BLOCKER: the bank's honest reference —
  tried rear-four trot, 4-beat crawl (duty 0.75), and a two-phase
  distance-clock crawl with feasibility-GO statics (mid splay,
  body-back, leading sway, lift-first swings): trot is stable but
  translates 0.00 m; crawl pins a mid leg (CoM outside the mid-swing
  triangle); two-phase steps but drifts BACKWARD 0.02–0.10 m with
  rear-leg chatter. Static feasibility (c57 GO) does NOT extend to
  open-loop stepping. All schemes reproducible via
  `rl_move/sim/probe_quad_crawl.py` (supports --video). The ordering
  tests SKIP with a loud reason (`QUADWALK_REFERENCE_BLOCKED`), so
  every quadwalk PPO arm stays MDP_PREFLIGHT-blocked — by design.
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

- **08-13 (diag session): route (1) — scripted-reference iteration —
  is CLOSED with a measured geometric proof.** The train-0
  instrumented session (`probe_quad_crawl.py --diag`, 14 configs
  spanning every physical lever: stance translation/rotation, pitch
  to the tilt limit, front-tuck yaw, adaptive 2-D weight shift,
  slow periods) shows a statically-stable open-loop quad crawl with
  both fronts lifted is INFEASIBLE on this robot: the mid-swing
  support triangle needs the CoM ~5-7 cm further back (or 9-13 cm
  lateral) than the ±35° hip-yaw workspace can EVER put it — the
  mid-swing margin measured −33..−70 mm in every config while rear
  swings are +35..+55 mm; commanded body x-shifts don't physically
  realize (yaw-saturated), lateral realizes ~30%; the swinging mid
  is pinned 0.65-1.0 of its window and the tip+recovery rectifies
  the gait backward. Full numbers + geometry argument in the probe
  docstring. **Consequence: only DYNAMIC (closed-loop) balance can
  walk this robot on four legs — the honest scripted reference the
  bank was waiting for cannot exist. Route (2), the operator ruling
  (accept the first RL/feedback policy showing genuine rear-four
  stepping as the bank trajectory — an MDP_PREFLIGHT
  chicken-and-egg only the operator can approve), is now the ONLY
  route. Until it lands, NO quadwalk training arm is launchable.**
- A legal interim arm once the operator weighs stance priorities: a
  quad-HOLD continuation pricing the stance creep with the new
  `k_quad_still` (bank-proven, cheats priced) — fixes the measured
  0.33 m/15 s drift on the existing quad-hold skill. Not queued this
  cycle: one-variable discipline says it rides with the next
  quad-hold consolidation, not as a lone 10M retrain of a solved
  trick (excess-capacity track; hw keeps pod priority).
- Then front-pair posture control while moving.

Detail: ledger cw-quad-* lineage.

# quad — Quadruped with two hands

W&B: tag `track:quad`. Excess-capacity research.

**Goal:** walk on four legs with the front pair lifted as hands/arms.
Stand on four, walk on four, front pair free for tricks.

## Now

- **08-13 (~18:xx UTC): the structural CODE fix the quadwalk5 ruling
  demanded is BUILT, bank-proven and TRAINING — `cw-quadwalk6`
  (quadwalk5 + `reward.walk_gait_gate=1.0`, ONE lever, 2M discovery)
  VERIFIED RUNNING train-0 (~15.3k fps, commit 1a21060, tag
  exp/cw-quadwalk6).** Root-cause pass (behavior <- incentive <-
  pricing <- sim defect): the scoot's income survives every existing
  lever because (a) the anchor/loadslip gates' fractions span LOADED
  feet only — an AIR-parked leg silently drops out of the
  denominator; (b) the sitting income (quad clear ~1.5 + plant
  ~0.75/tick) plus prog-gated kernel flows regardless of WHICH legs
  move the body; (c) all counter-pricing (k_park_duty,
  k_drag_loaded, k_quad_lift_contact) is additive, and quadwalk3/5
  measured that fines up to ~40-55% of return are simply paid (sim
  physics already audited clean). The fix is the campaign's proven
  "worth less by construction" pattern, extended to leg-sacrifice:
  velocity income (kernel + positive progress + quadwalk clear/plant
  on commanded ticks) is multiplied by the **MIN over commanded
  SUPPORT legs** of a per-leg "completed a real swing recently"
  score (2 s window of commanded ticks + 2 s linear fade — the
  holdstill1 no-hard-zeros lesson; swing = >=2 ticks airborne + XY
  stride >= 10 mm; lift legs exempt; penalties never shrink;
  episode start counts as just-stepped). MIN, not mean: fractional
  discounts are measured-payable — sacrificing ANY subset of support
  legs collapses transport income to the (1-g) floor. Default 0 =
  bit-exact off (verified); semantics bank 91 pass with 4 new
  `test_walk_gait_gate_*` cases: the scripted MID-PIN scoot
  (quadwalk4/5's cheat family) loses 72% of income / 54% of return,
  a one-leg walk flag collapses to walk_gait_min=0, and the honest
  six-leg walk gait keeps walk_gait_min=1.0 and ~99% of income.
  Instructive probe finding along the way: the scripted splayed
  rear-four trot scores 0 on the gate BECAUSE its mid legs are
  genuinely pinned (0 completed swings — the probe_quad_crawl
  geometry, seen per-leg), so the gate agrees with the geometric
  infeasibility proof and only a feedback policy can pass it —
  exactly the operator's route (2). Pre-registered outcomes ride in
  the ledger entry (freeze = known one-line STOP -> exploration/
  curriculum next, NOT pricing; token-hop -> gate parameter fix;
  falls -> balance curriculum).
- **08-13 (~17:xx UTC): `cw-quadwalk5` (quadwalk4 + `reward.k_park_duty`
  1.0->6.0, ONE lever) STOP — same known exploit, price hike did
  NOTHING. Det+sto both sacrifice the identical mid legs [1,4] in
  12/12 episodes (gait_valid 0/6, slip 13.25 med det / 11.31 sto,
  fwd med 0.17m det vs 0.05-0.06 commanded), and fronts_lifted
  actually regressed to 3/6 det (was 11/12 in quadwalk4) — a 6x
  charge that should dominate the ~-0.12/tick park cost produced
  ZERO behavior change. Quad-hold retention held (6/6 survived,
  creep 0.49m det / 0.40m sto — in the 0.43-0.50m lineage band;
  roll_tail 2.5/0.7deg, within the 3deg cap). **RULING: pricing-only
  is now measured-exhausted for BOTH quadwalk cheat families found
  so far** — fronts-down (quadwalk1-3, repriced 3x, still failed)
  and mid-leg-park (quadwalk4 discovery -> quadwalk5 repriced 6x,
  still failed identically). A coefficient that should have been
  overwhelming changed nothing, so the next lever is a structural
  CODE fix that closes the "sacrifice any subset of support legs"
  loophole generically (mirrors the stance line's per-foot-price ->
  bc_anchor_foot_z lesson: duty/contact pricing alone gets evaded by
  parking, whichever legs are cheapest to park). No further
  reprice-only quadwalk arm until that lever exists and its own
  semantics-bank case is written; this needs a root-cause pass
  (behavior <- incentive <- pricing <- sim defect), not another
  coefficient scan, and is not a same-cycle triage fix — flagged for
  a deeper pass. Fleet stays idle on quadwalk in the meantime
  (excess-capacity track; hw keeps pod priority; an idle pod here is
  fine, a blind 6th reprice would not be).
- **08-13 (~16:xx UTC): `cw-quadwalk4` (quadwalk3 + `goal.quadwalk_start
  ="quad"` spawn) FAIL — the spawn fix WORKS (fronts_lifted 11/12
  det+sto, tail lift duty <0.15, first quadwalk arm EVER to clear
  the fronts-down cheat) but the policy found a NEW flag-leg cheat:
  it parks the two MID support legs instead (duty 0.0-0.06 vs rears
  0.60/0.45 med) and scoots forward on the rear pair alone —
  gait_valid 0/6, rear-two speed 0.017 m/s vs 0.05 commanded, slip
  ~13 m/m, height sagged ~35mm, roll_tail 0.7deg, 0 falls, fwd med
  det 0.14m. Mechanism (measured, not guessed): `k_drag_loaded=10`
  prices touching a mid foot down and scuffing it far above what
  `k_park_duty=1.0` charges for leaving it parked in the air
  (~100x underpriced) — an active choice to keep the cheaper leg up,
  not a basin-crossing accident. Quad-hold retention: lineage
  baseline (fwd creep 0.46m ≈ quadwalk1-3's 0.43-0.50m, k_quad_still
  terms still 0 — the creep-fix miss from quadwalk3 is UNCHANGED,
  not re-litigated here). One-lever follow-up queued:
  **`cw-quadwalk5`** (quadwalk4 + `reward.k_park_duty` 1.0→6.0, ONE
  lever, 2M discovery) — backlog, drain in progress.
- **08-13 (~15:xx UTC): `cw-quadwalk3` (per-tick lift-leg ground-
  contact charge, k=3.0) FAIL — same pre-registered fronts-down cheat,
  THIRD time, and this one is decisive: the charge verifiably FIRED
  (`env/reward_quad_lift_contact` −1.54/tick ≈ −575/ep, matching the
  bank's −547) and the policy simply PAID it — ~40% of episode return
  — rather than leave the warm-started six-leg basin (tail lift duty
  0.58/0.36 med vs quadwalk2's 0.62/0.32; fronts_lifted 0/12; fwd med
  0.60 m, 0 falls, roll clean). PRICING (income and charge) is now a
  measured-exhausted lever CLASS for quadwalk; per the arm's own
  prediction-if-false the blocker is EXPLORATION from the six-foot
  plant spawn (quad-hold proves lifting IS learnable from plant under
  a hold command — under a walk command the six-leg basin dominates).
  Secondary: quad-hold retention survived 12/12 but the stance still
  creeps 0.41–0.43 m/15 s (k_quad_still=1.0 didn't fix creep).
  CODE lever landed per that branch: `goal.quadwalk_start="quad"` —
  episodes spawn ALREADY in the four-leg stance (support legs at the
  splayed freeze stance, `quadwalk_mid_splay_m` 0.06; lift legs at
  the feasibility tuck claw; tilt refs anchored LEVEL because the
  limp settle sags the stance ~16° nose-down onto the claws — runs
  must widen safety.max_roll/pitch_deg to 25). Default "plant"
  bit-exact; 3 new semantics tests, full bank 87 pass. Six-leg
  walking now requires actively PLANTING the charged fronts from a
  fronts-up start — **`cw-quadwalk4`** (quadwalk3 + this spawn, ONE
  lever package, 2M discovery) is the refill.
- **08-13 (~14:xx UTC): `cw-quadwalk2` (3× lift income) FAIL — the
  same pre-registered cheat (b), milder: fronts_lifted 0/12 det+sto,
  front tail contact duty 1.0 → 0.62/0.32 (still ≫ the 0.15 bar),
  fwd med 0.61 m, 0 falls, roll clean, quad-hold retention ok.
  Pricing moved behavior but cannot clear the bar → per the
  pre-registered two-miss fork, PURE REPRICE IS CLOSED as a lever.
  The prescribed CODE lever is BUILT and checked in (tag
  exp/cw-quadwalk3): `reward.k_quad_lift_contact` — per-tick charge
  on the fraction of commanded lift legs in ground contact after
  grace; default 0 = bit-exact off; semantics bank green (sixleg
  charged −547/ep, frontdrag −882/ep, honest tucked gait exactly 0;
  full test_task_semantics 84 pass). **`cw-quadwalk3`** (quadwalk2 +
  k_quad_lift_contact=3.0, ONE lever, 2M discovery) VERIFIED RUNNING
  train-0. Pre-registered STOPs unchanged (freeze/park now the
  likelier escape — it's already a one-line STOP); if quadwalk3
  freezes, the next lever is exploration/entropy, not pricing.
- **08-13 (~13:2x UTC): `cw-quadwalk1` (first quadwalk discovery arm)
  FINISHED and hit exactly the pre-registered cheat (b) — fronts-down
  six-leg gait. Det+sto harness: quadwalk gait_valid 0/6,
  fronts_lifted 0/6 both passes; frame strip/video shows all six
  legs planted and cycling like an ordinary walk, no lift attempt at
  all; net forward is positive (med 0.64m, no backward rectification)
  and 0 falls, but the fronts-down clause alone is an automatic
  STOP — no forensics needed, no continuation of this exact spec.
  Root read: ordinary six-leg walking already pays well from the
  warm-started lineage and the lift-leg income (k_quad_clear=1.5,
  k_quad_plant=1.0) is too small a side bonus to make the policy
  even try lifting under a 2M-step, low-entropy discovery budget.
  Quad (hold) retention side of the same run stayed clean (survived
  6/6, roll settled 6/6). Follow-up queued per the hypothesis's own
  pre-registered fix for class (b) — **`cw-quadwalk2`** (respec,
  same base/mix/budget, ONE lever: triple the lift-leg income,
  k_quad_clear 1.5→4.5 / k_quad_plant 1.0→3.0) — queued to the
  backlog, drain in progress. Per RESEARCH_RULES "two misses in a
  row": if quadwalk2 also lands at fronts_lifted 0/6, pure reprice
  is CLOSED as a lever and the next move is CODE — an explicit
  penalty for front-leg ground contact during quadwalk (or an
  entropy/exploration lever), not another coefficient scan.**
- **08-13 (~13:xx UTC, post-ruling cycle): the pre-registered
  reference-acceptance gate is WRITTEN (`rl_docs/tracks/quad/
  QUADWALK_REF_GATE.md` — the ruling's required first artifact:
  multi-seed det bars, dr-scale 0.5 + friction panel, zero falls,
  exemption verification, stillness bars, backward-rectification
  auto-FAIL), and the FIRST quadwalk training arm is launched:
  `cw-quadwalk1` (2M discovery, warm from `cw-quad-hold2`, mix
  quadwalk=0.7/quad=0.3, + k_quad_still=1.0). Binary question: does
  genuine rear-four stepping (all four support legs cycling,
  positive forward translation, fronts lifted) emerge at all?
  Known cheats pre-registered as one-line STOPs: freeze-in-stance,
  fronts-down six-leg gait, backward-rectified shuffle. Passing
  discovery does NOT make it the bank reference — that needs the
  full REF_GATE. Ordering tests still SKIP (message updated to
  cite the ruling + gate doc; REWARD.md row updated).**
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
  **08-13 ~12:4x UTC OPERATOR RULING: route (2) APPROVED — a
  feedback/RL policy showing genuine rear-four stepping is
  PERMITTED as the quadwalk bank reference, WITH AN EXPLICIT
  ROBUSTNESS GATE.** MDP_PREFLIGHT unblocks for quadwalk under
  these binding conditions: (a) "genuine rear-four stepping" must
  be evidenced the way every gait claim is — video/contact-sheet
  showing all four support legs stepping (no pinned/dragged mid
  leg, no backward rectification), positive translation, fronts
  lifted; (b) before ANY candidate is accepted as the bank
  reference it must pass a pre-registered ROBUSTNESS GATE, written
  down BEFORE the first arm launches — at minimum: multi-seed det
  (≥2 seeds × ≥6 eps), a DR panel at the walk bank's standard axes
  (dr-scale 0.5 + friction), zero falls, fronts_lifted + lift-leg
  no-credit exemptions verified in eval, and stance stillness
  (k_quad_still's measured creep ≤ a stated bar) — the gate spec
  is the FIRST artifact of the first arm, subject to the normal
  one-variable/matched-parent rules; (c) until a candidate passes
  that gate, the ordering tests keep SKIPPING and no scripted-bank
  claim may cite the RL reference. Reference-quality bar mirrors
  the walk bank's: the reference is data, so a sloppy reference
  poisons every downstream BC/anchor use — gate hard.
- A legal interim arm once the operator weighs stance priorities: a
  quad-HOLD continuation pricing the stance creep with the new
  `k_quad_still` (bank-proven, cheats priced) — fixes the measured
  0.33 m/15 s drift on the existing quad-hold skill. Not queued this
  cycle: one-variable discipline says it rides with the next
  quad-hold consolidation, not as a lone 10M retrain of a solved
  trick (excess-capacity track; hw keeps pod priority).
- Then front-pair posture control while moving.

Detail: ledger cw-quad-* lineage.

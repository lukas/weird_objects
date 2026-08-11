# RISE — standing up inside the walking policy (full plan + history)

Moved out of RL_GOALS.md / RL_PLAN.md 08-10 (operator doc-shrink
ruling). RL_PLAN.md "Open problems" item 2 is the binding summary;
this file is the detail. Status date: 2026-08-10.

## Where it stands

Unified rise is UNSOLVED. Every arm to date satisfied the height
criterion via a cheat (flag-leg / tripod / stilt) or froze. Working
fallback that does not block the joystick MVP: the stance champion
(`ppo_goal_cw_stance_dr10`) performs a genuine feet-down belly-rise,
a scripted 1.5 s blend reaches the walkable plant stance, and the walk
champion drives — sim-proven end to end (sim viewer key `7`).

## Root causes found 08-10 (each once cost a multi-M-step run)

1. Reward paid FREEZING more than trying (arrival-gate sign bug).
   Fixed 69e00c0: `reward.rise_finish_gate_signed=1` +
   `rise_income_prog_gate=1` — ALL rise/lower arms set both.
2. The walk-lineage warm start is measurably blind to the height
   command channel.
3. Training reward paid TORSO HEIGHT only → fresh policy hit height
   with feet 30 cm in the air ("6/6" on training's criterion, 0/6
   posture-strict). Longer training optimizes the cheat harder.
4. Posture gate priced lower's feet at the 20 mm stand allowance while
   an honest belly-down lower leaves pads 20–45 mm up → honest≈cheat
   income, outrigger cheat won (postgate1, ERODED lower). Fixed
   08-10: pf uses the 60 mm lower allowance.
   Pre-273ebde rise/lower checkpoints stay invalid near the ground.

## The literature route (HumanUP, HoST RSS 2025; HiFAR 2025)

Never learn a deployable get-up from a bare task reward in one shot.
Discover the motion once (loose limits, sparse reward), then train the
deployable policy to TRACK the discovered trajectory under strong
smoothness/torque regularization and randomization, with posture-aware
staged rewards and curricula. Our discovery stage is DONE — the stance
champion's belly-rise (re-verified 08-10: det flat start, ends 5 mm
off target, worst pad clearance 4 mm).

## Landed machinery (all cfg-gated, default-off)

- **Posture gate** (`reward.rise_posture_gate=1`): rise/lower income
  scales with the fraction of feet within the mode's clearance
  allowance (20 mm stand / 60 mm lower) — geometric, matching the
  harness's end_posture_ok, not touch force.
- **Reference tracking** (`reward.k_rise_ref_track` +
  `reward.rise_ref_path`): dense joint-space kernel against the
  champion's recorded rise, time-aligned at the height-ramp start.
  Champion earns ~full pay on its own reference; a frozen robot ~13%.
  A scaffold — anneal to 0 across arms.
- **Walkable-height reference** (operator ruling 08-10 ~11:00 ET: the
  ~70 mm crouch-stand is "a terrible stand"; rise must end in the
  ~142 mm plant stance): `extract_rise_ref --blend-to-plant` →
  `rl_move/sim/refs/rise_ref_belly2plant.npz` (+111 mm, all pads
  down, 7.4° RMS from plant). Commanding it needs
  `goal.rise_height_mm=[108,114]` AND `actions.max_height_mm=115`.
- **Stand-score income routing** (`reward.rise_score_income=1`,
  landed 08-10 evening after plantgate1 FAILED — gates leak, so the
  income itself moved): rise-episode height income (progress,
  milestones, finish, kernel) is ZEROED; the only rise income is a
  progress ratchet (`k_rise_score_prog`, pays once per new best) +
  post-ramp hold pay (`k_rise_score_hold` × S²/tick) on stand-score
  S = height-kernel × feet-down² × HARD no-flag × plant geometry,
  plus a ramp-weighted airborne-feet rent (`k_rise_posture_pen`) —
  the bank showed cheats otherwise win on the PENALTY side by dodging
  `reward_height` (torso up any way possible). The rent charges
  feet-in-the-air only, NOT the geometric fades: the honest reference
  itself moves through wide-footprint poses mid-rise. Lower keeps the
  legacy (solved) stack. Bank, all seeds: replay +91 ≫ stilt −9 >
  flag-leg −165 (income ~0) > freeze −218. First arm:
  `cw-stand-score1`.

## The standing SPECIFICATION (landed 2026-08-10, PLANT_SPEC)

"Standing" is now a geometric predicate, not a torso height —
`PLANT_SPEC` / `valid_plant()` in `rl_move/sim/sim_env.py`, ONE
criterion shared by the training reward, the eval harness, and the
semantics bank (never let them drift). A stand is valid iff at
episode end:

| check | threshold | kills which cheat |
|---|---|---|
| height | \|err\| ≤ 15 mm of target | parking short of the plant |
| attitude | \|roll\|, \|pitch\| ≤ 10° | leaning-tower stands |
| feet down | ≥ 5/6 pads ≤ 20 mm | tripod-at-height (b2p1) |
| no flags | no pad > 60 mm | flag-leg (b2p1, fresh1) |
| support | CoM ≥ 20 mm inside down-feet polygon | edge-balanced poses |
| footprint | body-frame feet ≤ 40 mm from plant anchors | stilt/splay (~50 mm out) |
| effort | max servo ≤ 2.0 A | fighting poses (real once current model lands) |

Consumers:

- **Eval harness**: `valid_plant` + `plant_fail` + `plant_margin_mm`
  reported per stand-ending episode (always); `--valid-plant-gate`
  wires it into rise/raise success — OFF until champions are
  baselined (same rollout as end_posture_gate on 08-08).
- **Training reward**: `reward.rise_plant_polygon_gate` (0..1,
  default 0) scales the same income terms as the posture gate by a
  continuous factor (CoM margin × attitude × footprint). Bank smoke
  08-10: replay +946 (full pay kept), stilt +217, freeze −188.
- **Bank**: `test_rise_valid_plant_separates_stand_from_cheats` —
  replay ends valid on every seed; stilt / freeze / partial all fail.

## The binding preflight

`rl_move/tests/test_task_semantics.py` (RISE bank): replaying the
demonstrated belly→plant path must dominate the stilt exploit and the
freeze under the FULL reward stack (was +952 / +225 / −195), and
freeze must be net negative. The b2p1 lesson: individually-validated
terms still lose to the height cheat when composed — the bank always
runs the full stack the arm will train with.

## Evidence trail (run docs have the full facts)

- `cw-uni-rfix-warm1` — lower 6/6 posture-strict after the pricing
  fix; KEEP fine-tune grafting (distill refuted by fresh1).
- `cw-uni-rfix-fresh1` — strictly worse (tripod + over-current).
- `cw-stance-postgate1` — FAILED and ERODED lower (allowance bug,
  since fixed).
- `cw-stand-b2p1` — FAILED (08-10 ~16:5x): height nailed (<6 mm err
  both modes) but rise 0/12 AND lower 0/12 posture-strict; flag-leg/
  tripod cheat video-confirmed (rise: 2 legs 80–131 mm up; lower: 1
  leg up to ~288 mm). Same pathology class as rfix-fresh1 and
  cw-stance-riseproof1 (open DIG-IN).
- `cw-stance-riseproof1` — control probe: stance-line joint_goal
  recipe from scratch on today's sim; decides whether the walk-env
  task construction or near-ground sim contact is implicated.
- `cw-stand-score1` — FAILED (08-10 night): score-income routing
  (income moved to a stand-score S = height-kernel x feet-down^2 x
  no-flag x plant geometry, + airborne-feet rent) warm-started from
  the HONEST stance champion (not a cheating checkpoint) — still
  0/12 valid_plant / end_posture_ok at both DR0 and own-DR0.2, EVERY
  start_kind (flat/bridge/crouch), every episode flagged
  `feet_down`+`no_flag` with one leg 40–150 mm off the ground.
  `env/rise_score` stayed flat ~0.01–0.02 for the full 2M steps (the
  pre-registered early-call trigger). Same pathology class as b2p1
  and plantgate1 — third distinct mechanism, same cheat. hold/track
  unaffected.
- `cw-stand-scoreref1` — FAILED (08-11): score1's stack unchanged +
  a cheat-proofed belly→plant reference-tracking crutch
  (`k_rise_ref_track=2.0`, feet-gated kernel, sigma 12→6°) warm-
  started from `cw-stand-score1`'s lineage on the honest stance
  champion. Still 0/6 valid_plant/end_posture_ok det AND sto at DR0
  gate and own-DR0.2, worst-foot clearance 160–188mm (leg held in
  the air the whole episode, video-confirmed), `env/rise_score` flat
  ~0.01–0.02 the entire 2M steps. Fourth distinct mechanism, same
  cheat — this was the pre-registered test of lever (a) below and it
  is now closed too.
- `cw-stand-scoreref1-dr0` — FAILED (08-11), control arm: identical
  to scoreref1 but DR-scale 0 (pre-registered to test whether DR
  noise was washing out the two thin-margin mechanisms). Same
  flatline at DR0: `env/rise_score` 0.01–0.03 the whole 2M steps,
  training's own diagnostic 0/2 rise every window. **DR is
  exonerated** — not the blocker. New clue from the W&B curve:
  `env/reward_rise_ref` starts at 0.65/tick at the warm-started
  checkpoint (the crutch was briefly engaged) and is eroded to
  ~0.02/tick within the first few updates — looks like early PPO
  updates destroying an aligned start, not undiscovered exploration
  (plausible cause: the warm-started critic is miscalibrated for the
  new score-routed reward). Follow-up `cw-stand-scoreref1-dr0-lowlr`
  (LR 3e-4→5e-5, one variable) is in flight to test the erosion
  hypothesis directly.
- `cw-stand-scoreref1-dr0-lowlr` — FAILED (08-11): the LR-erosion
  follow-up above. Cutting LR 6x did NOT slow the collapse —
  `env/reward_rise_ref` still crashed from 0.66→~0.002/tick within
  ~20-30 update steps (run median 0.021, gate needed ≥0.3) and
  `env/rise_score` never left the floor (median 0.021, max 0.096).
  Erosion-by-oversized-updates is REFUTED. Two live explanations,
  neither tested yet: the summed reward genuinely disfavors the
  tracked behavior once the rest of the stack is added in, or the
  tight 6° tracking sigma is measuring ordinary rollout stochasticity
  as "the behavior is gone." No further LR/coefficient variant
  queued — RISE.md's ruling below already closes that line; the next
  real lever is the structural height↔contact coupling (CODE).

## Direction (binding, 08-11 — supersedes the 08-10 night entry)

Four distinct mechanisms have now ALL been beaten by the identical
flag-leg trick: detect-and-discount (b2p1's posture gate), a
multiplicative PLANT_SPEC gate (plantgate1), moving the income source
itself (score1), and — 08-11 — a cheat-proofed reference/trajectory-
tracking crutch layered ON TOP of score1's income routing
(`cw-stand-scoreref1`: `k_rise_ref_track=2.0`, feet-gated kernel,
sigma tightened 12→6°). scoreref1 was the pre-registered attempt at
lever (a) below ("re-open the waypoint option" — operator's own
hardcore-disagreement test) and it did NOT stop the cheat: rise 0/6
det+sto at both DR0 and own-DR0.2, worst-foot clearance 160–188mm,
`env/rise_score` flat ~0.01–0.02 the entire 2M steps (its own early-
stop trigger). Ruling: **reward-income shaping AND reference-
tracking-as-crutch are BOTH now closed for rise.** Do not propose a
5th income/tracking variant on top of the existing reward stack —
showing the policy the right motion doesn't help while a fake stand
can still collect nearly full pay somewhere else in the same stack.
**The only remaining lever is (b): a structural coupling between the
commanded height goal and measured foot contact** — e.g. the height
*reference* itself refuses to rise on a leg the moment that leg loses
contact, rather than paying/penalizing after the fact. That is CODE
work (new mechanism, not a reward-coefficient respec) and must go
through SPECIFICATION (bank the exploit) before any DISCOVERY run.

## Direction (binding, operator-supervised, 08-10 late — supersedes
## the 08-11 ruling above)

The 08-11 ruling diagnosed the failures as reward-side and closed the
tracking line. A five-run controlled forensic ladder run WITH the
operator on 08-10 evening overturns that diagnosis:

- `cw-stand-scoreref1-dr0` (one change: DR 0.2→0): the warm start
  BEGINS with the crutch engaged (reward_rise_ref 0.65/tick, feet
  factor 0.87) and training erodes it. DR exonerated as root cause.
- `cw-stand-scoreref1-dr0-lowlr` (one change: LR 3e-4→5e-5): same
  erosion, slower. Update size exonerated.
- `cw-stand-scoreref1-dr0-riseonly` (one change: goal-mix rise=1.0):
  same erosion. Cross-mode interference exonerated. Also surfaced:
  train/std pinned at 0.198 all run.
- Noisy-replay probe: the reference replayed under the FULL 0.198
  action noise still earns +357 and stands 2/3 — the summed reward
  stack orders noisy-honest ≫ every cheat. Pricing exonerated,
  including the "6° sigma just measures stochasticity" theory.
- Re-read of the "early tracking pay": it was the PRE-RAMP hold
  window (lying at ref start), not rising — the warm start never
  rises at +111mm commands (outside its trained band).

Net: the reward is right and the path pays, but **training never
visits the paid states** — a pure state-distribution problem that no
reward term can fix, which is why four reward-side mechanisms in a
row failed the same way. The standard fix (DeepMimic RSI; also load-
bearing in HumanUP/HoST stage 2) landed 08-10 late: rise episodes
spawn ON the reference at a random phase (`goal.rise_rsi_frac`,
default-off; sag-robust via nearest-neighbor ref-clock re-alignment
at settle; remaining-rise schedule from the npz height profile).
Validated: forced-RSI spawns across the whole path continue to a
valid plant 7/8 with returns +400..+860, late spawns paying best;
bank green, default-off exact. First arm: `cw-stand-rsi1` =
scoreref1-dr0 stack + `rise_rsi_frac=0.5`. Lever (b) (structural
height↔contact coupling) stays open as the NEXT step only if RSI
with a correctly-priced stack still fails.

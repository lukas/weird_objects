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

## Direction (binding, 08-10)

The geometric criterion the 08-10 ruling demanded is LANDED (PLANT_SPEC
above): next rise arm sets `reward.rise_plant_polygon_gate=1` on top of
the standard rise stack, cites the passing bank as `--evidence`, and
gate-evals with valid_plant reported (baseline the champions before
flipping `--valid-plant-gate` into success). Do NOT re-run b2p1's
recipe with more steps or a different k_rise_ref_track weight. Big
rise consolidation runs wait for the loaded-actuator model (rise is
the motion most sensitive to it). Any new rise mechanism goes through
DISCOVERY (≤2M steps, early video) before any hardening budget.

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
- `cw-stand-scoreref1-dr0-riseonly` — FAILED (08-11), forensic probe:
  identical to scoreref1-dr0 but `goal-mix rise=1.0` (no lower/hold
  mixed in), pre-registered to split cross-mode interference from a
  within-rise cause. Harness: rise 0/6 det+sto, worst-foot clearance
  151–164mm, video the same held-leg cheat as the whole score/
  scoreref family. `env/reward_rise_ref` crashed 0.51→~0.03–0.05/tick
  within the first ~130 sampled ticks — the SAME fast timescale as
  the mixed-mode runs, even with zero lower episodes in the batch.
  **Cross-mode interference (Suspect A) is REFUTED** — the erosion is
  intrinsic to the rise task/reward itself, not diluted-by-lower
  gradients. Narrows to the two live explanations above (summed stack
  genuinely disfavors the tracked motion once composed, or the 6°
  sigma is measuring ordinary rollout noise as "gone") — neither is
  worth a further reward-coefficient arm; go straight to the
  structural coupling.

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

**08-11 ~03:30 — ACTUAL root cause found (supersedes the exploration
framing above).** `cw-stand-rsi1` still eroded, but its `env/rise_rsi`
tick share decayed 0.58→0.15 with ZERO terminations — impossible for
a constant 0.5 spawn fraction, i.e. a CODE smell, not behavior. Trace:
the warp/MJX vec envs recycle episodes from a reset pool and restore
per-episode host state from `mjx_host.SNAP_ATTRS` — and none of the
score-stack episode attrs added 08-10 were in that list. Every
pool-restored episode inherited a random other episode's
`_score_best` ratchet high-water mark (so progress income ~never
paid), `_rise_ramp_i0` ramp anchor (mis-clocked ref tracking and
post-ramp checks) and, in rsi1, lost its RSI clock entirely. This is
mechanically the observed signature everywhere: first-generation
episodes pay (warm 0.65-0.82/tick ref income at step 0), pooled
generations take over within ~20-30 updates and the pay — not the
behavior — collapses. Local probes never showed it (host env has no
pool). Consequences: **the score1/scoreref1/plantgate "the cheat
beats N mechanisms" verdicts are all contaminated** — those stacks
were never actually paid as designed on the GPU path; the closures
of income-shaping and tracking-as-crutch are REOPENED pending a
clean re-run. Fix landed: `SNAP_ATTRS` += `_score_best`,
`_rise_ramp_i0`, `_end_posture_from`, `_rsi_pending`,
`_rsi_ref_tick0`, plus a rule note that any new per-episode attr
read in the step path must join the list. First clean arm:
`cw-stand-rsi2` (rsi1 args, one change = this fix).

**08-11 — `cw-stand-rsi2` reports, RE-CLOSES the reopened verdicts.**
Mechanism health this time is genuinely clean: `env/rise_rsi` held
0.48–0.58 the ENTIRE 2M steps (mean 0.52, no decay — the pool-restore
fix works, no more state corruption). And it still failed the exact
same way: `env/reward_rise_ref` crashed 0.83→0.02–0.09/tick within
the first logged window and stayed there; `env/rise_score` never left
the 0.01–0.02 floor the whole run. Harness confirms: rise 0/6 det+sto,
worst-foot clearance 146–161mm, video-identical tripod (three legs
never leave the ground, three legs held 20–146mm up the whole
episode; duty cycle ~0.8–0.95 on the down legs vs ~0.01–0.13 on the
up legs). This is a CLEAN read (no corrupted state to blame) that
reproduces the identical failure. Ruling: the pool-restore bug is
EXONERATED as the cause of any prior rise verdict; **income-shaping
and reference-tracking-as-crutch are RE-CLOSED, on stronger evidence
than before the bug was found.** RSI (state-distribution fix) is
ALSO now refuted as a fix for this failure mode — it does what it was
designed to do (episodes visit the paid states) but the visited pay
still doesn't stick during training. Do not requeue another RSI/
income/tracking coefficient variant. The only remaining lever is (b):
structural height↔contact coupling [CODE] — RL_PLAN.md queue item
2b. No further DISCOVERY arm on the current reward stack until that
lands.

**08-11 ~04:50 operator session — `cw-stand-rsi3` and the two-lever
choice (reconciles with the cycle verdict above).** `cw-stand-rsi3`
(one change vs rsi2: `reward.rise_score_strip_pen=1` — the
k_height=100 PENALTY was still live and made flag-leg the reachable
optimum: belly rest −1.2/tick vs cheat rent −0.5/tick; bank green
with the strip) STILL collapsed, with an identical curve. Decisive
observation across ALL SIX runs (rsi1/2/3, dr0, dr0-lowlr,
dr0-riseonly): the feet-factor collapse (0.87→~0.17 by the 25% mark)
has the same shape and timescale under materially different rewards.
Behavior that does not respond to reward changes is not
reward-driven: this is **warm-start drift at out-of-distribution
observations** (the 108–114mm command band is ~2.2x the stance
champion's trained range; its behavior there is un-anchored and
update noise erodes it), and no reward stream can anchor it because
the 6° kernel only pays a policy that is ALREADY nearly perfect —
the cycle's persistent-tripod video above is exactly what unanchored
drift settles into. Widening the kernel is bank-blocked (measured
08-11: at sigma 10° flag-leg farms 108 vs replay 648 = 17%, over the
10% bar; at 15° three bank tests fail). Agreed with the cycle:
no more reward/income/tracking/RSI coefficient variants. Two CODE
levers remain, both through SPECIFICATION first:
(a) **BC anchor in the TRAINER** — auxiliary loss pulling the
    policy's action toward the reference action at RSI-spawned
    states (DeepMimic-family standard; supervises actions, immune to
    pose-farming, needs no rollout luck);
(b) **structural height↔contact coupling** (the cycle's queue item
    2b — the height reference itself refuses to rise on a leg that
    lost contact).
Operator leaning: spec (a) first — it attacks the measured mechanism
(drift with no anchoring gradient) directly, while (b) reshapes the
goal but still pays through the same RL gradient that the drift
out-runs. Everything else on this stack is now verified honest:
pricing (bank + noisy replay +357), state coverage (RSI holds 0.5),
state restore (pool fix), penalties (strip_pen).

**08-11 — lever (a) LANDED: reference BC anchor in the trainer
(SPECIFICATION pass green).** `rl_move/sim/bc_anchor.py`: BCAnchorPPO
adds one supervised step per update AFTER the untouched PPO update
(MirrorPPO pattern — no SB3 internals copied), minimizing
`coef * mse(pi_mean(obs), a_ref)` on a 131k ring buffer of
(post-step obs, reference action) pairs collected from live rise
rollouts. The env emits `info["bc_target"]` — the normalized action
whose joint target is the reference pose one ref-tick ahead of the
episode's live ref clock (`sim_env._rise_ref_clock`, shared with the
tracking reward so the two clocks can never disagree) — gated on
`train.bc_anchor_coef` riding into the env cfg; RSI and legacy
ramp-aligned episodes both emit. This supervises ACTIONS, not visited
rewards: at drifted states the target points back onto the
demonstrated path, which is exactly the anchoring gradient the
6° kernel cannot provide. Reward stack untouched (not a reward term;
rise bank unaffected — full bank re-run green post-refactor, 23
passed/1 pre-existing skip). Validation: 10/10 new tests
(`rl_move/tests/test_bc_anchor.py` — default-off exactness, RSI +
legacy clock alignment, target-chain tracks the path <8° RMS, aux
step provably moves pi_mean, done-boundary pairs skipped, loud
refusal on missing ref path, composes with MirrorPPO), MJX-pod smoke:
anchor engages, buffer fills, `train/bc_anchor_loss` 0.198→0.04
within one smoke. Knobs: `train.bc_anchor_coef` (0=off exact),
`_minibatches` (8), `_batch_size` (4096), `_buffer` (131072).
First arm: `cw-stand-bc1` = rsi3 stack + `train.bc_anchor_coef=1.0`
(ONE change), discovery 2M. Decisive signal: `env/rise_feet_factor`
must stop collapsing (all six prior arms: 0.87→~0.17 by the 25%
mark) while `train/bc_anchor_loss` stays low; if feet hold and
rise_score climbs, harden with an anneal schedule so the final
policy is not trajectory-locked. Lever (b) (structural
height↔contact coupling) stays next if the anchor fails.

## `cw-stand-bc1` (08-11) — PASS (partial): lever (a) WORKS, first
## honest rise in 7 stand-arms

`env/rise_feet_factor` dipped to 0.32–0.37 through 260k–590k (would
have fired the pre-registered <0.4-by-500k kill if live-monitored —
note for future kill rules: needs a sustained window, not a first
crossing, on this mechanism) then RECOVERED, climbing to 0.75 by 2M —
categorically different from all six reward-only arms, whose curve
never recovers. `env/reward_rise_ref` climbed to 0.6–0.77 (vs the
0.01–0.03 floor every prior arm flatlined at) and `env/rise_score`
left the floor (0.01→0.21).

Harness confirms this is real, not another metric artifact. Gate
pass (RSI 0.5, as trained): rise valid_plant **3/6 det** — honest
six-foot plants, height_err 4–7mm, all feet <20mm off the ground,
video-confirmed (frame strips show a genuine belly→spread-leg stand,
nothing resembling the flag-leg/tripod cheat). A follow-up probe run
directly on the pod (`--modes rise --per-mode 30 --seed 7
--cfg-set goal.rise_rsi_frac=0.0`, isolating the anchor from RSI's
help) on the SAME checkpoint: **bridge 7/12 and crouch 6/8 pass the
full geometric valid_plant** check; **flat-belly cold start (the
hardest case — legs straight out, belly down, exactly the operator's
placement) reaches a real six-foot stand 10/10 times** (correct
height, no flag leg, current in-band) but misses only the
walk-anchor **footprint** tolerance every time (0/10 valid_plant,
worst foot clearance 7.8mm — a foot-XY positioning gap, NOT a
height/posture cheat; video-confirmed indistinguishable in kind from
the passing bridge/crouch stands). Zero flag-leg/tripod cheat across
42 video-checked episodes total. The identical-recipe parent
`cw-stand-rsi3` (only missing `bc_anchor_coef`) shows the flag-leg
cheat 0/12 valid_plant on this exact reward/goal-mix stack — the
causal attribution to the anchor is clean (one variable).

Cost (skill interference, weak evidence — n=2 training-diagnostic
probe samples, not harness-verified): `raise` and `tipped_recovery`
both read 0/2 at 2M vs rsi3's 1–2/2 and 2/2; hold/track angle error
3.0° vs rsi3's 1.2–1.6°. The anchor only supervises rise-tick
actions but shares the network with every mode — plausible bleed
into nearby height/posture manifolds. `ep_rew_mean` fell to −29 by
2M (rising tilt/over-current terminations during genuinely riskier
rising attempts, not hold/track breaking — video confirms hold/track
look normal).

**Ruling: lever (a) is validated — reward-income shaping was
correctly diagnosed as exhausted, and moving supervision OUTSIDE the
reward (BC anchor on actions) is what unblocks flat-start rise.**

`cw-stand-bc1-coef03` (08-11, same protocol, coef 0.3 vs bc1's 1.0)
**FAILED — dose-response refuted, decisively.** RSI-off harness
probe: valid_plant **0/16 across every start kind** (bridge 0/7,
crouch 0/5, flat 0/8 det), vs coef=1.0's 13/30 (bridge 7/12, crouch
6/8, flat 0/10-but-honest). Video still shows a genuine stand
attempt (no flag-leg regression) but flat starts now fall SHORT of
full height (h_err ~26mm vs coef=1.0's ~11mm) and every episode still
trips the current ceiling. Training's own diagnostic did not improve
either (hold/track angles the same or worse, raise/tipped/rise-flat
all flat-or-worse vs coef=1.0). **Lowering the dose does not buy
cleaner cross-mode behavior — it just weakens the anchor, on every
axis measured.** Ruling: keep `bc_anchor_coef>=1.0`; do not queue
another coefficient-reduction variant.

`cw-stand-bc1-hard1` (08-11, 10M steps, same coef=1.0) **PASSES
(partial) on rise, but surfaces a real, pre-existing hold/track
cost that WORSENS with more training.** Rise consolidates further:
gate valid_plant 5/6 det (83%, up from bc1's 3/6=50%), tight height
errors (0.2–2.2mm on most episodes) — confirms the fix holds and
improves with budget. But re-checking hold/track's per-episode
`duty_cycle`/`swing_count`/`end_clear_mm` fields (NOT examined at
bc1's original verdict — a sparse 10-frame video strip missed it)
shows hold/track are not quiet stands: alternating legs cycle
continuously (duty ~0.85–0.9 / 0.06–0.09, 6–19 swings per 15s
episode) ending 12–50mm elevated at bc1 (2M), **100–161mm at bc1-
hard1 (10M)** — hold/track harness success 0/6 both modes at both
checkpoints. This pathology PRE-DATES the anchor (present already
at 2M) and is not primarily an anchor side-effect — more likely a
pre-existing hold/track income-pricing gap (continuous stepping
isn't charged; `k_still` as written scopes to belly-rest/lower, not
general hold/track) that the anchor's shared-network pull amplifies
with more steps. **Lesson for future triage: check
duty_cycle/swing_count/end_clear_mm for every stand-line mode, not
just valid_plant plus a sparse frame strip — a checkpoint can look
static in 10 sampled frames while stepping continuously between
them.**

Ruling: do not keep blindly hardening this lineage hoping hold/track
self-heals (the trend is the wrong direction). Next step is a
SPECIFICATION pass auditing hold/track's stillness pricing (why
continuous leg-cycling isn't charged) — a reward-mechanism change,
so it needs its own `test_task_semantics.py` HOLD-mode bank entry
before any training. Not yet queued (08-11) — this cycle only
diagnosed it.

**08-11 dig-in addendum — matched-parent control settles the
"regression" question; hard1 promoted to RISE SPECIALIST.** The
escalated "hold/track/raise/lower collapsed under hardening /
protected-skill erosion" read is REFUTED: the identical RSI-off
probe (seed 7, per-mode 12, same cfg) run on the PARENT
`cw-stand-bc1` (2M) shows every one of those modes was ALREADY 0/12
before the extra 8M steps — hold 0/12 (worst foot 51mm), track 0/12
(65mm), raise 0/12 (40mm; `p_raise=0` in the goal mix, the mode is
untrainable in this arm and the gate's raise criterion was
ill-posed), lower 0/12 with a **166mm flag-leg at 2M** (child:
189mm — same cheat, pre-existing; training's `SCORE/lower_success`
=1.0 is the height-only criterion and is blind to it). Nothing the
parent could do was lost; the crown-jewel lower lives in the
rfix-warm1/vref1-r1 lineages, untouched by this arm. Meanwhile the
child's rise is now **12/12 valid_plant RSI-off incl. flat 4/4
(worst foot 7mm)** — bc1's flat-start footprint miss resolved with
budget — and `rise_feet_factor` held 0.69–0.82 for all 10M (no
re-drift; the pre-registered kill signature never appeared;
trajectory-lock refuted by cold-start success). Verdict recorded:
`ppo_goal_cw_stand_bc1_hard1` = the rise specialist (SKILLS.md);
lineage closed for hardening; next = HOLD-stillness SPECIFICATION,
then the composition test (learned rise → walk/hold champion
handoff, replacing the stance-champion + scripted-blend fallback).
Harness fix landed same cycle: `eval_checkpoint.py` now refuses
unknown `--modes` loudly ('tipped' is a trainer periodic-eval axis,
not a harness mode — passing it used to zero every goal probability
and NaN-crash after the good modes had run).

## Hold/track stillness pricing — SPECIFICATION LANDED (08-11 idle-kick cycle)

The bc1-hard1 dig-in's queue item is done. HOLD bank added to
`test_task_semantics.py` (plant start, hold mode, the exact
stand-line stack): scripted `quiet` (hold the settled plant),
`stepping` (alternating tripods at ~1 Hz, swing peak ~40 mm — the 2M
pathology's honest-magnitude form) and `flag` (one front leg parked
189 mm up, five planted, frozen — the 10M pathology; a both-front-legs
splay nose-dives and terminates under position control, so the single
flag is the stable scripted member of that class). Measured legacy
returns (seed 0, 15 s): quiet 367.9, stepping 300.7, **flag 368.0 — a
frozen flag-leg park LITERALLY TIES the honest quiet stand** because
the tracking kernel has no opinion on legs and `k_still` (a bonus,
default 0) charges nothing. That is the whole pricing hole in one
number.

Fix: `reward.hold_still_gate` (default 0 = legacy; REWARD.md row).
Scales kernel income on hold/track ticks by feet-down² × HARD no-flag
zero (PLANT_SPEC flag_leg_mm 60 mm; honest adjustment swings stay
below it) × stillness Gaussian applied only while the reference is
stationary (TRACK's commanded attitude motion never charged). Scoped
strictly to hold/track — quad lifts legs and unload opens a contact on
purpose. Implemented in `sim_env._step_finish` (shared by the CPU
harness and the warp/MJX host-worker path). Gated bank ordering:
quiet 367.9 > stepping 107.2 > flag 9.5; gate-bite and no-tax tests
both pass; full task-semantics suite green (29 passed, 1 pre-existing
skip).

Next: `cw-stand-holdstill1` — discovery 2M, warm from
`ppo_goal_cw_stand_bc1_hard1` (the rise specialist), ONE variable
(`hold_still_gate=1.0`), same goal mix. Binary question: does hold
converge to a quiet valid plant (worst-foot <20 mm, swings →0)
without losing the honest rise? After that: the rise-specialist →
walk-champion handoff composition test.

### `cw-stand-holdstill1` (08-11) — FAIL on hold, rise retention PASS;
### plateau diagnosis → fade lever landed

The gate priced the pathology out but did not fix the behavior:
hold/track 0/12 det+sto with the IDENTICAL parent fingerprint (leg 0
parked 107–116 mm, legs 1/3/5 cycling duty ~0.9) — while
`env/hold_feet_factor` sat at ~0.1 (income ~0) from 260k on. The
pre-registered kill signature occurred; the ~4-minute run outran any
kill. Rise: det 4/6, sto 6/6 valid_plant, feet factor 0.53–0.79 all
run — retraining under the gate cost the honest rise NOTHING (better
than the parent's 2M band).

Diagnosis (the bc1 lesson again, in miniature): earning zero is not
being pushed back. The hard no-flag zero makes the whole splay
neighborhood a flat zero-income plateau, so PPO gets no slope telling
the parked leg WHICH WAY to move, and hold is only 10% of the mix.
Lever landed same cycle: `reward.hold_flag_fade=1` (REWARD.md) — the
no-flag factor becomes a linear ramp over 60→120 mm, so the observed
113 mm park earns 51/ep (0.14× quiet, scraps) with monotone slope to
full pay at feet-down; the 190 mm class stays at 0. Bank extended
(3 new tests: ordering preserved, gradient exists, park stays <25%
of quiet). `cw-stand-holdstill2` = holdstill1 + the fade, one
variable. If the fade also fails, the next lever is BC-style
supervision on hold ticks (target = the episode start pose), the
mechanism already validated on rise.

### `cw-stand-holdstill2` (08-11) — fade directionally right, still 0/12;
### hold pricing levers EXHAUSTED, next is BC supervision on hold ticks

One variable vs holdstill1 (`hold_flag_fade=1`). The slope works as
designed: parked leg 107–116 → 86–101 mm, `env/hold_feet_factor`
0.1 → 0.19–0.35 (still rising at 2M), track episodes down to
29–56 mm. But hold det+sto 0/12 — the quiet stand was never reached
in-discovery; rise retention held (det 4/6, sto 4/6). Ruling (two
pricing misses in a row = change the hypothesis; discovery rules
forbid extending a run whose target behavior has never been seen):
**no third pricing/mix/step variant on hold.** The gate+fade STAY
(bank-proven correct pricing — they will pay the real behavior). The
next lever is the rise playbook repeated: BC-style supervision on
HOLD ticks, target = the episode start pose (trivially available;
extend `bc_anchor.py`'s bc_target emission beyond rise ticks) — a
SPEC/CODE item (trainer change + bank re-run) before any further
stand-line launch. After that lands and a hold arm passes: the
rise-specialist → walk-champion handoff composition test (still the
plan's next composition milestone).

### CODE landed (08-11, idle-kick cycle): bc_anchor covers hold/track

`sim_env._is_hold_bc` (new per-episode flag, added to
`mjx_host.SNAP_ATTRS` — the pool-restore lesson applies to every new
per-episode attr, not just the rise ones) fires on hold/track ticks
and emits `info["bc_target"] = q_rad_to_action(self._q_nom)` — a
CONSTANT target for the whole episode (the pose it actually settled
at post-reset; already captured for the hold-current reward term, so
"trivially available" was correct). Rise ticks are unaffected (kept
in a separate branch, mutually exclusive with hold/track by
construction). `bc_anchor.py` docstring updated; no reward-stack
change (the anchor is a trainer loss, same as rise). Bank: 4 new
tests in `test_bc_anchor.py` (hold emission + value, track emission,
default-off-on-hold, rise/hold flags mutually exclusive across
rise/hold/track/lower) — 14/14 green; full `test_task_semantics.py`
re-run 32 passed/1 pre-existing skip (reward stack confirmed
untouched). `cw-stand-holdbc1` launched same cycle: respec of
`cw-stand-holdstill2` (byte-identical cfg — same
`hold_still_gate=1`, `hold_flag_fade=1`, same warm start from the
rise specialist, `bc_anchor_coef=1.0` was already set and is now the
ONE thing that changed behavior, since the code under it changed).
Binary question: does the mechanism that fixed rise also fix hold,
or is hold's "earning zero → no pushback" failure mode not fixable
by BC alone? If it also fails: three hold misses in a row, fall back
to the rise-specialist + scripted-blend handoff without a learned
quiet hold, and stop spending discovery arms on hold pricing.

### `cw-stand-holdbc1` (08-11) — PASS: HOLD SOLVED, third lever works

Answer to the binary question above: **yes, the BC-anchor mechanism
that fixed rise also fixes hold.** Harness (DR0 gate, det+sto,
per-mode 6): hold 12/12 valid_plant, worst-foot clearance 2–13mm,
height_err_end_mm ≈2 — every episode ends level, six feet down,
motionless, both deterministic AND stochastic. Video-checked (both
modes): the frame strips show zero movement across the full 15s clip
— no shuffling, no flag-leg, no drift. `env/hold_feet_factor` (the
gate's pre-registered mechanism-health signal) cleared the 0.1–0.35
plateau both `holdstill1`/`holdstill2` sat in for their entire runs,
climbing to ~0.99 by the FIRST logged point and holding ~0.99–1.0 for
all 2M steps — the earning-zero plateau never formed.

Rise retention (the gate's other conjunct, pre-registered floor
det >=3/6): bridge starts clean 2/6→2/2 valid, sto clean 6/6 (2 of
those flagged only on the soft current-limit check, not posture).
Det crouch starts came in at 2/6 valid (2 tilt_roll falls + 2
height-overshoot misses) — below the pre-registered floor taken at
face value. Checked against the lineage's own history before calling
this an erosion: `holdstill1`'s rise/det report has ZERO falls (2
height misses only); `holdstill2`'s rise/det report has exactly ONE
tilt_roll fall on a crouch draw (return −44.2, valid_plant False) —
the SAME failure signature, same magnitude, one draw. `holdbc1`
adding a second crouch fall on n=6 is the identical pre-existing
fingerprint recurring one more time on a 6-episode sample, not a
new pathology introduced by the hold-BC code change — video of the
failing episodes (`rise_det_2.png`/`rise_det_3.png`) shows a genuine
tip-over (body rolls onto its side), the same visual signature as
`holdstill2`'s single fall, not a flag-leg/tripod cheat. Ruling:
**PASS overall** — the headline mechanism (hold) is decisively fixed;
the crouch-start rise dip is a known, small, pre-existing fragility
to track, not a new regression, and not a known-exploit stop
(no cheat pattern on any episode, any mode).

Track-mode command-tracking accuracy stayed weak (det 2/6, sto 0/6 on
the tracking-error success metric) while posture stayed valid
throughout (end_posture 6/6 both passes, worst_clear 7–10mm) — not
part of this arm's pre-registered gate, noted for later (tracking
precision, not a posture/stillness problem).

Checkpoint `ppo_goal_cw_stand_holdbc1` (SKILLS.md: Hold row). The
"three hold misses in a row -> scripted-blend fallback" contingency
is now moot. Next: hardening continuation `cw-stand-holdbc1-hard1`
(10M steps, `--evidence` citing this discovery pass) to see if extra
budget also closes the crouch-start rise gap, the same way bc1's
flat-start footprint miss resolved with budget in `bc1-hard1` — then
the rise+hold → walk-champion handoff composition test, the plan's
next named composition milestone.

### `cw-stand-holdbc1-hard1` (08-11) — PASS: hardening consolidates,
### matches every pre-registered gate condition, lineage CLOSED

Binary question was whether 5x the budget (2M→10M) would also close
the crouch-start rise gap left by discovery, without eroding hold.
Answer: it holds hold and slightly improves crouch, exactly the
"no worsening" bar the gate asked for.

Harness (DR0 gate, det+sto, seed 0): hold valid_plant 11/12 (det
6/6, sto 5/6 — the one sto miss carries only a `current` soft-limit
flag, height_err 4.1mm, posture otherwise fine; not a posture/cheat
failure). `env/hold_feet_factor` held 0.990–1.0 for the entire 10M
steps (min 0.9904) — the pre-registered mechanism-health floor
(>=0.9) cleared with wide margin, no re-drift toward the
earning-zero plateau. Track valid_plant: det 5/6 (again one
`current`-only miss), sto 3/6 (three `current` misses) — not part
of this arm's gate (track command-tracking accuracy, tracked
separately below).

Det crouch-start rise: 2/4 valid (50%), vs discovery's 2/6 (33%) —
improved, not flat, comfortably inside the gate's "improve or hold
flat" bar. The one fall (`rise_det_2`, tilt_roll, return −49.6) is a
genuine tip-over on video — the robot rises normally for the first
half of the clip then rolls onto its side, no flag-leg/parking
signature. The one miss (`rise_det_4`, `plant_fail=['height']`,
height_err 22.8mm) is a correct-looking six-foot stand on video,
just outside the height tolerance — not a cheat either. Bridge
starts stayed clean (det 2/2, sto 3/3); sto rise overall 5/6 valid
(one `current`-only miss on a bridge start). Zero flag-leg/tripod
cheat pattern across all 24 det+sto episodes reviewed (hold, track,
and rise strips) — every failure mode is either a soft current flag,
a genuine fall, or a height-precision miss, never a frozen/splayed
park.

Reward-quarters [148.8, 271.5, 282.5, 283.5] show the usual
fast-rise-then-plateau shape, consistent with a converged, not
budget-starved, run.

Ruling: **PASS, lineage CLOSED for further hardening.**
`ppo_goal_cw_stand_holdbc1_hard1` is the hardened HOLD+RISE
checkpoint (SKILLS.md new row). No further step-count/coefficient
variant on this stack — matches the "two clean passes, stop
tuning" pattern from the bc1 lineage. Track-mode command-tracking
precision (sto tracking-error success still weak, though posture
holds) stays noted for later, not gated. Next, per RL_PLAN queue
2.3 and the plan's next named composition milestone: the rise+hold
→ walk-champion handoff composition test — swap the current
scripted-1.5s-blend handoff (stance champion → walk champion) for
this learned specialist's own settled pose. This needs a small new
piece of CODE first (an eval/drive script that runs the specialist
to a settled hold, then switches control to the walk champion and
checks the walk champion doesn't stumble on the specialist's exact
final pose/velocity state) — not yet built, so not launch-ready this
cycle.

### Handoff composition test (08-11) — PASS: the specialist's stand
### is a walkable start; the scripted blend is obsolete

The plan's next named composition milestone, now built and run:
`rl_move/sim/eval_handoff.py` (snapshot 7f91f87). Three arms, 3
episodes per rise start kind, two physics variants (default air +
`bus.servo_params=loaded`, the specialist's training physics):

- **direct**: `ppo_goal_cw_stand_holdbc1_hard1` runs a genuine
  training-distribution rise episode (env's own goal generator,
  RSI off, plant band 108–114 mm) to a settled hold; control then
  switches to `ppo_goal_cw_walk_longdist_r2` ON the specialist's
  exact physical state — episode bookkeeping re-anchored to a clean
  plant frame (the champion's training frame, same trick as play.py
  key-7), qpos/qvel/ctrl and the safety slew memory carried over
  unchanged; then 1 s settle + 6 s fwd @0.05 + 2 s stop.
- **blend**: identical plus the incumbent scripted 1.5 s joint blend
  to the walk plant pose between the policies.
- **plant**: walk champion from its own clean plant reset — the
  drive-metric noise band.

Results (per-episode records + strips:
`logs/ckpt_eval/handoff_holdbc1hard1_{air,loaded}.json`, strips dir
alongside): every successful rise handed off with ZERO falls in both
arms and both physics. Direct-arm drive metrics sit inside the plant
band: air trk_err 0.032–0.036 vs plant 0.031, dist 0.394–0.443 m vs
0.431, stumble-window max tilt 1.2–2.6° vs 1.5°; loaded trk_err
0.041–0.052 vs plant 0.050, dist 0.348–0.421 vs 0.360, tilt 1.7–4.5°
vs 1.9°. Blend-arm numbers are indistinguishable from direct —
**the scripted 1.5 s blend adds nothing; the specialist's settled
pose (ends ~124 mm chassis height, worst-foot 1.8–4.9 mm, height_err
|<6| mm) is already in the walk champion's start distribution.**
Video: flat/bridge strips show curl → six-foot rise → level stand →
normal all-six-legs gait after the switch; no flag-leg, no dragging,
no lurch at the switch tick.

Caveat, pre-existing and now sharpened: crouch-start rises fell
(tilt_roll, within ~2 s of ramp start) before the handoff in 6/6
episodes across both physics (RSI-off). The lineage's own gates saw
2/6–2/4 crouch failures (RSI on); RSI-off crouch appears worse than
the gate suggested. Not a handoff defect — flat (the realistic
operator placement) and bridge rises went 12/12. Tracked as the
lineage's known fragility; do not reopen hardening for it (two-pass
rule) — if crouch matters for the joystick chain it needs its own
mechanism question.

REVERSE handoff (walk → stop → lower/sit) DONE 08-11
(`rl_move/sim/eval_handoff_reverse.py`, same reanchor pattern; arms:
spec = specialist's own clean lower episode / direct = specialist
lowers on the walk champion's exact stopped pose+slew state /
scripted = 6 s glide to the zero pose, the hardware go_zero("sit")
analog). 6 eps/arm, air AND loaded
(`logs/ckpt_eval/handoff_rev_holdbc1hard1_{air,loaded}.json`):

- **Handoff itself: CLEAN.** direct == spec on every axis (4/6
  posture-strict both physics, zero falls anywhere, same failure
  signature, height_err 0.4–9mm) — the walker's gait residue costs
  nothing, mirroring the forward result.
- **Specialist lower post-holdbc1: mostly intact, not posture-strict.**
  8/12 pooled. Every miss is the SAME fingerprint: belly down at
  target height and level, but ONE foot (leg 2, of the elevated
  {0,2,4} triple) dangles 62–99mm > the 60mm belly allowance —
  video-confirmed a cosmetic dangling foot, NOT the old 130–190mm
  weight-bearing flag-leg cheat (huge improvement over bc1-hard1's
  lower 0/12 @189mm).
- **Scripted glide: 6/6 both physics, deterministic, gentle**
  (tilt ≤2.5°, all pads 34–38mm) — and it is already the
  operator-prescribed hardware sit (go_zero("sit") slow glide,
  never refuses). The learned lower is NOT needed for the joystick
  deliverable.

Ruling: sit side of the chain is COVERED by the scripted glide; the
full sim joystick motion cycle (specialist rise → walk champion
drive → stop → scripted sit) is now composed with zero falls.
OPTIONAL polish, not queued (prime directive): extend the BC anchor
to lower ticks (reversed rise ref / glide-to-zero target) to fix the
dangling foot if a one-policy stand/sit specialist ever matters more
than the scripted path.

### Deploy-side port (08-11 late) — the specialist is the robot's live
### stance policy; bench validation is all that remains

The RL_PLAN critical-path [CODE] item after both handoff PASSes:
`ppo_goal_cw_stand_holdbc1_hard1` exported
(`export_policy_np.py`, SB3↔numpy parity 2.65e-07) to
`linux_control/policies/stand_holdbc1_hard1.json` AND copied live over
`linux_control/rl_policy_weights.json` (the stance slot the web UI's
STAND/LOWER buttons run).

Design point, mirroring the rot60-port "no second implementation"
rule: the runner used to hardcode the stance_dr10-era goal ramp
(hold 5 s / ramp 4 s / +50 mm). Feeding that ramp to the specialist
would command a half-height stand. The trained profile now rides
INSIDE the weight file (`meta["profile"]`, new `--extra-meta` export
flag): specialist stand = hold 5 s / ramp 6 s / target +111 mm (mid of
the trained 108–114 band) / total 12.5 s (the handoff eval's validated
switch point, inside the 15 s training horizon). `rl_policy.py` reads
the profile per episode (`policy_profile()`); files without one keep
the legacy constants, so picker rollback to `stance_dr10.json` is
behavior-identical to the pre-port runner. Also: stance-slot obs-dim
guard (refuses a mis-slotted file), `deploy_adb.sh` now ships
`rl_walk_weights.json` + `policies/` (both were hand-staged before).

Verification:
- `rl_move/tests/test_stand_runner.py` (5 tests, green; full suite
  119 pass / 6 pre-existing skips): live-file == picker-file bytes,
  meta/profile values, runner ramp == training `GoalGenerator` rise
  trajectory (<2 mm), 68-wide obs layout block checks (goal height
  scaling, prev-action block), numpy-vs-SB3 parity on the source zip.
- Closed-loop sim smoke with the DEPLOYED artifacts (numpy weights +
  meta profile driving the env): flat-belly start, chassis 38 mm →
  149 mm (+111 mm, exactly the commanded target), zero falls,
  2/2 episodes.

NOT an assumption after all — the OPERATOR independently exported and
ACTIVATED the specialist in the robot's live stance slot the same
morning (commit 1e64263, bench session 08-11, md5 6620705c; see
HARDWARE.md "Bench state — drive session 08-11 morning"). That copy
predates this port and carries NO profile meta, so the on-robot
runner would feed it the LEGACY ramp: hold 5 s / ramp 4 s / **+50 mm**
— an out-of-distribution height command for a policy trained only on
108–114 mm targets, at a faster-than-trained ramp. **Re-push (deploy_
adb.sh) or re-select this repo's profile-bearing export before
pressing STAND**; the picker JSON's notes now say the same. Rollback
either way: `POST /api/rl/policy_select {"file":"stance_dr10.json"}`
(profile-less → legacy constants → behavior-identical to the old
runner).

Bench protocol (attempt #2 addendum): fresh set_zero, robot flat on
belly → STAND button (operator watching, 10° trip, 2.5 A trip) →
verify quiet hold → WALK button forward (existing captured-plant
preflight gates the pose) → go_zero("sit"). Every episode logs
obs+profile to CSV for offline replay parity.

### `cw-stand-crouchrise1` (08-11) — crouch fragility FIXED by
### start-mix bias; promotion declined on the hold current bar

Binary question: does biasing the rise START DISTRIBUTION (60%
crouch / 30% partial / 10% flat vs legacy 25/40/35; keys
`goal.rise_flat_frac` / `rise_partial_frac`) fix the lineage's one
residual defect — crouch-start tip-overs — where more undifferentiated
budget only nudged it (2/6 → 2/4)? Answer: YES, decisively.

- Gate harness (DR0, det+sto, seed 0): rise det 6/6 valid_plant
  (crouch 5/5, bridge 1/1), sto 6/6 success (3 current-only vp
  flags); hold det 6/6 vp; ZERO terminations in all 36 episodes.
- Dig-in RSI-off ALL-CROUCH probe (seed 1, `rise_rsi_frac=0`, 8 det
  + 8 sto), matched-parent control on `holdbc1_hard1`, same seed and
  cfg (`logs/ckpt_eval/{crouchrise1,hard1}_rsioff_crouch`):
  child **det 8/8 valid_plant, 16/16 stands, zero falls**; parent
  **det 0/8 with 8/8 tilt_roll falls** (sto parent 7/8 — the det
  policy is where the fragility lived). The RSI-on gate numbers had
  been flattering the parent; RSI-off shows the true gap.
- Videos honest both checkpoints' passing episodes: crouch/bridge →
  progressive leg gathering → level six-foot stand, no
  flag-leg/tripod/stilt.

MISSED pre-registered bar: hold det+sto valid_plant 7/12 (needed
>=10/12). Every miss is the PLANT_SPEC final-0.5s-tail current soft
flag (>2.0A) under sto — hold sto 1/6 vp vs parent 5/6; det episode
Imax 2.31A vs parent 1.96A. Posture, stillness, feet, height are
identical to the parent (end_posture 12/12, worst clearance 2–10mm,
track_err 0.19° det). Real but small current-hungriness increase on
a sim metric CURRENT_TRUTHS marks untrusted (sim hold 0.11A vs real
0.59A). Per the gate's own terms: **no promotion** —
`ppo_goal_cw_stand_holdbc1_hard1` stays the deployed stance policy;
`ppo_goal_cw_stand_crouchrise1` (md5 3877e16c) is pulled to the
controller and banked as the crouch-robust variant. Track-mode sto
also dipped (1/6 vp, current flags only) — same fingerprint, same
non-gated status as the parent's known track weakness.

Lesson worth keeping: after seven reward-mechanism failures and two
budget passes, the lever that finally killed a start-kind fragility
was the START DISTRIBUTION, one cfg key, 2M steps. Stand lineage now
fully closed: rise (all start kinds, between the two checkpoints),
hold, lower, both handoffs — all sim-solved; remaining work is
bench-owned.

## LOWER semantics bank (08-11 idle-kick cycle — SPECIFICATION, no training)

The last owed `test_task_semantics.py` bank is LANDED: lower mode,
under the deployed specialist's exact stack (holdbc1-hard1 cfg-sets,
loaded servos). Honest = FixedFootBodyIK descent anchored at the
SETTLED stance (anchoring at the ideal plant leaves a 16 mm sag
error), tracking the commanded ramp with all feet planted.
Measured (3 seeds): honest 540 > aloft 461 (85%) > outrig 383 (71%)
> partial 103–182 > refuse −2..−51 > thrash −77..−107; posture-strict
(60 mm pads / 15 mm h_err) accepts honest and rejects both cheats on
every seed (~300 mm aloft pads). Bank PASSES → lower-mode arms are
unblocked.

FINDING (encoded as a strict-xfail in the bank): the pf=5/6
posture-gate pricing lets a one-leg-aloft lower keep ~85% of honest
income — cheating out-earns refusal. This is the incentive behind the
deployed specialist's cosmetic 62–99 mm dangling foot seen in
eval_handoff_reverse. Not a joystick blocker (scripted sit glide
covers the deliverable; eval catches the posture), but any future
lower-MECHANISM arm (e.g. the optional BC-anchor-on-lower polish)
should strengthen this margin first and flip the xfail.

## `cw-stand-holdload1` (08-11) — reward-side hold-cheat hypothesis REFUTED

Pre-registered mechanism test off the crouchrise trio's unresolved
hold-park cheat: is the legs-1+4 hover profitable only because the
reward/eval was BLIND to it (clearance-priced, `foot_down_mm`/
`flag_leg_mm` both miss a 1–19mm hover), or is it an anchor-bleed
artifact the reward can't touch? New `reward.hold_feet_load=1.0`
prices hold/track income on MEASURED per-foot touch force (not
height), validated by its own FEET-LOAD bank in
`test_task_semantics.py` (hover reproduced at 4–13mm/duty<0.2 earns
only 0.25x of quiet-stand income under the gated stack vs 0.85+
parity pre-fix).

Result: **same recipe (crouchrise3, dose 0.45) + the new term still
parks legs 1+4.** Gate harness det-hold `duty_cycle` is [0.77, 0.04,
0.97, 0.73, 0.03, 0.99] — identical across all 6 deterministic
episodes (same start state) — vs crouchrise1/2/3's exact same two
legs. `valid_plant` reads True the whole time because both feet
happen to be within a couple mm of the floor at EPISODE END even
though they're airborne >95% of the episode — the same telemetry
gap flagged in crouchrise1's dig-in, now proven insufficient even
after the reward correctly prices the behavior mid-episode (the bank
confirms the term works in isolation; the trained policy still finds
it worth paying for). sto hold's vp misses are the pre-existing
>2.0A current-tail soft flag (5/6), unrelated. Crouch rise unaffected
(det 6/6 incl. crouch 4/4, zero falls) — this run does not touch the
crouch fix. det lower regressed to 2/6 (leg-2 dangling ~90mm, zero
falls) — matches the crouchrise2/3 lower regression exactly, not new.

Per pre-registration: reward-side hypothesis REFUTED by direct
measurement. Three distinct lever families are now closed on this
cheat (start-mix, dose, reward-pricing) — the state/height-aligned
BC anchor (clock-indexed anchor showing lifted-leg reference poses
in plant-adjacent states) is the sole remaining suspect, and it's
CODE/spec work, not another training variant. `hard1` remains the
deployed stance checkpoint; `cw-stand-holdload1`'s checkpoint is not
promoted or warm-started from.

## `cw-stand-anchorstate1` (08-11) — state-aligned BC anchor: PARTIAL mechanism confirmation, net FAIL

The last remaining suspect got its own test: `train.bc_anchor_state_
aligned=1.0` re-indexes the BC anchor every tick to the nearest
reference pose to the robot's CURRENT joints (+0.25s pursuit
lookahead) instead of a fixed clock, so a crouch/plant-adjacent state
can only ever be supervised toward the planted tail of the reference
path, never an early belly-path lifted-leg pose.

Result: **first fingerprint movement in five runs.** Det-hold
per-foot duty `[0.98, 0.04, 0.97, 0.97, 0.93, 0.99]` — leg 4
RECOVERED (0.93 vs 0.01–0.04 in every prior arm), leg 1 still parks
(0.04). So the state-aligned anchor moved the park where four
pricing/dose/mix changes could not — anchor-bleed is CONFIRMED as *a*
mechanism, not refuted, but leg 1 shows it isn't the whole story
(candidate: the lower-bank's own documented dangling-foot incentive
gap, see above). Cost of the fix: det flat rise stalled 62mm short
with all feet planted (0/1 — under-drive, not a cheat: a state anchor
only points 0.25s ahead of wherever the policy currently is, so a
policy that stalls gets barely-moving supervision, unlike the old
clock anchor which dragged it through regardless), and det lower
picked up three tilt_pitch falls (2/6, front feet lifting ~30mm
mid-descent — lower has no anchor, so this is an indirect
shared-network effect, worse than crouchrise3's fall-free
regression). Crouch rise itself stayed clean (det 4/4, sto 3/3), hold
otherwise clean (det 6/6, sto 6/6).

VERDICT: FAIL on gate (leg-1 duty, flat-rise stall, lower falls) but
NOT the pre-registered dead-line (the fingerprint changed instead of
reproducing identically) — anchor-bleed is a real, partial mechanism.
Follow-up per pre-registration: `cw-stand-anchorstate2`, ONE axis
(`bc_anchor_lookahead_s` 0.25→0.5) to restore flat-rise drive while
keeping the state-locality that fixed leg 4 — already launched,
result pending. `hard1` stays deployed; do not warm-start or deploy
from anchorstate1.

## `cw-stand-anchorstate2` (08-11) — lookahead dose 0.25→0.5s: axis EXHAUSTED for the park, leg-1 fingerprint isolated

One axis vs anchorstate1: `train.bc_anchor_lookahead_s` 0.25 → 0.5,
doubling how far ahead the state-aligned anchor pulls (still clamped
at the reference path's planted-tail end, so it should not reopen
the belly-path lifted-leg leak that caused the original clock-anchor
cheat).

Result: **both anchorstate1 regressions fixed exactly as
hypothesized.** Det flat rise restored to 1/1 (was 0/1 stalled 62mm
short). Det lower falls 3→0 (still short of the target: 2/6 success,
sto 4/6, zero falls — a shortfall, not instability). Crouch rise
stayed clean (det 4/4, sto 3/3), bridge 1/1. Leg 4 stays recovered
(det-hold duty 0.95). Hold overall: det 6/6 + sto 6/6 success,
valid_plant 11/12 (clears the ≥10/12 gate clause).

**Leg 1 still parks: det-hold per-foot duty
`[0.99, 0.03, 0.98, 0.96, 0.95, 0.98]`** — the SIXTH consecutive run
with that exact fingerprint on that exact leg, unmoved by four
pricing changes (start-mix, dose, reward-pricing/feet-load) and now
two anchor lookaheads (0.25s, 0.5s). `valid_plant` reads True anyway
(blind to per-foot duty mid-episode — the leg drifts back down at
episode end).

VERDICT: FAIL on the leg-1 duty clause and the lower-success clause,
but this is the pre-registered "leg-1 persists + flat rise restored"
branch verbatim — the lookahead axis is EXHAUSTED for the park, not
inconclusive. Per pre-registration, the next lever is the one
documented incentive gap not yet attacked: the LOWER bank's own
strict xfail (`rise_posture_gate` prices one-leg-aloft at pf=5/6, so
it keeps ~85% of honest lower income) — leg-1's hold-park and
det-lower's dangling-leg shortfall share the same class. `hard1`
stays deployed; anchorstate2 is otherwise the strongest unified-stand
checkpoint yet (crouch+flat+bridge rise, six-foot-minus-one hold,
zero falls anywhere det) but does not clear the gate to replace it.

Follow-up `cw-stand-loweranchor1` (`train.bc_anchor_lower=1.0`: BC
supervision on LOWER ticks toward the lower bank's own honest
demonstration — a per-tick `FixedFootBodyIK` descent anchored at the
settled stance, body at the next commanded height; tests pin
default-off, IK-exact emission, and a feet-planted chained descent,
`rl_move/tests/test_bc_anchor.py`, all pass) **RAN: LOWER SOLVED, hold
park REGRESSED — outside every pre-registered branch, and it names
the next mechanism.** The IK-descent anchor delivered det lower 6/6
AND sto lower 6/6 (from 2/6, zero falls anywhere det) — the lower-bank
xfail lever works exactly as specced, sitting/lowering is done. But
det-hold duty flipped BACK to a two-leg park ([1.0, 0.02, 0.89, 1.0,
0.02, 0.91]) — leg 4 was recovered (0.93–0.95) in anchorstate1/2, now
parks again, and det flat rise re-stalled 96mm short (worse than
anchorstate1's 62mm). This REFUTES the "independent mechanisms"
PARTIAL-branch premise: the park did move, but WITH the lower-anchor
mix, not because of a shared taught habit. Root cause: all three
per-mode anchors (rise/hold/lower) share ONE ring buffer and ONE MSE
step with uniform sampling, so each mode's effective supervision
strength is proportional to its emission share — adding thousands of
lower pairs diluted the rise/hold gradient. **ANCHOR DILUTION** is a
new, directly testable mechanism (not another blind pricing/dose
retune). Follow-up (ran as -r1 — outcome in the next section): `cw-stand-anchormix1`
(`train.bc_anchor_stratified=1.0`, mode-tagged ring buffer with equal
per-mode minibatch quotas, everything else frozen at loweranchor1) —
if stratified sampling recovers hold+rise while keeping lower's 6/6,
the unified stance line is SOLVED; if the park still moves with the
mix even at equal quotas, dilution is wrong/incomplete and the next
step is inspecting `train/bc_anchor_loss` per mode. hard1 stays
deployed meanwhile; `ppo_goal_cw_stand_loweranchor1` is the strongest
lower-specialist checkpoint to date if one is ever wanted standalone.

## cw-stand-anchormix1-r1 (08-11 late): stratified quotas — FAIL, dilution incomplete

The dilution fix ran clean (first launch crashed on a warm-start
`_bc_mode` pickle gap, fixed in bc_anchor.py; -r1 trained the full 2M,
`train/bc_anchor_loss` converged 0.033→0.012, buffer full at 131k).
Gate result (harness det+sto, DR0, videos watched):

- RETAINED: lower 6/6 det + 6/6 sto (loweranchor1's win kept under
  equal quotas), det crouch rise 4/4 valid_plant, zero falls in all
  36 episodes, no non-park cheat.
- STILL BROKEN: det flat rise stalls 105.6mm short (height+footprint
  fail — splayed low the whole strip, an under-drive not a cheat);
  det hold parks ONE foot (duty 0.02 vs 0.90–0.99 on the other five;
  `env/hold_feet_factor` pinned at ~0.14 the entire run vs holdbc1's
  ~1.0); hold det+sto valid_plant 9/12 (three sto 'current' fails).
- THE TELL: the park MOVED. Both legs that parked in
  crouchrise1/2/3/holdload1 (and the one that persisted through
  anchorstate1/2) now read duty 0.90–0.97; a DIFFERENT single leg
  parks. Which foot rests is a function of the anchor configuration,
  not a fixed learned habit — the seventh distinct anchor/pricing
  configuration to produce a one-or-two-foot park, each time
  "solving" the previous fingerprint.

Verdict: pre-registered FAIL branch — anchor dilution was real (the
quotas did protect lower + crouch rise) but is NOT the whole
mechanism. The branch's mandatory next step applies: per-mode
`train/bc_anchor_loss` logging (CODE — only the aggregate exists
today) before any further stand arm, so the next hypothesis is chosen
on measurement. hard1 + specialist handoff stays the deployed stance
stack.

## `cw-stand-riserock2-r1` (08-12) — rise-rock DR: matched-parent null, both sides fail

Separate blocker from the park/hold-cheat lineage above: the 08-11
bench found the real belly-curl rocks 10°+ and trips `tilt_roll`
5/5 at the same tick while sim's own curl stays ≤1.7°. `dr.rise_rock_*`
(commit 36076a6) gives rise-mode episodes a persistent one-side
hip/knee fold bias on the physical servo command (same fold→roll
mapping as tipped-start/walk-kick) with the tilt reference kept
LEVEL, so leveling out is paid. Config: hard1 + `rise_rock_prob=0.5`,
`rise_rock_deg=6,12` (half the episodes rocked, half nominal).

Gate (pre-registered, matched-parent): `eval_checkpoint
--cfg-set dr.rise_rock_prob=1.0 --cfg-set dr.rise_rock_deg=10,10
--baseline hard1`, det — the exact bench trip threshold, guaranteed
every episode. Result: **child rise 0/6 valid_plant (1/6 tilt_roll
fall, 5/6 stall well short of plant height/footprint); hard1 under
the IDENTICAL injection ALSO 0/6 valid_plant (2/6 tilt_roll falls,
4/6 stall).** Zero separation — video confirms the identical
fingerprint both sides: rises partway, then tips and settles splayed
to one side. Own-distribution retention (prob 0.5, deg 6-12, as
trained) is clean and matches hard1's own probe: det rise/hold/lower
6/6, end_posture 6/6 — no regression anywhere; the failure is
specific to the guaranteed near-threshold dose, not a general
breakage of the checkpoint.

Reading: unlike `cw-dep-tip1-kick1` (walk-mode dynamic roll-kick,
ALSO zero separation from its frozen parent but in the OPPOSITE
direction — both sides pass clean), rise-rock's null has both sides
FAIL. Two command-bias roll-injection axes now show zero measurable
learned effect vs a frozen parent, in opposite directions. Neither
looks like "training worked, just needs more"; both look like this
DR family (temporarily bias the commanded fold to fake a body roll)
doesn't give PPO a learnable, generalizable signal at the doses
tried. Per the gate's own pre-registered FAIL-A branch, one gentler
retry (`dr.rise_rock_deg=6,10` or `dr.rise_rock_prob=0.3`) is queued
before fully closing the axis — but do not schedule a third dose
after that miss; the remaining lever for both takeoff/rocking
transients is contact/pinning modeling (no-skate feet), not more
command-side perturbation. `hard1` remains deployed; do not warm-start
or deploy from `cw-stand-riserock2-r1`.

### `cw-stand-riserock3` (08-12) — gentler dose: LOWER regresses into a
### fresh flag-leg/outrigger cheat; family CLOSED

The gate's own pre-registered gentler retry (`dr.rise_rock_deg=6,10`,
same `prob=0.5`, warm from `hard1` — narrower band so the 10° bench
threshold sits at the edge of training exposure instead of the
middle). Own-mix retention pass (det, the run's own trained
distribution): **LOWER collapsed from riserock2-r1's clean 6/6
(worst foot clearance ≤46 mm, under the 60 mm bar) to 1/6 (worst
clearance up to 126 mm on 5/6 episodes).** Video confirms a NEW
three-leg flag-leg/outrigger cheat, not present in riserock2-r1:
`duty_cycle` is bimodal every episode — legs 1/3/5 plant hard
(≈0.9, clearance ≈0 mm) while legs 0/2/4 stay splayed in the air
(duty 0.05–0.14, clearance 10–126 mm) — the robot props itself on a
tripod of three legs instead of tucking all six. This is exactly the
"flag-leg/outrigger cheat" class the LOWER MDP_PREFLIGHT ordering
already names; per the known-exploit rule this is a complete verdict
with no forensics required. It breaks the gate's own mandatory
"nominal rise/hold/lower retention, no duty regression" clause
outright, so the arm fails regardless of how the rise-rock injection
performed in isolation (own-mix det rise 5/6, zero falls, roll tail
<2° — looked fine, but moot once retention broke).

Reading: two dose points now tested (riserock2-r1's harder
prob=0.5/deg=6–12, this arm's gentler prob=0.5/deg=6–10) — one shows
zero learned separation from a frozen parent, the other actively
introduces a new cheat on an unrelated protected skill. Neither
result supports the "randomize a temporary body-roll bias during
rise" mechanism. Per the two-miss rule, **the rise-rock DR family is
CLOSED** (joins walk-kick, the walk-mode sibling, also closed on
zero separation) — do not schedule a third dose or a prob-only
variant. `hard1` remains deployed; do not warm-start or deploy from
`cw-stand-riserock3`. The remaining lever for the hardware
belly-curl rocking gap (bench: 5/5 tilt_roll trips at 10.1–10.6°,
sim curl ≤1.7° without the axis) is contact/pinning modeling
(belly/foot contact geometry, no-skate feet), not more command-side
perturbation.

### 08-12 park audit (`cw-stand-margin1`/`cw-stand-transdrag1` dig-in):
### the park is INVISIBLE to joint-space supervision — foot-z anchor landed

Both 08-12 pricing arms (support-margin on rise, trans-drag on
stand/sit) FAILED and both re-awakened the idx1 hold park (duty
0.03–0.05 vs the frozen parent's 0.90) — third and fourth independent
pricing terms to find parking as the escape valve. The pre-registered
anchor-side spec pass then ran on train-0 (live hard1 cfg):

1. **`_q_nom` (the hold anchor reference) is CLEAN**: 48/48 hold
   resets settle with all six feet loaded 3.2–3.6 N (none <0.5 N).
   "The anchor teaches the park" is falsified in its literal form.
   Suggestive: feet 1/4 are the two lightest at settle (3.19 vs
   3.57 N) — the only two feet any park in six runs ever chose; the
   park picks the foot that is cheapest to unload.
2. **The anchor cannot SEE the park**: rolling the parked margin1
   policy through a det hold and scoring per-leg action MSE against
   its own anchor target gives the parked leg 0.0032 vs the clean
   parent's 0.0031 on the same leg — not even the worst of the six.
   A mm-scale hover is fractions of a degree of hip lift, 3 dims of
   18: ~1e-4 of joint MSE. Six anchor variants "converged" while the
   park persisted because the supervision signal was geometrically
   blind, not because PPO defied it.

Fix (landed 08-12, `rl_move/sim/bc_anchor.py`):
`train.bc_anchor_foot_z` (+ `bc_anchor_foot_z_mm`, default 10) — an
additional anchor term on commanded FK foot HEIGHTS (differentiable
torch twin of `body_ik.fk_all_feet`; z = −FEMUR·sin(hip) −
TIBIA·sin(hip+knee), yaw-free). A 10 mm commanded hover costs ~1.0
at default scale. Default 0 = off, update bit-exact (test-pinned);
FK matches body_ik to 1e-6; the park/joint-MSE visibility ratio ≥50x
is a bank test. First arm: `cw-stand-footz1-r1` (2M discovery, one
variable vs holdbc1-hard1).

### `cw-stand-footz1-r1` (08-12) result — PASS (partial): the fix works

Det hold: **all six feet duty 0.92–0.98 across all 6 episodes**
(the frozen parent `margin1`, identical setup, scores 0.05 on leg
idx1 in the same test) — valid_plant 6/6, video-confirmed level
quiet stand, zero flag-leg on any of the 6 clips. This is the first
clean six-foot det hold after minfeet1/margin1/transdrag1 (and the
whole crouchrise/anchorstate/loweranchor/anchormix line before them)
all reproduced the identical one-foot park. Root-cause diagnosis
(the park is invisible to joint-space MSE, see above) is now
CONFIRMED by the fix that follows from it.

Two clauses miss narrowly, neither park-related, both roughly at the
frozen parent's own miss rate:
- sto hold valid_plant 4/6 (2/6 trip the `current_ok` sub-check,
  `max_current_a` > 2.0A in the final 0.5s tail — margin1's own sto
  hold report shows the identical check tripping 1/6). No leg drops
  below 0.26 duty in any sto hold episode (vs the park's historical
  0.02–0.05) — the park itself does not reappear stochastically.
- det rise 5/6 valid_plant (parent was clean 6/6) — one flat-start
  episode misses on `height_ok` only (a few mm short at episode end,
  same "det flat rise stalls" fingerprint this whole lineage has
  shown before), zero falls, video reads as an honest crouch-to-
  stand with no cheat.

Det lower stays at 4/6 (matches "4/6 baseline this seed" exactly).
Per-foot `end_clear_mm` shows the SAME three-legs-proud pattern
(idx 0/2/4 hovering 14–100mm, idx 1/3/5 flush) that `margin1`'s own
lower/det report shows at the same three indices with matching
magnitudes — confirmed pre-existing, not a new cheat this run
introduced. Hold drag/slip 188mm vs the parent's 159mm (+18%,
plausibly the cost of a foot that now actually loads on the ground
instead of hovering free of friction). `train/bc_anchor_footz_loss`
fell 5.1→1.3–1.5 over the 2M steps and plateaued there rather than
converging near 0 — the residual commanded hover is likely
concentrated in rise/lower reference ticks, not hold.

Verdict: the mechanism is CONFIRMED on its primary target (the
hold park). Not yet a clean champion-replacement PASS — queued a
10M hardening continuation `cw-stand-footz1-hard1` (same recipe,
more steps, evidence = this run) to see whether the rise miss and
sto-current noise clear the way bc1→bc1-hard1 and
holdbc1→holdbc1-hard1 both did on the identical discovery→hardening
pattern. `hard1` remains the deployed stance checkpoint until a
footz-lineage arm passes clean.

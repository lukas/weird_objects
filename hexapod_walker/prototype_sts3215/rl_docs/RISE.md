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

# SESSION BULK GATE — held-out joystick-session cohort (pre-registered)

Registered: 2026-08-14 ~21:0x UTC, BEFORE any cohort shard ran.
Authority: operator directive fb_20260814T205137_33f21c (Lukas via
Codex) + same-day operator focus note. Scope: sim-only, idle pod
CPUs; no hardware motion; train-10 CPUs untouched (Cursor dynrep).

## What is being decided

The hierarchical frozen-skill controller (stance specialist + tall
walk specialist behind the explicit session grammar with per-mode
re-anchor, entry-slew at walk engage, STOP routed to stance hold) is
now the PRODUCT BASELINE (operator ruling in the directive). This
cohort measures it honestly on FRESH held-out seeds at n=300 per
pass, next to both single-model distill artifacts, so the promotion
story rests on intervals instead of 12 repeated cases.

## Cohort spec (locked)

- Instrument: `rl_move.sim.eval_modeseq` `--drive-random
  --entry-slew 1.5,0.25`, grammar `rise,walk,lower,rise,walk`
  (~60 s session: REST→RISE→SETTLE→WALK_ENTRY→randomized DRIVE with
  guaranteed stop-go + direction flip→STOP_SETTLE→LOWER→RISE→DRIVE),
  speed band 0.05–0.06 m/s, drive 14 s/walk segment.
- Candidates (matched seeds AND therefore matched joystick
  schedules; entry-slew ON for all three — it is the deploy-side
  switch, policy-agnostic):
  - `spec` (PRODUCT): stand=`ppo_goal_cw_stand_footlow2_hard1` +
    walk=`ppo_goal_cw_dep_bcgait1_hard1` (own-cfg
    `goal.walk_obs_body_vel=2`).
  - `td2`: `ppo_goal_cw_gru_dual_bc_transdagger2` single model.
  - `td3`: `ppo_goal_cw_gru_dual_bc_transdagger3` single model
    (diagnostic).
- Volume: 50 shards × 6 episodes = 300 deterministic + 300
  stochastic sessions per candidate (1,800 total). Cold-start kinds
  rotate flat/bridge/crouch, exactly balanced (100/100/100 per pass).
- Held-out seed banks, NEVER used by any prior eval (history uses
  seeds 0/1): det = 900000..900049, sto = 910000..910049 (shard
  seeds; `--torch-seed` = shard seed makes sto reproducible).
  Manifest sha256 (cohort `c1`, test-locked deterministic plan):
  `2591cc8a57941447451f81f3f88d6354e9e21b6b64af5ff72d9735c297e57bd9`
- Runner: `rl_move.sim.bulk_session_eval` (plan/worker/aggregate/
  rerender; resumable, shard-skip idempotent, no video in bulk;
  tests `rl_move/tests/test_bulk_session_eval.py`).

## Pre-registered readout + decision rules

1. **Product adoption (spec arm):** PASS if det complete-session
   zero-fall ≥ 0.95 AND every det segment-type success rate ≥ 0.95
   AND every det cold-start stratum zero-fall ≥ 0.90 (all reported
   with Wilson 95% CIs). On PASS: the hierarchy is packaged in the
   runner as the product baseline and joystick usability STOPS
   BLOCKING on one-policy consolidation (deploy/bench remains
   operator-only). On FAIL: the failing boundary (specific
   transition/stratum) becomes the named training target — train
   ONLY that transition/residual with the skills frozen.
2. **Weakest link naming (sto):** the minimum sto stratum and
   minimum sto segment are named in the verdict as the next
   training target (12-ep pilot predicts: in-sequence RISE,
   over_current/tilt). First-rise vs post-lower-rise reported
   separately.
3. **Hierarchy vs single models:** if spec's zero-fall CI lower
   bound exceeds td2's CI upper bound (per pass), single-model
   consolidation is quantified as not-product-ready; any later
   single-model arm must use structural parameter isolation
   (operator ruling in the directive), not shared-core sweeps.
   If td2 is within CI of spec on every clause, that is reported
   as-is (no promotion consequence without operator).
4. **Early stop:** a candidate below 0.40 zero-fall after ≥100
   episodes may be stopped (spec always runs to 300). (Bulk
   parallel dispatch may finish before any look — then moot.)
5. **Zero-command creep:** stop_settle stats reported per arm;
   regardless of value the runner keeps STOP→stance-hold (measured
   fact, hw/STATUS.md 08-14: walker creeps ~0.04 m/s at zero cmd).
6. **No pooled number hides a skill:** verdict quotes per-segment
   rates + fall reasons + visual-quality medians (slip/m, drive
   height, switch tilt) alongside zero-fall; failures re-rendered
   (every failure + 12 random clean sessions per pass) and watched
   before any PASS is reported as good news.
7. **Seed retirement:** once aggregate.json is read, seed banks
   900000..900049 / 910000..910049 and their schedules are RETIRED
   for tuning — no arm may train or model-select against them;
   future cohorts bump the bases.

## Artifacts

`logs/bulk_session/c1/aggregate.json` + `episodes.jsonl` +
`rerender/` (controller copies; shards on train pods), verdict in
hw/STATUS.md + STATUS.md headline, RL_LOG line citing
fb_20260814T205137_33f21c.

---

## RESULTS (2026-08-14 ~21:4x UTC — read AFTER the cohort completed; seed banks now RETIRED per clause 7)

All 300 shards ran (1,800 sessions), zero shards failed/missing;
manifest sha256 verified == the pre-registered hash. Full numbers:
`logs/bulk_session/c1/aggregate.json` (+ `episodes.jsonl`); failure
re-renders + clean samples: `logs/bulk_session/c1/rerender/strips/`
on train-1 (spec), train-2 (td2), train-3 (td3).

**Clause 1 — product adoption: PASS (mechanically checked).**
`spec` det complete-session zero-fall **290/300 = 0.967, CI
[0.940, 0.982]** (≥0.95 ✓); det segments rise 0.983 / walk 1.000 /
lower 1.000 (all ≥0.95 ✓); det strata flat 0.95 / bridge 0.99 /
crouch 0.96 (all ≥0.90 ✓). clean_session == zero_fall (no non-fall
segment misses hid anywhere). Walk gait_valid 590/590, slip/m med
1.75, drive height med 135 mm, switch tilt med 1.8°. Notably ALL 10
det failures are POST-LOWER rises (first rise 300/300 across all
three cold starts); reasons 7 tilt_roll / 2 tilt_pitch /
1 over_current.

**Clause 2 — weakest link (sto): the in-sequence RISE, specifically
post-lower.** spec sto zero-fall 256/300 = 0.853 CI [0.809, 0.889];
rise segment 0.852 (first 0.903, post-lower 0.801); worst first-rise
stratum crouch 0.84; fall reasons rise:over_current 30 /
tilt_pitch 10 / tilt_roll 4. Walking + lowering are NOT the problem
at n=300: walk 552/552, lower 296/296, zero drive falls under
stop-go + direction flips in 1,104 spec drive segments (det+sto).
Named next training target: the post-lower stochastic rise
(over_current-dominated), skills frozen, transition/residual only.

**Clause 3 — hierarchy vs single models: separated.** sto: spec CI
lower 0.809 > td2 CI upper 0.705 and > td3 upper 0.746 — DECISIVE
both. det: spec lower 0.940 > td3 upper 0.929 (separated); vs td2
upper 0.946 marginal overlap (not separated at 95% on det alone).
Single-model quality gaps the pooled numbers hid: td2 det
clean_session only 0.597 (its crouch cold rise finishes short
0/100 — the 12-ep read "5/12" resolves into a clean stratum split:
flat 100/100, bridge 96/100, crouch 0/100), both td2/td3 walk
~115-117 mm drive height (vs spec 135 mm, visibly lower posture),
sto first-rise collapses (td2 0.233, td3 0.320). Consequence per
pre-registration + directive: single shared-policy consolidation is
research, NOT a product blocker; any future single-model arm needs
structural parameter isolation.

**Clause 4 — early stop:** moot (bulk parallel finished in ~3 min).

**Clause 5 — zero-command creep: confirmed at scale.** stop_settle
< 0.02 m/s in **0 of 2,773** drive segments across all arms (spec
med 0.036-0.040 m/s; td2/td3 ~0.05). The runner's STOP→stance-hold
switch is mandatory; "walk policy at zero command" is not a stop —
now a measured fact at n=1800.

**Clause 6 — eyes:** every failure (719 episodes) + 12 random clean
sessions re-rendered with frame strips; reviewed samples (spec
post-lower-rise falls, td2 crouch stall, clean sessions) documented
in hw/STATUS.md.

**Verdict: the hierarchical frozen-skill controller
(footlow2_hard1 + bcgait1_hard1 + explicit session grammar +
entry-slew + STOP→stance-hold) is the measured PRODUCT BASELINE at
n=600 held-out sessions — det 0.967 / sto 0.853 complete-session
zero-fall — and the single boundary that needs work is the
post-lower stochastic rise.** Bench promotion of the pair stays
operator-owned.

---

## Cohort c2 — pre-registered gate for `cw-stand-postlower1`

Registered: 2026-08-14 ~22:2x UTC, BEFORE the arm trained (snapshot
`exp/cw-stand-postlower1` 950e496 + the bank commit). Arm: the c1
verdict's named next lever — rise-mode training with POST-LOWER
start-state exposure. Mechanism: `goal.rise_start_bank` (new,
default-off) samples harvested settled lower-endpoint poses of
`footlow2_hard1`'s OWN lower skill
(`park_banks/footlow2_hard1_lower_endpoints.npz`: 300/300 settled,
0 falls, seed 5000; knees sit ~+113 deg and hips ~-18 deg from the
flat-zero pose rise training saw before — the state family was
literally unseen) as the start of 35% of rise episodes. Everything
else is the footlow2_hard1 recipe unchanged (skills frozen: no
walk/hold diet changes; walk ckpt untouched).

- Instrument + grammar identical to c1 (`bulk_session_eval`,
  entry-slew on, `rise,walk,lower,rise,walk`).
- FRESH held-out shard-seed banks (c1's 900000../910000.. are
  RETIRED): **det = 920000..920049, sto = 930000..930049.**
- Candidate: `spec-pl` = new stance ckpt
  (`ppo_goal_cw_stand_postlower1`) + `bcgait1_hard1` walk (frozen).
  Baseline numbers = c1 spec (parent stance): det 0.967
  [0.940, 0.982]; det post-lower rise 290/300 (0.967); sto
  post-lower rise 0.801 CI [0.752, 0.842]; sto zero-fall 0.853.
- **PASS iff ALL of:** (1) sto post-lower rise ≥ 0.90 AND its
  Wilson 95% CI lower bound > 0.842 (parent's CI upper — true
  separation, not noise); (2) det complete-session zero-fall ≥ 0.95;
  (3) det post-lower rise ≥ 0.967 (no det regression); (4) det
  first-rise strata (flat/bridge/crouch) each ≥ 0.95 AND lower
  segments ≥ 0.99 (cold-rise/lower retention — the crown jewels);
  (5) the standard stance gate + eval_session hard gates pass
  (watcher auto-eval), with roll_tail/drag/slip quoted vs
  footlow2_hard1 (no visual-quality regression).
- **FAIL** if any clause misses. Two data-mix/exposure misses on
  this boundary = change mechanism, not dose (RESEARCH_RULES).

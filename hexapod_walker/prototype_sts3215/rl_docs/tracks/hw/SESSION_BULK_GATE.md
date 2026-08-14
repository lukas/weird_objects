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

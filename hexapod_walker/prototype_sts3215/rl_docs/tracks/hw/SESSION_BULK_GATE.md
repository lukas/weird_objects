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

### RESULTS (2026-08-14 ~22:5x UTC — read AFTER the cohort completed; banks now RETIRED per clause 7)

**FAIL — REGRESSION, not a trade-off.** 100 shards (11 idle pods,
`--cohort c2 --cands spec-pl`), 0 missing, 600 sessions total.
`logs/bulk_session/c2/aggregate.json`, `episodes.jsonl`, rerender
sample (6 fails + 4 clean) in `rerender/strips/` (video-reviewed:
no exploit — `over_current` fails show the robot genuinely stuck
straining against the deep-knee bank pose, same failure mode as
before, not a reward hack).

| clause | need | got | verdict |
|---|---|---|---|
| (1) sto post-lower rise | ≥0.90, CI lower >0.842 | **0.7167** [0.663,0.765] | **FAIL — worse than parent** (CI upper 0.765 < parent's own CI lower 0.752: real separation the WRONG way) |
| (2) det session zero-fall | ≥0.95 | 0.9233 [0.888,0.948] | FAIL (parent 0.967) |
| (3) det post-lower rise | ≥0.967 | 0.9358 [0.902,0.959] | FAIL (regression) |
| (4) det first-rise strata / lower | ≥0.95 / ≥0.99 | flat 1.00, bridge 0.96, crouch 1.00 / lower 1.00 | pass (crown jewels intact) |
| (5) visual stats vs parent | no regression | det hold drag 623mm vs parent 136mm (worse); sto rise roll_tail 2.3° vs parent 0.7° (worse); walk untouched (slip 1.74, height 135mm — unchanged, walk ckpt frozen) | FAIL (drag/roll-tail both worse) |

Only clause (4) passes. Clauses (1)(2)(3)(5) all miss, and (1) is a
genuine regression below the parent's own confidence interval, not
noise or a null. **35% exposure to the harvested post-lower bank
made the exact skill it targeted WORSE, plus a small hit to
det-session/cold-rise retention and hold/rise drag.** Two live
hypotheses this run cannot separate: DOSE (0.35 too aggressive,
diluting the general rise skill) vs MECHANISM (the fixed
`rise_ref_track` reference is shaped for flat-topology starts and
mis-prices the escape motion from this pose family regardless of
dose). Verdict + full reasoning: `rl_docs/runs/cw-stand-postlower1.md`.

**Next:** `cw-stand-postlower2` (discovery, 2M steps, frac 0.15,
same recipe otherwise) launched same cycle to separate the two
hypotheses cheaply before any further hardening or reward-side
change. Product baseline is UNCHANGED (still the c1 hierarchy);
this FAIL does not touch it.

---

## Cohort c3 — pre-registered gate for the postlower3 lineage (in-context sequence training)

Registered: 2026-08-15 (idle-kick cycle), BEFORE the arm trained
(snapshot tag `exp/cw-stand-postlower3`). Arm: `cw-stand-postlower3`
(discovery, 2M) — the c2 dig-in's named mechanism change. The
cold-spawn exposure class (postlower1/2, `goal.rise_start_bank*`) is
CLOSED by two misses + the impossible-target root cause; this arm
trains the transition IN CONTEXT instead: `goal.mode_seq_stance=0.5`
(new default-off key, stance-only grammar rise→hold→lower→rise on the
joint_goal task, tests `test_mode_seq_stance.py`) gives half of all
episodes an in-episode mode sequence — a lower-first sequence IS the
post-lower rise with real transition context (warm policy state,
canonical per-family re-anchor, blend window), and the mid-sequence
rise target is anchored at the sequence's OWN commanded stand height
(`_seq_stand_z`), mechanically reachable by construction (the c2 bug
class is impossible here; locked by
`test_lower_to_rise_targets_remaining_rise`). Everything else is the
footlow2_hard1 recipe unchanged (warm from footlow2_hard1; walk ckpt
frozen and untouched).

- Instrument + grammar identical to c1/c2 (`bulk_session_eval`,
  entry-slew on, `rise,walk,lower,rise,walk`, 300 det + 300 sto).
- FRESH held-out shard-seed banks (c1's 900000../910000.. and c2's
  920000../930000.. are RETIRED): **det = 940000..940049,
  sto = 950000..950049** (`COHORT_SEED_BASE["c3"]`).
- Candidate: `spec-pl3` = `ppo_goal_cw_stand_postlower3` +
  `bcgait1_hard1` walk (frozen). Baselines = c1 spec (parent stance):
  det 0.967 [0.940, 0.982]; det post-lower rise 0.967; sto post-lower
  rise 0.801 CI [0.752, 0.842]; sto zero-fall 0.853.
- **PASS iff ALL of (same clauses as c2):** (1) sto post-lower rise
  ≥ 0.90 AND Wilson 95% CI lower bound > 0.842 (true separation
  above the parent); (2) det complete-session zero-fall ≥ 0.95;
  (3) det post-lower rise ≥ 0.967 (no det regression); (4) det
  first-rise strata (flat/bridge/crouch) each ≥ 0.95 AND lower
  segments ≥ 0.99 (crown jewels); (5) standard stance gate +
  eval_session hard gates pass, roll_tail/drag/slip quoted vs
  footlow2_hard1 (no visual-quality regression — the c2 623mm hold
  drag counts as a FAIL here even if counts pass).
- **Discovery framing:** the arm is a 2M mechanism probe. Full PASS
  on c3 = a promotion-grade candidate (proceed to bench wait). A
  PARTIAL (sto post-lower rise separated ABOVE parent but short of
  0.90, retention clean) = mechanism confirmed → one 6M hardening
  re-run of the identical recipe, judged on a fresh cohort c4
  (bases bump per clause 7). Post-lower rise ≤ parent OR any
  retention/visual clause broken = the in-context class misses its
  first shot; per two-miss discipline the NEXT change must be
  mechanism-level again (e.g. sequence-RSI mid-lower spawns or
  reward-side rise pricing), not a dose/diet resweep.
- Banks retire the moment aggregate.json is read, pass or fail
  (clause 7 convention).

### Cohort c3 — RESULTS (2026-08-15; dig-in verdict at the end of this section)

n=600 (300 det + 300 sto), `spec-pl3` only (parent `spec` baseline
numbers reused from c1, not re-measured), banks 940000../950000..
now RETIRED. Full aggregate: `logs/bulk_session/c3/aggregate.json` +
`episodes.jsonl`; failure/clean re-renders (13/14 shards, one
timed out mid-render, easily finished): `logs/bulk_session/c3/
rerender_pulled/`.

- **Clause (1) sto post-lower rise ≥0.90, CI lower >0.842: FAIL.**
  Measured 0.6305, CI [0.5741, 0.6836] — WORSE than the c1 parent's
  0.801, and worse than postlower1's 0.717 miss.
- **Clause (2) det complete-session zero-fall ≥0.95: FAIL, badly.**
  Measured **0.4133**, CI [0.359, 0.4698] — vs parent 0.967. This is
  not a marginal miss, it's a collapse.
- **Clause (3) det post-lower rise ≥0.967 (no regression): FAIL,
  badly.** Measured **0.4189** (124/296) vs parent 0.967 — over
  half of all det post-lower rises now fail, where the parent almost
  never does.
- Clause (4) first-rise strata + lower retention: PASSES cleanly —
  det first rise 0.9867 overall, every stratum ≥0.96 (flat/crouch
  1.0, bridge 0.96); lower segments 1.0 det+sto. The COLD-START rise
  (the original crown jewel) and the lower segment are UNTOUCHED.
  The damage is isolated to the post-lower (in-context) rise —
  exactly the mechanism this arm targeted, now much worse instead of
  better.
- Fall reasons (det): `rise:over_current` 170/176 rise-segment
  falls, `rise:tilt_roll` 6 — the SAME qualitative failure mode
  named in postlower1/2 (a genuine current stall, not a new
  exploit; sto adds a few `tilt_pitch` falls too). Frame strips of
  4 det failures reviewed (`rerender_pulled/strips/fail_*`): the
  cold rise and the walk-drive segment both look like an honest,
  full-height six-leg gait — no flag-leg/tripod/park visible before
  the failure. Did not resolve pixel-exact where in the failed
  post-lower rise the stall bites (chase-cam framing changes with
  height, ordinary crop-and-eyeball couldn't localize it) — that
  localization plus the "why is this worse than not training the
  mechanism at all" question is exactly the dig-in's job.
- **Odd/notable for the dig-in:** det (0.42) does WORSE than sto
  (0.63) on the post-lower rise — backwards from the usual
  more-noise-in-sto pattern, and from this same arm's OWN training
  telemetry, whose last video reel read `rise:ok lower:ok rise:ok
  hold:ok` (i.e. the in-context sequence the arm was trained on
  reads as succeeding on-policy). That gap — trains-fine /
  held-out-collapses, with the WRONG mode (det) hit hardest — is a
  generalization-failure signature (RUN_INTERPRETATION_RULES #3),
  not a straightforward "not enough exposure" story; a plausible
  first hypothesis is a mismatch between the training env's own
  segment-transition re-anchor (mode_seq_stance, hold/rise/lower
  only, never sees a walk hand-off) and `eval_modeseq.py`'s
  `reanchor_to()` used for the real rise→walk→lower→rise chain —
  but the PARENT (footlow2_hard1) has that exact same train/eval
  mismatch and scores 0.967/0.801, so "different reanchor path" by
  itself doesn't explain it; something about mode_seq_stance
  training specifically (interference between the two 50% diets,
  or the sequence's own re-anchor logic) is the more likely culprit.
  Needs the deep-model root-cause chain before naming the next
  mechanism-level lever.
- **Verdict: FAIL by the letter of the pre-registration** (clauses
  1–3 all miss, clause 3 by a huge margin) — this is a clean THIRD
  miss for the post-lower-rise problem (postlower1, postlower2,
  postlower3 all FAIL) and, per two-miss discipline, rules out
  another simple resweep of this exact recipe. But the MAGNITUDE and
  DIRECTION of the miss (regression far below not just the gate but
  the parent and the prior misses, with det hit harder than sto) is
  surprising enough that the ledger verdict + "what mechanism comes
  next" call is being left to a dig-in cycle rather than guessed at
  triage. `cw-stand-postlower3` is deliberately left UNVERDICTED in
  the ledger pending that read. Product baseline (c1 hierarchy)
  unaffected either way — walk ckpt untouched, stance candidate not
  promoted.

### Cohort c3 — DIG-IN VERDICT (2026-08-15, deep cycle)

**FAIL — the mechanism taught a belly detour; root cause is a
reference-construction defect in `_seq_segment_traj`, found and fixed
same-cycle.** The chain, each link checked:

- **Behavior (video, re-renders):** after the lower, the policy
  RE-DESCENDS, splays flat to the belly pose, and re-runs the
  flat-rise choreography — visible in det failures
  (`fail_spec-pl3_det_s940000_ep0/ep2`, `s940001_ep1`) AND in det
  SUCCESSES (`ok_spec-pl3_det_s940009_ep0` tiles ~50-55: splay →
  curl → stand). The behavior change is global in the post-lower
  state family; failures are the >50% of det curls that stall
  (`over_current` 166/172 det post-lower falls; switch_peak_a pinned
  at 2.64 A in every fail).
- **Incentive (code):** at the lower→rise switch the rise schedule
  starts at **0 in the belly frame** (`height` zeros + 1 s hold
  before the ramp) and the blend interpolates the ref DOWN from the
  lower-end height to 0 — the reward's height tracking literally
  pays a descent to belly. Once low, the state-aligned flat-demo BC
  anchor supervises the flat-rise choreography (its whole path
  starts splayed). Both teachers price the detour; the c3 fix
  anchored the segment's ENDPOINT (`_seq_stand_z`) but left the
  STARTPOINT at belly 0 — the same bug class c2 died of, inverted.
- **Why training telemetry disagreed with the cohort:** on-policy in
  the training env the detour completes and the reward pays it
  (ref-compliance + low anchor loss → `rise:ok` reels). The
  instrument only scores end height/no-fall; the detour routes every
  post-lower rise through the max-strain curl, which over_currents
  >50% det on held-out sessions. det < sto because the deterministic
  policy fully commits to the taught detour; sto noise sometimes
  breaks it into the parent's direct push-up basin.
- **Why worse than the parent:** the 50% sequence diet actively
  overwrote the parent's direct push-up in exactly this state family
  (the parent never trained there and generalizes fine, 0.967).
  Cold rises/lower untouched (clause 4 passed) because the flat demo
  IS the right teacher for cold starts.
- **Controls:** standard DR-0 gate (single-mode cold spawns) is
  clean — hold/rise/lower det+sto 35/36 ok, no terms — confirming
  the damage is confined to the in-context transition.

**Fix (landed this cycle, snapshot `exp/cw-stand-postlower4`):**
`goal.mode_seq_rise_from_h` (default 0 = bit-exact legacy; tests
`test_rise_from_h_starts_at_current_height` + suites green) — with
the key on, a mid-sequence rise schedule starts AT the robot's
current height above the just-installed belly frame ("stand up from
where you ARE": hold at h_now, ramp h_now→amp), never commanding the
belly descent. rng streams unchanged. NOT a dose/diet resweep — a
construction-defect fix in the same mechanism class, per the c3
pre-registration's FAIL branch. CROSS-TRACK: the shared
`goal.mode_seq` (walk-task grammars, arch) has the identical
descent-commanding rise branch — escalated in arch/STATUS.md, no
arch launch from here.

---

## Cohort c4 — pre-registered gate for `cw-stand-postlower4` (rise-from-height fix)

Registered: 2026-08-15 (dig-in cycle), BEFORE the arm trains
(snapshot tag `exp/cw-stand-postlower4`). Arm: `cw-stand-postlower4`
(discovery, 2M) = the postlower3 recipe with exactly ONE change:
`goal.mode_seq_rise_from_h=1` (mid-sequence rise schedules start at
current height — no commanded belly descent). Everything else
identical (mode_seq_stance=0.5, warm from footlow2_hard1, walk ckpt
frozen).

- Instrument + grammar identical to c1/c2/c3 (`bulk_session_eval`,
  entry-slew on, `rise,walk,lower,rise,walk`, 300 det + 300 sto).
- FRESH banks (c1-c3 bases RETIRED): **det = 960000..960049,
  sto = 970000..970049** (`COHORT_SEED_BASE["c4"]`).
- Candidate: `spec-pl4` = `ppo_goal_cw_stand_postlower4` +
  `bcgait1_hard1` walk (frozen). Baselines = c1 spec (parent stance):
  det 0.967 [0.940, 0.982]; det post-lower rise 0.967; sto
  post-lower rise 0.801 CI [0.752, 0.842].
- **PASS iff ALL of (c2/c3 clauses + one new eye clause):** (1) sto
  post-lower rise ≥ 0.90 AND CI lower > 0.842; (2) det session
  zero-fall ≥ 0.95; (3) det post-lower rise ≥ 0.967; (4) det
  first-rise strata each ≥ 0.95 AND lower ≥ 0.99; (5) stance gate +
  eval_session hard gates pass, roll_tail/drag/slip vs
  footlow2_hard1; **(6) watched post-lower-rise re-renders show a
  DIRECT push-up — any splay-to-belly detour on a post-lower rise is
  a FAIL regardless of counts** (the c3 lesson: counts can pass while
  the behavior is a taught detour).
- PARTIAL (sto post-lower rise CI-separated ABOVE parent but < 0.90,
  retention + eye clauses clean) = mechanism confirmed → one 6M
  hardening rerun on fresh cohort c5.
- FAIL (post-lower rise ≤ parent or any retention/visual/eye clause
  broken) = SECOND miss of the in-context class → the class is
  closed for dose/diet AND schedule fixes; next lever must be a
  different mechanism entirely (align the instrument/runner rise
  schedule to remaining-rise semantics so train==deploy — an
  operator-facing product-contract question — or reward-side rise
  pricing), taken to the operator.
- Banks retire on aggregate read, pass or fail.

### Cohort c4 — RESULTS (2026-08-15)

n=600 (300 det + 300 sto), `spec-pl4` only (parent `spec` baseline
numbers reused from c1). Banks 960000../970000.. now RETIRED.
Aggregate: `logs/bulk_session/c4/aggregate.json` + `episodes.jsonl`;
10 watched re-renders (6 failures + 4 clean, sample-seed 0):
`logs/bulk_session/c4/rerender/strips/`.

- **Clause (1) sto post-lower rise ≥0.90, CI lower >0.842: FAIL.**
  Measured 0.6902, CI [0.6355, 0.7401] — below the c1 parent's 0.801,
  though well above c3's 0.6305.
- **Clause (2) det complete-session zero-fall ≥0.95: FAIL.** Measured
  0.8633, CI [0.8198, 0.8976] — below parent 0.967, but a MASSIVE
  recovery from c3's 0.4133 (collapse) — no longer a collapse, a
  clean shortfall.
- **Clause (3) det post-lower rise ≥0.967 (no regression): FAIL.**
  Measured 0.8721 (259/297) vs parent 0.967 — again a big recovery
  from c3's 0.4189, but still short of parity.
- **Clause (4) first-rise strata + lower retention: PASS.** Det first
  rise 0.99 overall, every start-kind ≥0.97 (flat 1.0, bridge 0.97,
  crouch 1.0); lower segments 1.0 det+sto (n=297). Crown jewels
  untouched, same as c3.
- **Clause (5) stance gate / visual quality vs footlow2_hard1:**
  drive height 135.0/135.2mm det/sto (in-band, matches c3's 135);
  slip/m 1.71 det / 1.73 sto (in line with c3's own-cfg numbers, no
  new drag signature); the pre-staged interactive session-gate canary
  (12-ep, informational only) reads HARD no_falls/rise/sit_descends
  all PASS. No new failure mode in the fall-reason histogram
  (`rise:over_current` 33/41 det falls, `rise:tilt_roll` 6,
  `rise:tilt_pitch` 2 — same qualitative stall as every postlower
  arm, at roughly 1/4 the c3 incidence).
- **Clause (6) eye clause — watched re-renders: PASS, cleanly.** 10
  re-renders reviewed (6 of the 41 det failures + 4 clean draws,
  frame strips at `logs/bulk_session/c4/rerender/strips/`): every
  post-lower rise is a DIRECT push-up from a crouched/low posture —
  no splay-to-belly detour in ANY of the 10, failing or clean. The
  fix (`goal.mode_seq_rise_from_h`) did exactly what it was built to
  do: it eliminated the c3 detour behavior. The remaining failures
  are a straightforward over_current stall partway up the push
  (switch_peak_a pinned ~2.6A at the stall tick, matching the
  qualitative pattern of postlower1/2's cold-bank stalls) — a
  genuine actuation-effort ceiling, not an exploit.
- **Verdict: FAIL by the letter of the pre-registration** (clauses
  1–3 all miss the parent-parity bar) — this is the SECOND miss of
  the in-context sequence-training mechanism (c3 = wrong mechanism
  fully taught a detour and collapsed; c4 = right mechanism, detour
  gone, big recovery, still short of parity). Per two-miss
  discipline the class is CLOSED for further dose/diet/schedule
  resweeps of this recipe. Unlike c3, this is NOT a surprising or
  disagreeing-with-telemetry result — it is exactly the pre-
  registered "if-false" branch of the hypothesis (mechanism doesn't
  fully transfer to the runner's reanchor semantics), confirmed by
  clean, video-checked evidence — so no dig-in is needed to interpret
  it; the pre-registration already named the next step. Per the
  pre-registered FAIL branch, the next lever is an operator product-
  contract call (align the runner/instrument rise schedule to
  remaining-rise semantics so train==deploy, or price post-lower
  rise directly in reward) — escalated to `[operator]` WAITING-ON in
  STATUS.md / hw/STATUS.md, not launched. Product baseline (c1
  hierarchy) unaffected either way.

## Cohort c5rr — pre-registered "remaining-rise" semantics probe (prices the postlower `[operator]` fork)

Registered: 2026-08-17 ~22:0x UTC (idle-drain cycle), BEFORE any
shard ran. Not a promotion gate for a new arm — an EVAL-SIDE probe
that answers the exact open question the c4 verdict escalated: c4's
FAIL was attributed to a train/eval schedule MISMATCH (postlower4
was trained under `goal.mode_seq_rise_from_h=1` — the mid-sequence
rise holds at the robot's CURRENT height then ramps to the stand
target — but `bulk_session_eval`/`eval_modeseq` always reanchors the
post-lower rise with the LEGACY schedule, hold-at-belly then ramp,
i.e. it evaluates the robot on a task it wasn't trained for). If
c5rr shows c4 crossing the c1 parent's bars under the MATCHED
schedule, operator fork option (a) — "align the runner/instrument to
train==deploy semantics" — is measured-best and the pick becomes
data, not taste; if it still falls short, the mismatch was not (the
whole) story and option (b)/(c) (reward-side pricing / accept the
ceiling) stay live.

- **New code** (this cycle, tests green): `eval_modeseq.py
  --remaining-rise` (default off, bit-exact legacy when unset) —
  after the post-lower reanchor, rewrites the rise segment's height
  schedule with `remaining_rise_height()` (pure affine value remap:
  old range `[height[0]=0, amp]` -> `[h_start, amp]`, so the ramp's
  exact tick timing is preserved, no index search) where `h_start`
  is the robot's real height above the just-installed belly frame at
  the reanchor tick. Mirrors `SimHexapodGoalEnv._seq_segment_traj`'s
  `mode_seq_rise_from_h` branch (goal_task.py), applied post-hoc
  instead of in-episode. `bulk_session_eval.py`: shards carry a
  `remaining_rise` bool (`COHORT_REMAINING_RISE`), `shard_cmd` passes
  `--remaining-rise` only when set. Tests: `test_bulk_session_eval.py`
  +4 (pure-math identity/clamp check, shard-cmd opt-in, cohort
  wiring + fresh bank) — 11/11 in that bank; two 2-episode smoke
  reads on real checkpoints (`spec`, `spec-pl4`) confirm the
  mechanism fires only on post-lower rises (`remaining_rise_h_start_mm`
  present) and cold rises are untouched.
- Instrument + grammar identical to c1-c4 EXCEPT `--remaining-rise`
  is now on: `rise,walk,lower,rise,walk`, entry-slew on, speed
  0.05-0.06, drive 14s. Candidates: **`spec`** (parent,
  `footlow2_hard1`+`bcgait1_hard1`, re-read under the new schedule —
  its OWN c1 numbers are NOT reusable here since the schedule
  changed) and **`spec-pl4`** (`postlower4`+`bcgait1_hard1`, the c4
  candidate). Volume: 50 shards x 6 episodes = 300 det + 300 sto per
  candidate (1,200 total) — same n as c1/c4 so the read is directly
  comparable to their bars.
- FRESH bank (c1-c4 bases all retired): **det = 980000..980049, sto =
  990000..990049** (`COHORT_SEED_BASE["c5rr"]`).
- **Readout (no pass/fail bar — a comparative price, not a gate):**
  report, for BOTH candidates under the matched schedule: det/sto
  post-lower-rise (`rise_by_ordinal.1`) rate + CI, det complete-
  session zero-fall + CI, first-rise/lower retention (must stay
  parity — any drop here is a NEW problem, not this probe's
  question), fall-reason histogram, visual medians (slip/m, drive
  height, switch tilt), and a handful of watched re-renders of any
  post-lower-rise failure (the eye clause: confirm no NEW exploit
  from the schedule change itself).
  - If `spec-pl4`'s post-lower rise (det AND sto) is >= `spec`'s
    OWN number under this SAME schedule (not the stale c1 number),
    with CIs not both worse: **fork (a) is supported** — write it up
    for the operator as the measured answer, do not auto-promote
    (postlower4 is not yet a product change without the operator's
    sign-off on the runner/instrument contract).
  - If `spec` ALSO changes materially (up or down) from its c1
    numbers under `--remaining-rise`, that is itself a finding about
    the runner (the legacy schedule was doing SOMETHING for the
    parent too) and gets reported plainly, not smoothed over.
  - If `spec-pl4` still trails `spec` under the matched schedule,
    the mismatch was not (the whole) explanation for c4's FAIL — say
    so plainly; fork (b)/(c) stay live for the operator.
- Banks retire on aggregate read regardless of outcome.

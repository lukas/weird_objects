# STAND_HEIGHT — commandable standing-height curriculum

Operator MCP request `fb_20260825T195117_3dce6e` (2026-08-25, GPT-5 via
Codex, asked by Lukas): once the robot is in a solid standing position,
a joystick axis/button should move the body up/down to a specified
standing height. If the robot is not already solid, it should ignore
arbitrary height commands and first get into a safe solid stand. This
doc is the concrete reward/eval/curriculum target the note asked for,
scoped and reasoned through this cycle (assume-and-go, per the
standing prompt's "no operator pauses" rule) rather than left as a
vague future feature.

## Design decision: reuse HOLD, don't build a new goal kind

The note's own prior-art warning ("old height-only gates existed but
were fixed target rise/lower, not continuously commandable") and its
suggested reward shape (a supported-stand score `S` gating height
income) both already have working, PROVEN implementations in this
codebase, scoped to the `hold` goal kind:

- `reward.hold_still_gate` / `hold_feet_load` / `hold_feet_load_min` /
  `hold_flag_fade` (REWARD.md §1) already implement the feedback's `S`
  almost exactly — feet-loaded² (measured touch force, not clearance)
  × no-flag pad-spread fade × stillness, scaling hold/track's income —
  proven in the HOLD/HOLD_LOAD/HOLD_MINLOAD banks and already the
  DEPLOYED hold champion's recipe
  (`ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_
  stdanneal`, Stage-1 HOLD SOLVED per STATUS.md 08-25 ~13:1x).
- `reward.k_drag_trans` (REWARD.md §2 note) already prices loaded-foot
  slip on every non-walk tick including hold, with a PROVEN scrape
  pattern (opposing-tripod yaw sweep) and operating point (k=400) in
  the TRANS-DRAG bank.
- The base tracking kernel (`rl_move/env.py compute_reward`) already
  prices `height_err = measured_height - (start + goal.height_ref)`
  with a Gaussian income for EVERY hold/rise/lower/raise episode —
  `goal.height_ref` is already an observed, scaled channel
  (`TaskGoal.as_obs`) fed to the policy. The only genuinely missing
  piece was a `height_ref` that MOVES over a wide, joystick-like
  range instead of sitting at one fixed per-episode target.

Building a brand-new goal kind (own reward strip/replace block, own
BC-anchor wiring, own obs channel) would duplicate all of this and
introduce a second, unproven S-gate. Instead: `goal.hold_height_cmd_*`
(new keys, default OFF, `rl_move/sim/goal_task.py`
`GoalGenerator._hold_height_schedule`) makes `hold`'s height_ref
follow a scripted, rate-limited command schedule instead of the
legacy flat 0 — everything downstream (S-gate, slip charge, height
kernel, BC-anchor eligibility since `_is_hold_bc` keys on
`mode in ("hold","track")` regardless of what height_ref does) applies
unchanged. Full key table: REWARD.md §4d.

**Note on `train.bc_anchor_coef` for hold:** the deployed hold
champion trained WITH a pose-imitation BC anchor toward the fixed
settled `q_nom` (a height-BLIND target — see the "TIP-AWARE HOLD
REFERENCE" block, `sim_env.py`). That anchor would actively fight a
moving height command. The first height-cmd canary must NOT carry
`train.bc_anchor_coef` forward (simply omit the cfg-set; default 0 =
off) — the S-gate + height kernel alone should be enough signal given
the champion already nails static height=0 tracking; if this proves
insufficient, the correct fix is a height-AWARE anchor (IK-solved at
the CURRENT commanded height every tick, exactly the mechanism
`lower`'s own `train.bc_anchor_lower` already uses via
`BodyOffset(height=g_next.height_ref)`), not the height-blind one.

## What is built (this cycle)

- `rl_move/sim/goal_task.py`: `GoalGenerator._hold_height_schedule` +
  `_rate_ramp` helper — piecewise hold/ramp/sine/pulse command script,
  every transition rate-limited (`hold_height_cmd_rate_mm_s`, default
  15 mm/s), range-clipped (`hold_height_cmd_range_mm`, default
  [-40,20] mm, clipped to ± `actions.max_height_mm`), settles at 0
  before the first command (shared `ramp_s` window, no step at spawn).
  Truncating episodes slow the transition down, never speed it up.
  Default `hold_height_cmd_frac=0.0` — zero extra rng draw, bit-exact
  for every existing hold-including lineage.
- `rl_move/tests/test_hold_height_cmd.py` — generator unit tests:
  default-off bit-exact, range/rate respected (normal + short
  episodes), settle window, pinned-profile hook reaches its target,
  range clipped to the action envelope.
- `rl_move/tests/test_task_semantics.py`, "HOLD bank, COMMANDABLE
  HEIGHT variant" — the REQUIRED preflight bank (RESEARCH_RULES.md /
  the 08-21 ruling's reward<->eval-alignment demand) before any GPU
  arm: on the SAME pinned exact ramp-to-target schedule, scored under
  `HOLD_MINLOAD_ON` (today's deployed recipe),
    - `track` (six-foot `FixedFootBodyIK` solve to the CURRENT
      commanded height, fresh every tick) beats `stale` (freeze at
      the OLD height) by a wide margin, both for a down-step (-30 mm:
      2551 vs -67) and an up-step (+20 mm: 1269 vs 126) target;
    - `flagleg` / `hover` (nominally reach the same height through an
      unsafe pose) earn under half of honest tracking, several of
      them net-negative (20.8 / -23.0 at -30 mm; 5.6 / -36.8 at
      +20 mm) — the S-gate proxy still bites while height varies;
    - `skate` (loaded-foot slip during the adjustment, the trans-drag
      bank's own proven opposing-tripod scrape riding on top of
      height tracking) loses to the quiet planted adjustment under
      `reward.k_drag_trans` (2187 vs 2551 at -30 mm; 1213 vs 1269 at
      +20 mm);
    - `hold_height_cmd_frac=0` reproduces the legacy flat-height hold
      episode exactly.
  All four tests green; numbers above are the 3-seed means measured
  this cycle. Full run: `uv run pytest rl_move/tests/test_task_semantics.py -k hold_height`.
- REWARD.md §4d — the key table + bank summary.

## What is deliberately deferred (rungs 4-5)

The feedback's own fallback: "if the current rise model still cannot
reliably reach solid stand from flat, restrict early stand_height
starts to already-solid/near-solid states, then add guarded entry
once rise improves." As of this note the standwalk rise mechanism is
STILL mid dig-in (tuckfloor0/tuckexempt0/tuckrise15/45 all
FAIL-MECHANISM on the mesh-native reference's height-flat tuck
segment; the surviving lever, a script-index/commanded-height-keyed
BC-anchor progress metric, is DIG-IN flagged for the deep cycle, not
yet proven). Rungs 4 ("start non-solid, must acquire S before
tracking") and 5 ("compose with rise/lower: rise -> height-cmd
sequence -> lower") therefore stay OUT OF SCOPE until rise is solved —
funding them now would train against non-solid starts this codebase
cannot yet honestly present (no believable "shaky but recoverable"
distribution exists without the rise fix). Do this "after/alongside
the current mesh2 standwalk rise work, not as a detour" (the note's
own words) — this doc is the alongside part; rungs 4-5 are the after
part.

## Curriculum rungs (config sweep, not new code — matches this
## codebase's convention: every rung is the SAME generator, different
## cfg values)

- **Rung 0** (implicit): `hold_height_cmd_frac=0` — the existing,
  solved static hold-at-plant behavior. Sanity baseline.
- **Rung 1**: small range (e.g. `hold_height_cmd_range_mm=[-15,15]`),
  slow rate (`hold_height_cmd_rate_mm_s<=10`), `kinds=("hold","ramp")`
  only — the first thing to actually train on.
- **Rung 2**: full default range `[-40,20]` (the plant-to-lower-ish /
  plant-to-slightly-raised envelope already proven reachable by the
  isolated `lower`/`raise` champions), still ramp-only.
- **Rung 3**: full kind mix (`hold, ramp, sine, pulse`) at the default
  15 mm/s rate — a genuine "joystick up/down" replay target.
- **Rung 4** (deferred, needs rise): non-solid starts (crouch/tangle/
  park via the existing `start_at="any"` machinery already proven in
  GETUP/RECOVER) that must recover to `S>=0.85` before any height
  income pays — requires either porting GETUP's staged S-ratchot
  onto hold, or wiring RECOVER's PBRS Φ as a pre-stage gate; needs its
  own dig-in once rise is solved.
- **Rung 5** (deferred): mode-sequenced `rise -> hold(height-cmd) ->
  lower` episodes via the existing `goal.mode_seq_stance` planner
  (`goal_task.py`, already builds rise/hold/lower sequences) — a
  drop-in once rung 3 passes and rise is solved.

## First science arm (funded once the bank stays green under review;
## NOT launched this cycle — see below)

Respec of the deployed hold champion
(`ppo_goal_cw_standwalk_stance_mesh2_holdminload40_bcanchor3_
stdanneal`): warm-start FROM it, `--goal-mix hold=1.0`, `--cfg-set
goal.hold_height_cmd_frac=1.0 hold_height_cmd_range_mm=[-15,15]
hold_height_cmd_rate_mm_s=8` (rung 1, ramp-only via
`hold_height_cmd_kinds='["hold","ramp"]'`), carrying forward EVERY
other cfg key from the champion's own command EXCEPT
`train.bc_anchor_coef` (omit — see the height-blind-anchor note
above). Gate: DR-0 + own-DR(0.2) det+sto, height MAE over the
commanded schedule, zero `hold_min_load`/`walk_low_height`-style
terminations, valid_plant on every hold tick (S-gate proxy at or
above the static champion's own band). 2M canary, judged as a
mechanism-health check (does the champion's skill survive a moving
target at all?), not a skill-acquisition claim.

## Eval-gate sketch (`eval_stand_height_gate`, not yet built)

Reuse `eval_checkpoint.py`'s existing `--cfg-set` probe convention
(the same mechanism `goal.rise_flat_frac=1.0` pins a flat-only probe
today) rather than a bespoke harness: `--goal-mix hold=1.0 --cfg-set
goal.hold_height_cmd_frac=1.0` plus the standard DR-0/own-DR det+sto
sweep already gives command-schedule coverage; a dedicated gate script
becomes worth building once rung 3 needs FIXED, reproducible command
scripts (step/ramp/sine/pulse/random-hold) for cross-run comparison —
mirroring `make_rise_ref_scripted.py`'s geometry-only, no-checkpoint-
extraction philosophy. Composite success per the operator's own spec:
both solid support (S proxy) AND height tracking (MAE/p95) required;
height matching alone is explicitly not success.

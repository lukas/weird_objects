# multitask — Multitask learning

W&B: tag `track:multitask`. Excess-capacity research; run prefix
`cw-mt-`. Design + verdict labels: **rl_docs/MULTITASK.md** (read it
before triaging anything here — this track has its own binding rules
for what counts as forgetting vs acquisition failure).

**Goal:** test whether a fresh command-conditioned generalist
(stand + forward + small yaw/lateral trained SIMULTANEOUSLY, one
coherent reward) beats the sequential specialist-fine-tuning pattern —
both at zero-shot command interpolation and at acquiring a genuinely
new command later (the phase-2 transfer test).

## Now

- **Wave 1 A/B/C cohort FAILED at 2M — under-budget, not an
  acquisition/interference result (08-12 ~20:00 UTC).** All three
  arms hit the same low-crouch splay creep, video-confirmed, with a
  monotone progress ordering that tracks command-distribution width:
  `cw-mt-a1` specialist det prog med 0.22 (gait_valid 0/6, 1 leg
  sacrificed every ep), `cw-mt-b1` narrow generalist 0.16 (gait_valid
  0/6 det, 3 legs sacrificed), `cw-mt-c1` broad generalist 0.10
  (gait_valid 0/6, 3-leg sacrifice). The control arm's own
  pre-registered gate (`prog med > 0.3`) fired its FAIL(budget)
  branch, which per MULTITASK.md stops independent B/C judging —
  no yaw/lateral/interpolation verdict is possible at this budget.
  Informative residual: diversity cost is mild, not catastrophic
  (0.22 → 0.16 → 0.10), so command width alone isn't what's starving
  discovery at 2M — the donor recipe (`cw-dep-fresh1`) simply needed
  20M steps to walk from scratch, same as this cohort.
  Re-queued UNCHANGED at the matched 20M budget per the
  no-per-arm-tweaks rule: `cw-mt-a2`/`cw-mt-b2`/`cw-mt-c2` (backlog,
  `cw-mt-b2` draining first). Ledger + wandbnotes done for a1/b1/c1.
- Fresh init IS the hypothesis (warm-start default waived, recorded
  in each spec) — unchanged for the a2/b2/c2 re-queue.
- **08-12 ~21:1x: `cw-mt-a2` (specialist control arm) PASSES at 20M —
  the budget-only diagnosis is confirmed.** Own-cfg(DR0.2) det prog
  med 1.30 (gate >0.8), gate(DR0) det prog med 1.23, gait_valid 6/6
  det+sto both passes, 0 terminations. Video/contact-sheet: genuine
  six-leg cycling (per-leg duty_cycle 0.26-0.72, swing_count 7-25/leg,
  zero sacrificed legs) — real stepping, not the a1-at-2M flag-leg
  exploit. Quality caveat (not gated, but note it): slow (~0.06-0.14
  m/s vs 0.05 cmd) and drags a lot (slip_per_m ~1.4-1.5, roll stays
  clean, tail ≤1.8°) — a research control, not a deployment
  candidate. Per its own pre-registered gate this **unlocks B/C
  triage** (`cw-mt-b2`/`cw-mt-c2`, in progress by other cycles).
- **08-12 ~21:3x: `cw-mt-b2` (narrow generalist: forward+small
  yaw+stops) FAILS at 20M, informatively — real gait, but short on
  both remaining gate clauses now that `a2`'s numbers exist.** A
  genuine six-leg gait finally shows up (gait_valid 6/6 det+sto,
  both DR0 and own-DR0.2 — unlike wave-1's 0/6 paddle at 2M), but
  det prog med is only 0.51 (gate)/0.53 (own-cfg) vs the required
  >=0.5x `a2`'s 1.23/1.30 (need >=0.615/0.65) — short on both. An
  extra `eval_yaw` probe (not auto-staged; run this cycle) also
  fails: turn |wz_err| med 0.137 (gate<=0.10), 9 falls across 10
  scripted turn/hold scenarios — yaw shows *some* directional signal
  (arc-left/right differ from hold) but is unreliable, not clean
  both-directions tracking. Stop-segments hold still cleanly (0
  falls, low wz on the isolated probe). Residual worth watching:
  leg index 3 sits at duty 0.11-0.35 in every one of 24 episodes
  (gate+owncfg x det+sto), near the 0.10 auto-sacrifice cutoff,
  while `a2` (same seed/recipe, no yaw/stop) shows a healthier
  0.26-0.72 spread with no such leg — the added command diversity,
  not the recipe/seed, looks like the proximate cause. Verdict +
  full numbers: `rl_docs/runs/cw-mt-b2.md`.
- **08-12 ~21:2x: `cw-mt-c2` (broad generalist: +small sideways
  commands) FAILS at 20M, worse than `b2` — not a valid gait at
  all.** Leg 2 is a true flag leg in every one of 24 episodes
  (duty_cycle 0.01-0.03 vs 0.35-0.69 for the other five) and the
  robot falls (`term_reason=tilt_pitch`, `roll_class=fell`) in 10/12
  gate episodes and 9/12 own-cfg(DR0.2) episodes. gait_valid: gate
  det 0/6, gate sto 2/6, owncfg det 1/6, owncfg sto 2/6 (vs `a2`'s
  6/6 everywhere, 0 terms). prog-med numbers look nontrivial (gate
  det 1.01) only because progress is measured on the drag-then-fall
  trajectory, not a real walk cycle — video: near-normal 5-leg
  stance with leg 2 splayed rigid, drags forward a few tenths of a
  metre, then topples nose-first around frame 8-10 of every clip.
  Never reaches its own gate's lateral/interpolation probes (moot —
  can't clear plain forward without falling). Verdict + full
  numbers: `rl_docs/runs/cw-mt-c2.md`.
- **WAVE 1 CLOSED, decisively — command-width interference is real
  and monotonic at matched 20M budget, not a 2M budget artifact:**
  `a2` (specialist) clean pass -> `b2` (narrow generalist: +yaw
  +stops) real gait but short on speed/yaw accuracy -> `c2` (broad
  generalist: +sideways) no valid gait, flag-leg + falls. Widening
  the command set genuinely interferes with learning to walk at
  all with this recipe/architecture; it is not a "needs more steps"
  story, since `a2` proves 20M is enough for THIS recipe at narrow
  command width. No more retries of this exact cohort.

- **08-12 ~21:5x: `cw-mt-b-arch256-1` (wave-2 capacity probe: 256×256
  net, fresh init, 2M, b1's command distribution) FAILS its
  pre-registered gate — width alone is NOT the discovery lever.**
  gate(DR0) det prog med 0.11 vs b1's 0.16 baseline (needed >=0.32),
  gait_valid 0/6 det (needed >=1/6); same low-crouch splay with leg-1
  near-sacrifice in 11/12 det eps, video-confirmed. Per the FAIL
  branch: no further net-arch retry at this budget; planning stays on
  `a2`/`b2` (128×128). Next lever launched same cycle:
  `cw-mt-widen1` (staged widening — b1's command distribution
  warm-started from `a2`'s walking checkpoint, 2M discovery).

- **08-12 ~22:2x: `cw-mt-widen1` (staged widening, 2M) FAILS(acquisition)
  per its pre-registered branch, but CONFIRMS the staged-widening half
  decisively.** The `a2` walking prior fully SURVIVES command widening
  at 2M (gate(DR0) det gait_valid 6/6, prog med 1.57 vs a2's 1.23;
  own-DR0.2 6/6/1.81; zero terms/sacrificed legs, duty_min 0.23-0.44 —
  b2's leg-3 near-sacrifice absent; roll_tail 0.5-0.9° flat-to-better
  than a2; cost: slip_per_m 1.92-2.19 vs a2's 1.38-1.50). Neither NEW
  command is acquired yet: stop-hold speed_med (0.058 m/s) equals
  fwd-hold's — the policy marches through zero-commands — and yaw
  sign-response is absent both directions (tip-left/right wz
  +0.103/+0.086, wrong sign right; eval_yaw |wz_err| 0.155 vs b2's
  0.137, but 0 falls vs b2's 9). Confound: no mt arm has ever acquired
  a new command at 2M (b2 needed 20M from scratch for partial yaw), so
  this doesn't yet distinguish "budget" from "representation limit".
  Discriminator **`cw-mt-widen2`** (same recipe, continue to b2's
  matched 20M budget) queued+launched same cycle to settle it before
  reaching for the representation lever (`obs.history_frames`).

- **08-12 ~23:1x: `cw-mt-widen2` (staged widening, budget-matched 20M)
  FAILS(no-acquisition), decisively — the budget confound is CLOSED.**
  Same 20M budget that let `b2` partially acquire commands from
  scratch; the walking prior stays perfect (gate(DR0) det gait_valid
  6/6, prog med 1.54, own-DR0.2 6/6/1.68, zero terms, roll_tail
  0.4-1.0°, video-confirmed genuine six-leg cycling — no regression
  from widen1) but NEITHER new command arrives: `probe_signed_yaw`
  stop-hold speed_med 0.0417 m/s vs fwd-hold 0.0688 (gate needed
  <=0.02; >0.04 trips FAIL outright), tip-left/right yaw differential
  0.0032 (gate needed >=0.10) — weaker directional signal than
  widen1's already-failing 0.017. `eval_yaw` agrees: turn |wz_err|
  med 0.122, hold |wz| med 0.107, both over gate. **20M of the exact
  b2-matched budget does not teach a staged-widened policy new
  commands — this was never a too-short-fine-tune story.** No further
  budget/width variants on this recipe (per its own pre-registered
  FAIL branch).
- **Same cycle, refill: `cw-mt-b-hist16-1`** (the pre-registered
  representation lever, obs.history_frames 1→16 on `cw-mt-b1`'s exact
  2M discovery recipe) hit a DEFECTIVE launch — 0 steps, all 24
  workers SIGBUS at first reset (the documented hist16+model-DR
  `/dev/shm` cap from the arch track's hist16 death chain, COMMANDS.md
  #13c: hist16+DR at n-envs=4096 needs ~78MB against the pod's 64MB
  cap). A concurrent cycle diagnosed it and queued the fixed retry
  **`cw-mt-b-hist16-r1`** (--n-envs=3072, same hypothesis/gate) to the
  backlog.
- **08-12 ~23:3x: `cw-mt-b-hist16-r1` (representation lever, 2M) FAILS
  per its pre-registered gate — 16-frame history does not change 2M
  discovery on b1's recipe.** gate(DR0) det prog med 0.21 (needed
  >=0.32; b1 baseline 0.16, delta inside noise), gait_valid 0/6 det
  (needed >=1/6); video shows the identical low-crouch splay with
  legs 1+4 parked every det episode, matching b1/arch256's shape
  exactly. sto shows 5/6 gait_valid but that is noise-twitch over the
  same crouch (prog med 0.28, fwd 0.12m/15s), not stepping — not a
  discriminator. slip_per_m med 4.4 det / 6.8 sto, 0 terms. Per the
  pre-registered FAIL branch: temporal window is NOT the 2M discovery
  lever either; no further capacity/representation retry at 2M on
  this recipe — the cheap-2M-probe menu for this track (capacity,
  staged-widening, history) is now exhausted. Next call per the gate:
  a straight-to-20M hist16 arm citing the b2/widen2 budget precedent.
  Refill (same cycle): **`cw-mt-b-hist16-20m1`** (respec, continue
  the r1 checkpoint to the b2-matched 20M budget) — RUNNING (train-0).

## Next

- **[RUNNING, train-0] `cw-mt-b-hist16-20m1`** — the real
  command-acquisition test for the representation lever: same
  16-frame recipe as `r1`, budget-matched to `cw-mt-b2` (20M). Gate
  (vs `b2`'s matched numbers): PASS = gait_valid >=5/6 det AND prog
  med >=0.615 (0.5x-of-a2 bar b2 missed) AND signed-yaw
  differential >=0.10 sign-correct AND turn |wz_err| med <=0.10 with
  <9 falls. FAIL(match-b2) = prog/yaw within noise of b2 -> history
  lever closed at this recipe, next is escalation (arch recurrence /
  operator command-width-curriculum call), no further hist-frames
  variants. FAIL(worse/no-gait) = hist16 actively hurts from-scratch
  multitask learning. Report slip_per_m + roll_tail vs b2 regardless.
- Phase 2 / wave-2 planning should start from `b2` (real gait,
  closest to passing) or `a2` (clean specialist), never from `c2`'s
  broad-command recipe as-is — any future wide-command attempt
  needs a design change (curriculum on command width, more
  capacity, or staged widening from a walking checkpoint), not just
  more steps at the same width.
- [CODE, when wave 1 lands] fixed retained-command suite runner
  (`eval_cmd_suite`) if `eval_drive.py`/`eval_yaw.py` can't already
  probe exact (vx, vy, wz) triples per checkpoint — per-command
  tracking error/falls/slip/current, det+sto, machine-readable.
- Phase 2 transfer test after wave 1: warm-start A/B/C on the SAME
  new command (larger yaw or backward), fixed 1M/2M/5M budgets,
  measure acquisition speed AND retained-suite erosion.
- Wave 2 levers (one at a time): 256×256 net arch on the best
  generalist; obs history; k_drag_stance; body-height command; seed
  twins of the winner.

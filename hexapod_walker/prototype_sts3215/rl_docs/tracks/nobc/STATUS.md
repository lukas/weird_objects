# nobc — Learn without BC anchor

W&B: tag `track:nobc`. Excess-capacity research.

**Goal:** learn standing — and honest skill discovery generally,
clean non-scraping gait included — by RL alone: structural rewards,
curricula, terrain, RSI. NO imitation losses of any kind. hw ships
whatever works; this track exists to retire the crutch.

## Now

- Stand-from-scratch: six reward-side arms collapsed identically; the
  collapse traced to warm-start OOD drift, and the Warp pool-restore
  bug contaminated early verdicts — income-shaping/RSI verdicts are
  REOPENED for from-scratch designs. RSI machinery works.
- Gait-from-scratch: terrain clamp bug fixed (all past terrain was
  really <=18mm); terrain2 (true 72mm) FAILED into leg-sacrifice;
  dragstep1 FAILED — but its effective charge never exceeded
  ~0.09/tick vs ~1/tick income, so "really big penalty" was UNTESTED.
  **CROSS-TRACK: item 1 below is now ANSWERED, by an hw-tracked run**
  (`cw-gait-dragstance1`, GAIT.md) — the audit-derived structural
  per-stance charge (k=8000/m, correctly sized this time) trained
  from scratch and FAILED into freeze/park (charge paid the whole
  episode, never resolved, near-zero travel), matching its own
  pre-registered false branch. Solo structural charge does not
  induce stepping from scratch; move to item 2 (RSI-for-walk).
  **08-12: item 2 (RSI-for-walk mid-stride spawns, `cw-gait-rsi1`)
  also FAILED — CLOSED.** Identical training mechanism to
  dragstance1 (loadslip factor floored by step 49, drag charge never
  resolved); gate 0/6, slip/m 6-18, video is marching-in-place. Mid-
  stride state injection gives the from-scratch policy nothing to
  build on. **08-12: lever 5 (`cw-gait-slowfirst1`, target speed
  0.02-0.025 vs the champion's 0.05-0.06, same charge, no RSI) ALSO
  FAILED** — det fwd travel 0.00m every episode, video-confirmed legs
  shuffle in place with zero net displacement, same fingerprint
  regardless of command speed. Every no-new-code lever (2, 4, 5) is
  now closed on the identical mechanism. **Re-opened same cycle, a
  different no-new-code angle on lever 2:** `cw-gait-anneal1` —
  instead of an in-run anneal (needs a scheduler, CODE), warm-start
  the structural charge onto `cw-gait-dragstep1`, a genuine
  from-scratch RL-only paddler that already translates (by skating)
  before the charge ever turns on, testing whether an already-mobile
  prior reshapes into stepping instead of freezing like a random
  init did. **08-12 ~08:30 verdict: `cw-gait-anneal1` FAILED,
  informatively.** The mobile prior survives the charge (det fwd
  0.37-0.46 m — NOT the frozen fingerprint) but never cleans up:
  leg-3 flag-leg skate (gait_valid 0/6, sac [3] all det episodes),
  slip/m 4.3-5.1 vs the 1.5 bar, reward diving -335 → -4744 by
  quarters as the charge goes unresolved for 2M steps. Every
  no-new-code form of lever 2 is now closed. ~~The track is now a
  genuine WAIT on unwritten code~~ **08-13: the code-wait CLEARED —
  the in-run coefficient scheduler LANDED** (`sched.*` cfg keys,
  applied in `sim_env._step_begin`, so it runs on both trainer
  stacks by construction — the MJX vec envs call the same hook;
  default off = bit-exact, 10 new tests `test_coef_sched.py` + full
  semantics bank green, REWARD.md row; monotone per-process clock,
  never rewound by pool-restores). First arm `cw-gait-sched1`
  (2M discovery, from-scratch on the dragstance1 stack, charge
  ramped 0→8000 over global steps 0.5M→1.5M) is the LAST untried
  form of lever 2: let the paddle form first, then price the skate
  out from under a policy that already knows how to move. Gate
  pre-registered (mobility retained AND skate resolving). If it
  fails the same way as its siblings (freeze OR unresolved-charge
  skate), the from-scratch gait line has no levers left and closing
  it is the recommendation to the operator.
  **08-13 ~01:2x: `cw-gait-sched1` RESULT — FAIL, the predicted false
  branch exactly.** Gate: det fwd travel med 0.01m (bar >=0.3m),
  slip/m 9.4 det / 18.4 sto (bar <=3.0) — both clauses miss badly.
  `env/sched_value` confirms the coefficient ramped correctly
  (0→8000 over the 0.5M-1.5M window, verified in the training curve
  — the scheduler CODE works exactly as built), but
  `env/walk_loadslip_factor` stayed floored at 0.03-0.09 the entire
  run and `env/reward_drag_stance` paid -6..-8/tick unresolved to
  the end — the identical mechanism as every prior lever-2 form.
  Video/contact-sheet: near-static stance across all 10 sampled
  frames, a freeze, not a paddle. **GAIT P3 LEVER 2 IS NOW CLOSED IN
  EVERY FORM** (fixed-rung pricing, warm-start anneal onto a mobile
  prior, and now a true in-run schedule) — three distinct mechanisms,
  same collapse. **The nobc gait-from-scratch line's entire
  no-new-code lever menu (2, 4, 5) is exhausted.**
  **08-13 ~04:xx: the physics-easing code-wait CLEARED — lever 3 is
  BUILT and running.** The 01:2x entry's build-vs-close question was
  answered assume-and-go (BUILD, per the CODE-FIRST directive; a
  dedicated idle-kick build cycle was available — ASSUMPTION
  recorded in top-level STATUS.md for operator review). The build is
  much smaller than the spec above feared: no `DomainRandomizer`
  range-refresh plumbing at all. New cfg keys `ease.gravity_scale` /
  `ease.vel_ceiling_scale` (default off, bit-exact when off,
  snapshot e40a3ea) scale the per-episode DR **draw** (`_ep_rand`)
  in `_reset_begin` — the single choke point BOTH trainer stacks
  already consume (private model via `apply_to_model`; batched MJX
  via `ModelDrScratch.rows_for`/`tp_rows`) — re-read every reset so
  the `sched.*` engine anneals them in-run; slope-DR direction is
  preserved, per-episode physics never changes mid-episode;
  randomize=False private-model envs get a direct fallback and
  shared-model shims raise loudly. 8 tests `test_physics_ease.py` +
  sched tests + full semantics bank green; REWARD.md row documents
  the eval-safety design rules (schedules must end at v1=1.0 and
  keep t1 under one eval episode's global-step equivalent so all
  gate evals run nominal physics). First arm **`cw-gait-ease1`**
  (2M discovery, train-0): byte-identical dragstance1 stack (full
  k_drag_stance=8000 from step 0), ONE variable = gravity 0.5→1.0
  annealed over 0.4M–1.1M. Pre-registered: PASS (fwd ≥0.3 m AND
  slip/m ≤3.0 or gait_valid ≥1/6 at nominal-physics DR0 det) =
  first-ever from-scratch gait signal → longer-anneal 20M hardening
  (vel-ceiling ease is the pre-built second axis); FAIL = P3 levers
  1–5 ALL closed under honest trials → recommend closing the
  from-scratch gait line (the hardware-bound gait keeps coming from
  the BC-anchored lineage regardless — nobc-charter research, not an
  hw-mainline blocker).
  **08-13 ~03:1x: `cw-gait-ease1` RESULT — FAIL, the false branch
  exactly. THE FROM-SCRATCH GAIT LINE IS OUT OF LEVERS; recommend
  CLOSE (operator call, surfaced top-level STATUS WAITING-ON).**
  Gate: det fwd travel med 0.00 m (bar ≥0.3 m), slip/m med 7.21 det
  / 19.19 sto, zero terms; video/contact-sheet a motionless splayed
  crouch in all frames — the identical freeze fingerprint as
  dragstance1/rsi1/slowfirst1/anneal1/sched1 (`gv True` here is the
  known zero-travel artifact, not stepping). The decisive mechanism
  read: `walk_loadslip_factor` floored at 0.04–0.09 from the FIRST
  W&B sample — i.e. through the entire HALF-GRAVITY eased phase —
  while `env/sched_value` confirms the anneal ran exactly as coded
  (0.5→1.0 over 0.4M–1.1M) and the drag charge paid −6.5..−7.9/tick
  unresolved to the end. Easier physics gave exploration nothing:
  the freeze forms before the eased window even closes. Levers 1–5
  all closed under honest trials; `ease.vel_ceiling_scale` stays
  pre-built but UNQUEUED — its licensing condition was the marginal
  branch, which did not occur. No nobc gait run queued pending the
  operator's close call.

- CROSS-TRACK INSIGHT (hw P0 probe, 08-11 late, GAIT.md bottom): the
  crouch-paddle is a sim-EFFECTIVENESS optimum, not a paid basin —
  it out-earns the plant-height gait ~495/ep on genuine progress
  income while tilt penalties are negligible (≤1.5/ep). Pricing
  magnitude alone must beat that moat without paying park; favors
  the per-stance charge + curriculum levers below over any further
  income shaping.

## Next

1. ~~Drag-charge magnitude audit~~ DONE 08-11 (see above) — solo
   structural charge does not induce stepping from scratch.
2. ~~RSI-for-walk mid-stride spawns~~ DONE 08-12 (see above) — CLOSED,
   same freeze/near-still mechanism as the drag charge alone.
2b. ~~Annealed-up charge (warm-start AND true in-run schedule)~~ DONE
   08-13 (see above) — BOTH forms FAILED, lever 2 closed for good.
3. ~~Physics easing~~ DONE 08-13 (`ease.*` built, snapshot e40a3ea;
   `cw-gait-ease1` FAILED on the pre-registered false branch — see
   above). **The gait-from-scratch lever menu is exhausted;
   recommendation to the operator: CLOSE the line.** Nothing queued
   pending that call. Stand-from-scratch (the track's other charter
   item) is unaffected.

Detail: GAIT.md P3 · RISE.md forensic ladder.

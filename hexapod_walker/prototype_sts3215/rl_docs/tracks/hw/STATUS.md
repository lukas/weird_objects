# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **08-15 ~22:3x (this cycle, triage + operator directive
  fb_20260815T222943_d019de): `cw-recover-any4-b0scratch1` STOPPED at
  ~10.8M/40M — the curriculum's judge, not the policy, was the bug.**
  Reward quarters were flat/worsening (-103.9→-108.3) and B0 EMA
  stuck near 0 under the stochastic rollout success signal used
  below, but a deterministic same-checkpoint eval on the reference
  C-MuJoCo simulator scores B0-B4=1.0, B5=0.667 — the policy already
  catches itself from small disturbances. The gap: action noise
  (~0.38 std) keeps nudging a foot off the ground and re-triggers the
  strict 0.5 s six-foot hold requirement, so the *noisy* rollouts that
  drove promotion never registered success even when the *calm*
  policy already could. Fix (operator-approved, commit `3589f418`):
  promotion now runs off a periodic deterministic certification pool
  on the exact training backend (Warp/MJX) — `CERT/recover_bucket_*`
  — with the old stochastic EMAs and the C-MuJoCo assay demoted to
  telemetry-only. Successor **`cw-recover-any5-mjxcert-scratch1`**
  launched FROM SCRATCH on train-1 (clones any4's full recipe +
  `--recover-cert-every 1000000 --recover-cert-envs 8`), verified
  live via `/proc`. Hard gate at ~1M: `CERT/recover_bucket_0_success`
  must be PRESENT (8-episode denominator); if CERT B0 fails while the
  background C-MuJoCo SCORE B0 still passes, that is a genuine
  Warp-vs-C backend mismatch, not more curriculum tuning — report it,
  do not promote. any4's checkpoint preserved on train-1 as the
  backend-mismatch diagnostic. Ledger verdict + W&B OUTCOME note
  complete on any4; no further action needed on it.
- **08-15 ~22:0x (operator-kick cycle, fb_20260815T214555_008f42):
  the RECOVERY LINE IS LIVE AGAIN — `cw-recover-any4-b0scratch1`
  launched FROM SCRATCH on train-1 (W&B brjnwcnb, 40M,
  hw/acquisition) on the operator's bucketed curriculum at exact
  main c60c7ac** (zero-indexed ladder B0 plant_catch ±2° → B1-B3
  onefoot micro/mid/full → B4 tripod park → B5 crouch/partial/bank
  → B6 zero/tangle → B7 flip; promote EMA≥0.8 n≥4, retreat+re-certify
  <0.2 n≥6, no harder probes). Genuinely scratch: no --init-from, no
  --obs-pad-transplant, parent null; any2b is comparison evidence
  only. Preflight green same cycle (recover 17/17, full bank 113
  pass / 4 skip / 1 xfail — matches operator's numbers). Verified
  live at ~1.2M: frontier B0-only (`env/recover_start_bucket=0`,
  `recover_frontier_bucket=0`, `recover_active_families=1`), valid
  settled B0 resets (tilt 0.44°, height 141 mm, min-load 3.3 N), BC
  recover anchor filling (131k, footz loss 0.16), first forced eval
  emitted `SCORE/recover_bucket_0..7_success` with explicit
  denominators (2/2/2/2/2/6/4/2 episodes). Train-side B0 EMA ~0.12
  (early, as expected from scratch). Watch: B0 success curve must
  rise; promotion only at EMA≥0.8 n≥4.
- ~~**08-15 ~20:5x (operator-kick cycle, fb_20260815T201417_5f7f0e +
  superseding fb_20260815T201712_39279d): the RECOVERY LINE IS
  PAUSED `[operator]`**~~ — CLEARED by the entry above; history:
  waiting on the operator's bucket-0
  curriculum (plant-catch / micro-onefoot rungs, tripod park moved
  later, per-bucket SCORE metrics), a new exact main SHA, and the
  final launch directive. Executed: `cw-recover-any2b` KILLED at
  2.75M under the stop-the-warm-arm order (checkpoint preserved on
  train-1 + W&B u9sp8dki, RESUMABLE); the ordered from-scratch
  replacement `cw-recover-any3-scratch1` was NEVER LAUNCHED — the
  supersede arrived first (ledger stub marked SUPERSEDED; RECOVER
  preflight bank 13/13 PASS on main 6f909719 stands for the future
  launch). **Honest read at kill: any2b was WORKING — split det eval
  onefoot success=1 (fixed the foot in 1.58 s) AND park success=1
  (rose from the crouch in 2.36 s), `SCORE/recover_success=1`,
  `tipped_recovery_success=1`, BC recover anchor filling, curriculum
  correctly bucket-1-only; train-side sto EMAs still ~0.1 (early).
  The "warm-start flatlined at zero" premise behind the pause traces
  to any2's blind evaluator, not to learning failure — decision
  asked in OPERATOR_QUESTIONS q_20260815T2050Z (resume any2b vs
  bucket-0 scratch, or both).**
- **08-15 ~20:3x (operator-kick cycle, fb_20260815T194955_9441a0):
  the replacement went LIVE as `cw-recover-any2b` (W&B u9sp8dki,
  train-1, 40M, hw/acquisition)** — first attempt (`cw-recover-any2`,
  W&B lf5afhd6) was a no-science FALSE START: train-1's fresh
  bootstrap lacked sb3-contrib, the trainer's bg eval/video/canary
  child died at first import and the run was permanently eval-blind;
  killed at ~5M, pod env fixed, `bootstrap_train_pod.sh` patched
  (sb3-contrib pinned + smoke import). any2b verified end-to-end at
  1.2M: bucket-1-only curriculum active, per-kind EMAs/counts and
  settled-reset telemetry live, `train/bc_anchor_*_recover` filling,
  and the split eval already shows det onefoot 2/2 AND park 2/2
  (`SCORE/recover_onefoot_success=1`, `park=1`) — the exact signal
  any1 never produced in 13.5M. Bucket-1 gate as pre-registered.
  (Superseded ~20 min later — see the entry above.)
- **08-15 ~20:0x (operator-kick cycle, fb_20260815T193318_2cc049):
  `cw-recover-any1` KILLED at ~13.5M by OPERATOR ORDER (failed
  diagnostic: aggregate success 0, Phi/quality declining, NO
  per-start-kind visibility — the video reel didn't even list the
  recover mode) and REPLACED by `cw-recover-any2`**, relaunched from
  the clean stance champion `cw-stand-footlow2-hard1` (NOT any1's
  degraded weights) on the operator's own bucket-1-fix commit
  (aa1023c6, pushed to main mid-cycle by his Codex session — this
  cycle's parallel duplicate implementation was dropped in its
  favor): curriculum starts at ONLY onefoot/park with no
  harder-family probe, retreat floor 1, admission n>=4 @ EMA-β .25
  per kind with retreat re-certification; always-on per-kind
  success/EMA/count + settled-reset height/tilt/load/spread + BC
  eligibility/ref-index telemetry; `recover_success` is an explicit
  termination reason; periodic eval + gate reports force equal
  onefoot/park episodes and split all recover rows by start kind;
  video reels caption the start kind; restored stance-lineage
  supervision (foot-z BC 1.0 @ 3mm, min-height-ahead 15mm above the
  BELLY datum, lookahead 0.5 s, height-matched state alignment).
  Preflight: plant-teacher held-success from settled onefoot AND park
  is now a semantics-bank test (PASS; full bank 109-pass green).
  any2 was launched by the CONCURRENT operator-kick cycle
  (fb_20260815T194955_9441a0) on train-1, W&B `lf5afhd6`, SHA
  local==pod 4f70cc13, verified advancing (1.44M @ ~20:1x) with
  per-kind counts/EMAs live. Its gate: both forced onefoot/park
  success curves must RISE, no promotion before per-kind EMA>=0.8
  n>=4, STOP EARLY on invalid settled resets or zero BC
  eligibility/fill; full-arm bar keeps any1's 95/85 held-recovery +
  video + no rise/hold/lower regression. BASELINE (this cycle's
  read): settled onefoot lands h~133mm tilt 3.3deg, park h~119mm
  tilt 1.3deg, ALL feet grounded but under-loaded (min_load
  2.3-2.7N — the limp settle mostly drops the lifted feet back
  down, so bucket 1 = a load/height correction, not a re-plant);
  zero-shot transplanted parent success = 0 through 1.4M (EMA ~0.23
  from the 0.5 prior) — learning is genuinely required, and the
  plant teacher proves the gate reachable. any1's other MDP pieces
  (PBRS reward, 5.1 s horizon, 185° envelope) carried unchanged.
  The paragraph below describes the original any1 spec, kept for
  the mode's design record.**
- 08-15 ~18:xx (operator-kick cycle): the getup/recovery sub-line
  REOPENED BY OPERATOR ORDER (authenticated KICK confirming the
  fb_20260815T165306_606974 directive after 5-6 correct
  channel-grounds declines of its unauthenticated MCP copies) and
  `cw-recover-any1` launched (now KILLED, see above) — a universal recover-to-plant
  specialist: from any recoverable state (near-stand w/ one unloaded
  foot, tripod park, crouch/interrupted rise, harvested post-lower
  bank, belly, random tangle, side/back/UPSIDE-DOWN drops) reach a
  full-height level quiet stand with ALL SIX feet loaded, hold 0.5 s,
  episode ends on held success. New `recover` mode (REWARD.md §4c):
  potential-DIFFERENCE reward (PBRS — no occupancy/ratchet/hold
  income, no alive bonus; smooth-min per-foot load keeps one unloaded
  foot visible → the getup3-c2/getup4 4-leg plateau cannot recur by
  construction), one-shot success bonus, time tax, fail cost ≥ max
  remaining tax (no early-abort), adaptive reset-family curriculum
  (frontier-weighted, ≥80% admit / <20% retreat, buckets 1-2 first),
  eligibility-gated state-aligned rise BC anchor (the cw-getup3
  lever, now orientation/height/contact-conditioned). Warm from
  footlow2_hard1 (obs-pad transplant = optimizer fresh, critic
  carried — recorded semantics), long-horizon PPO per directive
  (512 envs × 128 steps = 5.1 s span, γ=0.995, λ=0.98, batch 8192),
  40M cap, one-run bundle exception per the operator ruling
  (root STATUS.md). v1 DEVIATIONS from the directive spec (recorded,
  pre-registered next rungs): reset families 5-6 (pushed-walking
  falling states, on-policy failure harvests) + exact-qvel bank
  restore on the MJX path + the 1 s frozen-stance handoff INSIDE
  train-time eval are not built — handoff is checked at triage via
  eval_handoff; curriculum stats are per-env (4-worker sharded), not
  fleet-global; COM/support check is the footprint+all-loaded+level
  proxy. Gate: pre-registered in the ledger (held recovery ≥95% det /
  ≥85% sto across the ACTIVE mixture at 40M or early exploit stop;
  no regression on ordinary rise/hold/lower; research specialist —
  does NOT touch the product baseline).**
- CROSS-TRACK INSIGHT (08-15, from multitask): `cw-joystick-translate1`
  (walk-task, unrelated reward recipe) independently reproduced the
  parked/stilt-single-foot exploit gaming its progress proxy while
  real displacement stayed ~0 for 40M steps — corroborates, does not
  reopen, the hw "one-parked-foot hold habit" already TERMINALLY
  CLOSED on pricing above (`cw-stand-minfeet1` etc.); still points at
  the anchor/behavior side, not more per-foot reward tuning, as the
  only lever left. No hw launch from this.
- **08-15 (this cycle): `cw-stand-postlower4` FINISHED and its
  pre-registered Cohort c4 bulk read (n=600, fresh banks
  960000../970000.., now retired) is IN — VERDICT: FAIL, but the
  right kind of FAIL — mechanism CONFIRMED, magnitude still short.**
  The schedule fix (`goal.mode_seq_rise_from_h`, "stand up from where
  you are") worked exactly as designed: 10 watched re-renders (6 of
  the failures + 4 clean draws) show every post-lower rise is now a
  DIRECT push-up, zero belly-detours, and det post-lower rise
  recovered from c3's 0.419 to **0.872** (sto 0.631→0.690) — but both
  numbers land short of the parent (det 0.967, sto 0.801) and short
  of the pre-registered parity bar, so the letter of the gate reads
  FAIL on clauses 1-3 (det session zero-fall 0.863 vs bar 0.95, det
  post-lower rise 0.872 vs bar 0.967, sto post-lower rise 0.690 vs
  bar 0.90). Crown jewels clean (det first-rise 0.99 every stratum
  ≥0.97; lower 1.0 det+sto). Remaining falls are a genuine
  over_current stall (switch_peak_a ~2.6A), not a new exploit — an
  actuation-effort ceiling, not a behavior bug. This is the SECOND
  miss of the in-context sequence-training mechanism (c3 = wrong
  mechanism/detour, c4 = right mechanism/still short) — per two-miss
  discipline the class is CLOSED for further dose/diet/schedule
  resweeps of this recipe; unlike c3 this result is NOT surprising
  (it's exactly the pre-registered "if-false" branch, video-
  confirmed), so no dig-in was needed to call it. Next lever is an
  operator product-contract choice: align the runner/instrument's
  rise-schedule semantics to "remaining rise" (train==deploy exactly)
  or price post-lower rise directly in reward — escalated
  `[operator]` in STATUS.md WAITING-ON, no new postlower arm until
  picked. Full numbers: `SESSION_BULK_GATE.md` "Cohort c4 RESULTS".
  Product baseline (c1 hierarchy) unaffected.
- 08-15 (dig-in cycle, superseded by the c4 read above): `cw-stand-postlower3` VERDICTED FAIL —
  root cause FOUND, fixed in code, and `cw-stand-postlower4` launched
  on the fix (pre-registered Cohort c4, fresh banks 960000../970000..).
  The c3 collapse was not "more exposure needed" and not the reanchor
  path: the sequence rise schedule STARTS AT BELLY-FRAME 0 (blend
  down + 1 s hold at 0), so training PAID the robot to re-descend,
  splay flat and re-run the flat-rise demo choreography after every
  lower — the re-renders show that detour in failures AND successes,
  and the state-aligned flat-demo BC anchor reinforces it once low.
  The detour completes on-policy in training (hence the `rise:ok`
  reels) but routes every held-out post-lower rise through the
  max-strain curl → over_current on >50% det (det<sto because det
  fully commits to the taught detour). Fix (same-cycle, CODE-FIRST):
  `goal.mode_seq_rise_from_h` (default off, bit-exact, tests green) —
  mid-sequence rises start at the robot's CURRENT height, "stand up
  from where you are", never a commanded descent. Full chain +
  clause table: `SESSION_BULK_GATE.md` "Cohort c3 DIG-IN VERDICT" +
  "Cohort c4". If c4 misses too, the in-context class is done and
  the next fork (align instrument/runner rise schedules to
  remaining-rise semantics = a product-contract change) goes to the
  operator. Product baseline (c1 hierarchy) unaffected. CROSS-TRACK
  INSIGHT: the shared walk-task `goal.mode_seq` rise branch has the
  identical descent defect — noted in arch/STATUS.md, no arch launch
  from here.
- 08-15 (triage cycle, superseded by the dig-in above):
  `cw-stand-postlower3` FINISHED training and
  its pre-registered Cohort c3 bulk read (n=600, fresh banks) is IN —
  a CLEAN, BAD MISS: det session zero-fall collapsed to 0.413 (parent
  0.967) and det post-lower rise to 0.419 (parent 0.967, also worse
  than both prior FAILED attempts), sto post-lower rise 0.631 (worse
  than parent's 0.801). Cold first-rise and lower retention are
  untouched (still ≥0.96/1.0) — the damage is isolated to exactly the
  post-lower-rise mechanism this arm targeted, and it got WORSE, not
  better, training a NEW mechanism aimed straight at it. Same
  qualitative failure mode as before on video (over_current stall,
  no exploit, honest six-leg gait elsewhere), but det doing WORSE
  than sto — backwards from the usual pattern — and disagreeing with
  this arm's OWN training-time telemetry (last reel read
  `rise:ok lower:ok rise:ok`) is a generalization-failure signature
  that needs a root-cause read before naming the next lever, not a
  triage guess. **Left UNVERDICTED (DIG-IN flagged) per the model-
  tiering rule** — full numbers + a first hypothesis (train/eval
  reanchor-path mismatch specific to `mode_seq_stance`, not just
  "needs more exposure") in `SESSION_BULK_GATE.md` "Cohort c3
  RESULTS"; raw shards + failure re-renders saved
  (`logs/bulk_session/c3/`), no need to re-run the cohort. This is
  the THIRD miss on post-lower-rise (postlower1/2/3) — per two-miss
  discipline, no further dose/diet resweep of this recipe; the next
  mechanism is an operator/dig-in call, not a triage one. Product
  baseline (c1 hierarchy) unaffected.
- **08-15 (idle-kick cycle): `cw-stand-postlower3` LAUNCHED (discovery
  2M, train-0) — the c2 dig-in's named mechanism change is built,
  preflighted and pre-registered, all in one cycle.** New cfg key
  `goal.mode_seq_stance` (default OFF, bit-exact off; stance-only
  grammar rise→hold→lower→rise on the joint_goal task) delivers the
  in-context lower→rise SEQUENCE training the postlower verdicts
  called for: half of all episodes are two-segment stance sequences
  (7–8 s segments in 18 s episodes), a lower-first sequence IS the
  post-lower rise with real transition context (warm policy state,
  canonical per-family re-anchor, blend window), and the mid-sequence
  rise target anchors at the sequence's OWN commanded stand height —
  mechanically reachable by construction, so the c2 impossible-target
  bug class is locked out by a regression test
  (`test_lower_to_rise_targets_remaining_rise`). Bank exposure is OFF
  (`rise_start_bank_frac=0` — the cold-spawn class stays closed);
  everything else is the footlow2_hard1 recipe warm from
  footlow2_hard1, walk ckpt untouched. Implementation notes: the
  rise/hold/lower segment builder moved to the shared goal-task base
  (walk task delegates — statements verbatim, walk rng streams
  unchanged, `test_mode_seq.py` 11/11 green); the frame-capture and
  MJX mint gates now also fire on the stance key; the stance key on
  the joint_walk task raises loudly. Preflight: new
  `test_mode_seq_stance.py` (7 tests) + full `test_task_semantics.py`
  bank (91 passed) locally; `test_mode_seq_stance` +
  `test_mjx_vec_env` 16/16 on train-1 (pod MJX env). Snapshot tag
  `exp/cw-stand-postlower3`. Gate: **pre-registered Cohort c3**
  (SESSION_BULK_GATE.md "Cohort c3", FRESH held-out banks
  940000../950000.., candidate `spec-pl3` registered in
  `bulk_session_eval.py`) — full PASS = promotion-grade candidate;
  partial (sto post-lower rise separated above parent 0.801, retention
  clean) = one 6M hardening rerun on cohort c4; at/below parent or any
  retention/visual break = next change must be mechanism-level
  (sequence-RSI or rise pricing), never a dose resweep. Product
  baseline unchanged (c1 hierarchy).

- **08-14 ~21:4x UTC: BULK HELD-OUT SESSION COHORT (operator
  directive fb_20260814T205137_33f21c) — the hierarchical
  frozen-skill controller PASSES the pre-registered product gate at
  n=600 fresh sessions and is now the MEASURED product baseline;
  single-model consolidation is officially research, not a
  blocker.** New resumable sharded evaluator
  (`rl_move.sim.bulk_session_eval`, tests green, snapshot
  `exp/session-bulk-cohort1` d5aa13c) ran 300 det + 300 sto ~60 s
  randomized joystick sessions per candidate on 11 idle pods'
  CPUs (~3 min wall), matched seeds/schedules (held-out banks
  900000../910000.., now RETIRED), for `spec`
  (footlow2_hard1 + bcgait1_hard1 + entry-slew), `td2`, `td3`.
  Pre-registration + full numbers: `SESSION_BULK_GATE.md`. Headlines:
  - **spec det zero-fall 290/300 = 0.967 CI [0.940, 0.982]** — all
    gate clauses pass (segments ≥0.983, strata ≥0.95); ALL 10 det
    failures are POST-LOWER rises (first rise 300/300); sto 0.853
    with the weak link again the stochastic post-lower rise (0.801,
    over_current-dominated). Walking is clean at scale: zero drive
    falls + gait_valid 1104/1104 spec drive segments, slip/m 1.75,
    height 135 mm.
  - **Hierarchy vs single models: separated** — sto spec CI lower
    0.809 > td2 0.705 / td3 0.746 CI uppers; det separated vs td3,
    marginal overlap vs td2 (0.940 vs 0.946). td2's pooled det 0.92
    hid clean_session 0.597 (crouch cold rise 0/100 finishes short)
    and both singles walk ~116 mm (low posture) with sto first-rise
    collapse (0.23/0.32).
  - **Zero-command creep confirmed at n=1800**: 0/2773 drive
    segments settle <0.02 m/s — STOP→stance-hold stays mandatory.
  - Strips: every failure (719) + clean samples re-rendered on
    train-1/2/3 (`logs/bulk_session/c1/rerender/strips/`); reviewed
    samples confirm honest six-leg gait in clean sessions and real
    (not artifact) post-lower-rise falls.
  Next lever (pre-named): train ONLY the post-lower-rise
  transition/residual with both skills frozen; bench promotion of
  the pair stays operator-owned. **EXECUTED 08-14 ~22:3x UTC:
  `cw-stand-postlower1` is TRAINING (hardening 6M, train-0)** —
  new default-off `goal.rise_start_bank`/`_frac` (rise episodes
  start from harvested settled lower-endpoint poses of
  footlow2_hard1's own lower skill, walk-park-bank mechanism class;
  RSI skips bank episodes, canary force overrides bank, off-path
  bit-exact, tests green; eval start_kind label "post_lower") +
  `harvest_lower_endpoints.py` (bank: 300/300 settled, 0 falls,
  seed 5000, `park_banks/footlow2_hard1_lower_endpoints.npz` —
  knees ~+113°/hips ~−18° from the flat-zero pose, an unseen state
  family). Gate pre-registered BEFORE training on fresh c2 banks:
  SESSION_BULK_GATE.md "Cohort c2" (sto post-lower ≥0.90 CI-separated
  above parent 0.842 upper + full det/cold-start/lower retention +
  eval_session, visual stats vs parent). **RESULT 08-14 ~22:5x UTC:
  FAIL — REGRESSION, not a trade-off.** Full c2 bulk cohort (n=600,
  fresh 920000/930000 banks, 11 pods): sto post-lower rise
  **0.717** [0.663,0.765] — WORSE than the parent's own 0.801
  (CI upper below parent's CI lower: real separation the wrong
  way), det session zero-fall 0.923 (parent 0.967), det post-lower
  rise 0.936 (parent 0.967), det cold first-rise 0.987 (parent
  1.00), det hold drag 623mm (parent 136mm), sto rise roll_tail
  2.3° (parent 0.7°). Only the cold-rise-stratum/lower clause still
  passes. Video-confirmed no exploit (over_current fails show the
  robot genuinely stuck straining from the deep-knee bank pose, not
  a reward hack) — 35% exposure to the harvested post-lower start
  bank made the exact skill it targeted worse, diluting general
  rise quality with it. Full numbers + table: SESSION_BULK_GATE.md
  "Cohort c2 RESULTS". **Next (launched same cycle):
  `cw-stand-postlower2`** (discovery, 2M, frac 0.15, same recipe
  otherwise, train-0) to separate DOSE (0.35 too aggressive) from
  MECHANISM (the fixed `rise_ref_track` reference is shaped for
  flat-topology starts and mis-prices this pose family regardless
  of dose) before any further hardening or reward-side change.
  Product baseline is UNCHANGED — this FAIL doesn't touch the
  passing c1 hierarchy.
- **08-14 (late): `cw-stand-postlower2` FAIL — and the dig-in found
  the REAL bug: the bank mechanism trained on IMPOSSIBLE height
  targets.** Chain of matched controls (all rise-only, per-mode 6,
  parent = `footlow2_hard1`, reports `logs/ckpt_eval/*bank*/`):
  (1) postlower2 from the bank: 0/12, same stuck-straining stall.
  (2) PARENT from the same bank spawns: ALSO 0/12 (worse errors) —
  yet the parent rises from REAL in-session post-lower states at
  0.801 sto / 0.967 det (n=600). (3) Exact full-state restore
  (new opt-in `goal.rise_start_bank_exact`, harvest now saves
  qpos/qvel): parent still 0/12 — reconstruction exonerated.
  (4) Root cause MEASURED: rise height bands are z0-relative and
  BELLY-calibrated (flat z0=38mm), but bank spawns settle at
  82-99mm — the schedule commanded chassis ~190-213mm, ~50mm above
  standing. postlower1 (35%) and postlower2 (15%) trained on
  unreachable goals; max-current straining was the OPTIMAL policy.
  Explains the c2 regression outright. FIX LANDED: harvest saves
  per-row `z_stand` (the lower episode's own standing height);
  `goal.rise_start_bank_anchor_stand` (default OFF, bit-exact,
  tests green, snapshot f3b4902 tag exp/postlower-anchor-fix)
  rewrites the schedule to the REMAINING rise; v2 bank harvested
  (`park_banks/footlow2_hard1_lower_endpoints_v2.npz`).
  (5) Parent from the FIXED instrument: sto 2/6 real completions
  (first ever from bank spawns) but det 0/6 — the det parent
  COLLAPSES TO BELLY during the zero-height hold (constant 95.1mm
  err): a COLD single-mode spawn does not reproduce the in-session
  context (warm policy state + canonical re-anchor right after its
  own lower) where the same parent scores 0.967. **Two misses =
  hypothesis changed: cold-spawn exposure is the wrong lever class
  for a TRANSITION boundary. Named next arm (`cw-stand-postlower3`,
  to spec): train the stance policy with in-context lower→rise
  SEQUENCE episodes via `goal.mode_seq` (machinery landed + sharded
  mint proven 08-14 in arch; hw use here is stance-only, judged
  against the hw goal — not a cross-track launch) — spec needs the
  sequence-grammar check for stance-only pairs, mode-bank preflight,
  and a pre-registered c3 bulk cohort on fresh banks
  (940000../950000..) before training.** Product baseline still
  UNCHANGED (c1 hierarchy).
- **08-14 ~20:0x UTC: the SESSION-JOYSTICK product gate exists and
  the candidate specialist pair PASSES it deterministically —
  `session-joystick-handoff1`** (operator-requested action cycle;
  external notes fb_20260814T194245/194529). New default-off
  `eval_modeseq` flags (`--drive-random`, `--entry-slew`, snapshot
  `exp/session-joystick-handoff1` 4c9912d, legacy path bit-exact)
  turn the modeseq instrument into the ~60 s guarded session
  REST→RISE→SETTLE→WALK_ENTRY→randomized joystick DRIVE (fwd/
  diagonals at the trained band, guaranteed stop-go + direction
  flip)→STOP_SETTLE→LOWER→RISE→DRIVE, with canonical per-mode
  re-anchor at every switch and the TAKEOFF.md entry-slew ramp at
  walk engage. Result, `footlow2_hard1` (stance) +
  `bcgait1_hard1` (tall walk, own-cfg vel:=ref), 12 eps, matched
  A/B slew-on/off, strips watched (honest tall gait, no parks):
  - **det no-slew 12/12 zero-fall, every segment perfect** (rise
    24/24 incl. post-lower, lower 12/12, walk 24/24 gait_valid,
    slip/m med 1.80, drive height med 135 mm); det slew-on 11/12
    (one downstream rise tilt fall — noise-level; engage-window
    tilt med 1.6→1.3°, max 2.5→2.1°: the ramp is in-session
    OOD-safe and mildly quieter, its real justification stays the
    push-probe in TAKEOFF.md).
  - **sto: the weak link is the in-sequence RISE, not driving** —
    rise 18-19/24 (over_current + tilt falls), walk stays
    21-22/21-22 gait_valid with zero drive falls under flips and
    stop-go, lower 12/12 both arms.
  - **NEW measured fact for the session controller: at zero command
    the tall walker does NOT settle** — mean body speed over the
    trailing 1.5 s stop window is 0.035–0.040 m/s in all 4 arms
    (0/90 windows under 0.02 m/s). A joystick session MUST keep the
    runner's stop→stance-hold switch; "walk policy at zero command"
    is not a stop.
  Evidence: `logs/ckpt_eval/session_joystick_handoff1_{det,sto}_
  {slew,noslew}.json` + `_ep0.png` (controller copies; source
  train-1). Next hw session gate for any stance/walk candidate can
  now add these two flags for the session-level read.
- **08-13 ~2x:xx UTC: ruling 2's agent-doable half is DONE — the
  takeoff transient is instrumented and the staged gait-entry design
  exists, prototyped, with a bench-ready recommendation
  (`rl_docs/TAKEOFF.md`).** Tape analysis (26 walks): the transient
  is a DROP-IN POSTURE SNAP — the policy saturates the 1.5°/tick slew
  on all 18 joints from tick 0 at ZERO command; 14/26 tapes cross 5°
  roll before the runner's velocity ramp even starts (t=1.04 s), so
  the throttle must act at policy HANDOFF, not on the first step.
  Design built (default-off, bit-exact, tests + 91-bank green):
  `safety.entry_slew_ramp_s`/`entry_slew_start_deg` — per-tick slew
  starts at 0.25°/tick after engage and ramps to 1.5°/tick over
  1.5 s; shared SafetyLayer code path = same switch on hardware and
  in sim. Prototype (`probe_gait_entry.py`, 144 paired det rollouts,
  calibrated 2.6 N·m walk-push proxy): deployed walker tip1 falls
  9/12→4/12 (paired 5 saved / 0 caused), early peak 30.4°→7.2° med,
  walking resumes; bcgait1_hard1 fall count unchanged (push-dominated
  at that fixed dose) but rates halve; ALL no-push arms clean 12/12 —
  the throttle is OOD-safe. NEXT = OPERATOR BENCH: flip the two cfg
  keys on the runner's walk engage and re-run takeoff reps
  (fell/tail); training arms under the entry schedule only if the
  bench adopts it (MJX parity test for the new keys first).
- **08-13 ~12:4x UTC OPERATOR RULINGS — both open hw design forks
  are DECIDED:**
  1. **Standing lean (~8°): MECHANICAL TRIM.** The lean is ruled a
     hardware/mechanical trim problem OUTSIDE RL. Do NOT queue a
     lean-pricing reward term, tipped-exposure arm, or teacher
     redesign on any stance lineage (tiltcomp dossier stands as the
     closing evidence; the mechanisms `bc_anchor_tilt_comp` /
     `_tilt_from_settle` stay built, default-off). RL-side the lean
     is CLOSED; the fix moves to the bench (leg/servo trim,
     zero-calibration, or physical shimming — operator session
     work). `holdbc1_hard1` stays deployed meanwhile.
  2. **Walk-takeoff roll transient: STOP reward/DR sweeps;
     INSTRUMENT, then design a STAGED GAIT-ENTRY TRANSITION.** No
     further perturbation/DR/reward arm may target takeoff (the
     walk-kick / rise-rock / walk-push closures are now a ruling,
     not just evidence). The accepted direction: (a) INSTRUMENT the
     transient first — bench tapes + matched sim replays of the
     first ~1.5 s after gait start (roll rate, per-foot loading,
     which feet break contact, command ramp phase) so the
     transition is designed against measurements, not conjecture;
     (b) DESIGN a staged gait-entry transition — a deploy-side
     entry sequence that brings the gait up in stages (e.g.
     stance-settle → weight-shift → first half-step at reduced
     command, then blend to the policy's steady gait), rather than
     dropping the policy into full command from a cold plant.
     Deploy-side runner work + sim prototyping both in scope;
     training arms only AFTER an instrumented design exists.
- **08-13 ~12:xx: `cw-stand-tiltcomp3` (the 4× exposure follow-up)
  FAILS the same way, and the pre-registered FAIL branch FIRES —
  tipped-exposure training is CLOSED for the standing-lean fix,
  ESCALATED to the operator with a complete dossier.** Quadrupling
  hold/tipped-hold exposure (goal-mix hold 0.1→0.4) barely moved
  adoption: the policy's action-vs-teacher-target MSE closed only
  0.90×→0.80× of the full teacher-vs-nothing signal (bar: <0.5×,
  measured via the same `probe_tilt_teacher` policy arm). Forced-8°
  det tail median improved numerically (5.25°→2.55°) but
  settled/recovered count barely moved (0/12→1/12, bar ≥9/12) — 11/12
  episodes still classed "leaning", one foot still parked (det min
  duty 0.06–0.15, video-confirmed). **NEW cost, outside every
  pre-registered branch: the same park now leaks into NOMINAL
  (untipped) retention** — det hold min duty fell to 0.03 (was
  0.58–0.76 on tiltcomp2's nominal pass) even with zero tip and zero
  falls — more tipped-exposure made the ordinary stance worse, not
  just failed to fix the tipped one. Teacher capability and hold
  income are both measured innocent (tiltcomp2's dig-in); more
  practice is now measured insufficient too. **Both obvious sim
  levers (teacher design, exposure dose) are exhausted.** `hard1`
  (`holdbc1_hard1`) stays deployed, unaffected. ~~OPERATOR DESIGN
  CALL NEEDED~~ **DECIDED 08-13 ~12:4x UTC (ruling at the top of
  this section): mechanical trim, outside RL — no lean-pricing
  reward term, no further tipped-exposure or teacher-redesign arm.**
- **08-13 ~11:xx: `cw-stand-tiltcomp2` (the teacher-defect fix)
  FAILED its forced-tip gate — but cleanly, on the pre-registered
  discriminator: under-ADOPTION, not teacher design.** Det tipped
  tail med 5.25° (bar ≤3), leaning 24/24, parked foot persists —
  while the same checkpoint's teacher, rolled out as a perfect
  student on the same pod, levels to 1.76° at +0.384/tick. Adoption
  measured directly: the policy's actions sit at ~90% of the full
  distance from the teacher target (act-vs-tgt MSE 0.0131–0.0147 vs
  signal 0.0149) — 2M steps with hold=0.1×tipped=0.5 (~5% exposure,
  ~10% of anchor-loss mass) never competed with the warm-start
  habit. Nominal retention = sibling tiltcomp1's band exactly (hold
  det 6/6 vp tail 0.35°, zero falls; min-duty 0.72/slip 0.632 —
  pre-existing lineage cost, not new). Now running:
  `cw-stand-tiltcomp3` (ONE knob: goal mix hold 0.1→0.4, rise/lower
  0.3 each; gate adds an explicit adoption clause act-vs-tgt <0.5×
  signal). Pre-registered FAIL branch: adoption still ~0 at 4×
  exposure ⇒ tipped-exposure training CLOSED with a complete dossier
  (capable teacher + income pays leveling + refused adoption + the
  untrained parent's innate 1.45° recovery beats every tipped-trained
  child ⇒ training-dynamics, e.g. trip-fear near the 10° roll limit)
  → operator design call with data, not conjecture.
- **08-13 ~10:xx: tiltcomp1's mechanism read is OVERTURNED by
  measurement — the standing-lean line is UNBLOCKED and training
  again (`cw-stand-tiltcomp2`, train-0); the operator design-fork
  escalation is withdrawn.** New probe (`probe_tilt_teacher`,
  snapshot 0ca5c4f) rolled out the tilt-comp TEACHER ITSELF
  (bc_target fed as the action) on forced ~6.5° tipped holds:
  a PERFECT student settles at 3.95° — above the run's own 3° bar —
  exactly the closed-loop fixed point of a P-controller on the
  CURRENT lean ((L0+deadband)/2 = 3.98° predicted). And hold income
  DOES price lean (k_track tilt Gaussian σ1.5° vs the level ref on
  tipped episodes; teacher rollout −0.046/tick vs −0.150 staying
  tilted). So "two correct teachers converged ⇒ incentive gap" was
  wrong on both counts: neither teacher was capable (tilt-blind
  supervises the lean outright; tilt-aware backslides as the student
  levels), and the incentive already points level. FIX LANDED
  (snapshot fdc48d4): `train.bc_anchor_tilt_from_settle=1` sources
  the counter-rotation from the episode's post-settle lean (a
  per-episode constant, SNAP_ATTRS pool-safe; default-off bit-exact,
  4 new tests, 54-test anchor suite + 81-pass semantics bank green)
  — probe-verified the ideal student now levels to 1.76° earning
  +0.385/tick. `cw-stand-tiltcomp2` = tiltcomp1's exact recipe + that
  ONE switch (2M discovery, VERIFIED RUNNING ~15.3k fps); gate =
  matched forced-8°-tip probe (tail ≤3°, settle ≥9/12, no parked
  foot) + nominal retention in hard1's band. Residual caveat carried
  in the gate's FAIL branch: tiltcomp1's policy sat at 6.4°, never
  even reaching its teacher's 3.95° fixed point — if tiltcomp2
  under-adopts a probe-capable teacher the same way, the next lever
  is EXPOSURE (hold=0.1 mix / tip prob), not teacher design.
- SUPERSEDED by the above — **08-13 ~08:xx: `cw-stand-tiltcomp1`
  FAILED — the tipped-exposure
  route on the standing lean is CLOSED even with a correct teacher;
  the ~8° hardware lean escalates to an operator design
  discussion.** Matched forced-8°-tip probe (frozen hard1 baseline,
  seed 0): the child holds full height (valid_plant det 12/12,
  h_err 0.9mm — parent 0/12) but NEVER levels: roll_class "leaning"
  in all 24 det+sto episodes, tail med 5.75° (bar ≤3°; parent
  recovers to 1.45° in 11/12), one foot parked every episode (min
  duty 0.01–0.03). The residual lean sits at the 6° comp cap — the
  policy uses the teacher's correction authority to satisfy the
  height spec while staying tilted. Nominal retention milder than
  tip1's (no falls, hold det 6/6 at tail 0.4°) but below hard1's
  band (hold det min-duty 0.69 vs 0.95, slip 0.597 vs 0.136m).
  ROOT-CAUSE READ: two differently-designed teachers (tip1's
  tilt-blind q_nom, this run's tilt-aware counter-rotation) converged
  on the identical stay-tilted habit ⇒ the INCENTIVE is the blocker —
  hold income never prices residual lean, so RL happily trades
  levelness for height under any teacher. Next lever is an operator
  call (price levelness in hold income? non-RL trim on hardware?);
  pre-registered consequence forbids a dose retry. The mechanism
  code (`train.bc_anchor_tilt_comp`) stays built/default-off.
- 08-13 ~07:xx (superseded above): the anchor-side tip-aware
  reference (the lever the tip1 gate consequence prescribed for the
  hardware ~8° standing lean) was BUILT and its first arm launched.
  `train.bc_anchor_tilt_comp` (snapshot 1efc816, default off =
  bit-exact, 6 new tests + 50-test anchor suite + 78-test semantics
  bank green; design note in RISE.md): HOLD-episode anchor target =
  the IK pose counter-rotating the measured lean (soft deadband
  1.5°, cap 6° — the measured action-space expressibility boundary;
  track mode excluded), a proportional posture-feedback TEACHER, so
  tipped spawns supervise LEVELING instead of the tilt tolerance
  tip1 learned from the tilt-blind constant q_nom target. Composes
  with `bc_anchor_foot_z` (the foot-height term now prices the
  asymmetric extension in mm). First arm `cw-stand-tiltcomp1`
  (2M discovery, train-0, warm from footlow2-hard1, tip1's exact
  recipe + this ONE variable): gate = matched-parent forced-8°-tip
  probe (settled ≥10/12 det tail ≤3° AND valid_plant ≥9/12, zero
  park) + nominal retention at hard1's band; FAIL consequence
  pre-registered = tipped-exposure route closed even with a correct
  teacher → escalate the lean to an operator design discussion.
- **CORRECTION (08-13 ~06:xx, cross-track from arch — RETRACTS the
  earlier "warp under-charges slip" insight): the warp-vs-C contact
  parity audit RAN (`probe_contact_parity.py`, matched scripted-gait
  command streams from one settled start, iteration sweep) and the
  physics is IN PARITY.** Loaded-foot slip warp@1/4 vs C@50: within
  ~6% at 0.055 m/s and ~3% at the no-slip band's 0.012 m/s,
  iteration-INSENSITIVE (warp 1/4≈2/4≈4/8≈8/8), zero stance creep
  under pure load (warp cleaner than C); C itself explodes at 1/4
  (NaN), so warp's 1/4 was never the C solver truncated. The
  0.085-vs-0.31 "gap" was a stochastic on-policy TRAINING metric
  compared against a deterministic probe (C's own stochastic replays
  measured ratio 1.42–1.45 ≈ MJX's ~1.44), amplified by the steep
  loadslip-factor clip (raw ratios 1.44 vs 1.27, ~13%). Consequence
  for hw: NO campaign-wide physics fix is coming — the deployed
  crouch-shuffle's and bcgait1-hard1's slip numbers are honest
  properties of the policies, and slip levers stay reward/BC-side.
  Audit data: train-0 `logs/probe_contact_parity/`.
- **08-12 eve: `footlow2-tip1` FAILS both clauses — tipped-start DR
  on anchored stance is CLOSED as HARMFUL.** 50% tipped spawns
  taught tilt TOLERANCE, not correction: forced-8° probe holds
  height (det 12/12 vs parent 0/12) but never levels (tail med 7.2°,
  settled 0/12 vs parent 11/12, one foot parked every det ep), and
  nominal retention broke (untipped hold tilted 7.6°, 6 tilt_roll
  falls vs parent zero). Per its gate: anchor implicated, no further
  isolated-DR retries on the footlow2 lineage; tip robustness needs
  an anchor-side design if hardware demands it. Same cycle:
  `footzsharp1` PASSES — the sub-mm one-foot hover is
  supervision-resolution-limited; `bc_anchor_foot_z` norm 10mm→3mm
  closes the park at 2M (det hold all-six duty ≥0.96 vs parent 0.03).
  A lever for the next consolidation, not a new candidate; hard1 /
  stable1 stand unchanged, promotion call still open (bench-owned).
- **08-12 ~16:1x: `footlow2-stable1` PASSES (second stance candidate,
  real tradeoff) + the level1 lean-fix wait CLEARED with an
  existing-cfg probe, no new code.** `cw-stand-footlow2-stable1`
  (support-polygon gate + rise/lower ramp jitter) clears its own gate
  clean (rise det+sto 12/12 incl. a targeted all-flat cold-start
  probe, hold 6/6 no park, lower 12/12 flush) but hold-mode foot-drag
  is +75% vs hard1 (238mm det vs 136mm) — a second candidate, not an
  automatic upgrade; promotion call still open. Separately, the
  `footlow2-level1` FAIL's "needs a forced-tip probe" wait cleared for
  free: `dr.tipped_start_prob/deg` already override absolutely after
  dr-scale, so an isolated 8° forced tip (no other DR) on hard1 AND
  stable1 shows the hold policy already partially self-corrects (roll
  settles ≤2.6° in 11-12/12 eps) but misses strict height/current
  spec ~5/12 det, 9/12 sto on both — re-attributing level1's park
  reopening to its 3-variable confound, not the tipped axis itself.
  Refilled with the 1-variable isolation: `cw-stand-footlow2-tip1`
  (2M discovery, warm from hard1, tipped_start_prob=0.5/deg=6-10
  ONLY). Detail: STATUS.md WAITING-ON, `rl_docs/RISE.md`.
- **08-11 late MODEL TOUR (all 27 deployable ckpts through the
  interactive play.py session; rl_docs/MODEL_TOUR_2026-08-11.md):
  two NEW deployed-pair defects.** (1) `holdbc1_hard1` sit from the
  142 mm walk plant frame tips tilt_pitch at ~2.5 s,
  DETERMINISTIC (10/10 + clean-stand probe) — do not command
  sit-after-walk on hardware; (2) its belly rise stalls at 55 mm
  forever under the interactive goal ramp (training-profile rise
  passes — profile overfit, a separate axis from the hardware
  rise-rock). Landed in response: `rl_move.sim.eval_session` (the
  session gate, exit-code enforced — run on every stand/sit
  candidate) + `goal.rise_ramp_jitter`/`goal.lower_ramp_jitter`
  (default-off training axis, bank green). Family-wide walk notes
  (height collapse to ~70 mm, CCW veer ~3°/s unpriced in the dep
  lineage, reverse ~20 % of command) map onto the existing tall-wall
  / yaw-lineage / heading-exposure lines — no new axes there.
- **08-11 eve session 2 (19:07–19:19, four camera sessions,
  bench_blast_20260811_19*): learned rise is DETERMINISTIC-FAIL on
  hardware** — 5/5 tilt_roll trips (incl. 22:42's), every one at tick
  ~227 (~9 s, mid-curl) with roll 10.1–10.6° and currents ≤0.27 A.
  From verified clean zero (max pose delta 0.5°), so start pose is
  exonerated; sim keeps the same rise ≤1.7° roll. This is THE stand
  blocker; the queued `cw-stand-riserock1` drained as a STUB (the
  rocking-DR code was never written — run VOID, no science); the
  rise-rock DR axis is still unbuilt CODE work. Scripted `POST
  /api/zero pose=stand` is the working stand-up meanwhile.
- **FULL-NIGHT A/B (18 walks, bench_report): the takeoff transient is
  UNIVERSAL and there is NO policy winner.** Every walk crosses 5°
  roll within 0.6–1.5 s and peaks 13–27°; falls are ~a coin flip for
  BOTH policies (vref1 6/10 fell, tip1 4/7) with no predictor in peak
  size or direction. The early-evening "tip1 clean, vref1 3/3 fell"
  read did not survive the sample — and the A/B has a design confound
  (round 1 is always vref1-fwd/tip1-back, so tip1 never walked
  forward tonight). Verdict: the problem is surviving the takeoff
  transient, not policy choice. Sim-side: the queued takeoff arm
  drained as a STUB (default DR — VOID); the proper relaunch
  `cw-dep-tip1-takeoff25-r1` FAILED with a decisive read — under the
  identical 20–25° injection vs matched tip1 baseline, child==parent
  (0/12 valid both, zero falls both), sim ALREADY recovers static
  tipped starts at the hardware regime, dose lever CLOSED (2nd
  no-separation arm). The takeoff fix must be a DYNAMIC roll-rate
  perturbation during gait start (CODE) or contact/pinning work;
  gate on fell/tail, not peak.
- **Tonight's "thermal wall" was mostly PHANTOM BUS READS.** The
  "L4 hip 150 °C" abort read a steady 33 °C seconds later; the
  debounced watchdog never tripped all night. Single-read temp checks
  in safe_zero/pinned_tip were killing sessions on corrupted bytes —
  now debounced (two consecutive hot reads), and the always-on
  `servo_watch` gained a THERMAL PANIC that kills ALL motion (not one
  servo) on a real overtemp; busy cadence 10→5 s. All deployed. The
  19:18 "L2 hip 72 °C" stays unconfirmed-possible, not proven.
- Turn signs: **+0.3 = CCW from above (matches z-up convention)** off
  the 19:33 camera frames — single reading. **−0.3 still unmeasured**
  (first try silently refused on a pending measure record — fixed;
  rerun coincided with the camera being removed). First item next
  session.
- Recovery loop hardened from tonight's failures: recovery safe_zero
  now `force=true` (a fall always trips the tilt gate), scripted-stand
  fallback when the learned rise trips, demo-aware waits (`/api/zero`
  and `safe_zero` run as demos that `wait_idle` never saw — one abort
  came from reading a mid-glide pose), auto-safe_zero when the opening
  pose isn't belly zero (an earlier stalled safe_zero left L4 knee 78°
  off and quietly hold-hunting — the "twitching leg").
- **08-11 eve: fully-unattended camera bench IS the workflow now**
  (`bench_blast --go --auto --camera 0`: iMac camera records the whole
  session, exact unix sync, video_review cuts the sheets; fall-detect →
  safe_zero → stand recovery loop; terminal results recorded, never
  kickoff responses). Three unattended sessions run 08-11 eve
  (hardware_traces/bench_blast_20260811_18*).
- **Walking on hardware (08-11 eve, on camera):** both policies show a
  large TAKEOFF roll transient. vref1-r1: one clean-start fall (its 3rd
  runaway) and one full-6s walk that rode a 23–24° early transient and
  recovered to dead level — the "runaway" flag conflates recoverable
  transients with tips; judge by fell/tail. tip1 fwd tripped tilt_roll
  2/3 in the 21:4x attended A/B (robot's own log; the old "3 clean
  walks" summary was kickoff-response fiction). **First off-wedge rot60
  run (tip1 BACKWARD): FELL** (peak 27°) — rot60 port itself works
  (k engaged, terminal result logs it).
- **Stand specialist port: first honest hardware run FAILED with a
  REAL gap** (08-11 22:42): tilt_roll trip at 10.2° during the
  belly-curl. Sim probe: the same rise keeps roll ≤1.7° across 6 det
  seeds — hardware rocks over the tucked legs, sim doesn't. Trip
  threshold is correct; fix is training-side (rocking/tilt DR on rise
  ticks, loaded-knee actuator), NOT a threshold bump.
- 08-11 22:29 incident (resolved): unattended session 1 had no upright
  gate between steps → post-fall walks/turns ground the sprawled legs →
  board brownout; operator power-cycled, 18/18 healthy. The recovery
  loop + SessionAbort added in response and validated live in session 3.

## Next

- Sim-side (08-11 late): the two queued bench-answer arms drained as
  STUBS without their variables — both VOID (no science; verdicts in
  ledger). Proper relaunch `cw-dep-tip1-takeoff25-r1` then FAILED
  decisively (see Now bullet): tipped-start DOSE closed, sim
  saturates the static-tilt axis. **08-12: the dynamic follow-up
  landed AND ran (`dr.walk_kick_*` code, commit 7d34fc6;
  `cw-dep-tip1-kick1` trained) — SAME NULL: matched-parent probe at
  the gate's own dose (prob 1, 14–22°, n=24 seeds/side,
  `probe_walk_kick.py`) gives ZERO falls for BOTH child and frozen
  tip1, tail roll well under the bar for both. The WALK
  command-pulse family is now CLOSED (2nd axis, 3rd arm, to saturate
  with no separation) — do not schedule another dose. Remaining
  lever for the takeoff transient is contact/pinning modeling, not
  more command-side DR.** **08-12: rise-rock (same command-bias
  family, belly-curl mode) also FINISHED (`cw-stand-riserock2-r1`)
  — a null too, but in the OPPOSITE direction.** Matched-parent gate
  at the exact bench trip threshold (dr.rise_rock_prob=1.0,
  deg=10,10 fixed, det, baseline hard1): child 0/6 valid_plant (1/6
  tilt fall), hard1 ALSO 0/6 (2/6 tilt falls) — zero separation, both
  sides fail this specific guaranteed dose (own-mix retention at
  prob 0.5/deg 6-12 stays clean, 6/6, no regression). Two roll-
  injection axes now show zero learned separation from a frozen
  parent, in opposite directions (walk-kick: both pass; rise-rock:
  both fail) — mounting evidence this whole "randomize a temporary
  body-roll bias" family isn't teaching resilience either way.
  **08-12: the gentler dose retry ran (`cw-stand-riserock3`, deg
  6-10) — CLOSES the family, but on a new failure mode**: own-mix
  det LOWER collapsed from riserock2-r1's clean 6/6 to 1/6 (worst
  foot clearance up to 126mm vs the 60mm bar), video-confirmed a
  fresh three-leg flag-leg/outrigger cheat (legs 1/3/5 plant hard,
  legs 0/2/4 stay splayed 10-126mm off the ground) — a KNOWN LOWER
  exploit class, one-line STOP verdict, no forensics. Breaks the
  gate's own "no retention regression" clause outright, so it fails
  regardless of the rise-rock injection result in isolation (which
  looked fine: 5/6, no falls). **RISE-ROCK DR FAMILY NOW CLOSED**
  (2 doses, 2 misses: zero separation then a new cheat) — do not
  schedule a third dose. `hard1` stays deployed. Both command-bias
  roll-injection axes (walk-kick, rise-rock) are now closed; the
  remaining lever for takeoff/rocking transients on hardware is
  contact/pinning modeling (belly/foot contact geometry), not more
  DR dose. rot60 backward: one fall AND one clean walk — more reps
  when a takeoff-hardened checkpoint exists (none is coming from
  this lever; look to contact/pinning work instead).
- **08-12: the contact/pinning follow-up ran — open-loop trace
  replay (`rl_move/sim/replay_trace.py`) DIAGNOSED both transients**
  (full findings: rl_docs/SIM.md known-gaps §4). Ten stand-failure +
  nine walk tapes replayed action-for-action in the free-base sim:
  joints track at ~1° RMSE (actuator model exonerated); walk takeoff
  excursions reproduce open-loop (sim 8.7–29.5° vs hw 6–25°) — the
  policy never VISITS them in training; the stand failure is a
  support-geometry knife-edge (hw pivots on L4, left pads unload;
  sim keeps them planted — CoM/μ sweeps don't move it). Two
  calibrated MECHANISM-CORRECTED axes shipped: `dr.rise_rock_*` now
  RAMP-GATED (flat curl → last-1.2 s ramp, matching every tape —
  both riserock nulls tested the WRONG shape, a curl-long rock no
  tape shows; this is the replay-derived shape fix the closure's own
  "remaining lever" analysis called for, NOT a third dose of the
  closed persistent-bias axis) and NEW `dr.walk_push_*` (2.0–3.0 N·m
  half-sine chassis roll torque via xfrc, 0.8–1.5 s; reproduces the
  hardware coin-flip regime policy-in-the-loop where the command-side
  kick saturated at 5–10° — a TORQUE axis, not command-pulse family).
  Push works on both stacks (xfrc plumbed through the MJX batched
  stepper + both vec envs 08-12; warp parity test in
  test_mjx_parity.py). Bank tests green (`test_task_semantics.py`
  WALK-PUSH + rise-rock banks). OPERATOR-ORDERED retrains LAUNCHED
  08-12 (this cycle): `cw-dep-tip1-push1` (train-3, warm from tip1,
  dr.walk_push_prob=0.5 at the calibrated 2.0-3.0 N·m/0.8-1.5 s dose)
  and `cw-stand-riserock4` (train-4, warm from holdbc1-hard1,
  dr.rise_rock_prob=0.5, deg=8,18 — the ramp-gated calibrated
  default). **08-12 verdict: `cw-dep-tip1-push1` is PARTIAL/
  INFORMATIVE — the FIRST real (if sub-threshold) separation in
  this whole family.** New `probe_walk_push.py` (matched-parent,
  forced 2.6 N·m/1.5 s, n=12 seeds/side): child falls 5/12 vs frozen
  tip1 9/12 (1.8x lower, short of the pre-registered >=2x bar), but
  paired by seed all 4 disagreements favor the child and ZERO favor
  the parent — a real, directionally consistent effect, unlike
  walk-kick/rise-rock's exact-zero nulls. Nominal DR0 retention clean
  (gait_valid/slip/prog match tip1's own band, zero new falls).
  Per the campaign's own "more steps cleans up the rough edges"
  pattern (just re-confirmed on `cw-dep-bcgait1-hard1`), queued+ran
  `cw-dep-tip1-push1-hard1` (train-3, 10M, identical recipe) rather
  than closing the torque-DR family on a near-miss. **08-12 verdict:
  FAILS bit-for-bit** — the matched-parent `probe_walk_push.py`
  (n=12/side, forced 2.6N·m/1.5s) gives hard1 the IDENTICAL fall
  count as the 2M discovery arm (5/12 vs frozen tip1 9/12, same 1.8x
  gap, same 4 discordant seeds), tail-roll among survivors slightly
  worse. 10M more steps bought nothing. **TORQUE-DR (walk_push)
  FAMILY NOW CLOSED FOR GOOD** — all three perturb-during-training
  axes for the takeoff-roll transient (walk-kick, rise-rock,
  walk-push) are closed. **08-12 ~08:30: `cw-stand-riserock4` (the
  ramp-gated shape-corrected rise-rock, the family's last variant)
  FAILED the same way riserock3 did** — nominal det LOWER fell to
  4/6 with the video-confirmed outrigger/flag-leg park, rise sto
  2/6 tilt falls; disqualified by its own retention clause. And the
  contact/pinning hypothesis itself is now FALSIFIED: the
  `env.leg_chassis_collision` axis was built (default-off, tests
  green) and tape-replay shows the recorded curls NEVER touch the
  chassis — instead the support-polygon trace found the real
  mechanism: the deployed policy ends its rise on THREE feet
  (L0/L1/L4) with the CoM margin flickering **±25 mm every tick** —
  a knife edge sim survives by a hair-trigger catch and hardware
  doesn't (SIM.md gap 4). New arm `cw-stand-margin1` (2M discovery,
  warm from holdbc1-hard1) prices exactly that via the never-used
  `reward.k_support_margin` term; gate = det-rise plant_margin_mm
  up vs matched parent + full retention + no outrigger cheat.
  **08-12: `cw-stand-margin1` FAILS both pre-registered branches** —
  the margin stat itself never moved (det-rise plant_margin_mm 157 vs
  matched frozen parent 154, inside noise: the BC anchor pins rise's
  trajectory too hard for a new income term to shift it) AND a known
  exploit reappeared in retention: det hold parks foot idx1 (duty
  0.05 vs parent 0.90, visible outrigger in frame) even with
  hold_still_gate+hold_flag_fade already on. **Same conclusion from a
  totally different reward term:** `cw-stand-transdrag1`
  (`reward.k_drag_trans`, charging loaded-foot scraping during
  stand/sit — queued 08-11 night off the new drag-meter finding)
  also FAILS — drag dropped only 10-20% (hold 0.196->0.156m vs a
  <=0.05 bar, lower 0.736->0.658m vs <=0.20) and the SAME idx1 park
  reappeared (duty 0.03), because a foot that's mostly airborne is
  almost never "on" for two consecutive ticks and so can't accrue a
  per-tick drag charge — parking is a free escape valve from this
  charge too. **Between minfeet1 (hold pricing), margin1 (rise
  pricing), and transdrag1 (drag pricing), THREE independent
  reward-side levers now confirm the same closed door: any new
  pricing term on an anchored stand mode gets evaded by parking one
  foot, or doesn't move the anchored quantity at all.** Do not queue
  a fourth. Meanwhile the fleet also got the one CLEARLY ready,
  non-blocked lever on the board: the tall-walking champion
  `cw-dep-bcgait1-hard1`'s own DR/tipped-start retention panel (see
  "Next" below) — its first two axes (friction, ground-tilt) queued.
  **08-12 ~09:5x: the anchor-side spec/verify pass RAN (same cycle
  as the two verdicts) and settled the six-run mystery.** Audit on
  train-0 against the live hard1 cfg: (a) the `_q_nom` theory is
  FALSIFIED — 48/48 hold resets settle with all six feet loaded
  3.2–3.6 N, none under 0.5 N; the anchor reference is a genuine
  six-foot stance. (Suggestive detail: feet 1/4 are the two LIGHTEST
  at settle, 3.19 vs 3.57 N — exactly the only two feet any park has
  ever chosen.) (b) "PPO defies a working anchor" is falsified too:
  rolling the parked margin1 policy through a det hold and scoring
  per-leg action-MSE against the anchor target gives the PARKED leg
  0.0032 vs the clean parent's 0.0031 on the same leg — not even the
  worst leg of the six. **The park is geometrically INVISIBLE to
  joint-space supervision: a mm-scale hover is fractions of a degree
  of hip lift, 3 dims of 18, ~1e-4 of MSE.** That is why six anchor
  variants "converged" while the park persisted, and why every
  pricing term found parking as the escape valve (a hovering foot
  pays no per-foot charge). CODE landed same cycle:
  `train.bc_anchor_foot_z` (+ `_mm` scale) — an additional anchor
  term supervising commanded FK foot HEIGHTS (torch twin of
  `body_ik.fk_all_feet`, z = −F·sin(hip) − T·sin(hip+knee)); a 10 mm
  hover costs ~1.0 at default scale (bank test pins ≥50x the joint
  MSE ratio; default-off bit-exact; 41 anchor tests + 78-test
  semantics bank green). First arm: `cw-stand-footz1-r1` (2M discovery,
  warm from holdbc1-hard1, ONE variable, gate = all-six-feet det
  hold duty ≥0.5 + rise/lower retention vs the matched parent probe
  + no outrigger).
  **RESULT (08-12): PASS (partial) — the fix works.** Det hold: ALL
  SIX feet duty 0.92–0.98 across all 6 episodes (frozen parent
  `margin1` scores 0.05 on leg idx1 in the identical test), valid_plant
  6/6, video-confirmed level quiet stand with zero flag-leg — the
  first clean six-foot hold after 6+ straight pricing-arm failures.
  Two clauses miss narrowly, both matching the parent's own noise
  rate, neither park-related: sto hold valid_plant 4/6 (2/6 trip a
  >2.0A tail-current spec check, not a duty/park issue — no leg drops
  below 0.26 duty); det rise 5/6 vs parent's 6/6 (one flat-start
  height-only miss, zero falls, video reads as an honest crouch-to-
  stand). Det lower stays at 4/6, matching the parent's own baseline
  exactly with the IDENTICAL pre-existing 3-leg-proud pattern
  (confirmed against margin1's own report — inherited, not introduced
  by this run). Hold drag 188mm vs parent's 159mm (+18%, the
  plausible cost of a foot that now actually bears load instead of
  hovering free). `train/bc_anchor_footz_loss` fell 5.1→1.3-1.5 and
  plateaued (didn't converge near 0 — residual hover likely sits in
  rise/lower ticks, not hold). 10M hardening `cw-stand-footz1-hard1`
  queued to consolidate the rise miss and confirm durability; `hard1`
  stays the deployed stance checkpoint until a footz-lineage arm
  passes clean. **08-12: `cw-stand-footz1-hard1` FAILED its own
  gate** — hold survives hardening (det+sto all-six duty 0.92-0.99)
  but LOWER regressed to 0/12 (the known 3-leg outrigger, worse
  under budget, clearances to 170mm); this lineage never had the
  lower-mode anchor its sibling `anchormix1-r1` used to solve lower.
  **08-12 midday: the combination arm `cw-stand-footlow1`
  (footz1-hard1's hold fix + anchormix1-r1's lower-anchor bundle,
  one merge, 2M discovery) FAILED its own gate but is the most
  informative stance arm yet: HOLD stays clean (det duty ≥0.94
  every foot, 6/6) AND LOWER fully recovers (12/12 det+sto, feet
  flush sub-mm vs parent's 0/12 at up to 126mm — video-clean honest
  descent) — the first policy ever with both. The pre-registered
  dilution branch (hold park reopening) did NOT fire; instead RISE
  paid: det 3/6 / sto 2/6, stalling belly-down ~100mm short of
  target — the anchormix lineage's known det flat-rise stall
  (loweranchor1 96mm, anchormix1-r1 106mm), carried into the merge
  by the state-aligned/lookahead bundle. **08-12 (same day): the
  alignment audit RAN (`probe_anchor_align.py`, live stalled policy,
  the run's own cfg incl. loaded servos) and RESOLVED the mechanism —
  a PLATEAU FIXED POINT, correcting the "anchor-BLIND" read: the
  matched ref index PINS at j≈128–137 (0 ticks advance over the last
  3 s) inside the demo's 5+ s 0→25 mm prep crawl, so the +0.5 s
  pursuit target commands only 1–5 mm of height gain (ref_h 6.4–8.4
  mm vs chassis at 4–7 mm), loaded-servo sag (~0.3 s settle) cancels
  it, and the policy OBEYS — mse(act,target) 0.004–0.006 during the
  stall, its episode MINIMUM. The converged `bc_anchor_loss_rise`
  was the anchor actively supervising the stall. Fix landed
  (`train.bc_anchor_min_h_ahead_mm`: height-floor pursuit, target
  tick must command ≥Δmm above current chassis height; default off,
  bit-exact, 3 bank tests + 44-test anchor suite + 78-test semantics
  bank green): one-variable retry `cw-stand-footlow2-r1` (footlow1
  recipe + floor=15, 2M discovery) ran. Side finding,
  noted not attacked: off-path bridge starts (33° RMS from any ref
  tick) match the path END and get supervised straight to plant,
  ignoring the ramp. `holdbc1_hard1` stays deployed;
  rise-from-flat is still the last broken stance mode.**
  **08-12 midday+ RESULT: `cw-stand-footlow2-r1` FAIL per own gate,
  mechanism CONFIRMED, two new residuals (DEEP DIG-IN flagged).**
  The floor works where it aimed: det flat stall moved ~100 mm →
  15–16 mm short, sto rise 6/6 incl 4/4 flat (footlow1: 2/6),
  lower retained 12/12 flush. But (a) det flat still misses the
  height bar by ~15 mm on the eval's seeds while a seed-0 probe
  reaches 3 mm err with the anchor correctly targeting the demo's
  final plant frame (j=313, mse 0.003) — the residual is
  seed/start-dependent endgame, NOT the old plateau; and (b) the
  hold idx1 park REOPENED (duty 0.03 all 6 det eps, valid_plant
  still 6/6) despite `bc_anchor_foot_z=1` — the footlow1
  pre-registered rise/hold seesaw fired one arm late. Next arm
  waits on the seeded audit (which target/mse at the 15 mm-short
  states; why foot-z lost to the rise floor), not another dose.
  **08-12 afternoon DIG-IN RESULT: both residuals OVERTURNED —
  rise-from-flat is SOLVED in this checkpoint.** (a) The 15 mm-short
  "flat" episodes were RSI MID-PATH SPAWNS mislabeled by the eval
  (`rise_rsi_frac=0.5` rides into the gate; `_start_kind` couldn't
  see RSI): floored probe = 12/12 cold flat rises within ±3 mm
  across seeds 0–5 (anchor at path end, mse 0.0028, 6/6 contacts);
  RSI-off gate rerun = det rise 6/6 valid_plant, roll_tail ≤0.3°.
  eval_checkpoint now emits `start_kind="rsi"` (snapshot da367c9);
  judge cold-start clauses on the label. (b) The "park" is a
  +0.9 mm COMMANDED hover (FK probe vs q_nom; footlow1's same foot
  commands +0.4 mm at duty 0.97) — sub-resolution for the 10 mm
  foot_z scale, not the historical 10 mm weight-shed park. First
  policy with rise+hold+lower simultaneously clean to mm scale.
  Queued: `cw-stand-footlow2-hard1` (10M consolidation, PASS =
  deployment candidate incl. eval_session hard gates) +
  `cw-stand-footzsharp1` (foot_z_mm 10→3, one variable, closes or
  refutes the last-mm hover). Detail: rl_docs/RISE.md; artifacts
  logs/experiments/cw-stand-footlow2-r1/digin/.
  **08-12 midday+: `cw-stand-rampjit1` (model-tour ramp-jitter
  axis, holdbc1-hard1 + rise/lower_ramp_jitter=0.3) FAIL — axis
  CLOSED per its own gate.** Session hard gate still misses
  (interactive rise z_end 59.5 vs 60 mm @9.5 s; parent 55) AND det
  lower retention broke (2/6, sto 0/6, outrigger class, clearances
  to 147 mm). Honest positive: the parent's deterministic
  sit-from-142mm-plant tip did NOT occur (no_falls + sit_descends
  PASS, tilt peak 9.1°). Per the pre-registered gate: no dose-down
  retry (not retention-only); next lever is START-STATE exposure —
  and the stance candidate that should face eval_session next is
  the footlow lineage once its gates pass.
  **08-12 afternoon: `cw-stand-footlow2-hard1` (the 10M
  consolidation) PASSES — all four pre-registered clauses, first
  time.** Cold det rises all valid_plant ≤5mm (bridge 2/2, crouch
  1/1 from the gate draw; flat 12/12 via a targeted probe since the
  6-episode draw sampled none, h_err 0.5–3.4mm, roll_tail 0.0°).
  Det hold ZERO real park: all six feet duty 0.95–0.99 at ~0.1–0.2mm
  commanded hover, tighter than r1's own 0.9mm residual. Lower
  12/12 det+sto, feet flush (end_clear ≤0.3mm det/≤6.4mm sto). AND
  it clears `eval_session` HARD gates outright (no_falls/rise/
  sit_descends) — rise reaches full 148mm by t=9.5s under the
  interactive ramp, where the currently-DEPLOYED `holdbc1_hard1`
  stalls at 55mm on the identical protocol. Visual-quality stats
  (drag/roll_tail) flat-to-improved vs the r1 parent on every mode.
  Video-confirmed clean six-foot stance throughout, no flag-leg/
  park/stilt. `ppo_goal_cw_stand_footlow2_hard1` is a genuine stance
  DEPLOYMENT CANDIDATE (sim-only, not yet bench-tested) — the
  promotion-over-`holdbc1_hard1` call is next. `cw-stand-footzsharp1`
  (the paired last-mm-hover probe) still to triage.
  **08-12 ~15:2x: operator hardware milestone — `footlow2-hard1`
  completed the FIRST full belly-stand-belly round trips on the
  bench (2/2), but with a persistent ~8° standing lean.** Two
  hardening arms queued off the hard1 checkpoint to attack it:
  `cw-stand-footlow2-level1` (dr-scale 0.35 + ground_tilt 5° +
  tipped_start_prob 0.30, hypothesis: physics-on + tipped starts
  teaches IMU-feedback re-leveling) and `cw-stand-footlow2-stable1`
  (plant-polygon gate + rise/lower ramp jitter, still training).
  **`cw-stand-footlow2-level1` FAILS** — and not narrowly: on the
  PLAIN flat-floor DR0 retention check (no injection at all, the
  exact test hard1 aces 6/6 clean), det hold drops to 4/6 because
  2/6 episodes REOPEN the historical two-foot park (feet idx1+idx4,
  duty 0.03–0.09, end_clear 1.3–4.1mm, 5° lean, harness
  `success=False`) — the identical failure mode this arm was meant
  to cure, now appearing spontaneously without any DR tilt needed.
  Own-DR0.35 pass is worse (sto hold 2/6, roll_tail up to 9.5°).
  The arm's actual hypothesis (do tipped-start holds re-level to
  <=2°?) was never even tested — the standard eval draw sampled
  ZERO tipped-start episodes across all 24 hold episodes in both
  passes (`start_kind` stayed `plant` throughout); moot, since
  retention already fails on its own. One-line known-exploit stop
  (the park is a named recurring cheat), no forensics. `hard1`
  stays the sim-side deployment candidate; the DR/ground-tilt lever
  for the hardware lean is NOT validated and should not be retried
  blind — any retry needs a start-state probe that FORCES a tipped
  spawn (same fix class as the footlow2-r1 flat-rise mislabeling)
  rather than hoping the random draw includes one.
- Bench (blocked until operator resets): L2 hip hit 72 °C, so motion
  stopped for the night per safety rules. When resumed: wz turn-sign
  audit (STILL open — three sessions in a row died before reaching
  it), more A/B reps (vref1 3/3 fallen — consider dropping it from
  the rotation), learned-lower retry ONLY after the over_load trip is
  understood.
- Runaway metric fix in bench_blast: split "recovered transient"
  (peak high, tail level) from "fell" (terminal result / tail high).
- Gait cleanup (anti-scrape): P0 diagnostic DONE 08-11 late (tilt
  penalties exonerated; paddle is a sim-effectiveness optimum —
  GAIT.md bottom). Structural per-stance charge, FROM SCRATCH
  (`cw-gait-dragstance1`, audit-derived k=8000) FAILED 08-11: parked
  motionless instead of stepping, paying the charge the whole
  episode rather than resolving it (its own pre-registered false
  branch, verbatim) — GAIT.md. CROSS-TRACK: this is also nobc's
  drag-charge-audit item, same conclusion both tracks. Warm-start
  companion `cw-walk-dragstance1` (same k, on the actual champion)
  also FAILED, the other way: it neither parked nor stepped — kept
  full travel and simply absorbed −7/tick for 2M (slip only 1.1–1.3 →
  0.95–1.15). Static fine at either init is closed; the from-scratch
  40M `cw-gait-dragstance1-r1` was KILLED pre-verdict (known-exploit
  rule: identical recipe to the refuted 2M arm — RL_LOG 08-11 20:03);
  the anneal-up curriculum (CODE) carries the lever.
- **TALL LADDER (walk from a taller stance, same problem as
  anti-scrape): the wall is HABIT not kinematics** (`probe_tall_wall.py`,
  08-11 — GAIT.md/RL_PLAN queue -0.5). Ref-tracking alone is tradeable
  for speed (T1); a reachable income gate (T3, `cw-dep-tall-gate1`)
  buys 15mm at 2M but the trade WINS BACK under a 6M hardening budget
  (`cw-dep-tall-gate1-h1`, confirmed 08-11 late): steady-state walking
  height -72.6mm, statistically unchanged from the ungated -75mm wall,
  legs still pinned at the 35° yaw-splay limit (lateral-stability
  purchase). Gate-income alone CLOSED at this dose. **08-11 late:
  PRICING FAMILY CLOSED FOR POSTURE** — kh3 (-74.5mm), kh10 (-72.7mm,
  a 10x height charge that pays MORE than walk income rather than
  stand up), slow1 (-73.8mm, didn't even adopt its eased 0.03-0.04
  speed band, still walking 0.048-0.051) all flat at -72..-75mm, leg
  yaw pinned at the 35° limit in all six pricing arms tried (ref
  ladder, income gate, gate+budget, height 3x/10x, speed relief). The
  optimizer cannot FIND the taller basin at any dose — it isn't
  underpaying for it. RSI-for-walk (`cw-dep-tall-rsi1`, T6)
  was the last lever and it is FLAT TOO (-77.4mm mid-gait; the
  policy learned to recover from tall mid-stride spawns DOWN into
  the crouch — verdict 08-11 22:33, ledger recorded): neither
  pricing (6 arms) nor state injection moves posture.
  **08-12: BC-INIT BREAKS THE WALL (`cw-dep-bcgait1`)** — pure action
  pretraining on the scripted tall gait (`bc_init_gait.py`), then a 2M
  RL fine-tune: `probe_tall_wall` steady height -10..+6mm (every
  pricing/RSI arm above: -72..-75mm), leg-yaw margin now POSITIVE
  +17..+18deg (every prior arm: pinned negative at the 35° limit) —
  the crouch+splay habit is GONE, existence-proof-grade. Harness
  confirms real travel (prog_ratio 0.77, gait_valid 6/6, zero falls,
  roll settles clean). Not yet polished: secondary slip bar missed
  (det 2.12 vs the run's own <=1.8 bar, sto sacrifices a leg 1/6) —
  not hardware-ready, next is a hardening continuation. Detail:
  GAIT.md bottom.
  **Same cycle, the hardening continuation RAN: `cw-dep-bcgait1-hard1`
  (10M) PASSES decisively** — height stays in-band (-8.5..-9.8mm),
  yaw margin stays positive, and BOTH secondary misses are fixed
  (det slip/m 1.43, sto 1.51 with the sacrificed-leg episode gone,
  gait_valid 6/6 both passes, prog_ratio 1.05/0.91, zero falls). Now
  the strongest tall-walking candidate in the campaign; next is the
  standard dep-line DR/tipped-start retention panel, NOT yet run,
  before any Gate 0 consideration. **08-12: panel STARTED** —
  bcgait1-hard1 already trains with dr.tipped_start_prob=0.30 baked
  in (its own gate/own-DR evals already exercise that), so the panel
  gap is the per-axis stress arms the vref1-r1/tip1 lineage went
  through (friction, ground-tilt, latency, encoder noise, ...) that
  this NEW checkpoint has never seen. Queued to backlog (hardening
  phase, warm from bcgait1-hard1's own checkpoint): `-fric`
  (dr.friction_scale 0.4-1.6x) and `-groundtilt5` (dr.ground_tilt_deg
  5.0), both k_current=0 per the standing hardware-arm rule. Two
  axes only this cycle — the historical panel ran dozens one at a
  time over many cycles; treat this as started, not complete.
  **08-12: tipped-start-dose isolation and combined-axis DR-compose
  DROPPED from this panel** — both are already closed generic classes
  (would just reconfirm, not inform); CURRENT_TRUTHS corrected. The
  one real open question — does this lineage share the dep-line's
  walk-takeoff roll vulnerability? — is ANSWERED instead:
  `probe_walk_push.py` generalized to the bcgait1 lineage (no new
  reward stack needed, its physically-relevant cfg already matches
  VREF1_STACK exactly) and run as a pure diagnostic (no training):
  forced 2.6N·m/1.5s injection, n=12 seeds, `cw-dep-bcgait1-hard1`
  falls 6/12 vs frozen `cw-dep-tip1` 9/12 — LOWER, not worse, and
  already close to the push-trained lineage's 5/12 with zero push
  exposure. No push-training respec warranted (family closed anyway).
  Gate 0 for this lineage now needs hardware bench evidence, not
  another sim DR axis. Detail: GAIT.md.
- Crouch-start rise: the fix works (crouchrise1/2/3 all rise from
  crouch) but EVERY dose (0.60, 0.60+mix-restore, 0.45 — crouchrise3,
  08-11) reproduces the identical legs-1+4 flag-leg hold cheat; the
  dose/mix axes are closed. **08-11 later: the reward-pricing lever
  is closed too** (`cw-stand-holdload1` — measured-foot-load income
  correctly taxes the hover per its own bank, but the identical
  legs-1+4 park reproduces anyway, det duty 0.03–0.04, `valid_plant`
  blind to it mid-episode). **State-aligned BC anchor tested
  (`cw-stand-anchorstate1`, 08-11): PARTIAL confirmation** — leg 4
  recovers (duty 0.01→0.93) but leg 1 still parks, and the fix
  stalls flat-start rise + adds lower falls. **`cw-stand-anchorstate2`
  (lookahead 0.25→0.5s) fixes the flat-rise stall and the lower falls
  exactly as hypothesized, but leg 1 still parks (duty 0.03) — sixth
  run in a row, lookahead axis now EXHAUSTED for the park.**
  Follow-up `cw-stand-loweranchor1` (BC-anchor the LOWER ticks toward
  the lower bank's own honest IK descent — the last undocumented
  incentive gap) **SOLVED lower (det+sto 6/6, zero falls, from 2/6)
  but REGRESSED hold to a two-leg park + re-stalled flat rise 96mm —
  root cause found: the three per-mode BC anchors share one ring
  buffer/uniform sampling, so lower's pair volume diluted rise/hold
  supervision (ANCHOR DILUTION, a new testable mechanism, not the
  shared-habit theory).** `cw-stand-anchormix1-r1` (stratified
  per-mode minibatch sampling, equal quotas) **RAN 08-11 23:4x: FAIL
  per gate, LINE CLOSED — but the park MIGRATED.** Stratification
  fixed the seesaw as predicted (lower kept 6/6 det+sto, crouch rise
  4/4, hold det valid_plant 6/6) and the six-run foot-idx1 park
  finally recovered 0.03→0.90 — but foot idx4 parked at 0.02 in its
  place, and det flat rise still stalls 106mm. The persistent habit is
  SHED EXACTLY ONE FOOT; every lever so far only moves which foot.
  Per pre-registration: hard1 stays deployed, stand-specialist handoff
  stands, no further blind axes.
  **08-12: the reopened min-over-feet-load lever (`cw-stand-minfeet1`,
  with per-mode `bc_anchor_loss` logging landed) FAILS the same way —
  `env/hold_feet_factor` 0.105, deep in the same 0.1–0.35 failing
  plateau, while `train/bc_anchor_loss_hold` is LOW and converged
  (0.0107) — a working anchor, teaching the park. PRICING FAMILY NOW
  TERMINALLY CLOSED for the parked-foot habit** (min-over-feet was the
  last untried pricing axis). Rise/lower retention clean, hard1 stays
  deployed. Only remaining lever: anchor-side (find + patch the exact
  reference tick that shows a lifted-leg pose at a plant-adjacent
  state) — unqueued, needs a spec pass first. RISE.md.
- **New sub-line: unified get-up-and-walk (one policy, no scripted
  handoff).** `cw-getup1` (fresh init) and `cw-getup2-r1` (warm-started
  from the rise+hold specialist) both FAIL the same way: getup_S
  never nears the 0.3 gate target, and cw-getup2-r1 shows the
  specialist's inherited stand skill actively DECAYING (0.09→0.06
  over 2M steps) back into cw-getup1's exact static collapse — a
  warm-start prior alone doesn't survive this task. CODE landed +
  banked (`train.bc_anchor_getup`, default off, state-aligned pull
  toward the rise reference demo, 7 tests green): `cw-getup3` queued
  to test whether an explicit anchor (not just a head start) stops
  the decay. Not a joystick blocker (the working handoff already
  composes rise→walk cleanly); this is about replacing that two-piece
  handoff with one policy.
  **08-12: `cw-getup3` PASSES the pre-registered gate** — the explicit
  anchor stops the decay: `env/getup_S` climbed 0.09→0.17 (target
  >0.15) instead of falling, and video shows a genuine floor-to-stand
  rise (2mm→110mm over ~3s, level six-foot hold after, zero flag-leg)
  from one sampled floor-adjacent start. Not yet reliable — a second
  sampled start stayed stuck low the whole episode. Still not a
  joystick blocker; low-priority sub-line, no further budget queued
  this cycle while named stand/walk blockers are unattacked.
  **08-12: the 10M hardening `cw-getup3-c2` (identical recipe, "give
  it the steps it was still climbing at") FAILS — the extra budget
  entrenches a cheat instead of closing the gap.** `env/getup_S`
  plateaued 0.17–0.21 for the full 2M–10M range (never approached the
  >0.30 gate), `reward_getup_hold` stayed ~0.009 (needed >0.05), and
  video confirms the pre-registered "strongest alternative": height/
  footprint keep climbing (0.33→0.73 / 0.37→0.72) while `feet_loaded`
  sits stuck at ~2.7–2.9/6 the whole run — a partial (~4-leg,
  quadruped-like) stand, not a real six-foot one. "More steps" is
  refuted for this lineage (one-line known-exploit stop, no
  forensics); next lever is a pricing/anchor fix, same family as the
  now-closed stand-hold pricing line. Still a low-priority research
  sub-line (not a joystick blocker) — no further budget queued.
  **08-12: `cw-getup4` (one variable, `reward.getup_k_hold` 0.8→2.5,
  ~30x richer six-foot-vs-plateau payoff, warm from the c2 plateau
  ckpt) FAILS exactly per the pre-registered false branch — pricing
  depth is not the binding constraint.** `env/getup_S` ends 0.178
  (same 0.17–0.23 plateau) and `feet_loaded` 2.63–2.67/6 (pinned
  below the 3.5 bar); richer summit income moved nothing because the
  policy never explores far enough from the 4-leg pose to sample it.
  Per prediction-if-false, the next lever is exploration/anchor-side
  CODE (per-start-kind BC anchoring or start-mix reweighting), not
  another coefficient — unqueued, needs a spec pass. Sub-line stays
  deprioritized (the working rise→walk handoff already covers this
  on hardware); no further budget queued.

Detail: **rl_docs/BENCH_REPORT_2026-08-11.md** (tonight's consolidated
bench read + RL implications; regenerate tables with
`python -m rl_move.scripts.bench_report`) · RL_PLAN.md queue ·
rl_docs/HARDWARE.md · RISE.md · GAIT.md.

# multitask — Multitask learning

W&B: tag `track:multitask`. Excess-capacity research; run prefix
`cw-mt-`. Design + verdict labels: **rl_docs/MULTITASK.md** (read it
before triaging anything here — this track has its own binding rules
for what counts as forgetting vs acquisition failure).

**PAUSE LIFTED by operator 08-15 ~17:2x UTC (ops.sh cycle KICK focus
note): the 08-13 operator pause "no longer applies" and MUST NOT be
cited to decline multitask-adjacent work again. Normal launch rules
(blocker-reducing hypothesis, phases, preflights, track containment)
govern this track like any other. Historical context: the pause ran
08-13 ~12:2x → 08-15 ~17:2x with one partial reopening (below); the
withdrawn 08-13 direction call's options (arch recurrence transplant /
command-width curriculum / accept `b2` ceiling / reward geometry
diagnosis) are open again, now informed by the translate1 /
translate-scratch1 double-FAIL (recipe closed, see Now).**

**PARTIAL REOPENING by operator directive 08-15
(fb_20260815T114414_3c40d6, via ops.sh cycle focus note): ONE
authorized arm, `cw-joystick-translate1` (renamed from
"cw-mt-c2-fullcircle1" by fb_20260815T114937_f9078d, see below) — an
all-directions translation-only 40M continuation of `cw-mt-c2`. The
pause otherwise stands for agent-initiated mt work.**

## Corrections (08-15, operator directive + external audit)

- **`cw-mt-c2`'s "acquisition failure" verdict was premature** (ledger
  amended): W&B kvbcuqox shows active learning at 20M (return
  ->~166-172, reward_task 0.07->0.34, walk_prog_factor 0.34->0.80)
  with ep_len collapsing to ~148 — the return was optimized by
  drag-then-fall, an UNSAFE-REWARD/GAIT MISMATCH (flat -10 fall
  penalty + progress income without an all-leg gait gate), not
  demonstrated non-acquisition. Corrected label: PROMISING ACTIVE
  LEARNING + UNSAFE REWARD/GAIT MISMATCH AT 20M.
- **Yaw audit (fb_20260815T113718_baf9d6, VERIFIED in the ledger arg
  vectors): b2/c2 set `goal.walk_yaw_cmd=1` (wz sampled, in obs) but
  never set `reward.k_walk_yaw` (code default 0) — commanded to turn,
  never PAID to turn. The yaw clauses of the b2/c2 verdicts are
  invalid; the wave-1 monotone interference story stands only for
  the linear-command axis at the checkpoint level.** Any future yaw
  claim needs explicitly audited yaw pricing + command-bucket
  metrics.
- New machinery landed for the joystick-translate arm (all
  default-off, bank-tested): `reward.term_cost_per_remaining_s`
  (early-fall horizon cost, REWARD.md row), `goal.walk_cmd_metrics`
  (raw signed v_along/v_cross/wrong-way info keys, active ticks
  only) + headline `joystick/v_along_m_s` / `_cumulative` /
  `active_ticks` (secondary ratio-of-sums, cross, wrong-way under
  `train/`; NO per-heading training bins per fb_20260815T115650 —
  direction panels are eval-only) in the MJX trainer; FULLCIRCLE
  semantics bank (drag-then-fall < 0 at k=12; gait>stall>park all
  four directions).

**PAUSE EXCEPTION + NAMING CORRECTION (operator, 08-15 ~11:4x UTC —
read BEFORE speccing/launching anything from the c2 lineage):**
fb_20260815T114414_3c40d6 is an operator-authorized reopening of ONE
sim question — a joystick-commanded-translation continuation of
`cw-mt-c2` (headings sampled uniformly on [-pi,pi], wz/yaw
identically ZERO, explicit fall pricing, all-support-leg gait gate,
raw signed along-command m/s as the headline). fb_20260815T114937_f9078d
then CORRECTED THE NAME: the run is **`cw-joystick-translate1`**
(W&B display/output/ledger aligned), NOT "cw-mt-c2-fullcircle1" —
"fullcircle" is a banned mechanism-centric label and the launcher now
REFUSES any run name containing it (launch_run.py
`naming_correction`). Headline metrics should be plain/physical:
`joystick/v_along_m_s` (+`_cumulative`), "average signed m/s in the
requested joystick direction over nonzero-command ticks".
fb_20260815T115650_47010c then SIMPLIFIED the training metric
contract (applied 08-15 12:0x UTC, pre-launch): NO per-heading
direction bins in training — headline is only `joystick/v_along_m_s`
+ `_cumulative` + `joystick/active_ticks` (cross/wrong-way stay
secondary under `train/`); fixed 8/12-direction panels are held-out
EVAL only. The launcher refuses joystick launches if binned training
series reappear in the sim tree (`_joystick_metric_block`). General
rule (applies to future runs): name the operator-visible behavior
(joystick translate, stop, rise, lower); sampler geometry, reward
mechanism, architecture, curriculum go in config/tags/notes. The
name has no `cw-mt-` prefix by operator choice — pass
`--track multitask` explicitly so track inference doesn't misfile it.

**Goal:** test whether a fresh command-conditioned generalist
(stand + forward + small yaw/lateral trained SIMULTANEOUSLY, one
coherent reward) beats the sequential specialist-fine-tuning pattern —
both at zero-shot command interpolation and at acquiring a genuinely
new command later (the phase-2 transfer test).

## Now

- **CROSS-TRACK INSIGHT (08-15 ~20:0x UTC, from arch):
  `cw-arch-tf-joymodes-scratch1-fallfix1` (the same from-scratch
  transformer lineage already noted below) was continued to its full
  40M budget with a 14x harsher fall charge (operator-ordered,
  `reward.term_cost_per_remaining_s=12.0`) specifically to test
  whether underpriced falls, not the recipe, were the blocker.
  Result: FAIL — survival stayed at 0/6 det+sto (569/571 episodes
  tilt-terminated) even with falls costing ~-141 instead of ~-10,
  while direction-following held up fine (wrong_way_frac 0.042,
  dir_valid_frac 0.96). This rules OUT fall-pricing as the fix and
  sharpens the earlier finding: it isn't that the recipe rewards
  cheap survival over motion (undercharged falls) alone — under this
  stress-mix command curriculum, a walk that both follows commands
  AND survives doesn't currently exist for this reward/task setup at
  any fall price tried. Reinforces that a genuine command-tracking
  reward/curriculum redesign (this track's call) is the only lever
  left; no further arch-side resweep is coming. Detail:
  arch/STATUS.md "Now", ledger `cw-arch-tf-joymodes-scratch1-fallfix1`.
- **08-15 ~18:3x UTC: `cw-mt-b1-dualgru1` FINISHED (2M discovery) —
  VERDICT: FAIL(no-benefit), the arch-recurrence-transplant option
  is CLOSED.** This was the first of the 08-13 withdrawn-pause
  candidate directions to actually run: b1's exact fresh-init
  narrow-generalist recipe (walk 0-0.06 m/s, ±0.15 rad/s yaw on 20%
  of segments, 40% stop) plus a dual-core mode-gated GRU
  (`--gru-dual`) routed by the LIVE blended command
  (`obs.mode_onehot_cmd`, new 08-13 code) instead of the useless
  episode-constant one-hot. Result: det walk `gait_valid` 0/6 at
  both DR0 (sacrificed legs [1,3]) and own-DR0.2 (sacrificed legs
  [1,2,3]) — the identical splayed-leg paddle/park signature as
  b1's own 2M read (0/6), just a slightly higher `prog_ratio`
  (0.16→0.28-0.30) that never becomes a real gait. Confirmed the
  live-command routing DID engage (walk_stop_frac=0.4 forces
  hold/walk switches within every episode, ruling out FAIL(bug)) —
  so this is a genuine no-benefit result, not a wiring miss: giving
  the multitask policy two mode-gated cores does not fix its
  acquisition shortfall on this recipe. The remaining wave-1 fix
  candidates are command-width curriculum, accepting `b2` as the
  recipe ceiling, or a reward-geometry diagnosis — pick one of
  those next, not another architecture swap on this exact recipe.
  Ledger + W&B `gz8a103k` have full numbers/video refs.
- **CROSS-TRACK INSIGHT (08-15 ~17:5x UTC, from arch):
  `cw-arch-tf-joymodes-scratch1` (a from-scratch causal-transformer,
  arch track) independently re-ran this exact closed joystick
  command-tracking reward recipe and reproduced the identical
  signature a THIRD time: wrong_way_frac pinned ~0.43-0.44, near-zero
  real command-aligned motion (`v_along` ratio ~0.08), this time via a
  leg-sacrifice stilt cheat (video-confirmed, gait_valid 0/6 det).
  Three independent lineages (warm-stilt, fresh-march-in-place,
  fresh-transformer-stilt) now find a zero-net-motion cheat under this
  reward — further confirms the recipe itself, not init/architecture,
  is the blocker. Detail: arch/STATUS.md, ledger
  `cw-arch-tf-joymodes-scratch1`. No mt action taken; recipe already
  closed below.**
- **08-15 ~15:1x UTC: `cw-joystick-translate-scratch1` FINISHED (40M,
  from-scratch comparator) — VERDICT: FAIL, and it CONFIRMS the
  reward-setup hypothesis rather than the lineage hypothesis.**
  Acquisition: `joystick/v_along_m_s_cumulative` stayed flat
  ~0.003-0.006 m/s for the entire 40M steps (never trended) and
  `train/wrong_way_frac` stayed pinned ~0.43-0.44 from step 0 —
  the identical shape/magnitude as the warm twin
  `cw-joystick-translate1`. Harness confirms near-zero real motion
  across all 4 passes (gate det/sto, own-DR det/sto): `prog_ratio`
  med 0.11-0.17 (promotion band 0.75-1.25), `fwd` med 0.05-0.09 m
  over a 60 s episode commanding 0.03-0.06 m/s (~2-3 m expected),
  despite `env/reward_task` climbing to 0.85-0.89 — the reward/
  task-metric disconnect from RUN_INTERPRETATION rule 2. Survival:
  fully solved (ep_len saturates 1500/1500, ~1-2 terminations total
  across ~26k episodes). Gait quality actually DIFFERS from the
  twin: `gait_valid` 6/6 on every pass, no sacrificed leg at all
  (the twin has a persistent single-leg park/stilt, `sac [2]`,
  `gait_valid` 1-2/6) — but the video shows why that doesn't rescue
  it: all six legs cycle in a normal-looking pattern while the body
  sits on the same floor tile for the full episode (frame strips
  `walk_det_0/4`, `walk_sto_1`: identical checkerboard position from
  t=0 to t=48-60s). March-in-place is a listed known-exploit video
  pattern (RUN_INTERPRETATION_RULES #4) — one-line STOP, no
  forensics, no re-run. Because TWO independent initializations
  (warm c2-derived stilt-leg, fresh-random march-in-place) both
  found a different zero-net-motion cheat under the identical
  reward, the joystick-translate reward/command recipe itself (pays
  survival + leg-cycling, not real distance) is the problem, not the
  c2 lineage — the operator's "reward setup favors low-motion
  survival" prediction is CONFIRMED, not merely un-refuted. This
  exact recipe is CLOSED — no further re-runs in either lineage; a
  revisit needs a reward that prices real along-command distance
  much more explicitly than gait-validity/survival alone. The
  operator's one-time pause exception (fb_20260815T122345_2c039a) is
  now fully spent for BOTH arms; the 08-13 pause otherwise stands,
  no new mt launch this cycle. Full numbers: ledger entry + W&B
  tvzk2nn8 notes.
- **08-15 ~14:0x UTC: `cw-joystick-translate1` FINISHED (40M) —
  VERDICT: FAIL, known exploit (parked/stilt leg), not acquisition.**
  The fall-cost + gait-gate fix worked exactly as designed for
  survival (episode length 17->1478/1500, reward_task 0.66->0.83),
  but the actual task metrics never moved in 40M steps:
  `joystick/v_along_m_s_cumulative` stayed ~0.001-0.003 m/s the
  entire run (commanded 0.03-0.06, gate bar 0.015) and
  `train/wrong_way_frac` stayed pinned at 0.43-0.47 from step 0 —
  flat, no learning signal, ever. Video (rollout_627, ~31M steps)
  shows why: one front leg locks near-vertical like a stilt/prop
  while the body stays essentially stationary; `env/reward_park_duty`
  sits at -0.40 to -0.45 all run (chronic parked-leg penalty firing)
  and `env/walk_gait_gate_factor` never rises past ~0.2 (real
  all-leg gait needs ~1.0) despite this run's own gait-gate +
  park-duty pricing built specifically to price this out. So the
  reward_task climb is the shaping proxy being gamed by the stilt/
  park posture, not genuine command-following — this is a REWARD/
  METRIC specification gap (`walk_prog_factor`/`reward_task` payable
  by a planted leg), not proof broad-command walking is unlearnable.
  Per RUN_INTERPRETATION_RULES a known exploit in the video is a
  complete verdict: no forensics, no re-run, no continuation of this
  recipe. Confirms (does not newly discover) the standing
  "one-parked-foot hold habit" blocker already named in
  CURRENT_TRUTHS/hw — CROSS-TRACK INSIGHT noted there, no new hw
  launch from this (containment). `cw-joystick-translate-scratch1`
  (from-scratch comparator, same recipe) is still training under a
  concurrent cycle — same exploit is plausible there given it shares
  every reward term; its own verdict is not implied by this one and
  needs its own video read when it finishes. The one-time pause
  exception for this arm is now spent; the 08-13 pause otherwise
  stands. No new mt launch from this cycle.
- **08-15 ~12:2x UTC: `cw-joystick-translate1` LAUNCHED + VERIFIED
  (train-0, W&B ti7hygbp, 40M, warm from `cw-mt-c2`'s checkpoint) —
  the operator-directed joystick-translation continuation.** Full
  contract confirmed in the resolved W&B config: heading uniform
  [-pi,pi], speed 0.03-0.06, stop_frac 0, 8s±50% segments with
  0.5-1.0s blends, 60s episodes, wz≡0 (yaw obs kept,
  walk_yaw_zero_frac=1), `reward.walk_gait_gate=1.0`,
  `reward.term_cost_per_remaining_s=12.0`, `goal.walk_cmd_metrics=1`.
  Headline metrics live from the first rollouts:
  `joystick/v_along_m_s` ≈0.004-0.013 (cumulative ≈0.011),
  `train/wrong_way_frac` ≈0.35, active_ticks counting — exactly the
  near-zero-signed-projection starting point expected from the c2
  checkpoint under full-circle commands; NO `v_along_hbin*` series.
  Gate (pre-registered, EVAL-side direction splits only):
  eval_cmd_suite 12-direction panel + random 60s sessions, det+sto —
  zero falls, all-leg duty ≥0.10, raw det med v_along ≥0.015 m/s in
  every direction, v_cross med ≤0.03; no auto-FAIL while
  joystick/v_along_m_s + reward_task still rise; verdict needs the
  bulk cohort (EVALS.md §4).
- **08-15 ~12:3x UTC: `cw-joystick-translate-scratch1` LAUNCHED +
  VERIFIED (train-3, W&B tvzk2nn8, 40M) — operator-directed
  (fb_20260815T122345_2c039a) matched FROM-SCRATCH comparator to
  `cw-joystick-translate1`: identical arg vector at bit-identical
  trainer code (rl_move/sim+tests diff 8a8ee7e..HEAD empty), the ONLY
  difference is no `--init-from` (random init; fresh init IS the
  hypothesis, warm-start default waived per the directive).
  Purpose: separate "c2's inherited narrow/unsafe optimum × new gait
  gate" from "the all-direction reward setup itself favors low-motion
  survival". Per fb_20260815T121512_00533c, both arms report
  acquisition (joystick/v_along_m_s ± cumulative), survival
  (ep_len/falls), and gait quality SEPARATELY — no global PASS/FAIL
  from total return, no forced early binary verdict on translate1.
  **12:4x checkup note:** the watcher's low-fps SUSPECT on scratch1
  (~2.9k vs 5k floor) is RESOLVED-benign — slow since iteration 1 and
  climbing with learning while the twin holds 7538 fps on the same
  node; run-intrinsic contact-thrash cost of random init, left in
  place (expect repeat alarms until survival improves). **Triage
  precondition (fb_20260815T123151_a8660c, verified):** scratch1
  retains the twin's `--out-name` — its checkpoint file is ALSO named
  `ppo_goal_cw_joystick_translate1.zip`. Pull it only from
  train-3 / W&B tvzk2nn8 and rename to
  `ppo_goal_cw_joystick_translate_scratch1.zip` before any cross-pod
  use (full note in the ledger entry `checkpoint_provenance`).
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
  Refill (same cycle): **`cw-mt-b-hist16-20m1`** (respec, FRESH init
  — matched to b2's from-scratch design, NOT continued from r1's
  crouch checkpoint — at the b2-matched 20M budget) — RUNNING
  (train-0).
- **08-13 ~00:3x: `cw-mt-b-hist16-20m1` (representation lever, b2-
  matched 20M) FAILS(worse/no-gait), decisively — history CLOSED as
  a lever at both budgets tested.** gait_valid collapses to 2/6 det,
  2/6 sto (gate DR0) and 4/6 det, 3/6 sto (own-DR0.2) vs `b2`'s clean
  6/6 in all four passes: one front leg sits at duty 0.01-0.17 in
  every one of 24 episodes (video-confirmed rigid/splayed, barely
  touching ground) — worse than `b2`'s already-marginal leg-3
  (0.11-0.35). prog med looks flat-to-better (0.60-0.64 vs `b2`'s
  0.51-0.53) and slip_per_m better (2.5-2.8 vs 3.4-3.9), but that's
  the drag-exploit-inflates-progress pattern (five legs dragging the
  near-frozen one), not genuine improvement; roll_tail flat-to-worse
  (4.2-4.85 vs 2.9-4.45). Compound gate already fails on clause 1
  (gait_valid) so yaw probes weren't run — moot. **History (16-frame)
  actively hurts this from-scratch multitask recipe at the exact
  budget where the plain recipe learned a clean gait.** Combined with
  `r1`'s 2M FAIL, the representation lever is closed on this recipe
  at both budgets — no further hist-frames variants. Next per the
  gate's own escalation clause: arch-recurrence or an operator call
  on narrowing command width, not another cheap probe.
  CROSS-TRACK NOTE: not escalated to `arch` — that track's own hist16
  frame-stack line already PASSED on its (different, sequential-
  specialist) recipe, so this is a multitask-recipe-specific
  interaction (marginal weak-leg + history), not a general finding
  against temporal history.

## Next

- The cheap-lever menu for wave-1's acquisition shortfall (capacity/
  arch256, staged-widening, history, all tested at matched budgets
  where applicable) is now EXHAUSTED — every one FAILED or made
  things worse. The 08-13 operator direction call was WITHDRAWN by
  the pause (see banner above); the candidate directions (arch
  recurrence transplant, command-width curriculum, accepting `b2`
  as the recipe ceiling, reward-geometry diagnosis per MULTITASK.md's
  closing rule) are recorded for resumption. Top STATUS.md
  WAITING-ON entry updated to PAUSED.
- ~~08-13: the transplant's code is BUILT.~~ **LAUNCHED 08-15 ~18:1x
  UTC and TRIAGED 08-15 ~18:3x UTC: `cw-mt-b1-dualgru1` FAILED
  (no-benefit) — see Now.** The routing question this arm existed to
  test is settled (routing engaged correctly, architecture still
  doesn't fix acquisition); do not re-queue this transplant on the
  b1 recipe. Original spec kept below for the record.
  — b1's exact fresh-init narrow-generalist recipe (walk 0-0.06 m/s,
  +-0.15 rad/s yaw on 20% of segments, 40% stop) plus
  `--gru-dual --cfg-set obs.mode_onehot=1 --cfg-set
  obs.mode_onehot_cmd=1` (rollout geometry n-envs=256/n-steps=256/
  batch=8192/hidden=256, matching every other `--gru-dual` run's
  established precedent — a mechanical requirement of the recurrent
  architecture, not a second experimental lever), 2M discovery,
  VERIFIED RUNNING train-1 (W&B `gz8a103k`). The blocker being tested was routing: on
  this recipe every episode is mode "walk", so the episode-constant
  `obs.mode_onehot` never exercises the dual-core gate. New
  `obs.mode_onehot_cmd=1` (walk_task `_augment_obs`) derives the
  one-hot from the LIVE blended command instead — commanded stop
  (|vx|,|vy| ≤ `obs.mode_cmd_stop_m_s`, |wz| ≤
  `obs.mode_cmd_stop_rad_s`) lights "hold" (stance core), any motion
  lights "walk" (locomotion core); non-walk modes and the default-OFF
  path are untouched (bit-exact, `tests/test_mode_onehot.py`). Arm
  recipe = b2's cfg + `--gru-dual` + `--cfg-set obs.mode_onehot=1
  --cfg-set obs.mode_onehot_cmd=1`.
- Phase 2 / wave-2 planning should start from `b2` (real gait,
  closest to passing) or `a2` (clean specialist), never from `c2`'s
  broad-command recipe as-is — any future wide-command attempt
  needs a design change (curriculum on command width, more
  capacity, or staged widening from a walking checkpoint), not just
  more steps at the same width.
- ~~[CODE, when wave 1 lands] fixed retained-command suite runner~~
  **LANDED 08-13 (idle-kick cycle): `rl_move/sim/eval_cmd_suite.py`**
  — neither `eval_drive.py` (named linear panel, no wz) nor
  `eval_yaw.py` (fixed yaw panel) could probe an arbitrary exact
  (vx, vy, wz) triple, so it was built: explicit command list
  (`--cmd vx,vy,wz` repeatable / `--suite file.json` / default
  panel), det+sto passes, per-command falls / per-axis + magnitude
  v-tracking error / wz error / loaded-foot slip-per-m (harness
  definition) / servo current mean+p95, machine-readable `--out`
  JSON. Measurement suite, NOT a gate (always exits 0). Smoke on
  `vref1_r1` reproduced known facts (fwd slip/m ≈ 1.0 shuffle;
  wz_err ≈ 0.27 on a 0.3 tip = no yaw tracking). Ready for wave-2 /
  transfer-test retained-suite erosion runs whichever direction the
  operator picks.
- Phase 2 transfer test after wave 1: warm-start A/B/C on the SAME
  new command (larger yaw or backward), fixed 1M/2M/5M budgets,
  measure acquisition speed AND retained-suite erosion.
- Wave 2 levers (one at a time): 256×256 net arch on the best
  generalist; obs history; k_drag_stance; body-height command; seed
  twins of the winner.

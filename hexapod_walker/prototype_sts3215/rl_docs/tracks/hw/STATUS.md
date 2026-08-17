# hw — Hardware joystick robot

W&B: tag `track:hw`. THE MAINLINE — pod priority, operator bench time.

**SIM SPRINT (operator 08-17 ~18:05 UTC — binding while the robot is
off the bench for repair): the fleet's single deliverable is RELIABLE
RISE + WALK IN THE MUJOCO SIM, download-ready. The maintained download
answer is `rl_docs/DOWNLOAD_ANSWER.md` (hierarchy: `footlow2_hard1` +
`bcgait1_hard1` + session controller, det 0.967 / sto 0.853 at n=600).
Sprint gap status: post-lower rise = the ELEVATED `[operator]` fork in
STATUS.md WAITING-ON (+ a queued `[code]` remaining-rise eval probe to
price option (a)); takeoff transient = sim-side complete (entry-slew
composed), bench reps parked; no live session-gate regressions. All
bench-owned `[operator]` items stay parked; recover/tangle redesign
stays `[operator]`-gated and is NOT a sprint item. Full text:
RL_PLAN.md "SIM SPRINT".**

**Goal:** a walking, joystick-driven, standing/sitting/holding robot
working ON HARDWARE by any means necessary. Anchors, scripted blends,
rot-60 wrappers, specialist checkpoints — all fair game. KPI:
unresolved blockers between the robot and reliable joystick control.

## Now

- **08-17 ~22:3x (idle-drain): the postlower `[operator]` fork is
  PRICED — fork (a) (align the eval to train==deploy semantics) is
  the measured-best answer.** Built the `--rise-from-h` eval flag
  (calls the REAL trained `_seq_segment_traj` generator instead of
  the legacy cold sampler — a naive `--cfg-set` is a confirmed no-op
  through this harness) and re-read `spec` (parent) + `spec-pl4` (c4)
  under it, matched, n=1,200 fresh sessions (Cohort c5rr). Result:
  `spec-pl4` crosses `spec`'s OWN post-lower-rise number on BOTH det
  (0.963 vs 0.950) and sto (0.799 vs 0.779) under the fair schedule —
  recovering ~9-11pp of its apparent c4 deficit just from the
  schedule fix — and actually BEATS `spec` on overall sto session
  zero-fall (0.91 vs 0.84). Retention at parity, eye clause PASS (16
  reviewed episodes, both candidates: direct push-ups, no belly
  detour, no new exploit), visual medians in-band. This does NOT
  retroactively flip the c4 ledger verdict (correct under the
  schedule that existed then) — it answers exactly the question that
  verdict escalated. Two product-contract decisions still need the
  operator: upgrade the runner/instrument (and real hardware
  reference) to rise-from-h semantics generally, and/or promote
  `spec-pl4` over `footlow2_hard1` as the stance half of the
  hierarchy. No promotion made autonomously. Detail:
  SESSION_BULK_GATE.md "Cohort c5rr — RESULTS".
- **08-17 ~23:xx (operator order fb_20260817T221115_78b688):
  `cw-recover-any15-retentionrollback-cont1` re-verdicted
  scientifically INVALID — the dig-in bullet below is SUPERSEDED.**
  The cert callback called `_recover_update_admission` with
  `indices=0`, so only training env 0 advanced through the curriculum;
  the other 511 PPO envs trained on the base bucket all run
  (signature: `env/recover_focus_bucket=0.03125`,
  `env/recover_active_families=1.03125`; ~94,510 B0 training episodes
  vs 12-32 each for B1-B8). Every reward/frontier/stall/retention
  conclusion from that run describes the cert probes, not the PPO
  rollout distribution. Fix landed at main `4d1b45d` (admission
  broadcast to all 512 envs + divergence abort;
  `CERT/recover_training_envs_synchronized` must equal 512).
  Per the same order, launched the from-scratch synchronized cohort
  **recover-any16-pop3**: `cw-recover-any16-pop3-s11/s12/s13`
  (seeds 11/12/13, member 0/1/2, NO init-from, 40M each, any11
  recipe + 1M cert / 16 cert envs / retention gate / rollback 4M@0.60).
  **UPDATE (~22:4x-23:0x): the any16 cohort was STOPPED and marked
  INVALID_INTEGRATION_CANARY by operator amendment
  fb_20260817T223644_c8bc48** — the 512-env cert broadcast worked on
  all three, but population sync did not (member 0 self-adopted B1
  and ran ahead to B2/B3; s12/s13 never adopted; root causes: cached
  `wandb.Api` summaries in `_peer_rows` + no post-ACK release
  barrier). Codex landed the fix (72d4c53 + f5aee3f: bootstrap
  barrier at exactly 10 rollouts/655,360 steps, forced summary
  refresh, leader release_BNN after all identity-bound ACKs) and the
  successor cohort **recover-any17-pop3**
  (`cw-recover-any17-pop3-s11/s12/s13`, same recipe +
  `bootstrap-rollouts 10` / `barrier-timeout 900s`, still NO
  init-from) is live per operator directive fb_20260817T225114_a31958
  with a 7-point live integration gate (bootstrap WAIT before
  start_B00, sync=512 certs, single B1 winner, all-ACK before
  release_B01, fail-closed).
  **OUTCOME 08-17 ~23:2x UTC (operator MCP note
  fb_20260817T231211_ba01c4): any17 ALSO stopped at the bootstrap
  barrier, all three rows INVALID_INTEGRATION_CANARY.** The barrier
  itself worked exactly as designed (all three stopped at 655,360
  steps with valid ready_B00 records; no cert/candidate/winner/
  post-boundary training), but start_B00 was never released: a THIRD
  distinct bug — `wandb.Api.runs()` caches the initially EMPTY peer
  page queried before s13 existed, so s11/s12 were stuck at 2/3
  discovery forever. Fix landed at main `686f5628` (fresh
  `wandb.Api(timeout=15)` per unresolved-peer retry + regression
  test). PIDs verified absent on train-0/1/3; evidence preserved; the
  stale duplicate s11 REFUSED ledger row reconciled as
  STALE_DUPLICATE. Relaunch remains `[operator]`-gated on Codex's
  any18 directive (main STATUS WAITING-ON); do not resume any17
  names. No behavioral conclusions from any cohort member.
  **any18 LAUNCHED then FAILED CLOSED 08-17 ~23:2x-23:4x UTC per
  operator directive fb_20260817T231336_93cacc (exact any17 recipe,
  from-scratch, on 686f5628-or-descendant) — a FOURTH distinct sync
  bug, not the one 686f5628 fixed.** `cw-recover-any18-pop3-s11/s12/
  s13` ran on train-0/1/3 (distinct W&B ids `18q6to9f`/`e8qr91fq`/
  `1z5ejwe4`, no init_from). Gate items (1)-(2) PASSED live: all three
  stopped exactly at 655,360 steps with valid, distinct `ready_B00`
  records (correct member/run_id/run_name/population_id/
  root_fingerprint/bootstrap_steps — the exact spot any17 failed, now
  clean). **Item (3) FAILED differently**: the leader (s11) crashed
  with `RuntimeError` at its own 900s `barrier_timeout` having logged
  ZERO "start poll deferred" exceptions the whole wait (`_peer_rows()`
  never raised, it just silently never reached 3/3) — while a manual
  replica of the identical peer-discovery query, run from the
  controller at matching wall-clock times, resolved all three names
  instantly, and W&B confirmed all three correct `ready_B00` records
  were live well before the timeout. So the empty-page-cache fix
  (686f5628) is real but insufficient — something inside the leader
  process itself, not reproducible from the controller, still blocks
  peer resolution, and `wait_for_start` has no diagnostic print for a
  short peer count (unlike `poll()`), so only the crash surfaced it
  at all. Stopped mechanically + cleanly: s11 self-terminated; s12/
  s13 (pre-deadline) were killed once the leader was confirmed dead
  (no member 0 left to ever release the race); PIDs verified absent
  on train-0/1/3. All three ledger rows INVALID_INTEGRATION_CANARY,
  W&B notes updated. Escalated `q_20260817T2340Z` (full analysis + a
  concrete next diagnostic for Codex); per the directive's own
  instruction, no fix attempted this cycle. `[operator]`-gated again
  — do NOT relaunch any16/17/18 names without a fifth, root-caused
  directive.
  **OUTCOME 08-17 ~23:0x UTC (operator MCP note
  fb_20260817T223644_c8bc48): integration gate FAILED — cohort
  STOPPED, all three ledger rows INVALID_INTEGRATION_CANARY.** The
  512-env broadcast fix works (all certs synchronized=512), but the
  population sync does not: s11 (member 0) elected+ADOPTED its own
  B1 at local step 1,966,080 and ran ahead to publish B2/B3, while
  s12/s13 never adopted anything and promoted private B1/B2 lineages
  (log-verified on train-0/1/3). Root causes per the note: cached
  `wandb.Api` summaries in `_peer_rows` (needs `load(force=True)`)
  and no release barrier — all-ACK only blocks election, not the
  leader training ahead. Codex is fixing and will issue one clean
  relaunch directive; relaunch is `[operator]`-gated (main STATUS
  WAITING-ON). No behavioral conclusions from any of the three runs;
  checkpoints preserved on pods; /dev/shm cleaned on train-0/1/3.
- **[SUPERSEDED by the 08-17 ~23:xx INVALID correction above — its
  frontier/stall/retention conclusions are void] 08-17 ~22:xx
  (dig-in): `cw-recover-any15-retentionrollback-cont1`
  FAIL by its gate's frontier clause, but the clause was the invalid
  part — the retention-gated promotion mechanism is PROVEN and it
  proved the recovery line's ladder numbers were INFLATED.** Guard
  items (1)-(3) all live: 8 promotions (B1@1.05M … B8@25.03M), each
  with a saved policy ZIP + curriculum JSON on pod and W&B, every
  `recover_promoted=1` paired with a same-round
  `retention_suite_passed=1`, training-error priority responding to
  fumbles (b4 0.00→0.20, sample prob 0.45→0.06-0.26). Item (4)
  (timed rollback) never fired and is MIS-TARGETED: the observed
  failure is OSCILLATION (a bucket reads 0.00 then 1.00 in adjacent
  1M cert rounds, resetting the consecutive-failure timer; max age
  3.01M vs the 4M trigger), not the monotone forgetting the timer
  assumes. Root cause of the B8 stall: nothing was forgotten —
  over the last 15M steps buckets 0-3 pass 100% of rounds, b4 0.85,
  b5/b6 0.90; the wall is the frontier neighborhood, b7 `crouch_deep`
  (pass rate 0.47, mean 0.59) and b8 `partial_high` (0.14, mean
  0.31), and promotion was correctly denied at 28M/34M because b7
  dipped in the same round the frontier passed. **Measurement
  correction (the real result): `any11`'s "B15" was earned under
  rotating-subset certification with stale passes — its own history
  reads b1=0.06/b2=0.00/b3=0.00 at 15M with frontier 13, and
  b0=0.06/b1=0.31 at 38M with frontier 15 — while matched one-shot
  gate evals put any11 and any15 at EQUAL capability (det 10/18 at
  DR-0 and 11/18 at DR-0.1 for both; sto 0/18 for both). So the
  lineage's honest same-round-certified frontier is B8, not B15, and
  "B8 vs B15" is a certification-standard artifact.** Video honest:
  successes are genuine six-foot recoveries (no flag/stilt/park) and
  the b8 failures are not falls — upright, roll_tail 0.1°,
  end_posture_ok, valid_plant, just short of the held-success height
  (plant_margin 128.8mm vs 143.3mm on a success). Also noted:
  warm-start does NOT carry curriculum state (frontier restarts at
  0). No follow-up arm — recover/tangle redesign stays `[operator]`
  and is outside the SIM SPRINT. **Next, if the line reopens:** make
  the rollback trigger judge a windowed pass RATE instead of
  consecutive sub-threshold age, and aim the first lever at
  crouch_deep/partial_high STABILITY, not at more ladder rungs.
- **08-16 ~21:xx (triage cycle): `cw-recover-any13-tanglersi-bank1` FAIL —
  on-path-bank RSI does NOT crack tangle either; CLOSES the exposure-
  side lever class for tangle entirely (2nd new-mechanism miss,
  after 3 curriculum-weight misses any7/any11/any12) and surfaces a
  new starvation regression.** Tangle CERT success_fraction touched
  0.75 twice in a row (24.1M/25.0M) but never sustained ≥0.7 through
  the 30M budget — it decayed to a 0.1875 trough at 28M and closed
  at 0.5. Independently, the retention floor the gate was watching
  broke: zero-bucket (RSI-protected) read 1.0 four times through 24M
  then crashed to 0/16 at its last reading (29M); bucket 10 crashed
  to 0/16 at BOTH of its last two readings (28M, 29M) right after
  reading 1.0 at 23-24M — the same starvation signature any12 showed
  under 0.80 focus, now appearing even with the DEFAULT curriculum
  mix restored (16-ep readings, so 1.0→0/16 is a real regression,
  not noise). Root-cause note for the redesign: the recover BC
  anchor is eligibility-gated OFF whenever the robot isn't already
  near-upright/near-plant (08-15 anchor directive, by design), so it
  structurally cannot supervise the tangled→upright transition
  itself — fixing this needs a tangle-specific reference trajectory
  or a relaxed eligibility gate, a design call, not another
  automatic exposure knob (see WAITING-ON, `[operator]`). `any11`
  stays the recovery line's reference checkpoint — independently
  corroborated by MCP note `fb_20260816T203228_bc9bad` (operator
  review recommending any11 as canonical; already the parent here).
  No further recover/tangle arm queued this cycle pending that call.
- **08-16 ~18:xx (triage cycle): `cw-recover-any12-hifocus-cont1` FAIL —
  curriculum-weight for tangle is CLOSED (3rd miss: any7, any11,
  any12), extreme focus concentration also actively HURTS retention,
  and the named next-lever CODE (a harvested on-path RSI bank for
  tangle) is BUILT and launching this same cycle.** Pushing
  `recover_focus_mix` to 0.80 (from the proven 0.50/0.25/0.15/0.10
  default) did not just fail to crack the tangle-family plateau — the
  run's curriculum frontier never even reached the wall bucket (B15,
  `tangle`+`bank`) inside its full 20M budget, stalling at B13
  (`tangle_mid`) for the last 3M steps, TWO rungs behind any11's own
  pace at a matched step count (any11, default mix, reached B15 by
  21.1M from scratch). Extreme concentration made ladder-climbing
  throughput WORSE, not better — reinforcing rather than merely
  matching the closure. Unplanned second finding: buckets 0-10
  retention broadly COLLAPSED under the 80% frontier mass (final-cert
  training gate_fraction 0.0 on b0/b1/b5/b6/b7, 0.125-0.1875 on
  b2/b8/b9; harness det agrees, 7/11 early buckets 0/1) — RSI
  protection is wired only to the `zero` family, and the remaining 20%
  recent/weak/uniform mass can't sustain 10 other buckets under that
  much starvation. **`any12` must NOT replace `any11` as the recovery
  line's reference checkpoint — `any11` stays best** (it held buckets
  0-10 solidly at ≥0.8). Video: zero/tangle_mid/tangle_mild det
  successes are genuine six-foot settles; tangle/tangle_deep/bank det
  failures are a genuine low-splay stall, no exploit.
  **CODE-FIRST, built this cycle:** the pre-registered next lever
  (RSI generalized beyond the belly→plant reference, which has no
  equivalent for tangle's non-monotonic untangling motion) is now a
  second, independent RSI axis — `goal.recover_rsi_bank_frac`/
  `_bank_kinds`/`_bank_path` (default off, bit-exact, mutually
  exclusive with the ref-path axis per-episode) plus
  `harvest_recover_rsi_bank.py`, which rolls a checkpoint
  DETERMINISTICALLY (this campaign's own "sto collapses to 0" recover
  artifact would starve a stochastic harvester of successes even on
  solved kinds) through forced episodes of a target kind and keeps a
  subsample of the joint poses from the MIDDLE of every episode that
  reaches `recover_success` — an on-path bank built from the policy's
  own occasional wins, not a hand-choreographed reference. `test_
  recover_rsi_bank_*` (3 tests) + full RECOVER bank green (150/150),
  REWARD.md row, snapshot `1202b816`. Harvested a real bank from
  `any11` on all four tangle-family kinds (350 episodes/kind,
  deterministic, on train-0's idle CPU, ~4572 poses from 762
  successful episodes) — per-kind success rates 0.926 (tangle_mild) /
  0.497 (tangle_mid) / 0.377 (tangle) / 0.377 (tangle_deep), matching
  the historical 0.25-0.44 band on the two hard kinds exactly and
  confirming the source checkpoint's real, if partial, competence to
  harvest from. **`cw-recover-any13-tanglersi-bank1`
  LAUNCHED** (warm from `any11`, NOT `any12`; default curriculum mix
  restored; `recover_rsi_frac=0.5`/`kinds=zero` kept for zero-safety;
  new `recover_rsi_bank_frac=0.5`/`kinds=tangle,tangle_deep,
  tangle_mid,tangle_mild` pointed at the harvested bank). Gate: read
  at 30M or earlier plateau — `tangle` (the actual wall kind) CERT
  success_fraction must sustain ≥0.7 across ≥2 consecutive late certs
  once it reaches frontier; zero + buckets 0-10 retention must hold
  ≥0.8 at the final cert (a regression there would mean the new axis
  repeats any12's starvation mistake). FAIL closes the on-path-bank-
  RSI lever for tangle too and escalates to a reward/BC-teacher-side
  redesign (operator call, named in WAITING-ON if it comes to that).
- **08-16 ~16:xx (triage cycle): `cw-recover-any11-rsi-scratch1` PASS —
  RECOVER RSI genuinely generalizes to from-scratch protection; the
  zero-bucket (flat-belly) wall is now SOLVED as a training recipe,
  and the tangle wall gets its 2nd miss on curriculum-weight fixes.**
  Genuinely from-scratch (any6's exact recipe, `recover_rsi_frac=0.5`
  on kind `zero` from step 0), 40M budget. CERT frontier hit B11
  (zero) at 12.1M steps and cleared it in <1M steps (promoted to B12
  by 13.1M) — NO multi-cert stall, unlike any8/9/10's permanent
  stalls (>27M steps stuck, three separate mechanisms). Frontier then
  climbed cleanly through B12-14 and reached B15 (tangle+bank) by
  21.1M, holding there through 40M — matching/beating any6/7's clean
  pace. Harness eval confirms genuine capability, not an artifact:
  det zero (b11) 1/1 `recover_success`, bank (b16... b15 bank kind)
  1/1, video (recover_det_11, recover_det_16) shows real six-foot
  settle from a fallen/awkward start, no flag-leg/stilt/park.
  `tangle`/`tangle_deep` still fail (0/1 each) — this reconfirms the
  EXACT same statistically-solid tangle wall any7 already named
  (0.25-0.44 cert fraction band), now under a SECOND, differently-
  mechanisms attempt (any7: bigger cert sample + more time; any11:
  RSI + the current default spaced-replay sampler) — **two misses on
  "exposure/curriculum-weight cracks tangle," closing that avenue
  per the two-miss rule.** Secondary: a few early buckets (b0/1/2/9/
  10) miss their single harness sample despite ~1.0 CERT history
  throughout training — matches the already-named PPO-churn cert-
  oscillation pattern, not new forgetting (video for b0/plant_catch
  shows a stable, correctly-postured stance that simply never
  crosses the strict consecutive-hold success threshold). `sto`
  collapses to 0/18 across every bucket on both DR0 and own-cfg(0.1)
  — matches the already-documented any4 action-noise/hold-criterion
  artifact (video confirms a visually stable stance under noise,
  just never holds the required consecutive ticks), not new
  evidence. **Recovery line's best checkpoint is now `any11`**
  (matches any6/7's frontier with RSI protection baked in from
  scratch — the safer base for any future recover-line warm-start,
  since it can never re-entrench the zero stall). One untried,
  no-new-code lever launched same cycle as a genuine (not repeat)
  test before calling tangle a NEW-MECHANISM-ONLY wall:
  **`cw-recover-any12-hifocus-cont1`** (warm from any11, focus mass
  pushed well above the ~0.50 both misses trained under —
  `recover_focus_mix=0.80/recent=0.10/weak=0.05/uniform=0.05`, RSI
  kept on for zero-safety, 20M budget) — if tangle still plateaus in
  the same 0.25-0.44 band even at near-maximal frontier concentration,
  that is the THIRD miss and definitively closes curriculum-weight
  for tangle; the next lever would then be a genuinely new mechanism
  (a tangle-specific on-path RSI bank harvested from successful
  tangle-recovery rollouts, generalizing the rise-path RSI trick
  beyond the belly->plant reference it's hardcoded to today) — CODE,
  not yet built, named here for whichever cycle picks it up if
  any12 also misses.
- **08-16 ~13:xx (triage cycle): `cw-recover-any10-zerorsi-cont1` FAIL —
  RECOVER RSI does NOT rescue an already-stuck policy; the any8/any9
  stuck-lineage rescue is now CLOSED (no third warm-start).** Matched
  A/B vs any9 (same stuck any8 checkpoint, same diffuse masses, ONE
  delta: `recover_rsi_frac=0.5`). Bucket 11 (zero) CERT success stayed
  <=0.125 across the whole 20M budget (last 5 certs: 0, 0, 0, 0,
  0.0625 — no rising trend), and the single-sample harness gate eval
  agrees exactly (zero 0/1 det, `over_current` termination). Video
  (recover_det_11) is the same flat-splay-then-stall pathology as
  any8/any9 — genuine capability gap, not an exploit. Buckets 0-10
  retention also broke at the literal final cert (bucket 9
  gate_fraction 0.5625 < 0.8), consistent with the PPO-churn
  oscillation already named on any9, not new forgetting. Per the
  pre-registered gate: **stuck-lineage rescue CLOSED entirely** (three
  attempts now: any8 spaced-replay, any9 curriculum-mass, any10 RSI,
  all FAIL identically on bucket 11); RECOVER RSI is RETAINED as a
  mechanism, untested until now on a policy that ISN'T already stuck.
  **Refilled same cycle: `cw-recover-any11-rsi-scratch1`** (genuinely
  FROM SCRATCH, any6's exact recipe, `recover_rsi_frac=0.5` ON from
  step 0 — VERIFIED RUNNING train-0, `--phase acquisition`, 40M
  budget). Tests the live half of the hypothesis any8/9/10 never
  reached: does RSI keep a policy from ever entrenching the zero stall
  in the first place, rather than curing one that already has? Gate:
  zero CERT must reach >=0.8 within 3M steps of becoming frontier (no
  multi-cert stall) AND frontier must legitimately reach >=B12
  (tangle_mild) by 40M with buckets 0-10 retained >=0.8. FAIL closes
  RSI-for-zero entirely (from-scratch protection also fails) and calls
  for a genuinely new mechanism (reward/BC-teacher-side) on the zero
  family. Recovery line's best CURRENT checkpoint (until any11 lands)
  stays any7 (B15, tangle+bank, bank solved / tangle contested,
  10/10-episode video-confirmed genuine six-foot recover-to-stand,
  no flag/stilt/park).
- **08-16 ~12:xx (dig-in cycle): the zero-bucket (flat-belly) wall is
  root-caused and the mechanism fix is TRAINING.** Dig-in on the any9
  FAIL found the gap is start-distribution COVERAGE, not anchor
  pressure or pricing: the recover BC anchor already fires everywhere
  (eligible ~1.0) with a near-minimized loss (~0.05, no headroom), but
  the ladder's partial_high/mid/low rungs are LINEAR joint blends
  (f·q_crouch), NOT states on the executable belly→plant rise
  trajectory — so a policy entrenched in the splay-to-low-crouch/
  over-current local optimum never practices mid-rise states (the
  exact exploration gap `goal.rise_rsi_frac` closed for the rise task
  in the footlow2 lineage). The "retention regression" is cert-to-cert
  OSCILLATION (crouch_deep 0.0→1.0→0.0, B10 0.06→0.94), i.e. PPO churn
  from grinding a 0%-success frontier — secondary, not forgetting.
  **Built this cycle: RECOVER RSI** (`goal.recover_rsi_frac` /
  `recover_rsi_kinds`, default-off bit-exact; naturally drawn
  zero-family episodes spawn on a random rise-reference row; forced
  CERT/eval kinds never carry the flag so certification stays pure by
  construction; RSI episodes excluded from rollout/self-cert stats;
  `test_recover_rsi_*` + full RECOVER bank 24/24 green; snapshot
  `a1994dee`, REWARD.md §4c row). **`cw-recover-any10-zerorsi-cont1`
  VERIFIED RUNNING (train-1): matched A/B vs any9** — same stuck any8
  checkpoint, same diffuse masses, same seed, ONE delta
  (`recover_rsi_frac=0.5`, the footlow2-proven fraction). Gate: B11
  pure-CERT ≥0.5 in a late cert + buckets 0-10 ≥0.8 at final cert;
  FAIL closes the stuck-lineage rescue (no third warm-start) and
  returns the line's frontier to any7's tangle wall.
- **08-16 ~10:1x (triage cycle): `cw-recover-any9-lessfocus-cont1` FAIL —
  the SECOND miss on the curriculum-mass hypothesis, CLOSES that
  avenue, plus a new retention regression.** De-concentrating replay
  mass off the stuck B11 (zero/flat-belly) frontier (focus 0.50→0.20,
  recent 0.25→0.35, uniform 0.10→0.30, warm-started from any8's exact
  stuck checkpoint) did NOT unstick it: CERT bucket 11 success_fraction
  went 0.25→0.25 (inherited) then flat 0.0 for the last 7/9 certs over
  the full 20M budget, never reaching the pre-registered ≥0.5 bar —
  frontier never promoted past B11. Video (recover_det_11) confirms a
  genuine capability gap, not an exploit: robot stays flat/splayed on
  its belly and trips over_current before rising — the same flat-
  rise-stall pathology named elsewhere in the campaign. **Two misses
  now on curriculum-mass (any8 concentrated, any9 diffuse), both FAIL
  identically — the curriculum-mass avenue is CLOSED per the two-miss
  rule; do not schedule a third mass resweep.** NEW, unpredicted
  finding: the required buckets 0-10 ≥0.8 retention floor also broke —
  crouch_mid (B6) dropped to 0.25 and crouch_deep (B7) to 0.0 at the
  final CERT. Next lever named by the pre-registered gate: a
  mechanism-level fix (targeted BC anchor exposure or a reward term
  for the zero/flat-belly family), not another schedule tweak — this
  needs new reward/env code, **flagged DIG-IN** rather than designed
  here (see WAITING-ON). Universal-recovery's best result stays
  any6/any7's plain-curriculum lineage (B15, tangle wall, bank
  solved); no product baseline touched.
- **08-16 ~08:1x (triage cycle): `cw-recover-any8-spacedreplay-scratch1`
  FAIL, WORSE than the pre-registered if-false branch predicted —
  spaced replay got permanently stuck THREE RUNGS SHORT of the
  tangle wall, on a bucket neither sibling stalled on.** Same MDP/
  recipe as any6/any7, genuinely from scratch, but with bucket-level
  spaced replay (50% frontier / 25% previous-3 / 15% weakest / 10%
  uniform, default masses) + 16-episode certs (was 8) + 3 retention
  buckets. Frontier climbed cleanly B0→B11 by 13.0M steps (~1M/bucket,
  matching any6/7's pace) then FLATLINED at B11 (`zero` — belly-flat
  + small joint jitter) for the entire remaining 27M/40M steps: 28
  consecutive 16-ep certs average 1.6% success (mostly exact 0/16,
  best-ever 3/16) despite holding 50% of ALL training sample mass the
  whole time. Never reached B12-15 (tangle/bank) any6 AND any7 both
  climbed to — this is not "slower," it never got there. Video
  (recover_det_17, the zero-bucket episode) confirms a genuine
  capability gap, not a cheat: robot starts flat, splays into a low
  crouch by frame 2, never completes the rise in the full 16s episode
  — the flat-rise-stall pathology named elsewhere in the campaign, no
  flag-leg/park/stilt. Single-sample harness eval agrees exactly
  (zero 0/1 det AND sto, both DR-0 and own-DR-0.1 passes). Retention
  of buckets 0-10 is solid at the FINAL read (≥0.8125, mostly 1.0)
  but a non-frontier bucket dipped below 0.8 in 14/28 rounds while
  B11 was frontier — the no-forgetting promise only partially held.
  **Resolves any7's flagged open question: plant_catch (bucket 0) is
  NOT a real retention regression** (training cert 16/16 solid
  throughout, harness confirms 1/1 det; any7's sto 0/18-everywhere
  and this run's identical pattern are the already-documented any4
  action-noise artifact, not new evidence). **CONCLUSION: spaced
  replay as specified is measured WORSE than any6/7's plain cert-
  gated curriculum for climbing THIS ladder**, and additionally
  exposes a THIRD wall (zero, B11) neither sibling hit — plausibly
  because dedicating 50% of mass to a stuck frontier starves the
  varied earlier-bucket exposure that apparently bootstraps the
  flat-belly recovery. One miss on this exact hypothesis (not two) —
  no resample without a mechanism change (e.g. cap/anneal the
  frontier mass share, or attack the zero-bucket flat-rise stall
  directly). The universal-recovery project's best result stays
  any6/any7's plain-curriculum lineage (B15, tangle wall, bank
  solved); no product baseline touched. **Refilled same cycle:
  `cw-recover-any9-lessfocus-cont1`** (warm-started from any8's exact
  stuck checkpoint, not from scratch — 20M budget, VERIFIED RUNNING
  train-9) tests the named suspect directly: ONE coupled change (the
  4 spaced-replay mass shares must renormalize together) — focus
  0.50→0.20 / recent 0.25→0.35 / weak 0.15 unchanged / uniform
  0.10→0.30 — de-concentrating replay off the stuck B11 frontier.
  Gate: bucket 11 CERT success_fraction must show a clear rise and
  reach ≥0.5 in some late cert (vs any8's flat ~0.016 mean) AND
  buckets 0-10 stay ≥0.8 retained, or the curriculum-mass hypothesis
  is refuted and the next lever is mechanism-level (BC anchor /
  reward term for the zero family), not a third mass resweep.
- **08-16 ~06:3x (triage cycle): `cw-recover-any7-tangle-cont1` FAIL on
  its primary bar, but SHARPENS the any6 wall — bank is SOLVED,
  tangle specifically is a statistically solid wall.** Warm-started
  from any6, fresh 40M budget, doubled cert sample (8→16 eps/kind).
  Frontier climbed 0→15 by 34M (curriculum state resets on warm-start;
  network weights don't) then sat flat at B15 for the whole last 6M
  steps — never promoted to B16 (flip). Splitting the combined
  bucket-15 read: **bank cert fraction is now 0.56–1.0, trending to
  1.0 the last 3 reads — solved.** **Tangle cert fraction is flat at
  0.25–0.44 across all 6 late certs (n=16 each) — a real, statistically
  solid wall, not curriculum noise** (any6 could only say "contested";
  the bigger sample confirms it's a genuine plateau). Video (det, all
  4 tangle severities) shows the skill is REAL when it fires: legs
  start visibly crossed, the policy genuinely works them apart and
  settles into a clean six-foot stance within ~2s, holds clean to
  16s — no flag-leg/park/stilt. So this is a success-RATE gap under
  held-out variation, not a missing skill. flip (B16, never trained)
  fails as expected (genuine on-back flailing). Sample-size alone
  didn't move tangle, so the next lever must be different exposure/
  curriculum weight on tangle specifically, not more precise
  measurement of the same mix — `cw-recover-any8-spacedreplay-scratch1`
  (spaced replay, running) is testing one such lever now; if it also
  misses on tangle, per two-miss discipline the next design needs a
  named new mechanism, not a third resample.
- **08-16 ~02:5x (triage cycle): micro-bucket curriculum CONFIRMED —
  `cw-recover-any6-microbuckets-scratch1` PASSED its full 40M budget.**
  The frontier climbed cleanly (every promotion CERT-gated ≥0.8, no
  shortcuts) from B0 all the way to B15 by 22M/40M steps — the exact
  B4/B5/B6 cliff any4/any5 were built to diagnose is GONE, and this is
  the furthest the universal-recovery line has ever reached. It held
  the B15 peak (tangle+bank) for 5 straight certs (tangle 62.5–100%,
  bank 75–100%) before one bad bank-kind cert (12.5%) triggered the
  designed retreat at 28M; the remaining ~12M steps oscillated B13–B14
  (tangle_mid/deep, CERT ~0.25–0.75, never resettling ≥0.8), finishing
  at B13. Video confirms genuine six-foot-loaded stable stands (roll
  tail 0.1–0.3°, height err ~0mm) on every certified bucket incl.
  tangle_mild/mid and bank — no flag-leg/stilt/park anywhere. NEW WALL
  NAMED for the next arm: tangle-family recovery (self-tangled legs)
  + the harvested bank poses, still actively contested (not
  flatlined) when the budget ran out — a real capability gap, not a
  curriculum artifact. Next: warm-start continuation from this
  checkpoint targeting the tangle/bank wall specifically (see
  `cw-recover-any7-tangle-cont1`, launched this cycle).
- **08-15 ~23:1x (operator-kick cycle, fb_20260815T230538_a6f8d2):
  micro-bucket curriculum live.** Operator found B4→B5 and B5→B6
  were cliffs and replaced the coarse ladder with 17 baby-step
  buckets (B0 plant_catch … B16 flip, commit `3d556232`); promotion
  authority is now the literal latest deterministic held-out batch
  fraction (≥0.8 promote, <0.2 retreat; EMA never gates), with exact
  per-bucket stochastic training scores counting EVERY terminal env
  (`TRAIN/recover_bucket_N_success_fraction/_successes/_episodes`)
  and deterministic authority as
  `CERT/recover_bucket_N_success_fraction/_successes/_episodes`.
  `cw-recover-any5-mjxcert-scratch1` STOPPED at ~7.7M and preserved
  as the coarse-bucket cert diagnostic (bucket identities changed —
  no continuation). Successor **`cw-recover-any6-microbuckets-scratch1`**
  launched FROM SCRATCH on train-1 (W&B 78xjmlov; no --init-from, no
  transplant; clones any5's full 40M recipe), bank green (117/4/1).
  Verified live: B0–B16 map in W&B config, TRAIN B0 metrics logging,
  first 1M cert fired — CERT B0 fraction 1.0 (8/8 episodes), frontier
  0→1 promoted by the deterministic authority. Watch: how far the
  frontier climbs past the old B4/B5-equivalent rungs.
- **08-15 ~22:3x (superseded by the entry above — any5 stopped at
  ~7.7M under fb_20260815T230538_a6f8d2; triage + operator directive
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

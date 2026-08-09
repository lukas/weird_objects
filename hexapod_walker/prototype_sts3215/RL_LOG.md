# RL_LOG — condensed campaign log

Full detailed history through cycle 34 (verdict evidence, launch
records, operator notes): `archive/RL_LOG_FULL_2026-08-09.md`.
Per-cycle transcripts: controller `/workspace/cycle_logs/`. Run
specs + verdicts: the ledger (`orchestrator/experiments.json`) and
W&B notes.

**APPEND RULE (2026-08-09, operator): new entries are 1–3 lines —
run, verdict, takeaway, what launched.** Evidence lives in W&B, the
ledger, and the cycle log; do NOT reproduce it here. One `##` line
per cycle. If an entry needs more than ~5 lines, put the detail in
a file under `archive/` and link it.

## State (as of cycle 40, 2026-08-09 ~13:00Z)

- **WALK CHAMPION: ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c**
  (promoted cycle 44, seed-confirmed; DR1.0 det slip 1.06; NOT
  hardware-ready — paddle-creep persists, sto draw-stalls).
  Reward side of skating CLOSED pending loadslip-s1 concordance;
  slip root = sim contact/current pricing (operator calibration).
- **Stance:** heights 12/12 at DR 1.0; lower-line rework authorized
  per 08-09 rulings; current-economy arms operator-blocked.
- Compute: 16 GPU pods (train-0..15; 12 ready), backlog-drained
  automatically. capacity.py = live truth.
- Per-run summaries: rl_docs/runs/<run>.md (generated; do not edit).
## Walk line — what was tried and learned (chronological)

- **dr04b lineage** (pre-08-08): scalar champion, 0/9 gait-valid on
  video — shuffle/flag-leg. RETIRED as warm-start source. Lesson:
  scalars hide exploits; video eval is the promotion standard.
- **w07** widening, **flag/flagw** flag-leg penalty (both routings),
  **lp** speed curriculum, **speedhi** speed pressure: all REFUTED.
  Penalty-coefficient iteration on the shuffle is a dead approach.
- **nv/nv2** deployable-obs baseline: called at 8M — obs are not the
  blocker. **aac** asymmetric critic: retention tool (rise 11/12 vs
  3/12), not a gait fix; keep `--asym-critic` on warm continuations.
- **phase / phase-stance / phase-stance2**: phase reward as basin
  escape REFUTED in both basins.
- **step0** (08-08, operator recipe: step-event + drag + park-duty
  pricing, fresh init, walk-only, audited exploration): **FIRST
  genuine six-leg gait of the campaign.** Lineage standard since.
- **step0-c2 vs step0-lowent** A/B: entropy runaway confirmed as the
  plateau driver; warm starts use ent 0.001. lowent → champion.
  Identical-config continuations (c1/c2, lowent-c1, h15b-c1,
  anchorgate-c1, parkstart-c1) bought nothing 5x — CLOSED as a move.
- **h15b** (15 s horizon): real slip/std side-gains; 15 s is the
  lineage eval standard. **h15b-dr03 + probes (cycle 23): DR is NOT
  the bottleneck** — untrained parent passes DR 0.3/0.6 and misses
  DR 1.0 only on det slip. Fixing skating IS the DR 1.0 rung.
- **kgate** (progress-gated kernel income): cut park income ~5x,
  park unchanged — park PRICING refuted.
- **Cycle 27 investigation (big one): there is no park attractor.**
  Own-park rate ~0.15%; the recurring "park" episode was the fixed
  backward-command draw — champion has ZERO rear-hemisphere
  competence (operator-scoped). Real defects: **paddling** (91% of
  slip mid-stance, all legs) and **overspeed** selected by the
  fwd≥0.40 gate clause (operator ruling pending).
- **parkstart-mjx** (reset diversity): first mechanism to move det
  churn; c1 at update parity refuted the dose theory.
- **effort** (effort/CoT pricing): charge paid, nothing moved —
  paddling is not effort-reachable. **phaseprior**: clock locked
  (0.47→0.93), slip unmoved — timing orthogonal to anchoring.
- **anchorgate** (anchored-stance income gate, cycle 31): FIRST of
  four levers to move slip, det 1.543→1.240 → champion. **Income
  GATING works where charging and timing failed.**
- **anchorgate-c1 (cycle 32): cadence-inflation exploit** — policy
  inflated stance count +23% to re-buy the per-touchdown 10 mm
  allowance; free slip = cadence × tol, gate went non-binding.
- **anchortol5 (cycle 34): tolerance rung CLOSED.** At a binding
  tol=5 stake the policy neither anchored nor floor-rode — it PAID
  the gate (−18% walk income, frac stuck 0.74) and kept creeping.
  **Income gating has hit its price ceiling: no income lever can
  outbid in-sim-free sliding.** Slip root cause is contact/current
  pricing (an operator ruling, like stance).
- **step0-anchor** 40M fresh init (cycle 33): re-derives the paddle
  and the allowance-ride from scratch at 2x cadence + "drummer leg"
  (one leg 30–43 swings/ep; gait_valid can't see over-active legs).
  **Paddling is the sim's preferred transport under current pricing
  — a pricing problem, not a basin/history problem.** No more
  fresh-init walk arms without a pricing change.
- Exploit-watch columns (permanent): cadence/stance count, per-leg
  swing asymmetry, allowance-riding, unload-sweep.

## Stance line

- **stance-dr10 / stand-dr10**: heights SOLVED at DR 1.0 (crown
  jewels, canary-protected). Lower posture (flag leg) never solved.
- Refuted in order: raise-heavy mix (raise DEMOTED to canary),
  posture pricing (airborne legs = zero gradient), exploration
  (posture2: warm start + flat ent 0.01 = std runaway, 2-for-2),
  terminal pricing (endpost r1/c1: redistribution manifold), reset
  diversity (bellyrest: basin visited, hover still chosen), dense
  whole-episode charging (lowerdense: hover PAYS the ~2–5% charge,
  and the arm eroded rise — stop rule tripped, ckpt quarantined).
- **Root cause (cycle 28): the hover is INCOME-POSITIVE** — current
  model prices planted descent at 2.6 A (4x hover) and
  support_margin pays a tripod more than six planted feet.
  **BLOCKED on operator ruling** (60 mm allowance + current pricing
  + margin shape). No stance shaping arms until then.

## Infra lessons (all landed)

- Launcher is mandatory: capacity by node (not pod), host loadavg
  check, code-SHA gate (a pod ran 4M steps of pre-audit code
  silently — `lowent-dr03` INVALID), duplicate-name refusal.
- Ledger: locked writes via `launch_run.py update` only; the
  c1/c2 "clobbering" scare was a second repo tree on the controller
  writing a shadow ledger (now symlinked).
- Canaries + regression auto-stop work (saved 2.7M wasted steps on
  posture2) but must protect only skills the lineage HAS
  (`step0` walk-only lineage runs `--no-canary` after a false stop).
- Seed twins before the 08-08 seeding fix were bit-identical clones;
  those twin conclusions are void.
- Watcher: concurrent cycles, async checkups, event-driven triggers,
  auto-continue for flagged lineages (trailing cycle keeps kill
  authority — exercised on step0-anchor-c1). Snapshot-after-launch
  leaves lineage pods a commit behind auto-continue's code gate;
  sync the pod at snapshot time.
- MJX/GPU switchover (08-09, operator): 20–40M-step runs at ~15k
  fps; obs-pad transplant port validated for warm starts.
- Eval: fixed-draw sto panel can pin a "failure" to one command
  draw (cycle 27) — check WHICH episode fails before theorizing.

- Cycle 38 (08-09 ~12:1x): `cw-walk-longdist-r2` NEAR-MISS (strict
  gate FAIL, sto 5/6: one draw stalled 0.62m/slip 6.1) but det 6/6
  is CAMPAIGN-BEST: 1.57m@30s, det slip/m 0.96 (first <1.0; champ
  1.24), prog_ratio 0.98 (champ 1.43 overspeed), gait_valid 12/12,
  frames show level six-leg gait holding the full 30s. NOT
  hardware-ready (slip reduced, not gone). DR1.0 own-cfg eval
  running + `cw-walk-longdist-s1` seed twin queued (ruling-7 panel)
  — champion shift decided next cycle. `cw-walk-longdist` (r1):
  FAILED infra at init, 0 steps, carried by -r2. INFRA: fixed the
  fleet-stalling drain bug — snapshot.sh --sync marked pods
  "-dirty" off the watcher's own ledger/backlog churn, so EVERY
  drain launch was REFUSED while 9 GPUs idled; dirty-check now
  excludes the two runtime state files (exp/drain-dirtyfix,
  df10dbb). Synced+pushed champ ckpt to t4-11, ran
  bootstrap_train_pod.sh on all 8 (nobody had). Cleaned orphaned
  `cw-walk-fwdband` INTENT (authoring cycle zombied; 0 steps) ->
  requeued -r1. My 4 launches (cap): fwdband-r1, standwalksit
  (retry), loadslip-s1, longdist-s1 — 80M = cap. Drain verified
  working again (fast->t0, endur60->t4).
- Cycle 38 addendum: r2 DR1.0 own-cfg eval LANDED — det slip/m 1.06
  vs champion 1.240 (same condition, −15%, first sub-champion slip),
  prog 0.91, det 6/6/0-term; sto 3/6 (robustness = weak edge).
  Champion NOT shifted yet — decided by `cw-walk-longdist-s1` panel.
- Cycle 39 (08-09 ~12:3x): `cw-walk-dr05-r1` FAIL (gate) — DR0.5:
  0 term but det slip/m agg 1.56 (>1.24; one 3.66 blowout) + sto gv
  5/6 (leg-5 sacrifice, slip 23); DR0 retention PASS (det 6/6, prog
  1.00, slip/m 1.06). Gait largely survives DR0.5, stochastically
  brittle. `cw-steer-fdiag` FAIL/REFUTED — det diag tracking within
  noise of champ baseline, sto WORSE (tilt_pitch fall + 2 flag-leg
  eps, gv 4/6); fwd retention held → not dilution. Pre-registered
  consequence: paddle gait blocks steering; SCOPE-ONLY STEERING ARMS
  STOP (note for requeues of diag45/steer-explore class). Both point
  at sto instability → implementing plan rung 0-c(i) STABILITY
  pricing this cycle.
- Cycle 39 (cont): ROOT-CAUSE CHAIN for stability pricing (0-c(i),
  coefficient change): behavior = sto/DR episodes degenerate (tilt
  to ~11 deg, flag leg, slip/m 5-23, one tilt_pitch fall) <-
  incentive = outside the 1.5-deg kernel tilt is nearly free
  (k_roll/k_pitch=10 quad => ~0.3/tick at 10 deg vs ~2-3/tick walk
  income) and a fall costs 10 of a ~900 return <- pricing = both
  knobs sized in the balance-task era <- not a sim defect (frames +
  safety-layer trip mirror hardware). Scale audit: k=50 => 1.5/tick
  at 10 deg, 0.03/tick at 1.5 deg (normal gait unaffected); fall
  charge 300 ~= 1/3 episode return. Launching split one-variable
  arms off dr05-r1 (where the pathology lives): `cw-walk-dr05-tilt50`
  (k_roll/k_pitch 10->50), `cw-walk-dr05-fall300`
  (safety_termination_penalty 10->300). Idle-slot HARD reasons:
  remaining sound arms await in-flight results (fast/loadslip/
  longdist-s1) or are [CODE] (turning, payload, push-recovery,
  quadruped, mirror-sym — need a dedicated implementation cycle);
  scope-steering arms stopped by the fdiag verdict; concurrent c38
  holds 3 INTENTs.
- Cycle 40 (08-09 ~12:5x): `cw-walk-loadslip` FAIL / hypothesis
  REFUTED exactly on its pre-registered if-false: the policy paid
  the episode-level slip stake (walk income −32%, walk_prog −43%,
  speed flat) and KEPT sliding — DR1.0 own-cfg det slip/m 2.39 vs
  champion 1.240 (WORSE than parent), DR0 det 1.54; gait_valid
  24/24, 0 term; frames = the same paddle-creep. Takeaway: every
  reward-side lever (anchor gate, tol5 stake, stepdisp, episode
  loadslip) is now exhausted → reward side of skating CLOSED
  pending seed-1 concordance (`cw-walk-loadslip-s1` RUNNING t6);
  slip root = sim contact/current pricing (operator calibration
  class, cycle-28). NOT hardware-ready. Refills: queued
  `cw-pose-track` (wishlist #18, 10M, pose goal-mix off champion,
  canaries ON) and recovered `cw-walk-endur60` from backlog_failed
  (infra kill, wandb.env fix landed) → backlog depth 2 for 3 free
  slots; any residual idle slot = awaiting in-flight verdicts
  (fast/longdist-s1/loadslip-s1) + steering-scope stop (c39) +
  current-economy operator-block; rest [CODE].
- Cycle 39 (close, ~13:1x): stability twins LAUNCHED + verified
  two-phase: `cw-walk-dr05-tilt50` (t9, W&B ~10.9k fps) and
  `cw-walk-dr05-fall300` (t10, ~10.2k fps), both 20M off dr05-r1
  (13a668ea), snapshot 59f0b4a. First tilt50 attempt lost a pod
  race to c38's wander on t4 (launcher aborted cleanly, 0 steps;
  ledger REFUSED) — 2 more attempts refused on moving HEAD, normal
  concurrent traffic. 40M GPU steps, 2 launches this cycle
  (+1 aborted), caps respected.
- Cycle 41 (08-09 ~13:2x, checkup-triggered): SUSPECT flags on
  cw-walk-wander (t4, fps 4369) and cw-walk-highgait (t7, fps 2913)
  are FALSE ALARMS — instantaneous fps from log deltas: wander 10.9k
  @10M steps, highgait 7.3k @4.6M and climbing; the checkup window
  caught startup transients (compile + first eval/video). No kill, no
  rebalance; resolutions in ledger. 0 free slots (drain placed
  backforth on t3), no launches. Fixed ops.sh entry_field to prefer
  live entries over REFUSED/KILLED husks (loadslip-s1 trainlog had
  pointed at the wrong pod).
- Cycle 42 (08-09 ~13:3x): `cw-walk-fast` FAIL / REFUTED on its
  if-false — commanded 0.08-0.12 m/s but walked its usual ~0.065 in
  all 12 eps (prog 0.53-0.72 <0.75); gait fully retained (gv 12/12,
  0 term, det slip/m 1.14, no sacrificed legs; one sto blowout =
  known c39 brittleness). Speed ceiling is GAIT-limited → faster-
  walking / band-scope speed arms join skating behind the operator
  contact-pricing calibration; no follow-up queued. Pipeline: fixed
  the drain stall — recovered `cw-walk-endur60` + `cw-pose-track`
  from backlog_failed (parked on stale pod code markers, since
  synced) and dropped 2 stale parked entries for running runs;
  backlog depth 3, 0 free slots (hard reason: capacity).
- Cycle 43 (08-09 ~13:2x, steer-explore): `cw-walk-steer-explore`
  FAIL (exploratory, non-promotion) — full-heading (±π) training
  left the forward gait intact (fwd DR0 det 6/6 gv, 0 term, dist
  0.70 vs champ 0.73 m, slip/m 1.36 ~ champ 1.15) but NO omni base:
  off-axis/rear draws don't transport (prog 0.22-0.37, slip/m 5-7,
  body creeps forward instead of along command). Same contact-
  pricing root as skating/speed; confirms rear-hemisphere deferral +
  c39 steering stop; no requeue. Pipeline: backlog EMPTY (drain
  placed speedband→t0); every sound READY arm is in flight (12
  busy), speed/steer/current arms operator-blocked, payload needs a
  DR-range cfg hook = [CODE]; next cycle facing idle pods should run
  the mirror-symmetry implementation cycle (plan #3).
- OPERATOR (08-09 ~09:5x): killed `cw-walk-diag45` at 11.2M (near-
  duplicate of the refuted fdiag class; ledger updated). Landed the
  rough-terrain training hook (`env.terrain_amp`/`terrain_seed`, all
  env paths, warp-only guard; amp 0 = legacy exact). Local preview:
  champion paddles across even amp-1.0 bumps unimpeded — terrain is
  a ROBUSTNESS lever, NOT a slip fix; queued `cw-walk-terrain05`
  (backlog) framed accordingly. Slip exits stay: current-pricing
  calibration (operator) + the longdist task-pressure line.
- Cycle 43 (close): t4 freed mid-cycle (wander finished; its verdict
  = watcher's next cycle). Idle-slot HARD reason on t4: backlog
  empty + zero sound unblocked one-variable arms (READY wishlist all
  in flight; speed/steer/current operator-blocked; payload needs a
  DR-cfg hook). 0 launches this cycle. Next cycle with 2+ idle pods:
  mirror-symmetry implementation cycle (plan #3), or wander/tilt50/
  fall300 follow-ups once their verdicts land.
- Cycle 44 (08-09 ~13:5x): `cw-walk-longdist-s1` seed twin CONFIRMED
  r2 on every axis (DR0 det 6/6, slip/m 0.94, 1.63m@30s; DR1.0 det
  slip 0.98 vs r2 1.06 vs champ 1.240; sto misses = the SAME fixed
  draws in both seeds → lineage trait, not seed luck) → **champion
  PROMOTED to ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c**; NOT
  hardware-ready (slip ~1/m, DR1.0 sto 3/6). `cw-walk-diag45`
  operator-killed (c43); W&B outcome notes added for both. Killed
  drain-launched `cw-walk-speedband` at 6M: stale spec, its 0.11 m/s
  gate pre-refuted by c42 fast (ceiling ~0.065) — requeued as
  `speedband-r1` (0.02–0.06 band, resample kept). Queued
  `cw-walk-longdist-dr05` (DR-harden new champion; drain→t2
  verified). Idle slots = concurrent verdict cycles' refills for the
  ~10 runs that finished mid-cycle; terrain05 INTENT+live on t1 left
  to watcher checkup.

- Cycle 45 (08-09 ~14:0x): 4 triages. `cw-walk-lowgait` PASS (−20mm
  crouch: gv 12/12, 0 term, mean end-height err ~4mm; slip ~champ;
  one sto in-place-paddle ep = known lineage brittleness).
  `cw-walk-wander` PASS (±45°/5s resample/15% stops: gv 12/12,
  0 term, prog ~1.0 through changes; change-eps slip ~2x straight).
  `cw-walk-fwdband-r1` FAIL/refuted — DR1.0 det prog median 1.40 vs
  champ 1.43: command MIX is not a lever; with fdiag this closes the
  command-scope line. `cw-walk-dr05-fall300` NO-EFFECT on its
  if-false: new parent baseline dr05-r1@DR1.0
  (logs/ckpt_eval/cw_walk_dr05_r1_dr10) is ALSO 0-term with
  identical slip/prog — falls too rare to price; 0-c(i) now rides on
  tilt50 (reuse that baseline for its verdict). Refills:
  `cw-walk-wander30` (30s driving endurance, t3) + `cw-walk-lowgait30`
  (−30mm envelope, t4), 40M. t8/t10 idle HARD reason: no sound
  unblocked arms left (READY items all running/blocked/[CODE]);
  tilt50/loadslip-s1/highgait land within the hour and 0-c(ii)
  parents on tilt50's outcome; mirror-symmetry (plan #3) still needs
  its dedicated implementation cycle — next quiet cycle takes it.

- Cycle 46 (08-09 ~14:3x): `cw-walk-lowgait30` PASS — -30mm crouch: own-cfg
  DR0 gv 12/12, 0 term, height err ~4-5mm; det slip/m 0.95 vs parent 1.14
  (deeper crouch REDUCED slip, per-ep ranges non-overlapping); 1 sto
  draw-stall ep = known lineage trait, not worse. Refill: `cw-walk-lowgait40`
  (-40mm envelope rung, t3, 20M). Remaining free slots = the concurrent
  cycle's refills for its 10 triages (it was actively launching, e.g.
  wander-dr05); no other sound unblocked arms for this cycle (c45 reason
  stands); mirror-symmetry impl deferred — a code snapshot mid-refill
  would flip the pod SHA gate under the concurrent cycle's launches.

- OPERATOR (08-09 ~10:3x): c44 promotion ASSUMPTION **ACCEPTED**,
  and promotion re-based by ruling (8): "closest to deployable
  joystick robot" — physical metrics, det gain + sto parity is
  enough. External readiness review ADOPTED (P0 calibration →
  forward freeze → hardware ladder; P1 four-leg feasibility-first;
  no new anti-slip coefficient arms) →
  `archive/HEXAPOD_READINESS_RESEARCH_REVIEW_2026-08-09.md` +
  RL_PLAN rulings block. Triage bottleneck fixed:
  max_concurrent_cycles 2→4 (12 simultaneous finishes had idled 14
  GPUs behind 2 cycles).

## ASSUMPTION (operator to review)

- Cycle 44 champion promotion: adopted longdist-r2 as walk champion
  without an operator ruling. Question: does det slip −15%/−21%
  (r2/s1 vs anchorgate 1.240 @DR1.0, two seeds concordant, video
  clean) outweigh the unchanged sto draw-stalls (DR1.0 sto 3/6, same
  as champion's own panel)? Evidence: pre-registered s1 gate hit its
  if-true branch; anchorgate keeps its checkpoint (append-only).
  Revisit if the operator prefers promotion to also require a sto
  robustness gain; warm-starts since this cycle parent on r2.

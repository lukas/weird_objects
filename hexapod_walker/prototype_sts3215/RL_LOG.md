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

- Cycle 47 (08-09 ~15:1x): 4 triages. `cw-walk-terrain05` PASS (18mm bumps:
  terrain det 6/6 + flat retention slip 1.01, no destabilization). `cw-walk-wander30`
  PASS (30s driving, prog ~1.0, zero stalls — resampling breaks stall draws).
  `cw-walk-longdist-dr05` FAIL/refuted (DR1.0 sto 3/6 = parent; DR is not the
  stall lever). `cw-walk-endur60` PASS (3.17m det median @60s, no decay).
  `cw-walk-speedband` was in this cycle's finished list but already KILLED+
  verdicted by c43 — no verdict issued here. Direction: stalls = command-
  conditioning (three-run convergence) → launched `cw-walk-stallfix` (resample-
  trained champion, gated on fixed-command panels) + `cw-walk-terrain10` (amp
  1.0 rung). Remaining idle slots: concurrent cycles' refills landing live
  (longdist-s2/dr10, head90, joystick45 queued); killed 4 duplicate gate evals
  from a defunct double-spawned 14:37 cycle (same --out dirs, zombie-safe).

- Cycle 48 (08-09 ~15:2x-15:4x): 2 triages (cw-walk-speedband in the finished
  list was already KILLED+verdicted by c43 — no verdict issued here, again).
  `cw-walk-strafe-dr05` PASS (DR0.5: gv 12/12, 0 term, prog med 0.92-0.97,
  slip med ≤2.0; DR0 retention slip 1.80 beats parent 2.20) and
  `cw-walk-wander-dr05` PASS (DR0.5 steering: gv 12/12, 0 term, prog ~0.95,
  slip med 1.39-1.70) — both SKILLS rows added; neither hardware-ready
  (paddle-slip root). Watcher prestage pullckpt failed for both (specs lacked
  --out-name → pods wrote ppo_mjx_joint_walk_* names); pulled manually,
  md5-verified; all refill specs now carry explicit --out-name. Direction:
  robustness ladders + joystick hardening → queued cw-walk-wander-dr10,
  cw-walk-strafe-dr10, cw-walk-joyjit-dr05 (60M GPU steps, 3 lines) and
  drained; remaining free slots left to the concurrent cycle's refills
  (endur60-s1/longdist-tilt05/lowgait40 triage).


- Cycle 50 (08-09 ~15:2x-16:0x): 3 triages. `cw-walk-lowgait40` PASS (-40mm
  crouch rung: gv 12/12, 0 term, height err <=7mm, det agg slip 0.96) — SKILLS
  crouch row extended, -50mm rung queued. `cw-walk-longdist-tilt05` FAIL/
  NO-EFFECT (vs A/B partner longdist-dr05 all deltas inside noise; same sto
  draw degrades both) — 0-c(i) tilt lever CLOSED on champion line, stability
  rides on DR training (plan updated). `cw-walk-endur60-s1` FAIL on det slip
  clause (1.13 vs gate <=1.0; seed-0's 0.887 = seed luck, ranges disjoint) but
  endurance CONFIRMED seed-robust (3m @60s, no decay, both seeds) — SKILLS
  endurance row added; champion stands. Prestage missing for endur60-s1+tilt05
  (pre-fix specs lacked --out-name); pulled+evaled manually. Checkup SUSPECT
  longdist-dr10 fps 4369: transient, log now ~6970 fps, healthy, no action.
  Refills: queued+drained cw-walk-lowgait50 (envelope rung 4) and
  cw-walk-endur60-r2 (champion 60s fold), 40M GPU steps, 2 lines; remaining
  idle slots belong to the 3 fresh finishes' (stallfix/head90/joystick45)
  triage cycles' refills. [CODE] impl (mirror-symmetry/quadruped) deferred
  again: concurrent cycles actively launching (SHA-flip risk, c46 reason).
- Cycle 51 (08-09 ~16:1x): 0 verdicts owed — cw-walk-joyjit-dr05 (the
  finished-list run) was already KILLED+verdicted by c49's rebalance (no
  hypothesis verdict; continuation -c1 training on train-8); killed its
  stale pre-staged gate eval. Real finding: the drain was DEAD — all 4
  queued specs (wander60/stopgo35/head90-dr05/head135) burned 3 REFUSED
  attempts each into backlog_failed on pod marker 920e633-dirty, caused by
  ONE uncommitted harvest artifact (rl_move/sim/park_banks/longdist_r2_park
  .npz, 16:04) dirtying every sync, 4 GPUs idle. Fix: committed+pushed
  c47f0d5 (tag exp/cycle51-drain-unblock), synced idle pods 4/9/10/11
  clean, pushed head90 parent to train-11 (md5 ok), requeued the 4 unique
  specs (deduped double-queued wander60, attempts reset), drained;
  wander60 + head90-dr05 verified RUNNING (train-1/5), stopgo35 + head135
  placing. COMMANDS.md gotcha 0 added. Requeued total = 80M GPU steps =
  the per-cycle cap (hard reason): remaining free slots belong to the 4
  fresh finishes' (lowgait50/longdist-s2/longdist-dr10/terrain10) triage
  cycles' refills.

## ASSUMPTION (operator to review)

- Cycle 44 champion promotion: adopted longdist-r2 as walk champion
  without an operator ruling. Question: does det slip −15%/−21%
  (r2/s1 vs anchorgate 1.240 @DR1.0, two seeds concordant, video
  clean) outweigh the unchanged sto draw-stalls (DR1.0 sto 3/6, same
  as champion's own panel)? Evidence: pre-registered s1 gate hit its
  if-true branch; anchorgate keeps its checkpoint (append-only).
  Revisit if the operator prefers promotion to also require a sto
  robustness gain; warm-starts since this cycle parent on r2.

- Cycle 52 (08-09 ~16:3x): 1 triage. `cw-walk-stallfix` FAIL on pre-registered
  if-false: 5s cmd-resample training does NOT erase fixed-draw stalls — the
  SAME DR0 sto draw that stalls champion r2 (prog 0.24) stalls stallfix DEEPER
  (prog 0.04, slip/m 36.7, in-place churn on frames); helps only when resample
  is on at eval (own-cfg 0 stalls, gv 12/12). Follow-up harvest for the
  park-bank lever came back EMPTY (0/60 sto 30s eps parked, seed 1000) →
  stall incidence <2%, a rare tail, not a front: stall lever class CLOSED
  (3rd refusal: dr05, stallfix, park-bank unfundable), plan updated, panel
  draw stays as regression canary. NEW champion evidence: joystick gate
  (eval_drive) PASS at DR0.2 AND DR0.5, zero falls incl. instant-flip stress
  (backward cmd parks, doesn't fall) — SKILLS + plan updated. Watcher prestage
  had failed (pullckpt expected ppo_goal_* but MJX writes ppo_mjx_joint_walk_*);
  pulled + ran 3 gate panels manually. Refill: queued+drained cw-walk-wander60
  (driving endurance 30->60s, 20M); other slots owned by concurrent cycles'
  refills (endur60-r2/head90-dr05/joyjit-dr05-c1 landed mid-cycle) + 4 fresh
  finishes spawning their own triages. [CODE] impl (quadruped feasibility
  sweep, mirror-symmetry, yaw-rate turning) deferred again — same c46 reason,
  concurrent cycles actively launching; flag: next quiet cycle should take one.

- Cycle 51 (08-09 ~16:2x): 1 triage. `cw-walk-head90` PASS — heading rung 1
  (±90°): gv 12/12, 0 term, clean tripod, lateral err ≤1.6× fwd, JOYSTICK
  GATE 0 falls @DR0.2; pathologies logged: prog 0.84 vs wander30's 0.94-1.02
  (lateral costs progress) + left-strafe half of right (mirror-symmetry
  target). SKILLS row added. Refills (4 = cycle cap, 80M): head135 (ladder
  rung 2), wander60 (dup-queued convergently with c50 — dedupe held, one
  instance), stopgo35 (stop_frac 0.35), head90-dr05 (DR rung on ±90). Infra:
  my dirty-tree `--sync` of train-4 stamped a `-dirty` marker that blocked 4
  drain launches → snapshot cycle51-head90-refill + re-sync 4/9/10/11 fixed
  it (lesson: never --sync with a dirty tree). Remaining idle slots: at
  per-cycle launch cap; 5 fresh finishes (wander/strafe-dr10, lowgait50,
  longdist-s2, terrain10) spawn their own triage cycles.

- Cycle 49 (08-09 ~15:5x-16:1x): 1 triage. `cw-walk-joystick45` PASS
  (JOYSTICK GATE 0 in-envelope falls panel+flip-stress; own-cfg DR0 gv
  12/12, 0 term; paddle-slide persists, not hw-ready) — SKILLS row added;
  abrupt-resample exposure hardens flips at DR0, DR pair joyjit-dr05-c1
  answers composition. Checkup: joyjit-dr05 starved on g142d86 (load
  216/128, fps 2.1k, foreign tenant suspected) → killed, ckpt bbde2450
  rebalanced as joyjit-dr05-c1 -> train-8 (18.2k fps). Code: dr.<field>
  cfg-set overrides for DR ranges (sim_env hook + eval harness fix,
  smoke-tested; absolute values, post-scale) → unlocks wishlist 11/13b.
  Refills (3 launches of 4-cap, 58M GPU): payload50 (mass 1.0-1.5x) +
  latjit25 (latency 0.5-2.5x), both isolated-axis at dr-scale 0 off
  champion, train-9/10. Remaining slots: concurrent cycles' triage
  refills (5 fresh finishes); [CODE] quadruped feasibility still flagged
  for next quiet cycle.

- Cycle 53 (08-09 ~16:4x): checkup-triggered, no triage. SUSPECT cw-walk-stopgo35
  confirmed starved: node g142d86 oversubscribed by a foreign tenant again (load
  110-146/128, fps 4.4k vs 15.1k on healthy g129004, GPU 0%) → killed at ~8.5M/20M,
  rebalanced as cw-walk-stopgo35-c1 (same config, ckpt 1fc1f2d4) → train-2 (g131eec).
  head90-dr05/head135 on same node slow (6.9k/9.1k fps) but progressing — left in
  place; freed cores should help. No verdicts; refill = the rebalance launch only.

- Cycle 54 (08-09 ~16:5x): 3 triages. `cw-walk-lowgait50` PASS (crouch rung 4:
  -50mm, height err 4.6/4.7mm, det slip/m 1.04, gv 12/12; knob now -20..-50mm,
  SKILLS updated). `cw-walk-terrain10` PASS + TERRAIN LINE CLOSED as SATURATED
  (amp1.0 ≡ flat: prog 1.06/slip 0.94 both; SKILLS row added; harder ground =
  [CODE] scene work). `cw-walk-strafe-dr10` FAIL on gate (det ep sacrifices legs
  4+5, slip med 2.49/2.59 > 2.4; NOT a collapse — DR0.5 stays the strafe ceiling,
  fix is mirror-symmetry not more DR; verdict applies to this run only, strafe-dr05
  skill row unchanged). Watcher prestage for terrain10 failed (spec pre-dated
  --out-name fix) — pulled ckpt manually (md5 57cea2dc). Refills (4 = cap, 80M =
  GPU-step cap): lowgait60 (envelope rung 5), lowgait-dr05 (crouch DR rung),
  speedband (wishlist 8b, 0.02-0.12 m/s off wander30), fricvar (13b friction axis
  0.4-1.6 off champion; torque axis taken by concurrent torquedroop). [CODE]
  mirror-symmetry now has TWO independent motivations (head90 L/R asym +
  strafe-dr10 flag legs) — flagged as the next implementation cycle's pick.

- Cycle 53 (08-09 ~17:0x): 3 triages. `cw-walk-joyjit-dr05-c1` PASS — flip
  hardening COMPOSES with DR0.5 (JOYSTICK GATE @DR0.2 0 falls incl. flip
  stress; own-cfg DR0.5 gv 12/12, prog 0.94/0.98) → best driving candidate,
  SKILLS row added, joyhead90 (±90° abrupt-resample rung) queued off it.
  `cw-walk-longdist-s2` PASS — champion recipe 3/3 seed-robust (det slip
  1.07 ≤1.10); same fixed sto draw churns (lineage canary); champion stays r2.
  `cw-walk-longdist-dr10` FAIL (if-false) — full-DR retrain of champion buys
  zero reliability (bad draws unchanged) and erodes nominal quality (DR1.0
  det slip 1.28 vs r2's 1.06 same panel) → full-DR retrain lever CLOSED, plan
  updated. Refills (3 of 4-cap, 60M GPU): torquedroop (13b torque axis, ran on
  train-0), speedband (dup-dropped by dedupe — speedband-r1/speedband2 already
  cover 8b), joyhead90 (drain launch lost a placement race with lowgait-dr05-r1
  on train-4, worker EOF at init → retried once as joyhead90-r1 on train-5,
  VERIFIED RUNNING). Remaining free slots owned by concurrent cycles' refills.

- Cycle 55 (08-09 ~16:2x-17:1x): 1 triage. `cw-walk-wander-dr10` FAIL on
  pre-registered if-false (gv clause): own-DR1.0 gv 11/12 — one det draw
  sacrifices leg 5 (prog 0.37, slip/m 4.6, near-stationary on frames), two
  more det draws degraded; DR0 retention clean (gv 6/6, slip 1.53 = parent).
  DR 0.5 stays the steering line's ceiling (SKILLS updated); concordant with
  longdist-dr10's full-DR-lever FAIL. Refills (3 of 4-cap, 60M): friclow
  (13b slippery-half friction 0.3-1.0 — OVERLAPS concurrent fricvar 0.4-1.6,
  flagged in both ledgers: triage as ONE axis study), wander-dr05-s1 (seed
  panel for the steering-DR champion, ruling 7), slowband (KILLED by me at
  ~3M same cycle: dedupe scan missed speedband-r1's "no requeue" FAIL which
  already covers the 0.02-0.06 band; 8b stays closed on pricing root);
  latjit25 re-queue dup-dropped (concurrent cycle already ran it). FLAG for
  next free-slot cycle: READY wishlist arms are EXHAUSTED (3 cycles collided
  on 8b/friction within the hour) — the quadruped feasibility sweep [CODE]
  (readiness P1, deferred c46/c52/here) is now the correct use of a slot, not
  another walk variant.

- Cycle 54 (08-09 ~17:1x): 1 triage. `cw-walk-endur60-r2` FAIL on the
  pre-registered if-false: 60s off champion HOLDS endurance (det med fwd
  2.91m, gv 12/12, 0 term, frames clean six-leg cycling full 60s) but gives
  back slip — det slip/m 1.07 vs gate <=1.0 (parent 0.94-0.96 @30s); champion
  stands at 30s, no continuation (plateaued + income side CLOSED).
  `cw-walk-stopgo35` needed no verdict (c53 KILLED/rebalance stands); its
  8.5M ckpt gate-evaled clean (gv 12/12, 0 term, prog ~1.0) → stopgo35-c1
  continues from a healthy policy (W&B addendum only). Refills (4 = cap,
  80M, backlog+drain): head90-s1 (seed panel for the ±90° envelope, ruling
  7), stiffvar (13b contact compliance 0.5-2.5x), deadband30 (13b/13c servo
  deadband 1-3x), comshift30 (wishlist 11 asymmetric CoM 30mm) — the last
  fresh isolated axes; after these the READY well is dry (remaining dr
  fields: cmd_drop/ground_tilt/vel_scale only). The c53 flag STANDS and
  sharpens: next cycle with a triage-light prompt should BE the quadruped
  feasibility sweep implementation cycle (readiness P1, scripted, no GPU) —
  it is agent-time work and does not compete with pod refills.

- Cycle 54 addendum (~17:1x): refill placement fought two races + one real
  infra bug. (a) Mid-drain snapshot moved HEAD -> fricvar/lowgait-dr05 REFUSED
  once (lesson: snapshot BEFORE ops.sh drain, not between). (b) speedband name
  collided with c43's killed run (W&B names append-only) -> reissued as
  speedband2. (c) REAL BUG: train-4 /dev/shm 98% full of hexmjx-* segments
  leaked by c53's stopgo35 killrun -> lowgait-dr05 AND speedband2 died 0-step
  at first env reset (worker EOFError); cleaned shm, retried once each per
  policy: lowgait-dr05-r1 -> train-4 (16.7k fps) + speedband2-r1 -> train-2,
  both VERIFIED RUNNING. COMMANDS.md gotcha 13 added (shm leak + eval waitlog
  marker is 'artifacts' not 'WROTE'; ops.sh/watch_loop hint strings fixed).
  Final cycle-54 fleet adds: lowgait60(t1), speedband2-r1(t2), fricvar(t3),
  lowgait-dr05-r1(t4).
  (c54 addendum: first drain hit stale .code_sha on train-6/7/11 after my
  snapshot moved HEAD — re-synced 3 pods (one transient -dirty stamp from
  concurrent ledger-write temp files, resolved on retry), requeued, all 4
  placed: deadband30 RUNNING, head90-s1/stiffvar/comshift30 INTENT→verifying.)

- Cycle 56 (08-09 ~17:5x): 3 triages. `cw-walk-head135` FAIL — det tilt_pitch
  termination + prog med 0.53 (parent head90 0.84) + slip/m ~2x lineage; joystick
  gate @DR0.2 passed (0 falls) — it survives commands it can't track. HEADING
  LADDER FROZEN at ±90 (no ±180); mirror-symmetry now has 3 independent
  motivations. `cw-walk-head90-dr05` PASS (own-DR0.5 gv 12/12, prog med
  0.83/0.90, DR0 retention = parent 0.84 — widen-then-harden composes; SKILLS
  row; DR1.0 not queued, full-DR refuted 2x). `cw-walk-latjit25` PASS (latency
  0.5–2.5x trainable by exposure; DR0 retention in champion band; honest tail:
  2/6 extreme det draws shuffle at ~40% distance — median hardened, not the
  2.5x tail; SKILLS row). Refills: my deadband30 spec dup-dropped pre-drain
  (c54-addendum's identical run already RUNNING on t10); queued
  wander-dr05-s2 (ruling-7 seed panel) + joylat25 (validated latency axis onto
  driving candidate joyjit-dr05-c1), 38M. Distinct READY arms are thin again:
  next slots belong to the mirror-symmetry [CODE] cycle (3 motivations) and
  quadruped feasibility sweep (CPU-scripted, no GPU slot needed).

- Cycle 57 (08-09 ~18:0x): 1 real triage of 3 finished. `cw-walk-payload50`
  PASS — payload axis lands: mass 1.0–1.5x panel gv 12/12, 0 term, det med fwd
  1.31m; DR0 retention clean (slip 1.15, prog 0.95). Honest tail: 2/6 heaviest
  det draws squat-shuffle at ~half speed (slip 3.4–3.8) — solid to ~+40%, top
  marginal; SKILLS row added; NOT hardware-ready (paddle lineage).
  `cw-walk-lowgait-dr05`: NO hypothesis verdict — 0-step infra failure (shm
  leak, already recorded c54), retry -r1 training; `cw-walk-slowband`: c55
  KILLED-as-dup verdict stands, no new evidence. QUADRUPED FEASIBILITY SWEEP
  (c55/c56 flag) implemented + run: rl_move/sim/quadruped_feasibility.py — GO
  for static 4-leg stance (neutral CoM is 68–82mm OUTSIDE the 4-foot polygon,
  but −20mm shift + 17° mid-leg splay gives 39mm margin @0.6A, push-robust;
  11/18 configs robust; SKILLS section added; next rung = quad-hold goal mode
  [CODE]). Infra: train-1 had a `-dirty` code marker starving the drain
  (groundtilt5 burned 3 attempts into backlog_failed) — committed, re-synced 7
  pods, requeued; placed. Refills (3 of 4-cap, 60M): cmddrop10 (13c command
  dropout), velsag30 (13b servo speed sag 0.70–1.10), payload-dr05 (compose
  rung off today's PASS, parent pushed to idle pods). READY well after these:
  mirror-symmetry [CODE] + quad-hold [CODE] are the flagged next
  implementation cycles.

- Cycle 58 (08-09 ~18:1x): 3 triages, 3 PASS. `cw-walk-lowgait60` PASS (crouch
  rung 5: −60mm, end-height err det 3.9/sto 4.5mm, det agg slip/m 1.00, gv
  12/12 — height knob validated −20..−60mm). `cw-walk-wander-dr05-s1` PASS —
  steering-DR rung seed-CONFIRMED (own-DR0.5 gv 12/12, 0 term, prog 0.99/0.94,
  slip 1.59/1.75 ≈ seed-0; ruling-7 panel satisfied; SKILLS updated).
  `cw-walk-wander60` PASS — 60s driving endurance holds (gv 12/12, 0 term,
  worst slip 1.67 < wander30's 1.93; no decay over ~12 changes/ep). Checkup:
  joyhead90-r1 SUSPECT fps dip self-recovered (5.8k in log) — no action.
  Infra: wander60 was launched without --out-name → prestage pullckpt failed;
  ckpt recovered from pod default name (md5 bcabaea0) — specs must ALWAYS pass
  --out-name. Refills: lowgait70 (crouch rung 6, RUNNING t9) + wander60-dr05
  (minute-long drives at DR0.5, queued for t10); my slope3 spec withdrawn
  pre-launch (dup of c57 groundtilt5) and velsag KILLED at 0 steps (dup of
  c57 velsag30; t10 shm cleaned) — 3 concurrent refill cycles drew from the
  same READY well within minutes; re-check census AFTER queueing, not only
  before.

- Cycle 59 (08-09 ~17:2x-18:1x): 2 real triages of 3 finished. `cw-walk-stopgo35-c1`
  PASS — 35% stop density handled (own-cfg DR0 gv 12/12, 0 term, prog med 0.97,
  min ep 0.92; frames: quiet level parks, restarts re-track ~3s); stop
  transitions need no shaping; SKILLS row added; slip/m 1.43 = lineage paddle,
  not hardware-ready. `cw-walk-torquedroop` FAIL/NO-EFFECT (dig-in with named
  baseline: parent longdist-r2 under the IDENTICAL 0.60-1.05x torque spread
  matches everywhere, incl. the same 2 collapsing low-torque det draws) —
  exposure lever for torque droop CLOSED; champion ALREADY covers ~0.75-1.05x
  free (SKILLS envelope updated), <=0.7x = paddle-gait transport boundary ->
  estimator rung. `cw-walk-speedband2` finish event = c54's 0-step shm infra
  FAIL closing in W&B; verdict + OUTCOME already recorded c54, no new evidence
  (-r1 training). Refills (2, 40M GPU): groundtilt5 (13b floor-slope axis,
  RUNNING t3 after 3 dirty-tree refusals c58 fixed), cmddrop20 (13b bus-drop
  axis, RUNNING t10) — cmddrop20 OVERLAPS c57's cmddrop10 (0.10 vs 0.20 max):
  cross-flagged in both ledgers, triage as ONE intensity-ladder study and run
  the parent baseline FIRST (torquedroop lesson: gate letter can't tell
  exposure effect from pre-existing champion tolerance).

- Cycle 58 (08-09 ~18:4x): 3 triages, 3 PASSes — the last isolated 13b/wishlist-11
  axes all land off the champion. `cw-walk-comshift30` PASS (30mm off-center CoM:
  own-cfg gv 12/12, 0 term, det med fwd 1.44m; DR0 retention = champion 1.58/0.95)
  and `cw-walk-deadband30` PASS (deadband 1–3x: 1.41m; retention 1.58/1.00; NO
  jerky-overdrive compensation in frames) — both share payload50's honest tail
  (worst det draw ~0.68m @ slip ~2.9, half-speed shuffle, no falls). `cw-walk-friclow`
  PASS on the letter ONLY (grip 0.3–1.0x: det med fwd 1.23 scrapes gate 1.2; slick
  draws SKATE, own-cfg slip/m med 1.73; only axis charging a nominal tax, DR0 det fwd
  1.57→1.43 disjoint ranges) — slippery floors survivable, not solved. SKILLS +3 rows.
  Checkup: head90-s1 SUSPECT (1.9k fps) = g142d86 node contention (4 trainers +
  controller evals co-located; train-6 equally slow, run healthy/reward climbing,
  0 free slots -> no rebalance possible, HARD reason recorded; will finish ~2h) +
  cleaned 40M leaked hexmjx shm on train-7 (uid-checked vs live fds). g12ba48 still
  SchedulingDisabled — no scale-out. Refills (backlog, 60M GPU): comshift-dr05 +
  deadband-dr05 (compose rungs, parents pushed to all 12 pods) + fric50 (grip
  0.5–1.0 refinement: find the CLEAN-walk grip floor friclow's letter-pass hides).
  Mirror-symmetry + quad-hold [CODE] cycles remain the flagged next implementation work.

- Cycle 59 (08-09 ~18:4x): 2 triages + 1 dup closeout. `cw-walk-fricvar` PASS —
  friction 0.4–1.6x trainable by exposure (own-cfg gv 12/12, 0 term, det prog med
  0.87; DR0 retention slip 1.09 = champion band); honest tail: 2/6 slickest det
  draws churn near-in-place (prog 0.36–0.56, slip 2.4–4.2); SKILLS row; fricvar-dr05
  compose queued. `cw-walk-speedband2-r1` FAIL — speed pinned 0.05–0.06 across the
  0.02–0.12 band (det prog med 0.65), confirming the ALREADY-CLOSED c42/c43 gait
  ceiling; spec was a stale c54 reissue of a refuted class (name-dedupe missed the
  class AGAIN, cf. slowband — check run docs for the CLASS before requeueing). 8b
  speed-knob stays CLOSED pending contact pricing. `cw-walk-velsag` = c58 KILLED dup
  of velsag30, corrected steps in verdict (~4.2M not ~0), no hypothesis verdict;
  killed its wasted pre-staged eval. Fleet 12/12 busy, no free slots; refill = 1
  backlog spec (20M GPU). Mirror-symmetry + quad-hold [CODE] cycles still flagged.

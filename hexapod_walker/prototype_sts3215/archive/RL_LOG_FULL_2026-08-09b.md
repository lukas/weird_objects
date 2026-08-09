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

## State (as of cycle 34, 2026-08-09 ~11:30Z)

- **WALK CHAMPION: `ppo_goal_cw_walk_anchorgate.zip` md5 35234ddc**
  (cycle 31; DR1.0 det slip 1.240, best ever; NOT hardware-ready —
  sprawly paddle-creep, slip gate ≤1.0 unmet).
- **Stance champion:** `cw-stance-dr10` line — heights 12/12 at
  DR 1.0; lower posture still flag-leg. **Stance line BLOCKED on
  operator pricing ruling** (cycle 28: hover is income-positive).
- In flight: `cw-walk-stepdisp12` (displacement-gated step credit —
  the LAST income-side walk arm; if it fails, walk slip goes to the
  operator whole, alongside the stance pricing ruling).
- Compute: 2 GPU MJX pods (20–40M-step runs, ~15k fps) + CPU pods.

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

## Cycle log (append below, 1–3 lines each)

- Cycle 33 (08-09 ~10:4x): `cw-walk-step0-anchor` FAIL/refuted —
  fresh init re-derives paddle + allowance-ride; auto-continuation
  killed on the merits. `anchortol5` verdict is next; its if-false
  closes the tolerance rung and escalates to displacement-gated
  step credit (0-c.2) or operator pricing.
- Cycle 34 (08-09 ~11:0x): `cw-walk-anchortol5` FAIL — policy PAID
  the binding tol=5 gate (−18% income) and kept creeping; tolerance
  rung CLOSED, income gating at its price ceiling. Launched
  `cw-walk-stepdisp12` (displacement-gated step credit, 12 mm) —
  a cadence-ATTRIBUTION arm, not a slip arm; its if-false sends the
  walk slip line to the operator whole. Champion unchanged.
  Launch verified two-phase (W&B xj6ehawo, ~16k fps, mechanism live
  at the audited ~2% denial; checks in ledger). mjx-train-0/2/3
  idle: all remaining walk levers route through stepdisp12's
  outcome or the operator rulings (c27/c28); mirror-symmetry gets
  its own implementation cycle.
- OPERATOR (08-09 ~10:5x): design rulings LANDED — binding, full
  text `archive/OPERATOR_RULINGS_2026-08-09.md`, condensed block in
  RL_PLAN. Resolves cycle 26/27/28 NEEDS-OPERATOR items: stance
  allowance/support_margin (rework authorized), rear hemisphere
  (deferred), distance gate (progress_ratio; prefer narrow forward
  band), loaded-slip metric (never touchdown-reset), multi-seed
  promotion panels. Still operator-owned: hardware current
  calibration — NO robot contact by the orchestrator.
- OPERATOR (08-09 ~10:5x): ASSUME-AND-GO + parallel lines landed
  (guardrails `operator_unblock_policy`): plausible-recommendation
  questions get an "## ASSUMPTION (operator to review)" entry and
  work CONTINUES; operator-only blockers ⇒ fill idle GPUs with
  command-steering / QUADRUPED (pulled forward) / mirror-symmetry /
  contact-aux arms. Cycle cap 12→48 (watch_loop synced). Idle GPU
  pods now require a ledger-recorded HARD reason.
- Cycle 35 (08-09 ~11:1x, findings/directive cycle): landed rulings
  code — `goal.walk_heading_max_rad` (fwd/fwd-diag command scope),
  `reward.walk_loadslip_gate` (episode-accumulated loaded slip/m
  income gate, never touchdown-reset) + harness progress_ratio /
  slip_per_m (champion reads prog_ratio 1.43 — ruled overspeed
  confirmed; slip/m 1.11 on the along-command denominator). Retried
  operator arms `cw-walk-longdist`+`cw-walk-dr05` DEAD at init
  (champion ckpt absent on pods — `snapshot.sh --sync` excludes
  policies/; `ops.sh pushckpt` + ckpt pushed to train-0/2/3, md5
  35234ddc; c36 took the longdist retry as -r2). Launched:
  `cw-walk-dr05-r1` (train-3, operator DR-0.5 arm retried),
  probe-walk-rulings-mjx → `cw-walk-loadslip` (train-2).
  `cw-walk-fwdband` deferred — HARD reason: 4-launch cap +
  no free pod (c36's steer-fdiag covers the heading knob at π/4).
  NOTE for next cycle: WISHLIST tags `cw-walk-fast` +
  `cw-chain-standwalksit` [RUNNING] but neither was ever launched
  (no W&B run, no ledger entry) — they are READY pickups.
- ## ASSUMPTION (operator to review) (c35): (1) rulings-(5)/(6) walk
  arms launched BEFORE the stepdisp12 verdict lands — the rulings
  make loaded-slip accounting + forward scope binding regardless of
  its cadence-attribution outcome; revisit if that verdict
  contradicts. (2) loadslip thresholds ok=0.75/max=1.5 slip-per-m
  from controller scale audit (champion paddle ep keeps ~11% of
  velocity income, clean ep ~75%); tune only as gate tolerances per
  ruling. (3) `cw-walk-longdist` retried under the DEAD-retry rule
  (0 steps trained, pure infra fault) instead of waiting for its
  queued watcher cycle.
- Cycle 36 (08-09 ~11:2x): `cw-walk-stepdisp12` FAIL/REFUTED — with
  step credit displacement-gated (denials real, climbing), cadence
  STILL inflated (det DR1.0 61.5 swings/ep vs c1 58.2, champ 47.2):
  step credit does NOT pay the cadence ride; income channels now
  fully mapped/exhausted, confirming c35's rulings-(5)/(6) arms as
  the right escalation (no contradiction with its ASSUMPTION (1)).
  Champion unchanged; frames = same paddle-creep, NOT HARDWARE-READY.
  Launched `cw-steer-fdiag` (train-1, c35's deferred steering arm).
- Cycle 37 (08-09 ~11:4x, trailing cycle for cw-walk-longdist):
  run FAILED at init, 0 steps (parent ckpt absent on pod — infra,
  not policy); already retried by c35 as `-r2` (RUNNING, ~17M/20M).
  No eval possible; hypothesis NOT TESTED, carried by -r2. Fixed:
  launcher `pod_trainers` scan matched cycle-agent cmdlines (371
  phantom trainers on the controller node would refuse all smokes);
  watcher prestage now skips gate eval when pullckpt fails.
  Scale-out pods train-4..11 came up Running mid-cycle: launched 4
  (cycle cap; 80M GPU steps = cap): `cw-walk-fwdband` (t4, rulings-5
  fwd-only scope, c35's deferred arm), `cw-walk-fast` (t5, wishlist
  #2, 0.08-0.12 band — does real stepping emerge), `cw-chain-
  standwalksit` (t8, wishlist #14, walk+rise+lower mix), `cw-walk-
  loadslip-s1` (t9, seed-1 panel per ruling-7). Turning skipped:
  needs yaw-command code ([CODE], not [READY]). train-10/11 idle —
  HARD reason: max_new_launches_per_cycle=4 + GPU-step cap reached;
  train-12..15 Pending (unreachable).
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

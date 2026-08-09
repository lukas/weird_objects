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

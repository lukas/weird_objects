# CURRENT TRUTHS — read this BEFORE any history file

Accepted current facts + rulings only, never campaign narrative. If
RL_LOG/archive prose disagrees with a line here, this file wins.
Update ONLY when a ruling is accepted or hardware produces new
evidence; keep 50–80 lines. Reading order: RL_GOALS.md → this file →
RL_PLAN.md → RESEARCH_RULES.md → rl_docs/SIM.md.
Last regenerated: 2026-08-13 ~13:xx UTC. The prior 806-line version
(every resolution chain intact) is archived verbatim at
`archive/CURRENT_TRUTHS_2026-08-13_pre_regen.md`; chains also live in
track STATUS docs, RISE.md, GAIT.md, SIM.md, rl_docs/runs/. Live
waits/forks: STATUS.md WAITING-ON (all 12 pods idle on named
operator calls at regeneration time).

## Checkpoint state (sim gates; deployment/promotion = operator)

- DEPLOYED: stance `holdbc1_hard1` (FAILS the interactive session
  gate: tips on sit-from-walk-plant, belly rise stalls 55mm — no
  sit-after-walk on hardware), walk `dep-tip1`; scripted glide is
  the only working hardware stand-up (no learned STAND on hardware
  until a rocking-hardened checkpoint exists).
- Stance candidates awaiting bench promotion: `footlow2-hard1`
  PASSES the full gate (cold-flat rise 12/12, park-free six-foot
  hold, 12/12 lower, session hard gates) — first ever; `-stable1`
  also passes (hold-drag tradeoff, doesn't dominate).
- Tall walk: `cw-dep-bcgait1-hard1` (BC-INIT) broke the crouch-splay
  wall — in-band height, positive leg-yaw margin, slip <1.8, zero
  falls; fric/groundtilt5 panel PASS; takeoff push-probe no worse
  than tip1. Gate 0 rests on bench tape, not another sim DR axis.
- Handoffs compose in sim (rise→hold→walk, walk→stop→sit); scripted
  sit glide covers sit. rot60 (exact hexagon symmetry) gives the
  full command circle zero-training, deployed default-ON;
  MirrorPolicy = zero-training turning (deploy port operator-only).

## Real robot facts (these outrank any sim result)

- Scripted tripod gait walks/crabs/turns; travel ~50% of commanded;
  loaded-foot slip contributes — slip alone is NOT failure. Working
  gaits rock ±10–20°; envelope 25° + directional rate logic.
- 08-11 night bench truth: learned stand-up trips tilt_roll 10/10 at
  the same belly-curl tick (the 10° trip is CORRECT); every learned
  walk rides a universal 13–27° takeoff roll transient and falls
  ~half the time (NO vref1/tip1 A/B winner); feet drag in walk and
  stand/sit. Judge walks by fell/tail, not peak flags. After a tip
  the stance holds a ~8° lean (live operator fork).
- Current registers track the cmd-vs-settled FIGHT, not pose;
  k_current=0 on hardware arms until calibrated (sim torque-proxy
  overprices 3–25x; effort-inversion RETRACTED). Warp GPU physics
  EXONERATED by replay parity ("under-charges slip" RETRACTED).
- Rises must move feet UNLOADED then push; loaded servo settle is
  ~250–325ms (air ~9ms), loaded fit opt-in, treat with DR.
- Hygiene: fresh set_zero (all 18 servos healthy); TFT repaint
  stalls the bus unless bus_hot; 25Hz control, ~100–200ms delay.
  08-06 incident rules absolute: no motion without operator ask.

## Policy / deployment facts

- 18 raw joint targets via SafetyLayer; 1.5°/tick stateful slew in
  BOTH training and hardware; vx/vy_meas:=ref contract ACCEPTED
  (`cw-dep-vref1-r1` = hardware base); prev-action audited PASS.
- hist16 is the temporal base; dual-core mode-gated GRU removes
  shared-trunk walk-freeze; its rise gap was BC-demo poverty — the
  DAgger redistillation landed 08-13 (`ppo_goal_cw_gru_dual_bc_
  dagger1.zip`, rise in the BC init for the first time, lower
  collapsed 0/6; operator picks warm-RL vs variant distill —
  arch/STATUS.md).
- Stance goal profile ships in the weights meta (runner contract
  test-locked); eval_session gates every stance/walk candidate and
  runs automatically in watcher pod evals.
- Eval-history corrections: pre-08-13 `quad_*` eval rows actually
  ran WALK (fixed+asserted); pre-da367c9 rise rates mixed mislabeled
  RSI spawns in (`start_kind` label now).

## Campaign rulings in force

- **SIM SPRINT (operator 08-17, ~18:05 UTC — CURRENT top ruling):**
  robot off the bench ~a day for repair; until the operator says it
  is back, the fleet's single deliverable is RELIABLE RISE + WALK in
  MuJoCo at download quality (named checkpoints + gate evidence;
  post-lower rise and takeoff transient are the known gaps). No new
  research-track launches unless they directly serve that; bench
  [operator] items parked; no hardware steering. Full text:
  RL_PLAN.md "SIM SPRINT".
- KPI = blockers to the next joystick hardware test, not occupancy.
  08-11 night: sim stand/walk blockers are THE focus; write needed
  code same-cycle (cfg-gated, default-off, tests, snapshot).
  Peripheral runs banned. Phases + MDP_PREFLIGHT + matched-parent
  controls binding; new reward terms add a REWARD.md row; a closed
  sim hypothesis reopens only on new HARDWARE evidence.
- BC-anchoring is the proven lever class (solved rise, hold, lower,
  tall gait); pricing-only levers on anchored stance TERMINALLY
  CLOSED (parking one foot evades any per-foot price). The park was
  invisible to joint-space MSE → `bc_anchor_foot_z` (3mm norm kills
  even sub-mm hover); the flat-rise stall was a demo plateau fixed
  point → `bc_anchor_min_h_ahead_mm`. Anchors can TEACH a defect:
  probe the teacher itself before blaming incentives.
- Tipped/perturb DR: tipped_start 0.30 stays default-ON, but the
  DOSE is closed for takeoff (saturated), tipped DR on anchored
  stance is closed as HARMFUL (learns to stay tilted), and the ~8°
  lean's teacher/exposure levers are measured-exhausted (tiltcomp
  1/2/3; the untrained parent still recovers best). All takeoff
  perturb families closed (walk-kick, rise-rock, walk-push);
  contact/pinning falsified; the hardware stand fall is a 3-foot
  knife-edge sim survives (SIM.md gap 4). RULINGS 08-13: lean =
  MECHANICAL TRIM outside RL (no lean-pricing/exposure/teacher
  arms); takeoff = NO more reward/DR sweeps — instrument the
  transient, then design a staged gait-entry transition
  (deploy-side + sim prototyping; training only after an
  instrumented design).
- Turn: de-scoped from the deliverable, alive as a track; yaw income
  defects fixed+banked; price escalation closed; signs correct both
  ways (rate unmeasured). Omni: pricing exonerated, walk-tick BC
  failed — rot60 is the shipped answer.
- Quad: quad-hold solid, mixing erodes walk (specialist-only);
  open-loop statically-stable quad crawl measured GEOMETRICALLY
  INFEASIBLE. RULING 08-13: feedback/RL rear-four stepping is
  PERMITTED as the quadwalk bank reference behind an explicit
  pre-registered robustness gate (spec first artifact, conditions
  in quad/STATUS.md) — quadwalk launchable under those terms.
- Multitask: PAUSED by operator 08-13 (dynrep prioritized; wave-1
  read stands: interference real+monotone at 20M, all cheap levers
  failed; MoE only after real interference is shown). nobc: RULING
  08-13 — from-scratch GAIT line CLOSED (reopen on new hardware
  evidence only); stand-from-scratch charter stays.
- Watcher idle-kick backoff (15m→4h no-op spacing, snap-back on
  activity) operator-APPROVED 08-13.
- One-variable-per-run REPEALED by operator 08-15 (~18:20 UTC,
  relayed via authenticated Cursor session): multi-variable/coupled
  bundles are permitted when the operator orders them or the cycle
  judges the coupling necessary; pre-registration and honest
  verdicts still required (RESEARCH_RULES.md, guardrails.yaml
  updated; this file wins on conflict with any older restatement).
  SCOPE CONFIRMED GLOBAL (fb_20260815T184319_458b20, 18:43 UTC,
  operator-stamped/trusted-loopback client, answering the
  confirm-or-disavow below): the operator's verbatim words were "we
  removed the one variable rule" — the global repeal commit
  (24707196) stands as written; the earlier 18:15 KICK's
  one-run-exception phrasing for cw-recover-any1 is SUPERSEDED, not
  a live scope limit. Stop treating bundles conservatively; the
  WAITING-ON confirm-or-disavow entry is CLOSED (STATUS.md).
- cw-recover-any1 / universal-recovery: CONFIRMED GENUINE by the
  operator (08-15 18:15 UTC authenticated KICK) after 5-6 correct
  channel-grounds declines of its unauthenticated MCP copies —
  future cycles must NOT re-decline this line on channel grounds.
  Executed 08-15: `recover` mode (PBRS potential-difference reward,
  held-success termination, adaptive reset-family curriculum,
  default-off) built + banked (RECOVER section,
  test_task_semantics.py), REWARD.md §4c, run launched on the
  footlow2_hard1 lineage (hw track).
- Bench/UI: sit NEVER refuses on pose delta (stand-only refusal);
  errors stay copyable; thermal reads debounced (phantom wall).

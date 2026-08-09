# RL_LOG — condensed campaign log

Full history: `archive/RL_LOG_FULL_2026-08-09.md` (through c34) and
`archive/RL_LOG_FULL_2026-08-09c.md` (c35–c59 detail). Per-run facts:
`rl_docs/runs/<run>.md` (generated) + W&B notes. Skills inventory:
`rl_docs/SKILLS.md`. Cycle transcripts: controller `/workspace/cycle_logs/`.

**APPEND RULE (operator, 08-09, tightened after the log tripled in half
a day): ONE line per cycle, written ONLY via `ops.sh logline "..."`.**
Never `cat >>` this file. The line is: cycle, run(s) → verdict word,
one-clause takeaway, what launched. ALL evidence, numbers, and
narrative go in the ledger verdict (auto-renders `rl_docs/runs/`) and
the W&B OUTCOME note — if it matters, it lives there, not here.

## State (2026-08-09 ~19:00Z)

- **WALK CHAMPION: ppo_goal_cw_walk_longdist_r2.zip md5 bcddc65c**
  (c44, seed-confirmed 3/3, operator-accepted; DR1.0 det slip 1.06;
  NOT hardware-ready — paddle-slide persists, sto draw-stalls).
  Slip root = sim contact/current pricing → operator calibration
  (P0); reward-side anti-slip levers ALL closed.
- **Driving:** joystick gate (`eval_drive`) is the binding eval;
  champion + joystick45 + joyjit-dr05-c1 all PASS it. Heading ladder
  FROZEN at ±90° (head135 FAIL); rear coverage waits on
  mirror-symmetry [CODE].
- **Validated axes (see SKILLS.md for the full table):** crouch to
  −60mm, terrain amp1.0 (saturated), payload +40%, latency 2.5x,
  deadband 3x, CoM shift 30mm, stop density 0.35, 60s endurance,
  DR0.5 on steering/±90. Full-DR (1.0) retrain lever CLOSED (2x).
- **Next implementation cycles [CODE]:** mirror-symmetry (3
  motivations), quad-hold goal mode (feasibility sweep = GO).
- Compute: 12 GPU pods (train-0..11; g12ba48 cordoned, 12–15
  deleted). `capacity.py` = live truth; backlog auto-drains.

## Walk line — what was tried and learned

- **dr04b lineage** (pre-08-08): scalar champion, 0/9 gait-valid on
  video — shuffle/flag-leg. RETIRED. Lesson: scalars hide exploits;
  video eval is the promotion standard.
- **w07/flag/flagw/lp/speedhi**: penalty-coefficient iteration on the
  shuffle is a dead approach (all REFUTED).
- **nv/nv2**: deployable obs are not the blocker. **aac**: asymmetric
  critic is a retention tool, not a gait fix.
- **phase / phase-stance(2)**: phase reward as basin escape REFUTED.
- **step0** (08-08, operator recipe: step-event + drag + park-duty
  pricing, fresh init, walk-only): **FIRST genuine six-leg gait.**
  Lineage standard since.
- **step0-c2 vs lowent**: entropy runaway = the plateau driver; warm
  starts use ent 0.001. Identical-config continuations bought nothing
  5x — CLOSED as a move.
- **h15b**: 15 s is the lineage eval standard. DR is NOT the
  bottleneck (untrained parent passes DR 0.3/0.6).
- **kgate**: park PRICING refuted. **Cycle 27:** there is no park
  attractor — the "park" was one fixed backward draw; real defects are
  paddling (91% of slip) + overspeed.
- **anchorgate** (c31): income GATING works where charging failed —
  det slip 1.543→1.240, champion. **anchorgate-c1** (c32):
  cadence-inflation exploit. **anchortol5** (c34): the policy PAYS a
  binding stake and keeps creeping — **no income lever can outbid
  in-sim-free sliding; slip root is contact/current pricing
  (operator).** **step0-anchor** 40M fresh (c33): re-derives the
  paddle from scratch — pricing problem, not basin/history.
- **loadslip** (c40): episode-level slip stake also paid-not-fixed —
  reward side of skating CLOSED (seed-confirmed).
- **fast** (c42): speed ceiling ~0.065 m/s is GAIT-limited; speed
  arms join skating behind the calibration.
- **stalls** (c47→c52): fixed-draw stalls are command-conditioning;
  resample training fixes resample-on eval only, incidence <2% —
  stall lever class CLOSED, panel draw kept as canary.
- Exploit-watch columns (permanent): cadence/stance count, per-leg
  swing asymmetry, allowance-riding, unload-sweep.

## Stance line

- Heights SOLVED at DR 1.0 (crown jewels, canary-protected). Lower
  posture (flag leg) never solved; refuted in order: mix, posture
  pricing, exploration, terminal pricing, reset diversity, dense
  charging. **Root cause (c28): the hover is INCOME-POSITIVE**
  (current model prices planted descent 4x hover) — **BLOCKED on the
  same operator pricing ruling.** No stance shaping until then.

## Infra lessons (all landed in tooling — see rl_docs/COMMANDS.md)

- Launcher is mandatory (capacity, code-SHA gate, dup refusal);
  ledger writes via `launch_run.py update` only.
- Canaries protect only skills the lineage HAS (`--no-canary` on
  step0 walk-only lineage).
- Pre-08-08 seed twins were bit-identical clones; conclusions void.
- Sync the pod at snapshot time; snapshot BEFORE drain, never
  mid-drain (HEAD moves under the launcher).
- Specs must ALWAYS pass `--out-name` (prestage pullckpt breaks on
  MJX default names otherwise).
- Dirty-marker refusals: state files, .md, logs, zips, tmp files are
  all EXCLUDED from the dirty check now; a genuinely dirty tree means
  COMMIT FIRST. wandb/ run logs are gitignored (were tracked, poisoned
  every sync 08-09).
- /dev/shm leaks after killrun: clean hexmjx-* segments before
  relaunching on that pod (c54: two 0-step deaths).
- Node loss is normal (g12ba48 cordoned 08-09): fleet truth is
  `capacity.py`, never a doc.
- Fixed-draw sto panels can pin a "failure" to one command draw —
  check WHICH episode fails before theorizing.

## Cycle digest (c38–c59, 08-09; detail in archive + rl_docs/runs/)

- c38 longdist-r2 det slip 0.96 campaign-best; drain dirty-fix. c39
  dr05-r1 FAIL; stability pricing arms launched. c40 loadslip FAIL →
  reward side of skating CLOSED. c41 fps SUSPECTs = false alarms.
- c42 fast FAIL (gait-limited ceiling). c43 steer-explore FAIL (no
  omni transport); diag45 operator-killed. c44 **champion promoted:
  longdist_r2** (accepted by operator ruling 8); speedband killed as
  stale. c45 lowgait/wander PASS, fwdband-r1 FAIL (command mix closed),
  fall300 NO-EFFECT.
- c46 lowgait30 PASS. c47 terrain05/wander30/endur60 PASS,
  longdist-dr05 FAIL (DR not the stall lever). c48 strafe-dr05 +
  wander-dr05 PASS. c49 joystick45 PASS (joystick gate born);
  joyjit-dr05 starved→rebalanced; dr.<field> cfg hooks landed.
- c50 lowgait40 PASS, tilt05 NO-EFFECT (tilt lever closed), endur60-s1
  endurance seed-robust. c51 head90 PASS (±90 envelope; L/R asym →
  mirror-sym motivation); drain-dead root cause = one uncommitted
  artifact. c52 stallfix FAIL → stall class CLOSED.
- c53 joyjit-dr05-c1 PASS (best driving candidate), longdist-s2 PASS
  (3/3 seeds), longdist-dr10 FAIL (full-DR lever closed);
  stopgo35 starved→c1. c54 lowgait50/terrain10 PASS (terrain
  SATURATED), strafe-dr10 FAIL (DR0.5 = strafe ceiling); endur60-r2
  FAIL on slip clause; shm-leak bug found+fixed.
- c55 wander-dr10 FAIL (DR0.5 = steering ceiling); READY well dry →
  quadruped sweep flagged. c56 head135 FAIL (**ladder frozen ±90**),
  head90-dr05 PASS, latjit25 PASS. c57 payload50 PASS; **quadruped
  feasibility sweep = GO** (39mm margin w/ shift+splay). c58
  lowgait60/wander-dr05-s1/wander60 PASS; comshift30/deadband30 PASS,
  friclow letter-pass only (slick draws skate). c59 stopgo35-c1 PASS,
  torquedroop NO-EFFECT (champion already covers 0.75–1.05x free —
  run the parent baseline FIRST on exposure axes); fricvar PASS
  (0.4–1.6x, slick-tail churn), speedband2-r1 FAIL (stale reissue of
  the CLOSED c42 speed class — check run docs for the CLASS before
  requeueing).

## Cycle log (one line per cycle via `ops.sh logline` — no exceptions)

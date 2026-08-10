# AGENT.md — how the autonomous RL agent works, what we learned, what's next

Handoff brief for the next human or LLM taking over this campaign.
Read this, then: `../RL_GOALS.md` (mission), `../RL_PLAN.md` (current plan +
binding rulings), `../RL_LOG.md` (history), `COMMANDS.md` (how to run
things), `../rl_move/orchestrator/README.md` (architecture details).
Written 2026-08-09 after ~60 autonomous cycles in one day.

## The one-paragraph version

A watcher on the controller pod polls W&B; when training runs finish
it spawns concurrent LLM "decision cycles" (`claude -p` with
`ORCHESTRATOR_PROMPT.md`). Each cycle TRIAGES finished runs (video +
eval table + reward curve, ~10 min each), records a verdict in the
ledger, and REFILLS free GPU slots by queueing new experiment specs
into `backlog.json`. Mechanical workers do everything else: a drain
pushes queued specs onto free pods (self-repairing), checkups catch
dead/starved runs, a pre-stager pulls checkpoints and starts gate
evals before the cycle wakes up. **Software owns facts and
throughput; the LLM owns judgment.** That split is the single most
important design decision — everything in "what failed" below traces
back to some place where it was violated.

## How the agent picks the next run

Priority order (from `ORCHESTRATOR_PROMPT.md` §4):

1. **Continuations of near-misses** — one, not two; identical-config
   continuations are CLOSED as a move (0-for-5 historically).
2. **The plan's next rung** — `RL_PLAN.md` queue + its binding
   operator rulings and the readiness-review priorities (P0/P1/P2).
3. **`WISHLIST.md` topmost [READY] items** — the operator's backlog;
   items marked [CODE] need an implementation cycle first.

Non-negotiable spec shape: warm-start off the current champion, ONE
variable per run, pre-registered gate with explicit if-true/if-false,
plain-English hypothesis, always `--out-name`. More than 2 free slots
= queue multiple experiments from DIFFERENT lines in the same cycle.
Design questions with a plausible answer never idle a pod: ASSUME AND
GO, log the assumption for operator review. `guardrails.yaml` caps
everything (launches/cycle, GPU steps/cycle, concurrent cycles).

## What we discovered WORKS (keep doing this)

- **Mechanical verification of operational state.** The LLM is never
  authoritative about what launched/exists/runs. The launcher
  verifies INTENT→RUNNING (process + W&B + code SHA); the ledger is
  the only record; `ops.sh census` reads /proc, not memory. Before
  this, runs were recorded as launched that never existed.
- **Backlog + self-repairing drain.** Decoupling "what to try"
  (LLM, minutes-to-hours) from "keep GPUs busy" (software, seconds)
  ended the idle-fleet problem. A free slot + non-empty backlog is a
  bug by definition.
- **Triage-first review.** Most verdicts need one video + one eval
  table (`ops.sh review <run>`). Forensics only on a trigger (gate vs
  video disagreement, anomalous metrics, plan forks, reward/env code
  changes). The earlier heavyweight process burned 40 min/run for no
  added decision quality.
- **Pre-registered falsifiable gates.** A run whose failure branch is
  written down BEFORE launch produces a usable verdict even when it
  fails (most of today's lever-closures came from clean if-false
  branches). "We haven't tried this coefficient" experiments are
  banned and stayed banned.
- **Video over scalars, exploit-watch columns.** Every metric the
  campaign optimized has been gamed at least once (velocity → 
  shuffling, clearance → flag leg, touchdown allowance → cadence
  inflation). gait_valid + frame strips + per-leg swing asymmetry
  catch what scalars hide.
- **Isolated-axis robustness runs off a champion.** One physics axis
  per run (payload, latency, deadband, CoM shift, friction, terrain)
  at DR0 → cheap, parallel, composable; then a DR-compose rung. This
  filled 12 slots with informative runs after "more seeds" ran dry.
- **Ladders, not leaps.** Abrupt widening breaks the gait every time
  (±180 heading, 2x speed band — both refuted). One rung at a time
  off the previous rung's checkpoint works (heading ±45→±90 passed;
  ±135 failed and FROZE the ladder — that's the ladder working).
- **Purpose-built gates for new behaviors.** The generic harness
  samples the training distribution; it cannot prove direction
  coverage or command-flip robustness (backforth was gated
  meaninglessly). `eval_drive.py` (scripted panel + flip stress) is
  the joystick gate; build the eval WITH the behavior, not after.
- **Helpers over instructions.** Agents re-derived the same parsing
  500+ times in a day despite docs saying not to. The fix that
  worked: make the helper (`ops.sh review`), put it FIRST in the
  prompt, and ban the alternative. Instruction without tooling does
  not change agent behavior.

## What we discovered FAILS (stop/avoid these)

- **Prompt rules without enforcement.** "Keep RL_LOG entries to 1-3
  lines" was ignored by every cycle; the file tripled in half a day.
  Now the only write path is `ops.sh logline` (one line, locked).
  Generalize: if a rule matters, encode it in a tool or a tripwire.
- **Concurrent cycles racing on shared state.** Every incident class
  we hit: near-duplicate launches (velsag/velsag30), dirty code
  markers from another cycle's uncommitted artifact, HEAD moving
  mid-drain, RL_LOG merge conflicts. Mitigations that work: locks
  (git/ledger/backlog), name-stem dedupe warnings, launcher SHA
  gates, generated per-run files instead of a shared log. Re-check
  census AFTER queueing, not just before.
- **Analysis paralysis with idle GPUs.** The agent's natural failure
  mode is over-deliberation: waiting for operator input, re-reading
  history, writing essays while pods idle. The operator's standing
  order: idle pods are the failure, not imperfect experiments.
- **"More seeds" as a default refill.** It's a cop-out. Seeds are
  for PROMOTION panels (ruling 7), not for filling slots.
- **Coefficient iteration on a rejected behavior.** k=5,10,20,40 on
  the same penalty never fixed anything here. Close the lever class
  after two clean refusals and change the mechanism.
- **Verdicting runs the cycle didn't evaluate.** Class-stops must
  name the evidence run; the operator misread a class-stop as a run
  FAIL once. Also: check the run-doc CLASS before requeueing a
  name-varied spec (speedband2-r1 re-ran an already-closed class).
- **Infra assumptions that bit us** (all now in COMMANDS.md
  gotchas): pods have no ps/pkill/curl/git; kubectl exec has a 2-min
  timeout (launch looks dead, isn't); killrun leaks /dev/shm
  segments; W&B lags fresh launches ~8 min; naive /proc kill scans
  match themselves; hard-killing the watcher murders in-flight
  cycles (use `restart_watcher.sh`).
- **Campaign science** (what the ROBOT taught us — full list in
  RL_LOG "what was tried and learned"): income levers can't outbid
  in-sim-free sliding (slip root = contact/current pricing =
  operator calibration, P0); DR is not a gait fix; full-DR retrains
  buy nothing; there is no park attractor; entropy runaway is the
  plateau driver on warm starts.

## Future work (in rough priority order)

Campaign (see RL_PLAN for binding detail):

1. **Contact/current calibration against hardware (P0, operator).**
   Everything slip-related is blocked on it; no new anti-slip reward
   arms until then.
2. **Mirror-symmetry augmentation [CODE]** — 3 independent
   motivations (head90 L/R asymmetry, strafe-dr10 flag legs, rear
   coverage need). Needs mirror index maps + trainer support + probe.
3. **Quad-hold goal mode [CODE]** — feasibility sweep passed (GO):
   static 4-leg stance with −20mm CoM shift + mid-leg splay. Then
   weight shift, then quadruped stepping.
4. **Omnidirectional joystick composition** — combine the ±90
   envelope, flip hardening (joyjit-dr05-c1 is the best driving
   candidate), and steering-DR into one policy; gate with
   eval_drive. Rear hemisphere waits on mirror-symmetry.
5. **Estimator rung** (DreamWaQ-style concurrent estimator) — the
   settled next architecture step; unlocks the <0.7x torque/low-grip
   boundary cases exposure can't fix.
6. **Sim-to-real ladder** (readiness review Gate C): freeze a
   forward-only policy on physical metrics, supported hardware
   trials. Hardware safety rules in AGENTS.md are absolute.

Agent/process:

7. **Mechanical gate pre-check.** The pre-stager could diff gate
   scalars vs the pre-registered thresholds and mark "clear PASS
   candidate" — cycles then spend judgment only on marginal calls.
8. **Class-level dedupe.** Name-stem warning exists; a better check
   is against rl_docs/runs verdicts ("this axis/class is CLOSED").
9. **Watcher-side eval_drive pre-staging** for driving-line runs
   (currently only the generic gate eval is pre-staged).
10. **Doc-size tripwires** — warn when RL_PLAN/RL_LOG exceed their
    line budgets instead of waiting for an operator cleanup pass.
11. **Node-change automation** — g12ba48 loss was handled by hand;
    capacity.py could flag guardrails/manifest drift when nodes
    come and go.

## First 10 minutes on takeover

```sh
cd /workspace/weird_objects/hexapod_walker/prototype_sts3215  # controller
bash rl_move/orchestrator/ops.sh census        # what's actually training
python3 rl_move/orchestrator/capacity.py       # slots/nodes, live
bash rl_move/orchestrator/ops.sh triage 12     # anything lost/ignored?
python3 rl_move/orchestrator/launch_run.py backlog list
tail -20 /workspace/orchestrator.log           # watcher alive?
```

The operator watches a status webpage (http://127.0.0.1:8090 via
port-forward). If they say it's down, the setup/restart runbook is in
`COMMANDS.md` § "Operator status page" — two pieces: `statusweb` tmux
session on the controller + `kubectl port-forward … 8090:8090` on
their laptop.

Then read RL_PLAN.md end to end (it's ~170 lines and every line is a
ruling). Do not touch the physical robot, ever, without an explicit
operator instruction in the current message (AGENTS.md, hardware
safety rules — a real robot was damaged on 2026-08-06).

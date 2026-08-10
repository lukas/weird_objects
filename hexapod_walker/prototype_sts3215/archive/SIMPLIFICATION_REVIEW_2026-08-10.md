# Hexapod RL Campaign — Simplification Review (GPT, 2026-08-10)

> Operator-adopted 2026-08-10. Source: external GPT review of the full doc
> bundle (`~/Documents/hexapod_rl_docs_for_gpt_2026-08-10.md`), delivered as
> `CURSOR_RL_CAMPAIGN_SIMPLIFICATION_REVIEW.docx`; converted to markdown
> verbatim. Where this review conflicts with older process rules, THIS
> review wins. Adoption edits landed the same day: guardrails
> `idle_gpu_needs_hard_reason` → false, `auto_continue_lineages` → [],
> RESEARCH_RULES diagnostic-vs-integration + gate-invalidation rules,
> RL_PLAN critical-path header.

**Bottom line:** The campaign has corrected the original "random experiment
factory" problem, but is now at risk of becoming an RL bureaucracy. Simplify
before adding more process. Preserve mechanical safety/reproducibility;
remove rules that manufacture experiments or require elaborate analysis when
the video already gives the answer.

## 1. What to keep

- Robot safety, code/checkpoint provenance, exact deployment-contract
  evaluation, duplicate-launch prevention, and mechanical logging. These
  prevent real damage or invalid science.
- Video review. If the motion is physically wrong, scalar success does not
  rescue the run.
- Matched-parent controls for injected friction/noise/latency/physics axes.
- Short discovery runs and the rule that long runs harden behavior already
  seen.
- Two misses in the same behavioral class -> change the hypothesis, not the
  coefficient or step count.
- CURRENT_TRUTHS first. Historical log volume must never outweigh a newer
  accepted hardware fact.
- The compact RL run-interpretation rubric: physical behavior and held-out
  task success outrank training reward.

## 2. What to remove or weaken now

- Any rule that treats idle GPUs as a failure. Idle compute is acceptable
  when the next useful work is specification, hardware, or code.
- Reward-based automatic continuation. A rising training return alone must
  never mechanically buy more steps.
- Peripheral DR/robustness composes merely because capacity exists. The
  existing robustness campaign has already answered the broad question.
- Long forensic analysis of a known exploit. Flag-leg, tripod, freeze, park,
  obvious dragging, or march-in-place should usually be a fast STOP and
  task/reward/eval fix.
- Elaborate schemas for every obvious follow-up. Keep the ledger, but make
  scientific judgment fast.
- Any process layer that duplicates CURRENT_TRUTHS / RL_PLAN / the
  orchestrator without adding a mechanical guarantee.

## 3. Minimal research loop

This should be the mental model for every autonomous cycle:

1. What are we trying to make the robot do next? Current core: honest
   stand-up, commanded turning, then unified joystick control.
2. Run the simplest experiment that could unblock that behavior.
3. Look at the gated video first. If behavior is physically wrong, stop and
   fix the task, reward, evaluator, or simulator.
4. Check the pre-registered task metric and held-out eval against the
   correct parent/baseline.
5. If it helped, harden it. If the same idea misses twice, change ideas.
6. Do not invent unrelated work because a pod is idle.

## 4. RL interpretation basics — use before theorizing

| Observation | Interpretation |
| --- | --- |
| Training/task signal flat; eval flat; behavior wrong | FAIL — mechanism did not learn. |
| Reward rises but task behavior/metric does not | REWARD / TASK-SPECIFICATION BUG — likely shortcut. |
| Training task improves but held-out eval does not | GENERALIZATION FAILURE — diagnose train/eval mismatch, variance, curriculum, or overfit. |
| Metrics/eval improve but video is physically wrong | EVAL / REWARD LOOPHOLE, unless the mismatch is clearly simulator physics. |
| New skill improves but an old protected skill degrades | SKILL INTERFERENCE — not a clean pass. |
| Eval improves, video is good, protected skills hold | PASS — hardening/confirmation is justified. |
| Sim looks good but hardware anchor/replay gets worse | SIM / TRANSFER FAILURE. |

Important refinement: ask "did learning ever happen?" rather than only
looking at the final checkpoint. Inspect early / middle / final. NEVER
LEARNED and LEARNED THEN LOST are different diagnoses.

## 5. When more training is allowed

- The intended qualitative behavior already exists.
- The task-specific metric and/or held-out eval is still moving in the right
  direction, or there is concrete evidence the run is budget-limited.
- The run is not dominated by a known exploit.
- Protected skills are not being silently traded away.

Do not continue because "maybe another 20M will fix it." Flat reward + wrong
behavior + known exploit is a stop condition, not a continuation hypothesis.

## 6. Separate two experiment types

**Diagnostic experiments**

- Purpose: establish causality or test one mechanism.
- One variable per run is the default.
- Use short discovery budgets when behavior is new.
- Matched-parent controls are mandatory for injected axes.

**Integration/product experiments**

- Purpose: answer "does the complete controller work?"
- May combine already-validated ingredients intentionally.
- Do not pretend these runs isolate causality.
- The clean unified-controller flagship belongs here.

## 7. Architecture recommendation

- Keep the target ambitious: one genuinely unified joystick controller is a
  reasonable goal.
- Use hist16 as the preferred temporal starting point if it remains
  competitive. The robot has actuator delay, hidden contact/load state, no
  true body-velocity measurement, and useful recent proprioceptive history.
- Before MoE, run the clean control: hist16 + explicit mode/command
  conditioning + a somewhat larger plain MLP (e.g. 256x256 or 256x256x128).
- Train it with a curriculum on HOLD / RISE / LOWER / WALK / TURN after each
  task's semantics are sane.
- Move to MoE only if correctly specified multitask training shows
  reproducible skill interference. Do not use MoE to compensate for gameable
  rewards/evals.
- Do not run a 16 -> 24 -> RNN -> Transformer architecture ladder unless a
  real failure or controlled comparison motivates the next rung.

## 8. Stand-up: simplify the problem definition

Standing should be a state predicate, not "torso reached target height."
The next stand-up work should define a valid walkable plant geometrically:

- Body height in the walkable plant band.
- Roll/pitch within a reasonable band.
- All intended support feet geometrically near the floor; no flag legs /
  tripod cheat.
- Projected CoM inside the support polygon with margin.
- Feet/joints inside a broad plausible plant manifold.
- Current/safety reported separately while the simulated current model
  remains uncertain; do not let a known-bad current model define geometric
  standing success.

Then prove the easiest acquisition problem first: near-plant -> valid-plant.
Only expand starts toward belly after that works. If PPO cannot finish from
80-90% of the way there, fix the MDP/sim before asking it to discover the
whole maneuver.

## 9. Walking desynchronization: two high-value diagnostics

1. Replay the known-good scripted gait through the current
   simulator/actuator/contact stack. If it develops phase drift and
   dragging, fix the simulator before blaming RL.
2. For RL video/logs, overlay per joint: policy proposal ->
   SafetyLayer/applied target -> actual q, plus foot contact state. This
   separates impossible policy choreography from actuator lag /
   contact-induced phase loss.

## 10. Documentation: simplify, do not add another bureaucracy

- CURRENT_TRUTHS.md: short accepted facts only.
- RL_PLAN.md: current goal, 2-4 blockers, next experiments, closed moves.
- Orchestrator prompt: mechanical cycle procedure plus the compact
  interpretation rules.
- RL_LOG.md: navigational history, not a knowledge base.
- Detailed run docs/archive: retrieve only when needed.

Do NOT build a richer blocker registry or giant metric ontology unless the
agent demonstrably starts wandering again. A short blocker section in
RL_PLAN is enough for now.

## 11. Five-line critical-path section to put near the top of RL_PLAN

CURRENT GOAL: joystick-controlled real robot. BLOCKERS: honest rise to
walkable plant; commanded turning; deployment-equivalent loaded/contact
dynamics. DEFERRED: quad, generic DR composes, posetrack, architecture
curiosity work not tied to a demonstrated failure. RULE: idle GPUs are fine.
TEST: the next experiment should take less than one minute to explain and
should change what we do before the next useful hardware test.

## 12. Suggested orchestrator edits

- Keep: ops.sh review, WRAPUP, ledger write path, video review,
  matched-parent controls, DIG-IN escalation, snapshot/launcher safety.
- Delete/disable any mechanical idle-GPU violation or "fill spare capacity"
  requirement.
- Delete reward-quarter-based auto-continuation.
- Clarify that "one variable per run" applies to diagnostic experiments;
  integration flagships may intentionally combine validated ingredients.
- Clarify that a gate discovered to measure the wrong thing invalidates
  conclusions that depend on that gate until re-evaluated.
- Add: do not infer importance from how many lines a topic occupies in
  RL_LOG; CURRENT_TRUTHS and current blockers outrank historical token
  volume.
- Keep the simple evidence order: physical behavior -> held-out task
  success -> protected-skill retention -> training task metric -> training
  reward.

## 13. Prime directive for Cursor

Get to a reliable joystick-controlled physical robot with the least
unnecessary research. Automate boring correctness; do not bureaucratize
scientific judgment. Before training a new behavior, make sure the
reward/evaluator actually prefer it over known cheats. Use short runs to
discover, long runs to harden. Look at the video early. If behavior is
obviously wrong, stop. Hardware evidence outranks generic simulator
robustness. Idle compute is acceptable. Keep the unified hist16 controller
as the architectural target, but do not add MoE or deeper temporal
architectures until a clean experiment demonstrates the need.

## 14. Recommended immediate sequence

1. Prune stale throughput rules: idle-GPU pressure and reward-based
   auto-continuation.
2. Finish the geometric valid-plant stand definition and its tiny
   semantic/video preflight.
3. Run near-plant -> valid-plant discovery; expand toward belly only after
   it works.
4. Fix turning through command exposure/curriculum rather than another yaw
   coefficient sweep.
5. Use the scripted gait and proposal/applied/q overlays to diagnose motor
   phase drift/contact mismatch.
6. Then run the clean unified flagship: hist16 + explicit mode conditioning
   + larger plain MLP on HOLD/RISE/LOWER/WALK/TURN.
7. Only if that clean experiment shows genuine cross-skill interference
   should MoE become the next architecture experiment.

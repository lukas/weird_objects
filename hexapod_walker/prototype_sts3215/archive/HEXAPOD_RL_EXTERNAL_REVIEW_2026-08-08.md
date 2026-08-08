# External Review — Recommended Next Phase

The overnight campaign produced enough evidence to change the framing of the project. The main question is no longer whether raw-joint PPO can learn useful hexapod behavior. It clearly can. The next phase should focus on:

1. freezing and validating a safe stand/recovery hardware candidate,
2. continuing walking through curriculum rather than reward escalation,
3. understanding multi-task interference,
4. removing privileged observations from walking,
5. testing whether policy architecture/capacity is becoming a bottleneck.

## 1. Freeze the stand/recovery champion

If `cw-stance-dr10` completes successfully, freeze it as a versioned hardware candidate (`STAND_V1` or equivalent).

Do not continue training this checkpoint with walking. Walking has repeatedly shown that it can erode rise, so the known-good recovery policy should become an immutable reference.

Before hardware deployment, perform a much larger frozen-policy evaluation rather than more PPO training.

Suggested validation:

- 500–1,000 episodes per important start condition.
- Deterministic and stochastic inference.
- Full DR 1.0.
- Explicit parameter sweeps/corners in addition to random DR:
  - friction,
  - actuator strength,
  - actuator latency,
  - servo rate limits,
  - geometry,
  - IMU bias/noise/mount error,
  - start-pose perturbations.
- Test combinations of adverse parameters, not merely one-dimensional sweeps.
- Report distributions and worst cases, not just means.
- Automatically save telemetry/video for the worst episodes.
- Continue reporting per-servo current, hot time, imbalance, joint speeds, etc.

Random domain randomization can undersample dangerous combinations such as low friction + weak actuator + high latency. Explicit corner testing should therefore be a hardware gate.

## 2. Recalibrate torque → current before trusting the hardware safety gate

This is currently a blocker for interpreting simulated current.

The MuJoCo 2.3.7 → 3.11 transition moved quiet-hold predicted peaks from roughly 2.46 A to 2.60 A without a corresponding physical change.

Because simulated policies are showing approximately 2.5–2.7 A peaks near the hardware breaker threshold, do not treat those numbers as calibrated until the torque/current mapping is revalidated.

This should happen before aggressive hardware rise testing.

## 3. Treat slow walking as the beginning of a curriculum, not the final walking solution

The strongest walking result from the campaign is:

- increasing progress reward did not improve tracking,
- making commands achievable dramatically improved tracking.

That is strong evidence that curriculum is the correct axis.

Do not make another large jump such as 0.02–0.06 → 0.02–0.08.

Implement an adaptive speed curriculum.

Conceptually, begin with something like:

- 70% commands from mastered range,
- 20% commands near the current frontier,
- 10% commands slightly beyond the frontier.

For example:

- 70%: 0.02–0.06 m/s
- 20%: 0.06–0.07 m/s
- 10%: 0.07–0.08 m/s

Maintain evaluation statistics by speed bucket and only shift command probability upward when the frontier bucket becomes reliable.

Make speed-conditioned evaluation a first-class metric. Instead of relying primarily on a single mean velocity error, report curves/tables of:

- commanded speed,
- achieved speed,
- tracking error,
- success rate,
- stride length,
- swing count,
- slip,
- current,
- stability.

The important quantity is now the location and movement of the learning frontier.

## 4. Investigate multi-task interference before adding complicated regularization

Walk-heavy training repeatedly erodes rise.

Do not jump immediately to EWC.

First try simpler task-mix changes so that rise/lower/stand receive substantial continued training while walking progresses.

Also directly measure whether the objectives conflict.

If practical, estimate policy gradients on separate walk and rise batches and measure their cosine similarity:

`cos(g_walk, g_rise)`

If gradients are frequently strongly opposed, this is evidence of genuine destructive interference rather than simply insufficient replay of rise.

That result should determine the architecture strategy.

If interference is modest:
- continue with a shared goal-conditioned policy,
- improve task sampling/rebalancing.

If interference is severe:
- seriously consider specialist policies,
- or shared representation with mode-specific policy heads,
- with optional distillation later.

There is no requirement that the physical robot ultimately use one monolithic neural network. A high-level state machine selecting a frozen recovery controller versus locomotion controller may actually be safer and easier to validate.

## 5. Deprioritize `raise`

Do not spend substantial compute optimizing `raise` from 5/6 to 6/6 unless investigation reveals that the failure indicates a broader defect.

Belly → stand is already a more demanding height-changing behavior.

Keep `raise` in evaluation because it is a useful diagnostic probe, but it should not currently drive training decisions.

## 6. Start removing privileged walking observations now

This is becoming one of the largest deployment blockers.

Walking currently observes true simulated body velocity. Hardware cannot provide that exact observation.

Do not spend a large amount of compute producing an excellent policy whose observation interface cannot be reproduced on the robot.

Begin a hardware-realistic observation line now.

First experiment with frame stacking/history using only quantities available on hardware:

- joint positions,
- joint velocities if reliably obtainable/estimable,
- IMU orientation,
- IMU gyro,
- previous actions,
- commanded velocity.

Provide perhaps 200–400 ms of history initially.

If frame stacking is insufficient, test a recurrent policy (GRU/LSTM).

The existing privileged policy can become a teacher. A useful architecture is:

Privileged simulation teacher:
joint state + IMU + true body velocity → actions

Hardware-realistic student:
joint/IMU/action history → actions

Consider behavior cloning/distillation from the teacher combined with the normal RL objective.

The key next milestone should be a walking policy whose observation vector can actually be constructed on the robot.

## 7. Run a targeted model-capacity/architecture experiment

It is now reasonable to test larger policies, but this should be an ablation rather than the primary strategy.

Keep the environment, curriculum, reward and training budget fixed and compare approximately:

- current MLP,
- ~4× parameter count,
- ~10–16× parameter count.

For example, if the current policy is approximately 256×256, candidates might include 512×512 and 1024×1024 or a somewhat deeper 512-based network.

Use multiple genuine seeds.

Measure:

- final walking performance,
- sample efficiency,
- seed variance,
- rise retention,
- gait quality,
- policy entropy,
- value loss,
- current/jerk behavior.

A particularly interesting result would be if increased capacity substantially reduces catastrophic interference between walking and rise. That would indicate that model capacity is actually limiting multi-task learning.

However, architecture may matter more than raw parameter count.

Also consider a shared trunk with mode-specific heads, especially if gradient-interference measurements support that hypothesis.

For the hardware-realistic observation problem, prioritize memory over sheer MLP size. A modest recurrent policy may be substantially more useful than a very large feed-forward policy because velocity/state information must be inferred from temporal observations.

## 8. Repair confidence lost because of the seed bug

Do not rerun the entire historical experiment tree.

Do replicate the foundational claims using genuinely independent seeds:

1. full-DR stand/rise/lower champion,
2. six-foot even-stance champion,
3. slow-walk champion.

Three genuine seeds each would provide much stronger confidence.

From now on every run should record enough information to detect accidental seed reuse:

- requested CLI seed,
- `model.seed`,
- Torch seed,
- environment seeds,
- parent checkpoint checksum,
- initial policy checksum,
- final policy checksum,
- git SHA.

Independent-seed experiments should fail loudly if their RNG/model initialization state indicates accidental duplication.

## 9. Suggested next compute allocation

Once the currently running round finishes, a reasonable four-pod allocation is:

**Pod 1 — Stand validation**
Massive frozen `STAND_V1` DR/corner/worst-case evaluation.

**Pod 2 — Adaptive walking curriculum**
Main curriculum experiment.

**Pod 3 — Adaptive walking curriculum, independent seed**
Test whether the result is reliable rather than a lucky trajectory.

**Pod 4 — Hardware-realistic walking observations**
Frame-stack/student experiment using no privileged body velocity.

Model-size/architecture experiments can enter the queue as pods become available.

## 10. Hardware strategy

Do not autonomously deploy experimental policies to the physical robot.

Once `STAND_V1` passes expanded simulation validation and current calibration is trustworthy, begin hardware validation conservatively.

The first hardware test should not be belly → stand.

Start with the robot placed in a normal/supportable standing configuration and test:

- joint/action mapping,
- IMU sign/frame conventions,
- deterministic inference,
- SafetyLayer behavior,
- current behavior,
- stability under tiny commanded corrections.

Progressively expand the envelope only after these agree with simulation.

Keep recovery/get-up and experimental walking policies separately promotable so that walking research cannot silently modify the known-good hardware recovery controller.

## 11. Research-process guidance

The overnight campaign's process was very good and should be preserved.

The valuable outcome was not simply better scores. The campaign:

- falsified the warm-start-trap hypothesis,
- falsified progress-reward repricing as the main walking solution,
- discovered the tripod stance pathology,
- found a dense reward that fixed it,
- discovered the seed bug,
- established speed curriculum as the strongest walking direction.

Continue operating this way.

For every significant experiment, maintain an experiment log containing:

1. observation/problem,
2. hypothesis,
3. experiment designed to distinguish that hypothesis,
4. result,
5. interpretation,
6. next decision.

Avoid running experiments simply because a hyperparameter has not been tried.

Prefer experiments that distinguish between competing explanations.

## Overall priorities

The project now has two valuable assets that should be treated separately:

**Recovery/control:** a robust raw-joint stand ↔ belly policy approaching hardware readiness.

**Locomotion:** an emerging slow gait for which curriculum has finally produced a clear learning signal.

Protect the first while aggressively investigating the second.

The most meaningful next milestone is not simply lowering walking velocity error another few thousandths.

It is:

**A policy using only hardware-available observations that walks robustly in simulation, alongside a separately frozen and validated recovery policy capable of reliably taking the robot from belly to a safe six-foot stance.**

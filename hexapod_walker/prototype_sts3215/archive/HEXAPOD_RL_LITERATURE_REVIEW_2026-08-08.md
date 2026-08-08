# Literature-Informed Review — Updates to the Hexapod RL Plan

Date: 2026-08-08

## Executive summary

Recent sim-to-real legged/humanoid RL literature broadly validates the current direction: raw joint-position targets, PPO, domain randomization, actuator modeling, safety/rate limits, curriculum learning, and exact-path visual evaluation.

The literature suggests four important updates:

1. **Use asymmetric actor–critic before relying on teacher→student distillation.** The deployable actor gets only hardware-available observations; the critic may get privileged simulator state such as true body velocity and contacts.
2. **Replace manual walk-speed widening with an automatic learning-progress curriculum.** Allocate training to the current learning frontier instead of repeatedly changing one global command range.
3. **Treat temporal information as a first-class sim-to-real mechanism.** Frame stacking or a modest GRU/LSTM can infer velocity and hidden actuator/environment state. This is likely more useful than simply making the MLP much larger.
4. **Formalize mode-conditioned reward routing.** Keep genuinely universal safety costs global, but explicitly route task-specific objectives by mode.

These should rank above a large model-size sweep.

## 1. Asymmetric actor–critic

Current walking uses MuJoCo `vx/vy`, which is privileged and unavailable exactly on hardware. Rather than first zeroing it or immediately distilling a privileged teacher, test asymmetric PPO.

Actor observations:
- joint positions
- hardware-realistic joint velocities
- complementary-filter tilt
- gyro
- previous actions / action history
- commanded velocity / goal
- optional observation history

Critic observations:
- all actor observations
- true simulator body velocity
- true body pose if useful
- exact contacts
- other privileged simulator state useful for value estimation

The critic disappears at deployment. Never expose privileged information to the deployable actor.

Suggested order:
1. hardware-observation actor + privileged critic
2. add observation/action history
3. compare frame stack with modest GRU/LSTM
4. teacher/student distillation if still useful

References:
- Rapid Locomotion via Reinforcement Learning / privileged-learning patterns: https://arxiv.org/abs/2405.10830
- Gait-conditioned humanoid locomotion with privileged critic: https://arxiv.org/html/2505.20619v2

## 2. Promote temporal observations to a planned experiment

The deployable actor already has a concrete reason to need history: it must infer body velocity and hidden actuator/environment state from hardware-observable signals.

Recent humanoid locomotion demonstrates that histories of proprioception and previous actions can implicitly reveal velocity, actuator response, contact state, latency, terrain interaction, and other hidden dynamics. This acts as implicit online system identification.

Architecture ablation:
- A: current MLP + roughly 8–10 frame history
- B: MLP encoder → GRU/LSTM 128–256 → policy head
- C: only later, consider a small causal Transformer

At 25 Hz, 8 frames provide about 320 ms of history.

Memory/history should rank above making the feed-forward MLP 10–16× larger.

Reference:
- Berkeley history-based humanoid locomotion: https://learning-humanoid-locomotion.github.io/

## 3. Replace manual speed widening with learning-progress curriculum

Project evidence:
- original command range: velocity error ~0.064
- reachable slow range: ~0.028–0.032
- tripled progress reward: no meaningful tracking gain
- abrupt widening to 0.07/0.08: regression

This strongly points to curriculum rather than reward magnitude.

Divide command space into buckets such as:
- 0.02–0.03
- 0.03–0.04
- 0.04–0.05
- 0.05–0.06
- 0.06–0.07
- 0.07–0.08
- 0.08–0.10
- 0.10–0.12 m/s

For each bucket track:
- velocity tracking
- valid-gait success
- falls
- slip
- stride
- current
- recent learning progress

Preferentially sample buckets where performance is currently improving. Do not spend most samples on already-solved regions or currently impossible regions.

Make command-speed → performance curves first-class W&B metrics.

Reference:
- LP-ACRL / learning-progress automatic curriculum: https://arxiv.org/html/2601.17428v1

## 4. Formalize mode-conditioned reward routing

This project has repeatedly demonstrated reward interference:
- stance clearance fixed the tripod but destroyed `raise`
- exempting `raise` repaired it
- walk-heavy training erodes rise
- gait/flag-leg terms mean different things in different modes

Formalize reward routing instead of accumulating ad-hoc exceptions.

Common/global terms:
- safety
- joint limits
- excessive current
- violent motion
- genuinely universal smoothness costs

Mode-specific terms:
- WALK: velocity tracking, gait validity, contact quality, appropriate swing clearance
- RISE: rise progress, final pose, contact progress, controlled motion
- LOWER: lower progress, controlled descent, safe end pose
- HOLD: attitude tracking, six-foot stance, load balance

Reference:
- Gait-conditioned humanoid locomotion / reward routing: https://arxiv.org/abs/2505.20619

## 5. Recovery line: freeze success rather than endlessly modify it

Recent humanoid stand-up work such as HoST uses diverse randomized starts, curriculum, dense/progressive rewards, smoothness constraints, motion-speed constraints, and hardware-oriented regularization. These principles are already present in the raw-joint recovery line.

This increases confidence in freezing a successful stand/recovery champion instead of continuing to mutate it unnecessarily.

Do not retrofit complicated architecture into a solved skill solely because a paper uses it.

Reference:
- HoST: https://arxiv.org/html/2502.08378v1

## 6. Gait validity belongs in evaluation, not only reward

The flag-leg policy is another specification exploit: scalar velocity looked acceptable while the actual gait was unacceptable.

Add generic configuration/gait sanity metrics:
- per-foot contact fraction
- per-foot maximum clearance
- sustained clearance duration
- swing count
- stride length
- slip
- joint-angle occupancy
- fraction of time near limits
- number of contacting/functioning legs
- symmetry where appropriate

The evaluator should identify pathology first. Change rewards only after understanding the failure.

`k_flag_leg` is useful, but it should not be the only defense.

## 7. Consider a weak gait prior only if exploitation continues

Figure's engineering work illustrates that task rewards can produce effective but undesirable locomotion styles, motivating a motion prior.

For the hexapod, do not immediately impose hard-coded joint trajectories. First try gait-validity metrics, flag-leg prevention, adaptive curriculum, and better temporal/deployable actors.

If bizarre gaits continue, consider a weak alternating-tripod contact-phase prior rather than trajectory imitation. Keep it soft so PPO retains freedom.

Reference:
- Figure RL walking: https://www.figure.ai/news/reinforcement-learning-walking

## 8. Updated view on model size

A capacity sweep remains worthwhile, especially to test whether multi-task interference is capacity-limited. But simply making the MLP huge is lower priority.

Successful systems increasingly gain capability from:
- temporal history
- recurrent policies
- asymmetric actor–critic
- privileged critics
- automatic curricula
- teacher/student methods
- domain randomization

Priority:
1. automatic learning-progress curriculum — very high
2. asymmetric actor–critic — very high
3. temporal actor/history — very high
4. mode-conditioned reward routing — high
5. large feed-forward MLP — lower

Still eventually compare current size vs ~4× and ~10–16× parameters under identical conditions and genuine seeds.

Reference:
- NVIDIA Isaac Lab Spot sim-to-real example: https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab/

## 9. Eventually calibrate DR from real hardware data

Broad domain randomization is appropriate before first transfer. Longer term, use real robot trajectories to infer the uncertainty distribution that actually matters.

Hardware testing should create a structured sim↔real discrepancy dataset for:
- roll/pitch
- joint positions and velocities
- per-servo currents
- gyro
- timing
- SafetyLayer events

Use safe real trajectories to estimate realistic distributions for latency, strength, deadband, rate limits, torque/current mapping, joint-zero offsets, IMU errors, and contact/friction behavior where identifiable.

Then move from broad guessed DR toward empirically calibrated uncertainty around the actual robot.

Reference:
- LoopSR: https://arxiv.org/abs/2409.17992

## 10. Keep stochastic robustness curves

The Body-IK line showed that deterministic success can hide almost zero exploration-noise margin.

For each champion, prefer a noise-response curve such as:

| action std | success |
|---|---:|
| 0.00 | 100% |
| 0.02 | 100% |
| 0.05 | 98% |
| 0.10 | 91% |
| 0.15 | 73% |
| 0.20 | 31% |

Store this as champion metadata. A deterministic 100% policy with a cliff at tiny noise is not equivalent to one with broad margin.

## 11. Separate development gates from hardware gates

Twenty episodes is useful for normal experiment promotion but too small for final hardware promotion.

Development champion:
- ≥20 exact-path episodes
- fixed seeds
- deterministic + stochastic probe
- visual inspection
- mode-split metrics

Hardware candidate:
- hundreds/thousands of frozen-policy simulations
- nominal, full DR, explicit bad corners, and combinations of adverse parameters
- retain videos/telemetry for all failures and worst-N episodes

Evaluation is cheap relative to training; exploit that asymmetry before risking hardware.

## 12. Hardware progression

Recommended progression:
1. supported quiet hold
2. tiny roll/pitch commands
3. tiny body translation/height changes
4. larger supported movement
5. controlled lower
6. rise
7. walking

This validates coordinate signs, joint mapping, IMU frames, latency, tracking, current prediction, and SafetyLayer behavior before intentionally putting the robot on the floor.

No autonomous experimental hardware deployment.

## 13. Every hardware session should improve the simulator

Do not record only success/failure.

Run comparable policy/command trajectories in simulation and hardware and quantify divergence in:
- roll/pitch
- q/dq
- currents
- gyro
- timing
- SafetyLayer intervention

Convert vague discrepancies into measurable ones, e.g. “R2 knee responds 80 ms later and draws 0.3 A more than simulation.” Feed those measurements back into calibration and DR.

## 14. Recommended next CoreWeave allocation

**Pod 1 — Flag-leg fix + gait-validity instrumentation**
Produce a clean six-leg slow gait and ensure evaluation automatically rejects morphological/gait exploits.

**Pod 2 — Learning-progress speed curriculum**
Implement command buckets and adaptive sampling based on recent learning progress rather than manual global widening.

**Pod 3 — Asymmetric actor–critic**
Train a policy whose actor uses only hardware-available observations while the critic retains privileged simulator information such as true body velocity.

**Pod 4 — Temporal deployable actor**
Compare a short frame stack with a modest GRU/LSTM using only hardware-realistic observations.

Move model-size experiments behind these four.

## 15. Standing best practices for the project

1. Actor inputs must be deployable; privileged state belongs in the critic, teacher, or auxiliary targets.
2. Prefer automatic curriculum around learning progress over manually advancing difficulty.
3. Route rewards by skill/mode instead of assuming every term should apply everywhere.
4. Treat temporal history as implicit system identification, not merely a way to estimate velocity.
5. A checkpoint is not a result until exact-path evaluation and visual inspection agree.
6. Evaluation should detect exploits independently of the reward whenever possible.
7. Freeze solved recovery champions so locomotion research cannot silently erase them.
8. Use genuine independent seeds and log seed/checksum provenance.
9. Use large frozen-policy evaluation before hardware promotion.
10. Turn every safe hardware run into sim-calibration data.

## Updated highest-priority technical recommendation

If choosing only two new mechanisms to implement next:

**1. Asymmetric actor–critic**
- hardware-observation actor
- privileged simulator critic

**2. Learning-progress command curriculum**
- bucket command speeds
- estimate recent progress per bucket
- allocate training to the moving frontier

Then add roughly 300 ms of observation/action history to the actor and determine whether the deployable policy can approach the privileged walking policy.

This combination directly addresses the two clearest current blockers:

```text
privileged velocity
        ↓
asymmetric actor–critic + temporal actor

manual widening regressions
        ↓
learning-progress curriculum
```

## Sources

- Rapid Locomotion / privileged learning: https://arxiv.org/abs/2405.10830
- Gait-conditioned humanoid locomotion: https://arxiv.org/abs/2505.20619
- Gait-conditioned humanoid locomotion full text: https://arxiv.org/html/2505.20619v2
- LP-ACRL automatic curriculum: https://arxiv.org/html/2601.17428v1
- Berkeley history-based humanoid locomotion: https://learning-humanoid-locomotion.github.io/
- HoST humanoid stand-up RL: https://arxiv.org/html/2502.08378v1
- NVIDIA Isaac Lab Spot sim-to-real locomotion: https://developer.nvidia.com/blog/closing-the-sim-to-real-gap-training-spot-quadruped-locomotion-with-nvidia-isaac-lab/
- Figure RL walking: https://www.figure.ai/news/reinforcement-learning-walking
- LoopSR: https://arxiv.org/abs/2409.17992

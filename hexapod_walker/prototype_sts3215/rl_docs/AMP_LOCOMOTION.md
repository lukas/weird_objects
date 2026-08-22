# Hexapod “Modern RL Robot” Locomotion Program

Operator brief, adopted 2026-08-21 as the `amp` track's charter
(source: `hexapod_amp_rl_cursor_brief.md`). Track status lives in
`rl_docs/tracks/amp/STATUS.md`.

## Repo adaptation (operator, 2026-08-21 — binding deltas to the brief)

The brief below is binding, with exactly these adaptations:

- **No Isaac Lab.** The primary trainer is the existing GPU-native
  MJX/Warp stack (`train_ppo_mjx`, 12x H200 pods, ~4096 envs each),
  extended as needed. It already has per-world model DR, canary
  probes, eval/video logging, and episode desync; the track builds the
  missing pieces on top (AMP discriminator + demo replay, GRU/history
  actor, asymmetric-critic obs split, joystick command generator,
  fault injection, terrain curriculum, push disturbances). Everything
  else in §9.1's capability list still applies — build it, don't wait
  for it.
- **GPU plan (§10):** map "8 GPUs" onto the 12 single-H200 train pods
  (one run per pod). Wave 1 uses 8 pods per the brief's layout; the
  remaining pods absorb seeds or the next wave.
- **MuJoCo cross-engine role (§9.2):** the validator is the plain
  CPU MuJoCo eval harness (`rl_move/sim/eval_checkpoint.py` path),
  which is a genuinely independent contact/integration path from MJX.
- **Milestone M6 (hardware)** is operator-owned: the track is DONE at
  M5 (MuJoCo transfer). No physical-robot action without an explicit
  operator ask.
- **Status output (§18):** write to `rl_docs/tracks/amp/STATUS.md`
  (this repo's track-doc path) instead of
  `rl_docs/tracks/amp_locomotion/STATUS.md`.
- **Do not pause on operator input.** Design questions get
  assume-and-go with a recorded assumption; only physical-robot access
  and spend approvals may wait.

## Directive to Cursor

Implement the locomotion program described below as the new primary research line for the 18-DOF hexapod.

Do **not** continue the existing pattern of broad reward sweeps, forward-only PPO specialists, slip-gate optimization, or stand/walk choreography work unless it directly supports this program.

The target is not merely a gait that avoids falling. The target is one learned policy that:

- accepts continuous joystick commands `(vx, vy, yaw_rate)`;
- produces a smooth, visually convincing alternating-tripod gait;
- starts, stops, reverses, strafes, and turns without phase resets;
- recovers from hard pushes;
- retains useful locomotion when a joint or entire leg is impaired;
- transfers from the primary simulator to MuJoCo and then to the AK40 hardware.

The user accepts hardware risk. Optimize for ambitious locomotion performance, not conservative hardware preservation.

---

## 1. Core Decision

The main line will be:

> **Adversarial Motion Priors (AMP) + massively parallel PPO + privileged critic + observation history + explicit actuator/fault randomization.**

The demonstration gait is training data, not the deployed controller.

The learned policy should own:

- gait timing;
- stance/swing transitions;
- command-conditioned stride shape;
- balance corrections;
- push recovery;
- adaptation to actuator variation;
- adaptation to damaged joints or legs.

Do not replace this with a scripted wave gait. A scripted or optimized gait may be used only to create the motion-prior dataset and to initialize trajectory optimization.

---

## 2. Why the Previous Program Stalled

The previous raw-joint PPO experiments asked the policy to discover all of the following from sparse task rewards at once:

- useful phase relationships between six legs;
- alternating-tripod coordination;
- swing-foot trajectories;
- stance behavior;
- command following;
- turning and lateral motion;
- self-collision avoidance;
- actuator delay compensation;
- contact behavior;
- visually smooth movement.

That search has repeatedly converged to paddle-creep, dragging, narrow specialists, fragile choreography, or reward exploits.

The new program supplies a strong prior for what plausible hexapod locomotion looks like while preserving RL’s ability to discover feedback, robustness, recovery, and command-conditioned variation.

---

## 3. Required Architecture

### 3.1 Policy command

The policy receives a planar body velocity command:

```text
command = [vx, vy, yaw_rate]
```

The command must vary continuously during episodes. Forward, turning, strafing, reversing, diagonal motion, and stopping must all be represented in the same policy.

### 3.2 Actor observations

Use only signals that can exist on the real robot:

- commanded `vx`, `vy`, and `yaw_rate`;
- 18 joint positions relative to a neutral pose;
- 18 joint velocities;
- IMU angular velocity;
- projected gravity or equivalent body orientation representation;
- previous 18-dimensional action;
- measured or estimated motor current/effort when available;
- optional 18-dimensional joint-health vector;
- a short temporal history or recurrent hidden state.

The actor must **not** receive privileged contact forces, true world velocity, terrain height maps, or simulator parameters.

### 3.3 History

Implement one of these first, in priority order:

1. GRU actor with 128–256 hidden units; or
2. temporal encoder over the most recent 5–10 observation/action frames.

The history mechanism should infer:

- actuator lag;
- motor weakness;
- a stuck or unresponsive joint;
- body velocity not directly observed by the actor;
- recent contact transitions.

Do not start with a large transformer or world model. The first successful policy should remain compact and easy to deploy.

### 3.4 Actor output

The actor outputs 18 joint-position offsets around a neutral configuration:

```text
action[i] = normalized joint position offset for joint i
```

Then apply:

- fixed action scaling per joint;
- joint limits;
- target rate limits matching the actuator;
- low-level impedance or position control beneath the learned policy.

Prefer a 50 Hz learned policy initially. Run local motor control and sensing faster where the hardware permits.

### 3.5 Asymmetric critic

The critic may receive privileged simulation information:

- true body linear and angular velocity;
- contact state and contact force for every foot;
- terrain samples;
- friction coefficients;
- body mass and center-of-mass offset;
- actuator strength;
- latency and command delay;
- payload;
- joint damage state;
- applied external force;
- true foot positions and velocities.

The critic should improve sample efficiency without changing the actor’s deployable observation set.

### 3.6 AMP discriminator

Train a discriminator to distinguish motion-library transitions from policy-generated transitions.

Start with discriminator features such as:

- joint positions relative to neutral;
- joint velocities;
- base angular velocity;
- projected gravity;
- foot positions relative to the body, if derivable consistently;
- transitions across adjacent timesteps rather than isolated poses.

Do not include joystick command in the discriminator initially. The motion prior should answer “does this look like coherent hexapod locomotion?” while the task reward answers “does this follow the requested command?”

Normalize all discriminator inputs from the demonstration dataset. Add a gradient penalty and discriminator regularization so the style reward does not saturate immediately.

---

## 4. Motion-Prior Dataset

### 4.1 Goal

Create a small, excellent library of alternating-tripod locomotion for this exact morphology.

It does not need to be robust. It does not need to be deployable. It needs to demonstrate clean state transitions that look like the desired gait.

### 4.2 Required motion coverage

Include at least:

- forward at several speeds;
- backward at several speeds;
- left and right lateral motion;
- clockwise and counterclockwise rotation;
- forward while turning;
- diagonal translation;
- acceleration from rest;
- deceleration to rest.

Target 20–60 seconds of diverse, clean transitions after augmentation. A shorter base library is acceptable if symmetry, mirroring, speed scaling, and phase augmentation expand it cleanly.

### 4.3 How to produce it

Use one or more of:

- trajectory optimization/direct collocation;
- a clean hand-designed alternating-tripod kinematic generator;
- the current cycle-44 gait as an initialization only;
- optimization over foot trajectories and body motion;
- offline smoothing of a usable but imperfect gait.

The motion source may use privileged state because it is not the deployed controller.

### 4.4 Demonstration quality objectives

Optimize for:

- alternating-tripod phase structure;
- low stance-foot velocity;
- smooth body motion;
- smooth joint velocity and acceleration;
- adequate foot clearance;
- no self-collision;
- no repeated joint-limit contact;
- moderate torque and action rate;
- periodicity for constant commands;
- clean command transitions.

Reject clips containing obvious dragging, joint discontinuities, body collapse, leg tangling, or numerical contact explosions.

### 4.5 Dataset format

Store:

```text
obs_style_t
obs_style_t_plus_1
command_t
joint_position_t
joint_velocity_t
base_orientation_t
base_angular_velocity_t
foot_positions_t
phase_or_tripod_label_optional
metadata: speed, direction, turn rate, source, quality score
```

The discriminator training path should not require every optional field.

---

## 5. Reward Structure

Use a small number of interpretable reward families. Avoid another large reward-term search.

### 5.1 Task reward

Primary terms:

```text
r_linear_velocity = exp(-||v_xy - v_xy_cmd||^2 / sigma_v)
r_yaw_velocity    = exp(-(wz - wz_cmd)^2 / sigma_w)
r_upright         = function(projected_gravity)
r_height          = weak body-height regularizer
```

The actor may not observe true body velocity even though it is used for reward and critic training.

### 5.2 AMP style reward

Use a standard discriminator-derived style reward. Keep it strong enough that the policy cannot ignore it, but not so strong that it refuses necessary recovery behavior.

A reasonable first sweep is only three AMP/task mixtures:

```text
A: task 0.70 / style 0.30
B: task 0.50 / style 0.50
C: task 0.30 / style 0.70
```

Keep other terms fixed for this comparison.

### 5.3 Physical regularization

Keep these penalties modest:

- action rate;
- action acceleration;
- joint acceleration;
- torque or estimated electrical effort;
- destructive body/leg collision;
- excessive contact impulse;
- joint-limit contact;
- vertical body jitter;
- foot tangential velocity during confident stance.

Do not make stance slip the dominant reward or a hard early gate.

The policy may use small real slips while accelerating, turning, or recovering. Reject paddle-creep visually and through contact-sequence metrics rather than forcing every stance foot to be perfectly immobile.

### 5.4 Termination

Terminate for:

- unrecoverable body orientation;
- body collapse below a permissive threshold;
- numerical instability;
- extreme self-collision;
- prolonged inverted state.

Do not terminate for every brief tilt or stumble. Recovery must be learnable.

---

## 6. Joystick Training Distribution

Joystick behavior must exist from the first serious training run.

Within each episode:

- begin from both rest and moving states;
- sample commands continuously, not one fixed command per episode;
- hold some commands for 0.5–3.0 seconds;
- ramp some commands smoothly;
- change some commands abruptly;
- include sign reversals;
- include zero-command intervals;
- include mixed translation and yaw;
- include diagonal and lateral commands.

Initial command envelope:

```text
vx       in [-0.35, 0.60] m/s
vy       in [-0.30, 0.30] m/s
yaw_rate in [-1.0, 1.0] rad/s
```

Curriculum may begin at 30–50% of this envelope and expand automatically when tracking and stability thresholds are met.

Required scripted evaluation command sequence:

```text
0.0 s   stop
2.0 s   forward
5.0 s   forward + clockwise yaw
8.0 s   left strafe
11.0 s  reverse + counterclockwise yaw
14.0 s  stop
17.0 s  fast forward
21.0 s  hard right turn
24.0 s  stop
```

The policy must not reset gait phase or recurrent state when the command changes.

---

## 7. Domain Randomization

Randomize from the beginning, but use a curriculum rather than maximal chaos on step one.

### 7.1 Actuator randomization

Randomize:

- motor strength;
- joint damping;
- friction/backlash approximation;
- target tracking gain;
- torque/current saturation;
- control latency;
- observation latency;
- dropped commands;
- encoder noise;
- per-joint calibration offsets;
- bus update jitter.

Use measured STS3215 properties for software tests and measured AK40 properties for the deployment policy.

### 7.2 Robot randomization

Randomize:

- total mass;
- individual link mass;
- center of mass;
- payload position;
- leg length by small percentages;
- body inertia;
- foot radius/compliance;
- neutral-pose calibration.

### 7.3 Contact and terrain randomization

Randomize:

- foot-ground friction;
- contact softness/stiffness;
- restitution;
- flat ground;
- mild heightfields;
- rough ground;
- small steps;
- ramps;
- low-friction patches;
- isolated obstacles.

Do not spend weeks trying to identify one perfect MuJoCo contact model. The policy should survive a distribution.

### 7.4 Disturbances

Apply randomized pushes:

- lateral;
- fore/aft;
- yaw torque;
- short impulses;
- sustained force pulses;
- pushes during command transitions;
- pushes during swing and stance.

Curriculum should expand push magnitude once the policy has stable normal locomotion.

---

## 8. Fault-Tolerance Curriculum

Do not mix severe failures into the first hours of learning. First obtain a beautiful normal gait, then continue training with faults while retaining a majority of normal episodes.

Suggested episode mix after normal gait convergence:

```text
60% normal robot
15% weakened joint(s)
10% locked or frozen joint
10% one disabled leg
5% sensor/communication fault
```

Randomize faults including:

- one motor at 70%, 40%, 20%, or 0% strength;
- one joint frozen at its current position;
- one joint frozen at a random safe position;
- one complete leg disabled;
- intermittent command dropout;
- delayed position feedback;
- biased encoder;
- one low-friction foot;
- one shortened/bent-leg approximation;
- sudden torque sag;
- shifted payload after episode start.

### 8.1 Style reward under faults

A normal alternating-tripod style prior can punish correct asymmetric recovery. Use one of these approaches:

1. reduce AMP weight during known fault episodes;
2. condition the discriminator on the fault/health vector;
3. train a second recovery discriminator from successful fault rollouts;
4. allow a fault-specific adaptation phase with task reward dominant.

Begin with option 1 because it is simplest.

### 8.2 Health vector

If hardware diagnostics can identify an unhealthy joint, pass an 18-dimensional health vector to the actor:

```text
1.0 = healthy
0.0 = disabled
intermediate = degraded confidence/strength
```

Still retain history so the policy can infer unannounced failures.

---

## 9. Simulator Strategy

### 9.1 Primary trainer

Use Isaac Lab as the primary massively parallel training environment unless the existing repo already has an equally fast GPU-native path that can support AMP cleanly.

Required capabilities:

- thousands of environments per GPU;
- PPO with asymmetric actor/critic inputs;
- recurrent or history-based actor;
- AMP discriminator and demonstration replay buffer;
- domain-randomization callbacks;
- terrain curriculum;
- external pushes;
- per-joint fault injection;
- video capture and deterministic evaluation.

### 9.2 MuJoCo role

Keep MuJoCo as an independent validator:

- import the trained actor;
- reproduce the same deployable observations;
- replay the joystick evaluation script;
- replay actuator delays and force limits;
- compare gait style and command tracking;
- run fault cases;
- detect simulator-specific exploits.

Do not tune on MuJoCo until Isaac behavior is good. Do not require perfect metric agreement across engines. Require preservation of the qualitative gait, stability, and command response.

### 9.3 Cross-engine gate

A policy passes cross-engine validation when it:

- remains upright for the full evaluation;
- follows all command directions;
- retains alternating-tripod structure during ordinary motion;
- does not collapse into continuous foot dragging;
- recovers from moderate pushes;
- shows useful adaptation to at least one disabled-joint case.

---

## 10. GPU Utilization Plan

Use the 8×H200 node for parallel populations and environment count, not for a giant policy.

Start with compact networks and target as many stable environments as the simulator supports. An initial target is 4,096–16,384 environments per GPU, adjusted based on memory use and simulation throughput.

Run a controlled population rather than unrelated experiments.

### Wave 1: architecture validation

```text
GPU 0–2: three random seeds, AMP/task mix B
GPU 3:   no-AMP ablation
GPU 4:   recurrent actor
GPU 5:   fixed-history actor
GPU 6:   higher AMP weight
GPU 7:   lower AMP weight
```

### Wave 2: robustness

Clone the best architecture across all GPUs and vary only:

- domain-randomization scale;
- actuator latency curriculum;
- push curriculum;
- action scale;
- impedance settings.

### Wave 3: faults

Clone the best normal policy and vary only:

- fault frequency;
- AMP reduction during faults;
- explicit health vector versus history-only;
- disabled-joint versus disabled-leg curriculum.

### Experiment discipline

Every run must log:

- exact code revision;
- full config;
- random seed;
- simulator version;
- environment count;
- samples and wall-clock throughput;
- reward components;
- discriminator accuracy/loss;
- command-tracking errors;
- gait/contact metrics;
- push recovery;
- fault recovery;
- evaluation videos.

Stop broad exploratory sweeps. Change one or two meaningful dimensions per wave.

---

## 11. Hardware Track

The STS3215 chassis remains useful for:

- communications testing;
- observation plumbing;
- deployment code;
- joystick interface;
- validating recurrent-state handling;
- low-speed policy smoke tests.

The performance target should use the AK40-based chassis or another actuator setup with:

- low backlash;
- fast target response;
- torque or impedance control;
- high-rate position/velocity/current sensing;
- rigid metal load paths at horns and brackets;
- adequate joint-range margin around the neutral stance;
- low distal leg mass;
- compliant or rubberized feet;
- measured latency and torque limits.

Do not assume the policy can fully compensate for 150–200 ms of actuator delay while retaining high-frequency, lifelike movement. Train with measured delay, but improve the hardware bandwidth rather than treating delay as a pure learning problem.

### Required system identification

For each actuator type, measure:

- command-to-motion delay;
- step response;
- maximum speed under representative load;
- torque/current saturation;
- steady-state position error;
- damping/backdrive behavior;
- bus update rate with all 18 joints;
- feedback rate and jitter.

Feed these measurements into the randomization ranges.

---

## 12. Evaluation Metrics

### 12.1 Command tracking

Report separately:

- `vx` RMSE;
- `vy` RMSE;
- `yaw_rate` RMSE;
- response delay after command changes;
- overshoot;
- stop distance/time.

### 12.2 Gait quality

Report:

- leg phase relationships;
- percentage of ordinary motion matching alternating-tripod contacts;
- stance and swing duration distributions;
- foot clearance distribution;
- body roll/pitch RMS;
- body vertical acceleration RMS;
- joint acceleration/action-rate statistics;
- foot tangential velocity while loaded;
- self-collision count;
- visual evaluation video.

Do not reduce visual quality to one slip number.

### 12.3 Robustness

Report:

- maximum recoverable push impulse by direction;
- fall rate over randomized pushes;
- terrain completion rate;
- low-friction-patch completion rate;
- performance under actuator latency extremes;
- performance under mass and center-of-mass variation.

### 12.4 Fault tolerance

For each joint and each full leg:

- success rate after disablement;
- distance traveled after fault;
- command-tracking degradation;
- time to recover a stable pattern;
- fall probability;
- whether the policy develops a coherent asymmetric gait.

---

## 13. Hard Milestones

### Milestone M0 — Infrastructure

- primary simulator runs at scale;
- actor/critic observation split works;
- joystick command generator works;
- recurrent state resets correctly;
- fault injection works;
- evaluation videos and metrics are automatic.

### Milestone M1 — Motion library

- clean alternating-tripod clips exist for all command families;
- clips contain no obvious dragging or collision;
- dataset loader and augmentation work;
- discriminator can distinguish demonstrations from random/bad motion without instant saturation.

### Milestone M2 — Beautiful normal gait

One policy must:

- walk forward, backward, laterally, and diagonally;
- turn both ways;
- smoothly follow changing joystick commands;
- start and stop without collapse;
- show unmistakable alternating-tripod structure;
- look materially smoother than the current champion;
- reject paddle-creep as its primary locomotion mode.

Do not advance because of reward alone. Save evaluation videos and require human visual inspection.

### Milestone M3 — Push recovery

The M2 policy, after robustness continuation, must:

- survive repeated randomized lateral and fore/aft pushes;
- recover gait without episode reset;
- preserve useful command tracking;
- avoid becoming permanently crouched or stationary.

### Milestone M4 — Fault adaptation

A single policy must continue useful joystick-controlled movement when:

- one random joint is weakened;
- one random joint is frozen;
- one random leg is disabled.

Normal gait quality must remain close to M2 when no fault is present.

### Milestone M5 — MuJoCo transfer

The same actor weights must replay in MuJoCo with no retraining and preserve:

- command response;
- recognizable gait;
- push recovery;
- at least partial fault adaptation.

Small observation normalization fixes are allowed; policy retraining is not.

### Milestone M6 — Hardware deployment

On the AK40 chassis:

- deploy at low command scale first;
- verify observation ordering and normalization;
- verify recurrent-state lifecycle;
- verify action signs and limits;
- expand command envelope aggressively after the first stable run;
- record every test for sim-to-real comparison.

Hardware damage risk is accepted, but software limits must still prevent obviously invalid commands and uncontrolled numerical output.

---

## 14. What to Stop Doing

Stop spending primary research time on:

- raw PPO from random initialization with only task rewards;
- slip-reward coefficient sweeps;
- separate forward, steering, and reverse specialists;
- stand/rise/walk multitask training as a prerequisite to locomotion;
- distilling mediocre gait teachers;
- adding a larger dynamics model before the AMP baseline works;
- treating a perfect MuJoCo contact model as a blocking dependency;
- using giant networks in place of good demonstrations and training distributions;
- rejecting a policy solely because it exhibits a small amount of physically plausible slip;
- accepting a policy solely because it passes a scalar reward gate.

Dynrep can be revisited later as an adaptation or representation-learning enhancement. It is not on the critical path to the first excellent joystick gait.

---

## 15. Recommended Repository Structure

Adapt names to the existing repository rather than duplicating infrastructure.

```text
rl_move/
  amp/
    config.py
    dataset.py
    discriminator.py
    rewards.py
    replay_buffer.py
    augment.py
  envs/
    hexapod_amp_env.py
    observations.py
    commands.py
    randomization.py
    faults.py
    terrains.py
  policies/
    recurrent_actor_critic.py
    history_actor_critic.py
  motion/
    generate_motion_library.py
    optimize_tripod_motion.py
    validate_motion_library.py
    datasets/
  train/
    train_amp_ppo.py
    curriculum.py
    launcher.py
  eval/
    joystick_script.py
    gait_metrics.py
    push_eval.py
    fault_eval.py
    cross_engine_mujoco.py
    video.py
  deploy/
    export_policy.py
    runtime.py
    joystick.py
    observation_adapter.py
    actuator_adapter.py
rl_docs/
  AMP_LOCOMOTION.md
  tracks/amp_locomotion/STATUS.md
```

---

## 16. Minimum Configuration Skeleton

```yaml
experiment:
  name: amp_hexapod_joystick
  seed: 0

sim:
  backend: isaac_lab
  control_hz: 50
  physics_hz: 500
  num_envs: 8192
  episode_seconds: 20

command:
  vx: [-0.35, 0.60]
  vy: [-0.30, 0.30]
  yaw_rate: [-1.0, 1.0]
  resample_seconds: [0.5, 3.0]
  zero_command_probability: 0.15
  abrupt_change_probability: 0.35

actor:
  type: gru
  hidden_sizes: [256, 128]
  recurrent_hidden_size: 256
  action_dim: 18
  action_mode: joint_position_offset
  action_scale_deg:
    yaw: 20
    hip: 25
    knee: 30

critic:
  hidden_sizes: [512, 256, 128]
  privileged_observations: true

ppo:
  rollout_steps: 24
  epochs: 5
  minibatches: 4
  gamma: 0.99
  gae_lambda: 0.95
  clip: 0.2
  entropy_coef: 0.01
  learning_rate: 0.0003

amp:
  enabled: true
  task_weight: 0.5
  style_weight: 0.5
  discriminator_hidden_sizes: [256, 256]
  gradient_penalty: 10.0
  replay_size: 1000000

rewards:
  linear_velocity: 1.0
  yaw_velocity: 0.5
  upright: 0.25
  body_height: 0.05
  action_rate: -0.01
  action_acceleration: -0.005
  joint_acceleration: -0.001
  torque: -0.0001
  stance_foot_velocity: -0.02
  destructive_collision: -1.0
  joint_limit: -0.2

randomization:
  enabled: true
  curriculum: true
  friction: [0.5, 1.5]
  motor_strength: [0.8, 1.2]
  control_latency_ms: [0, 80]
  observation_latency_ms: [0, 40]
  mass_scale: [0.9, 1.1]
  com_offset_m: [-0.015, 0.015]

faults:
  enabled: false
  # Enable only after M2 normal-gait checkpoint exists.
```

These values are initial conditions, not sacred constants. Change them only through controlled, logged experiment waves.

---

## 17. First Implementation Sequence

Execute in this order:

1. Inspect the existing repo and reuse its robot model, observation normalization, actuator model, logger, evaluator, and checkpoint format where possible.
2. Add one unified joystick-command environment with the actor/critic observation split.
3. Add GRU actor support and deterministic recurrent evaluation.
4. Add motion-library generation and validation.
5. Add AMP discriminator, demonstration replay, and style reward.
6. Run a tiny smoke test proving gradients flow through PPO and the discriminator trains.
7. Run M1 library validation.
8. Run Wave 1 across the 8 GPUs.
9. Select by videos plus tracking/stability metrics, not scalar return alone.
10. Continue the best checkpoint with push and domain-randomization curricula.
11. Continue the resulting checkpoint with the fault curriculum.
12. Export the actor and run the fixed MuJoCo cross-engine suite.
13. Integrate the deployment runtime with the AK40 control layer.

Do not branch into additional research tracks until M2 either succeeds or fails with a concrete, instrumented diagnosis.

---

## 18. Required Status Output After Each Wave

Update `rl_docs/tracks/amp_locomotion/STATUS.md` with:

```text
Current milestone:
Best checkpoint:
Code revision:
Training samples:
Training wall time:
Primary simulator FPS:
Normal gait verdict:
Joystick tracking verdict:
Visual gait verdict:
Push recovery verdict:
Fault recovery verdict:
MuJoCo transfer verdict:
Top 3 failures:
Exact next experiments:
Operator input required:
```

Include direct paths to:

- best checkpoint;
- config;
- W&B run;
- normal joystick video;
- push video;
- disabled-leg video;
- MuJoCo replay video.

---

## 19. Final Success Definition

The program succeeds when there is one compact deployed policy that can be driven with a joystick and visibly behaves like a modern learned legged robot:

- smooth command-conditioned locomotion;
- natural alternating-tripod movement during normal operation;
- responsive turning and strafing;
- stable starts and stops;
- push recovery;
- graceful degradation under damaged joints or a disabled leg;
- no choreography or phase-reset machinery required for ordinary driving.

That is the objective. Do not optimize the project back toward “a careful hexapod that technically moves.”

---

## 20. Useful References

- Real-hexapod AMP-style locomotion example: https://arxiv.org/html/2511.03167v1
- Impaired-joint adaptation example: https://arxiv.org/html/2403.00398v1
- Isaac Lab: https://developer.nvidia.com/isaac/lab

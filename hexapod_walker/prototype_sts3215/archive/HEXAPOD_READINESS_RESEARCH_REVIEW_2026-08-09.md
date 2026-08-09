# External Review — How close are we to real joystick walking and four-leg mode?
**Date:** 2026-08-09  
**Audience:** autonomous coding/training LLM  
**Purpose:** Independent strategic review of the current hexapod RL plan, grounded in published sim-to-real legged-locomotion work and the campaign evidence supplied by the operator.

## Executive answer

### Six-leg joystick walking on the real robot

There are two different milestones hidden inside “joystick controlled walking”:

1. **Forward joystick demo** — push stick forward, robot walks forward; release, it stops.
2. **Real velocity joystick** — proportional forward/back/sideways/yaw control over a meaningful command envelope.

The project is **fairly close to milestone 1 in software, but not yet cleared for hardware**. The hard part is no longer discovering a six-leg gait: that exists, sustains for 15–30+ s in sim, and survives substantial DR. The remaining blocker is whether the simulator is teaching a physically wrong foot-ground economy. The policy still prefers loaded sliding/paddling, and repeated reward-side attempts could not dislodge it.

Engineering estimate from the evidence:

- **First supported forward-walk hardware experiment:** plausibly **1–3 focused engineering days** after the current/contact calibration is resolved and a frozen policy passes the hardware gate. A bad sim-to-real discrepancy could extend this substantially.
- **Repeatable forward-only joystick demo:** plausibly **several days to ~2 weeks after first hardware contact**, assuming the gait transfers without a major contact-model failure.
- **Full joystick locomotion (forward/back/lateral/yaw):** **not close in the same sense**. Backward competence is currently zero and forward-diagonal steering has already failed once. Treat this as a later phase, likely **weeks rather than days**.

The dominant uncertainty is no longer PPO compute. It is **sim-to-real fidelity and hardware safety**.

### Four-leg standing/walking with the front pair free

Split this into three milestones:

1. **Static four-leg stance with front legs lifted**
2. **Weight shifting / small joystick body motion on four legs**
3. **Four-leg walking while keeping both front legs free**

My estimate:

- **Four-leg static stance in sim:** possibly **hours to a few days** once implemented, provided a scripted feasibility check confirms the CoM can sit safely inside the remaining four-foot support polygon.
- **Four-leg stance on real hardware:** should wait until ordinary six-leg sim-to-real has been validated; after that, perhaps **days** if torque margins are comfortable.
- **Four-leg walking in sim:** likely **days to a couple of weeks** of focused work, with substantial uncertainty.
- **Four-leg walking on hardware:** later than six-leg joystick walking and should not be on the critical path before ordinary walking transfers.

The important caveat: do **not** assume “six legs minus two” is an easy policy edit. Raising the front pair changes support geometry, joint loading, body pitch torque, and current distribution.

---

## 1. What the campaign has actually achieved

The key accomplishment is not the old 6-D IK policy. The project now has an **18-joint position-target policy** producing a real six-leg stepping gait. The `longdist-r2` result is particularly important:

- sustained walking for the full 30 s evaluation
- all six legs cycling
- ~1.57 m deterministic travel
- progress ratio ~0.98 rather than the old champion’s ~1.43 overspeed
- lower deterministic slip than the formal champion
- substantial competence retained at DR 1.0

That is enough evidence to say **locomotion has been learned in simulation**. The project is not waiting for a gait to appear.

This is consistent with successful real-world legged-RL systems in the literature: policies often output joint-position targets rather than raw torques, and simulation-trained policies can track high-level velocity commands on real robots when actuator dynamics, state estimation and randomization/adaptation are handled carefully.

Relevant primary work:

- Hwangbo et al., *Learning agile and dynamic motor skills for legged robots* — simulation-trained neural policies transferred to ANYmal and followed high-level body velocity commands.  
  https://arxiv.org/abs/1901.08652
- Margolis et al., *Rapid Locomotion via Reinforcement Learning* — simulation-trained Mini Cheetah policy tracks velocity commands in the real world; adaptive command curricula and online system identification are central.  
  https://arxiv.org/abs/2205.02824
- Kumar et al., *RMA: Rapid Motor Adaptation for Legged Robots* — real-world locomotion uses online adaptation to infer persistent environment/robot variation from recent history.  
  https://arxiv.org/abs/2107.04034
- DreamWaQ — proprioceptive learned estimation/adaptation validated on real quadruped locomotion.  
  https://arxiv.org/abs/2301.10602

So the architecture direction is credible. The present risk is that **this simulator is rewarding a gait these servos/feet will dislike in reality**.

---

## 2. Critical path to a forward joystick demo

### Gate A — resolve the contact/current economics

This is now the highest-value unresolved technical issue.

The campaign has strong evidence that paddling is not merely a reward-local optimum:

- it reappears from fresh initialization
- effort/current reward changes did not eliminate it
- timing/phase changes did not eliminate it
- anchor gating helped once, then the policy routed around it
- explicit episode-level loaded-slip gating caused large income loss and the policy still slid

That pattern makes another anti-slip reward coefficient low-value.

What is not yet proven is which physical model component is wrong. “Contact/current pricing” is an umbrella; the root could be one or more of:

- tangential friction
- contact compliance / solver behavior
- foot collision geometry/material
- servo position stiffness
- latency/profile interaction under contact
- torque-speed behavior
- current conversion
- mass/CoM/load distribution

The STS3215 manufacturer lists 30 kg·cm stall torque at 12 V and provides current feedback/protection. A ~2.6 A condition should be treated as stall-scale, not benign operating current.  
https://www.feetechrc.com/525603.html

### Required operator calibration

Run a **safe, low-amplitude hardware comparison** of:

1. planted/supporting foot under slow body/joint motion
2. deliberately small tangential foot drag under controlled light and moderate load

Record per-servo current, joint position error, joint velocity, body IMU, bus voltage, temperature, commanded target, and duration.

Do not intentionally stall the motor. The goal is the **relative physical cost of planted support versus loaded sliding**.

Then reproduce the same conditions in MuJoCo/MJX.

If real dragging is dramatically more expensive or mechanically stalls while sim dragging is easy, fix the simulator before further gait optimization.

### Gate B — freeze a physical-quality forward policy

Do not require omnidirectional competence for the first candidate.

Define the first command interface simply:

- neutral/stop
- forward, preferably a narrow command band

The joystick mapping is straightforward:

```text
joystick Y
  -> normalized forward command
  -> policy goal observation
  -> 18 joint targets
  -> SafetyLayer
  -> STS3215 targets
```

A first real demo does not need precise speed tracking. It needs:

- neutral -> stable hold/stop
- forward -> sustained stepping
- release -> controlled stop
- no fall
- no hot servo
- little loaded scrubbing
- low SafetyLayer intervention

The decision to defer the rear hemisphere is correct. Margolis et al. used an adaptive velocity-command curriculum; broad command competence does not need to be prerequisite to a narrow first transfer.

### Gate C — supported hardware rollout

Recommended ladder:

1. torque-disabled/offline command trace inspection
2. unloaded/supported leg motion
3. supported quiet hold with policy active
4. tiny policy-driven joint/posture changes
5. supported stepping with meaningful foot contact
6. very short free forward walk
7. longer forward joystick demo

Safety should be treated as a distinct layer. Yang et al., *Safe Reinforcement Learning for Legged Locomotion*, demonstrated real quadruped deployment with a safety/recovery layer reducing falls.  
https://arxiv.org/abs/2203.02638

---

## 3. How close is “joystick” specifically?

### Forward joystick: close in software

Do **not** create a separate joystick RL project. The policy already consumes command information. Feed joystick input into the same goal channel.

### Full 2-D joystick: later

Eventually:

```text
left stick Y -> vx
left stick X -> vy
right stick X -> yaw rate
```

But the project does not yet have evidence across this envelope:

- backward/rear hemisphere: zero competence
- forward diagonal: attempted and failed
- yaw: not yet trained

Call the first milestone **forward joystick walking**.

After forward hardware transfer:

1. forward speed band
2. forward diagonals
3. lateral
4. backward/rear diagonals
5. yaw
6. combined commands

---

## 4. Four-leg mode: feasibility assessment

### First question is geometry, not RL

Before PPO, freeze the two front legs in the desired raised/manipulator configuration and sweep:

- body x/y shift
- body pitch
- body height
- front-leg raised configurations

Measure:

- projected CoM relative to the four-foot support polygon
- static stability margin
- joint torque/current
- individual supporting-leg load
- joint-limit margin
- tolerance to a few degrees of perturbation

If the CoM cannot be brought comfortably inside the support quadrilateral without saturating middle/rear joints, PPO will not rescue the morphology.

### Static four-leg stance should be its own skill

Task:

> front pair unloaded/raised; remaining four feet support; body level; low current; hold 10–15 s.

Success:

- both front feet clear and unloaded
- four support feet planted
- body attitude within tolerance
- adequate support margin
- no hot motor
- zero termination

Do not start by asking the existing six-leg walking policy to spontaneously stop using two legs.

Fault-tolerant locomotion research supports the premise that learned controllers can adapt to reduced limb availability when those conditions are represented during training:
- *Saving the Limping: Fault-tolerant Quadruped Locomotion via Reinforcement Learning*  
  https://arxiv.org/abs/2210.00474
- hexapod gait reconfiguration after damage has also been demonstrated algorithmically  
  https://arxiv.org/abs/2506.19968

These are not identical to “front legs become arms,” but they support the feasibility of reduced-support locomotion.

### Four-leg walking is a separate locomotion mode

Sequence:

1. four-leg static hold
2. small body shifts
3. one-step forward task
4. repeated four-leg forward walk
5. command-conditioned walk
6. front-leg manipulation objectives

Do not make simultaneous manipulation part of the first locomotion training. “Front legs remain free” is enough.

Recent hexapod whole-body loco-manipulation work explicitly exploits subsets of limbs for manipulation while other limbs provide support/propulsion, so the long-term concept is credible.  
https://arxiv.org/html/2509.23651v3

---

## 5. Why four-leg stance may be easier than the current lower-posture problem

The current six-leg `lower` pathology is partly reward/economy ambiguity: hovering a leg became profitable.

Four-leg stance has a cleaner target:

- exactly two named legs airborne
- exactly four named legs supporting
- body level
- current sane

That makes the task easier to specify and evaluate.

Therefore **do not wait for perfect six-leg lower posture before beginning four-leg simulation**. It is reasonable as a parallel GPU line.

But four-leg **hardware** deployment should wait until six-leg raw-joint sim-to-real has been demonstrated. First hardware contact should use the most forgiving six-foot morphology.

---

## 6. Recommended priority

### P0 — unlock real six-leg walking

1. Complete current/contact calibration.
2. Fix MJX/MuJoCo if real-vs-sim loaded slip/current differs materially.
3. Retrain/fine-tune forward gait under corrected physics.
4. Select frozen policy by physical metrics:
   - distance
   - zero falls
   - body attitude
   - loaded slip/m
   - per-servo current/thermal proxy
   - low SafetyLayer intervention
5. Begin supported real-world forward walking.

Do **not** block this on turning, backward locomotion, fall recovery, or four-leg walking.

### P1 — four-leg simulation in parallel

Use spare GPU capacity for:

1. scripted four-foot support feasibility sweep
2. static four-leg stance
3. four-leg weight shifting
4. short forward quadruped stepping

### P2 — expand real joystick envelope

Only after forward transfer:

1. forward speed variation
2. diagonals
3. lateral
4. yaw
5. rear hemisphere

### P3 — four-leg hardware + claws

After six-leg transfer is understood, deploy supported four-leg hold, then free stance, then walking.

---

## 7. Readiness score

These percentages are engineering judgments, not statistical probabilities.

| Capability | Readiness | Main missing piece |
|---|---:|---|
| Six-leg gait exists in sim | **90–95%** | already demonstrated |
| Clean forward gait in sim | **70–80%** | loaded-slip/contact economics |
| First supported real forward steps | **55–70%** | calibration + first transfer |
| Repeatable real forward joystick demo | **40–60%** | unknown sim-to-real gap |
| Full directional joystick walking | **15–30%** | lateral/back/yaw not learned |
| Four-leg static stance in sim | **30–50%** | not yet implemented/tested |
| Four-leg static stance on hardware | **15–30%** | establish six-leg transfer first |
| Four-leg walking in sim | **15–30%** | new gait/task |
| Four-leg walking on hardware | **5–15%** | reduced-support learning + transfer |

The lower four-leg percentages reflect **lack of attempts**, not disbelief in feasibility. With MJX throughput they could move quickly.

---

## 8. Strategic instructions to the autonomous LLM

### Do not confuse throughput with the critical path

4096 MJX environments are useful, but the next major uncertainty is physical calibration. Hundreds of millions of samples in a simulator with the wrong foot-ground economics can make the wrong gait more confidently optimal.

### Stop inventing anti-slip reward terms until calibration changes the premise

The campaign has tested the reward side extensively. New anti-slip coefficient arms are low-value unless they change the physics or task representation.

### Keep evaluator metrics independent of reward

Loaded slip/m, contact pattern, per-servo current, stability and distance remain external promotion metrics.

### Treat `longdist` as a research leader even if formal champion promotion is pending

Its reported multi-metric physical behavior is more interesting than the formal anchorgate champion. Replicate it, but do not let champion bookkeeping keep the campaign anchored to a mechanically worse policy.

### Four-leg mode starts with feasibility, not reward design

No RL reward work until support polygon, current margin and joint-limit feasibility are checked.

---

## Bottom line

**Forward joystick walking on the real robot is now a near-term engineering goal, not a speculative research goal.** A real gait exists in simulation. The project is approximately one simulator-calibration/transfer phase away from discovering whether it survives reality.

It is premature to call the robot close to unrestricted joystick locomotion: backward, lateral and yaw competence are not established, and the raw-joint policy has never yet driven the physical robot.

**Four-leg standing is a plausible near-term parallel simulation project.** Four-leg walking is credible but is a new locomotion mode and should not delay six-leg hardware transfer.

Fastest path to something visibly impressive:

> **clean forward six-leg gait -> supported real forward joystick walking -> four-leg static stance -> four-leg walking -> front-leg manipulation**

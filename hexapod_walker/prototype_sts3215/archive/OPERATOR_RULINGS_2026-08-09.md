# Hexapod RL Design Rulings — researched external review
**Date:** 2026-08-09  
**Purpose:** Operator rulings for the autonomous RL campaign. These replace the tentative answers on stance clearance/current economics, rear-hemisphere command scope, and the walk distance gate.

## Executive rulings

1. **Stance clearance:** Do **not** keep a fixed 60 mm free-hover allowance. As an immediate patch, shrink it to roughly **25–30 mm**, but the better fix is to stop using one global clearance allowance at all: evaluate each foot against its **episode-specific desired end clearance/contact state**. Price deviation around that reference on both sides. Treat ground penetration/contact compression separately rather than making negative clearance “free.”
2. **Stance current:** **2.6 A is physically possible, but it is essentially stall current for an STS3215 and is not a normal/safe steady-state operating point.** A slow maneuver can still draw near-stall current if the servo is producing near-stall torque, so “slow” does not prove the sim wrong. However, repeated ~2.6 A peaks during an ordinary six-foot planted descent are suspicious enough that the exact maneuver/current model must be calibrated on hardware before reward design is changed around it.
3. **Support-margin reward:** A lower/rest skill should never pay a tripod-hover solution more than the intended six-foot end posture. Audit the term. For lower mode, make support-margin a constraint/backstop or replace it with end-state contact/posture objectives; do not let it be positive income that competes with the requested final posture.
4. **Rear hemisphere:** Choose **2b: explicitly defer it.** Current walking scope is forward/forward-diagonal only. Remove backward/rear-lateral draws from promotion gates until forward locomotion is physically sane and reliable. Add the rear hemisphere later via a deliberate command curriculum.
5. **Distance gate:** Choose **3a over 3b**, but the framing should be simplified further. A fixed 0.40 m gate across variable speed commands is mathematically inconsistent and selects overspeed. If variable speed commands remain, gate displacement relative to commanded displacement, with a loose overspeed ceiling. Since the operator’s actual objective is distance/stability/reliability rather than speed tracking, the cleaner current-phase experiment is a **single/narrow forward walk command** and a direct distance gate; add broad speed conditioning later.
6. **Evaluation panels:** Keep one fixed panel as a regression canary, but **promotion must use multiple independently seeded command panels** plus explicit corner cases.

---

# 1. Stance line

## 1a. The 60 mm free-hover allowance

### Ruling

**Retire 60 mm.**

Immediate conservative patch:

- free clearance allowance: **25–30 mm maximum**
- error should be based on deviation from the intended end state, not `max(clearance - allowance, 0)`
- under-target/jammed configurations must not become free

Better structural version:

For each foot `i`, define an episode-specific desired end clearance `c_ref_i` (normally approximately zero/contact for a planted lower ending, or whatever the commanded geometry actually implies). Evaluate

```text
e_clear_i = abs(c_i - c_ref_i)
```

with a small tolerance band, e.g. ~10–15 mm around the intended reference, rather than a single 60 mm ground-clearance dead zone.

If the intended end state is genuinely “foot planted,” use **sim contact state/load as an evaluation signal as well as clearance**:

```text
foot_ok_i =
    recent_contact_i
    AND abs(c_i - c_ref_i) <= clearance_tol
    AND slip_i <= slip_tol
```

Do not train the deployable actor on privileged contact if hardware does not have it; this recommendation is for the **sim reward/evaluator/gate**, where contact is known.

### Important sign issue

Do **not** simply turn the clearance term into `abs(c)` if `c < 0` means MuJoCo penetration/contact compression. These are physically different failures:

- too high: hovering
- too low: penetration / geometry mismatch / excessive compression

Both should be non-free, but penetration should have its own contact/penetration/impulse treatment. The correct symmetric object is **error around a desired reference**, not raw signed world clearance.

### Why

The existing 60 mm dead zone is not a tolerance; it is a second valid posture. PPO is correctly occupying it.

The evidence already shows this:
- legs intentionally move into the 42–60 mm region
- a leg that began planted chooses to hover there
- the tolerance is several times the uncertainty it was intended to cover

That is a gate specification failure, not an exploration failure.

### Downstream consequence

Expect the reported lower-posture success rate to fall immediately. That is desirable: the old number counted behavior that the physical requirement does not accept.

Do not compensate by immediately increasing reward coefficients. First establish a physically correct evaluator.

---

## 1b. Is 2.6 A plausible for a slow planted descent?

### Manufacturer facts

Feetech’s official STS3215 page specifies at 12 V:

- no-load current: **180 mA**
- rated torque: **10 kg·cm**
- stall torque: **30 kg·cm**
- stall current: **2.7 A**

Therefore 2.63 A is about **97% of the published stall current**.

A simple DC-motor static approximation,

```text
I ≈ I0 + (Istall - I0) * tau / taustall
```

would place 2.63 A extremely close to stall torque. This approximation is not a calibrated STS3215 model, but it is sufficient to show the scale: 2.6 A is not an ordinary “somewhat loaded” operating point.

### Critical nuance: slow does NOT imply low current

A position servo moving slowly under large load can draw very high current. At low rotor speed, back-EMF is small; if the controller is asking for high torque to hold/force the commanded position, current can approach stall.

So the following inference is wrong:

```text
slow motion -> low current -> therefore 2.6 A must be a sim bug
```

It could be real if the geometry/contact constraints force a joint near stall.

### But the specific observation is still suspicious

For a deliberately slow, coordinated **six-foot planted** descent, repeated near-stall current on a servo should be treated as a red flag. Possible causes include:

- load concentrated on too few legs
- an unfavorable/singular joint geometry
- feet mutually fighting because the commanded joint configuration is incompatible with contact constraints
- servo position stiffness too high in sim
- incorrect torque→current scaling
- contact stiffness/friction mismatch
- wrong mass/CoM
- current model not reproducing the servo’s internal controller/profile behavior

This is particularly important because the MuJoCo 3.11 migration already moved the simulated quiet-hold peak across the 2.5 A safety threshold.

### Binding recommendation

**Do not decide the stance reward economy until this is measured.**

Run a small hardware calibration that reproduces the relevant load regime safely, rather than testing stall:

1. Robot physically supported against falling.
2. Use a known planted stance.
3. Execute a very small/slow downward body or representative joint movement.
4. Log commanded position, measured position, current, voltage, velocity and temperature for every servo.
5. Stop well below the known dangerous regime; use the existing current trip.
6. Compare current-vs-position-error/current-vs-load traces to the exact MuJoCo replay.

The goal is not to prove the 2.7 A stall spec. The goal is to answer:

> Does a normal planted stance/descent on this robot really put individual servos near 2.5–2.6 A?

If hardware says yes, the maneuver/reward design must respect that physical cost. If hardware is far lower, repair the sim current/actuator/contact model before using current penalties or gates.

### Hardware interpretation

A **brief** 2.6 A transient is physically possible.
A **sustained** 2.6 A condition is essentially near-stall operation and should be treated as unsafe for this project.

Do not turn “published stall current = 2.7 A” into “2.6 A is acceptable.”

---

## 1c. support_margin paying tripod hover more than planted stance

### Ruling

**Not defensible for the lower/end-rest skill.**

There are two separate concepts being conflated:

1. static stability of the current support polygon
2. correctness of the requested end posture/contact set

A wide tripod can be statically stable. That does not make it the correct terminal configuration when the requested skill is “lower into the defined planted/rest posture.”

### Audit the implementation

If `support_margin` is literally the distance from projected CoM to the boundary of the convex hull of active contact points, then adding contact points at the same foot locations cannot shrink the convex hull. If “six planted” scores less than a subset solely because of contact count, the implementation deserves inspection.

It is possible for the tripod episode to have a better geometric margin because the robot has rearranged its feet/body, but even then the reward is misaligned with this mode.

### Recommended shape

For **lower**:

- end posture/contact correctness is the task objective
- support margin should be a **minimum-stability constraint/backstop**, not open-ended positive income
- once margin is safely above a threshold, additional margin should pay approximately nothing

Conceptually:

```text
stability_ok = support_margin >= minimum_safe_margin
```

not

```text
more support_margin = more income forever
```

Then separately reward/gate:
- all intended feet in their desired end state
- low residual motion
- safe current
- low impact

This prevents “excellent tripod, wrong task” from out-earning the desired posture.

### Downstream consequence

The lower policy may initially become less stable as it loses the tripod-income shortcut. Keep a hard stability/safety guard, but do not restore a reward that makes the shortcut economically superior.

---

# 2. Rear-hemisphere command scope

## Ruling: 2b — explicitly defer

The current phase should declare its command distribution explicitly:

> **Forward locomotion phase:** forward and, if desired, modest forward-diagonal commands. Backward, rear-lateral, lateral and yaw competence are out of scope until the forward gait passes the physical-quality and reliability gates.

Therefore:

- remove rear-hemisphere samples from the current promotion gate
- do not count current failure on backward commands as a failure of the forward phase
- clearly label checkpoints as forward-domain, not omnidirectional

### Why

The policy has not yet solved the more fundamental physical behavior:
- skating/paddling
- occasional park basin
- stability/reliability
- hardware-safe contact/current behavior

Expanding the command task now increases multi-task difficulty while the base gait is still wrong.

This matches the lesson from high-performance legged-RL work: broad command spaces are commonly introduced using a **command curriculum** rather than assuming uniform competence over the entire space from the beginning. Margolis et al., *Rapid Locomotion via Reinforcement Learning*, explicitly found that broad velocity-command ranges can make training fail and used an adaptive curriculum to expand the feasible command set.

### Recommended later curriculum

After forward hardware-quality walking:

1. forward
2. forward diagonals
3. lateral
4. rear diagonals/backward
5. yaw
6. combined translational + yaw commands

Expansion should happen when the prior region retains its gate.

### Downstream consequence

This makes current evals easier, but not artificially so: they now match the declared skill.

The cost is that nobody may claim “general velocity-conditioned locomotion” yet. That is fine.

---

# 3. Distance gate and low-speed commands

## First: the current fixed 0.40 m gate is invalid

For a horizon `T = 15 s` and command `v_cmd = 0.03 m/s`, commanded displacement is:

```text
D_cmd = v_cmd * T = 0.45 m
```

Requiring 0.40 m means requiring ~89% of commanded distance at the lowest command, while the same fixed threshold is much less demanding at larger commands.

More importantly, if there are command draws below 0.0267 m/s, 0.40 m is literally impossible at perfect tracking.

A fixed distance threshold across a variable command distribution therefore changes the task from:

> obey the requested locomotion

to:

> move at least this far regardless of what was requested

That mechanically selects overspeed.

The observed ~43% overspeed is consistent with the gate/reward economy, not surprising PPO behavior.

---

## Ruling: choose 3a over 3b — but simplify the phase if possible

If variable velocity commands remain, use **command-scaled displacement**.

Recommended primary statistic:

```text
progress_ratio =
    along_command_displacement /
    max(commanded_speed * horizon, epsilon)
```

For nonzero walking commands, a reasonable initial promotion interval is approximately:

```text
0.75 <= progress_ratio <= 1.25
```

Tune these only as **gate tolerances**, not as reward coefficients. The upper bound prevents “always charge at maximum speed” from passing every command.

Also require:
- zero termination
- gait_valid
- slip/m below target
- attitude/stability gate

Velocity error can remain a diagnostic/secondary gate, but **3b should not replace the main gate**, because the operator explicitly does not care about precise speed tracking as the top-level objective.

---

## Stronger framing recommendation: stop varying speed for this phase

The stated objective is:

1. DISTANCE
2. STABILITY
3. RELIABILITY
4. not speed tracking

That conflicts with simultaneously asking one policy to track a broad set of speed commands.

The cleanest current-phase task is therefore:

> **WALK FORWARD** at one nominal/narrow operating point and cover as much ground as possible while remaining stable, reliable, and non-skating.

For example, train/evaluate one fixed or narrow command band during gait consolidation. Then the distance gate is honest and simple:

```text
distance >= threshold
AND no termination
AND gait_valid
AND slip/m <= threshold
AND stability gate
```

Only after this is a good physical gait should speed conditioning be reintroduced as a multi-task extension.

This is the recommendation I prefer if the goal is “something robust and cool on the physical robot soon.”

---

## Fix the cadence-reset exploit structurally

The current anchor allowance:

> touchdown resets slip budget

creates an exploitable accounting identity:

```text
free slip per episode ≈ number_of_touchdowns * free_tolerance
```

Therefore any per-touchdown resettable slip allowance will create pressure for cadence inflation.

Do not fix this by lowering the tolerance coefficient.

Use a quantity that cannot be reset into free income, such as:

- total horizontal foot travel while **loaded/in contact**, integrated over the episode
- loaded slip distance per meter of body progress
- contact-point velocity integrated while normal force/contact exceeds a threshold

For example:

```text
loaded_slip =
    sum_i integral[ contact_i ] ||v_foot_xy_i|| dt

slip_per_meter =
    loaded_slip / max(body_forward_distance, eps)
```

Touchdown can reset swing-phase bookkeeping, but it should **not reset accumulated loaded slip**.

This aligns the metric directly with the real failure mode: foot grinding on the floor.

---

# 4. Fixed panel vs multi-seed evaluation

## Ruling

Use both for different jobs.

### Fixed command panel
Purpose:
- regression canary
- apples-to-apples checkpoint comparison
- debugging known failures

Never discard it.

### Promotion panel
Purpose:
- estimate actual task reliability

Use:
- multiple independently seeded panels
- enough episodes to resolve the desired failure probability
- deliberately balanced coverage across the currently declared command domain

With MJX/4096 environments, evaluation is cheap enough that six stochastic command draws should not determine promotion.

### Corner-case panel
Always include explicit hard cases rather than hoping random sampling hits them:
- smallest in-scope command
- largest in-scope command
- edge forward-diagonal commands if included
- low/high friction
- latency extremes
- one weak/slow actuator case later in the system-ID/adaptation ladder

A policy passes only if the random distribution and the named corners both pass.

---

# 5. What is actually blocking each line now

## Stance

Do **not** launch another stance reward arm until:

1. the 60 mm gate is replaced with a physically meaningful end-state metric
2. `support_margin` cannot pay the wrong contact set more
3. the 2.6 A simulated descent is checked against hardware/current calibration

There is no value optimizing PPO against an economy whose desired maneuver may currently be labeled expensive and whose wrong endpoint is labeled successful.

## Walk

The walk line has two structural accounting bugs:

1. fixed-distance gates with variable velocity commands select overspeed
2. touchdown-reset slip allowances select cadence inflation

Repair those definitions before interpreting another reward ablation.

Then keep the current gait objective narrow:
- forward
- real stepping
- net distance
- stability
- zero falls
- low loaded slip

Rear/omnidirectional locomotion comes later.

---

# 6. Exact operator decisions to encode

Copy these into the binding plan:

### STANCE-ALLOWANCE
**Decision:** Retire the fixed 60 mm free-hover band. Interim cap 25–30 mm; preferred implementation is per-episode reference-relative clearance/contact error. Under-reference states are not free; penetration/contact compression is handled separately.

### STANCE-CURRENT
**Decision:** A 2.6 A STS3215 current is physically plausible but approximately stall-scale, not a normal safe descent current. Revalidate the exact planted-descent current regime on hardware and recalibrate MuJoCo current/actuation/contact behavior before allowing current economics to drive stance reward design.

### STANCE-SUPPORT
**Decision:** In lower mode, support margin is a minimum-stability constraint, not open-ended income. The requested six-foot/end-rest state must outscore any tripod-hover state.

### WALK-COMMAND-SCOPE
**Decision:** Rear hemisphere is explicitly deferred. Current promotion distribution is forward/forward-diagonal only. Broad command competence is added later via curriculum.

### WALK-DISTANCE-GATE
**Decision:** Retire fixed 0.40 m across variable commands. If speed commands remain, use command-scaled displacement with a loose overspeed bound; velocity error is secondary. Preferred current-phase simplification: narrow/single forward walk command because the operator objective is distance/stability/reliability, not speed tracking.

### WALK-SLIP
**Decision:** Loaded slip is accumulated continuously and never reset by touchdown. Use loaded foot-XY travel per meter of body progress as the primary skating metric.

### EVAL-PANELS
**Decision:** Fixed panel = regression only. Promotion = multi-seed panels + explicit corner cases.

---

# 7. Research basis

## Feetech STS3215 manufacturer specifications
Official Feetech product page:
https://www.feetechrc.com/525603.html

Relevant 12 V figures:
- 180 mA no-load current
- 10 kg·cm rated torque
- 30 kg·cm stall torque
- 2.7 A stall current
- current/position/speed/voltage feedback

Implication: 2.6 A is near-stall scale.

## Margolis et al. — Rapid Locomotion via Reinforcement Learning
https://arxiv.org/abs/2205.02824

Relevant findings:
- joint-position-command policy transferred to real quadruped
- policy consumes history of observations/actions
- broad velocity-command spaces can make learning fail
- adaptive command curriculum expands the feasible command set
- online system identification is a key sim-to-real component

Implication: staging the command domain is standard and principled; broad rear-hemisphere competence need not be a current gate.

## Kumar et al. — RMA: Rapid Motor Adaptation for Legged Robots
https://arxiv.org/abs/2107.04034

Relevant finding:
- real-world locomotion under changing terrain/payload/wear benefits from online adaptation using an adaptation module that infers latent environment properties.

Implication: domain randomization plus history/adaptation is useful for sim-to-real, but it does not justify broadening the current locomotion task before the base gait is physically sound.

## DreamWaQ
https://arxiv.org/abs/2301.10602

Relevant finding:
- robust real-world quadrupedal locomotion can be learned with limited/proprioceptive sensing by learning implicit environment/terrain representations.

Implication: privileged sim state can help training/critics, while deployable actor inputs should remain hardware-observable; history/estimation belongs in the robustness stack, not as a substitute for correct reward/gate economics.

---

# Bottom line

The campaign is currently at risk of solving its own accounting rules instead of the physical tasks.

The next gains should come from fixing the definitions:

- **stance:** desired contact/end posture must be the cheapest valid end state
- **walk:** actual loaded foot grinding must accumulate continuously; cadence cannot reset it
- **distance:** the gate must not require the robot to disobey low-speed commands
- **scope:** only evaluate command directions that are actually in the current training phase

After those are corrected, PPO results become interpretable again.

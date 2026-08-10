# SIM.md — what the physics simulation models, and how sure we are

One page of truth for the MuJoCo twin: what is simulated, where every
number came from, and which numbers are COVERED BY DOMAIN
RANDOMIZATION because we do not trust them as point estimates.

**Operator ruling (08-10, binding): quantities we are unsure of —
servo reaction times being the canonical example — go into domain
randomization as a range covering the uncertainty. Do not burn time
modeling them "perfectly", and never present an uncertain fit as an
exact nominal.** The confidence table below is the implementation of
that ruling; keep it honest when refitting.

## The simulation chain (one control tick, 25 Hz)

policy action → SafetyLayer (same rate clamp/limits as the robot,
`rl_move/safety.py`) → **servo model** → MuJoCo physics → obs builder
(same `build_obs` as the hardware runner).

The servo model (`rl_move/sim/servo_model.py`, `ServoProfile`) has
four stages, mirroring the real Feetech bus write
(`speed=400 counts/s, acc=20` — cfg `bus.write_speed/write_acc`, the
deployed runner's values):

1. **Latency** — each goal write matures after a per-axis delay.
2. **Trapezoid profile** — the servo's internal setpoint slews toward
   the goal (accelerate at `acc`, cruise at min(commanded speed,
   fitted velocity ceiling), decelerate).
3. **Deadband** — errors below it produce no motion (and no torque).
4. **Tracking** — MuJoCo position actuator `kp` + DOF damping `kv` +
   `frictionloss` + torque clamp track the profile target.

Identical semantics in the C env (`sim_env.py`, numpy) and the GPU
training path (`mjx_backend.py`, JAX pytree; parity-tested). The body
model comes from `mujoco_prototype.build_xml()` (CAD-derived
kinematics; primitive collision geometry; foot contacts softened 3× —
see `soften_contacts`). GPU port details: `../rl_move/sim/MJX_PORT.md`.

## Two fitted parameter sets (cfg `bus.servo_params`)

| Set | File | Data | Selected by |
|-----|------|------|-------------|
| **air** (default) | `rl_move/sim/sim_model.json` | 08-07 unloaded bench battery (`fit_motor_model.py`, legs straight out, fixed base) | nothing — legacy default, byte-exact |
| **loaded** | `rl_move/sim/sim_model_loaded.json` | 08-10 loaded step ladder at plant + rl_stand deployed traces (`fit_loaded_actuator.py`, free-base stance replay) | `--cfg-set bus.servo_params=loaded` |

Every consumer (both trainers, both eval harnesses, C and MJX envs)
resolves the set from the run's cfg, so a run's own cfg package evals
under its own actuator model. A missing file raises — no silent
fallback. The air fit is known-wrong under load (it also recorded the
commanded profile speed as the velocity ceiling, and its 0.494°
deadband is an unloaded-rig artifact); prefer `loaded` for anything
meant for hardware.

## Confidence table (loaded set, knee axis; DR bands at dr_scale 1)

| Parameter | Nominal | How we know it | Uncertainty | DR coverage |
|-----------|---------|----------------|-------------|-------------|
| deadband | 0.06° | loaded tracking 96.6% at 5° pins it (air's 0.494° was the whole sim tracking floor) | ~2× across joints/regimes | ×0.5–1.8 → 0.03–0.11° |
| velocity ceiling | 48.5°/s | ladder peaks 48–67°/s at non-binding profile speed | moderate — but the deployed write speed (35.2°/s) binds first, so it rarely matters | ×0.85–1.10 |
| **latency** | **85 ms (capped)** | **WEAKLY — ladder cmd→motion (110–210 ms) includes the HTTP hop; direct-bus air knee was 8.6 ms; fit sat on its cap** | **~25–160 ms plausible** | **×0.3–1.9 → 26–162 ms** (widened via `delay_ms_pct=0.45` in the file's spread — the ruling above, mechanized) |
| kp / kv / frictionloss | 916 / 0.17 / 0.02 | sim-in-loop fit vs step shapes; holdout ±5° 20–40× better than air | shape-fit, load-dependent in reality | kp ±20%, kv ±25% per joint |
| torque limit | 2.2 N·m | datasheet, never fitted | unknown under sag | torque_scale ×0.80–1.05 |
| hip / yaw (all of the above) | knee latency delta +76 ms, shared ceiling; air kp/kv/deadband | **ASSUMPTION — no loaded ladder for these axes** (HARDWARE.md wishlist item 4) | large | same DR bands; latency band spans the assumption |

Deployed-path sanity check: cmd→response lag from the rl_stand traces
(25 Hz derivative cross-correlation) is ~250 ms median on all axes —
consistent with latency + profile travel + tracking lag combined.

Mechanics of the ruling: `DomainRandomizer.from_params`
(`domain_rand.py`) widens DR from the params file's `spread` block, so
fit uncertainty travels WITH the file. Per-run widening needs no code:
`--cfg-set dr.latency_scale=0.3,1.9` (any `dr.<field>=lo,hi` is an
absolute range override applied after dr_scale).

## Also randomized every episode (`domain_rand.py` RandRanges)

Mass/inertia + CoM shift, per-link geometry scale, floor friction
×0.6–1.4, contact stiffness ×0.7–2.0, ground tilt (gravity vector),
battery-sag torque scale, dropped SyncWrites (≤5%/tick), IMU mount
offset + sensor noise, hand-placement pose slop, bad-start joints,
logical-zero drift. Model-field DR is applied in the C env and (as
shared-model per-env fields) in the MJX path.

## Known gaps — sim is NOT trusted here (open problems in RL_PLAN)

1. **Contact travel pricing: CALIBRATED 08-10, premise revised.**
   `rl_move/sim/calibrate_slip.py` replays the exact hardware tripod
   gait (same generator, plant +20/+80, hardware write profile
   1500/30) against the tape truth (ratio 0.50–0.51, speed-invariant).
   Result: sim travels 0.35–0.41 of commanded at BOTH speeds — sim
   does NOT price sliding as free; it loses slightly MORE stride than
   concrete (conservative), and matches the speed invariance and the
   walking current band (sim 0.36–0.45 A vs real 0.31–0.42). μ
   saturates above ~1.5 (sweep 0.2–8.0), so friction is not the
   limiting knob at nominal; XML default stands. New cfg hook
   `env.foot_friction_slide=<μ>` (C + both MJX stacks) recenters
   foot–ground slide μ if a future floor demands it; DR
   friction_scale multiplies around it. Re-run the script after ANY
   contact/servo-param change (Gate 0).
2. **Current/effort pricing inverted AT HOLD** — quantified 08-10 by
   the same replay: sim plant-hold mean current 0.11 A vs real 0.59 A
   (walking matches). The inversion is NOT fixable by scaling
   (ordering flips); it needs a load-dependent holding-current model
   fitted on the existing per-servo traces (tape CSVs + rl_stand
   logs). Until then `k_current=0` on hardware arms stands.
3. **Hip/yaw loaded dynamics assumed**, not measured (table above).
4. **Liftoff +roll collapse not yet reproduced in sim** — the loaded
   actuator set is the prime-suspect fix; re-run the reproduction
   fixture on it before trusting stance sim near liftoff.
5. STL *visual* meshes have stale offsets (June re-export) — physics
   uses primitives; render with `mesh_visuals=False`.

## Refitting

Air battery: `POST /api/rl/probe_dynamics` on the robot →
`fit_motor_model.py`. Loaded: new bench traces into
`rl_move/hardware_traces/`, then `python -m
rl_move.sim.fit_loaded_actuator` (`--measure` first to sanity-check
parsing; it fits ±2/±10°, holds out ±5°, and replays the rl_stand
cmd streams as multi-step validation). Both write provenance +
evidence into their JSON. After ANY sim-param change: scripted-gait
plant-calibration check (Gate 0).

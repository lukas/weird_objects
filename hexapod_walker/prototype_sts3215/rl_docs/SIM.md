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
logical-zero drift, tipped starts. Model-field DR is applied in the C
env and (as shared-model per-env fields) in the MJX path.

**Tipped starts (`dr.tipped_start_prob/deg`, added 08-10):** with prob
0.30×dr_scale a plant/park-start episode begins at a settled 6–18°
body roll (asymmetric hip fold, calibrated: settled roll ≈ 0.36×fold)
while the tilt reference stays LEVEL — the policy sees the lean in obs
and the attitude terms pay it to level out. Capped at 70% of the run's
tilt-trip envelope; belly-rise starts never tip; the DOSE is not
shrunk by dr_scale (probability is). Born from the dep-vref1-r1
hardware runaway roll (rl_docs/HARDWARE.md 08-10). Discovery result
(cw-dep-tip1, same day): the champion already recovered static leans
in sim — the hardware runaway is a contact/pinning sim-to-real gap,
so this axis is a hardening/regression floor, not the fix. Eval:
`SCORE/tipped_recovery_success` (rl_docs/EVALS.md, gate-fix caveat).

**Rise rocking (`dr.rise_rock_*`) — RAMP-GATED as of 08-12.** One-side
hip/knee fold bias on the physical command of rise-mode episodes,
scaled by the height-ramp progress. Recalibrated against the ten
08-11 stand-failure tapes via `replay_trace.py`: the hardware trip is
NOT a curl-long rock — roll stays flat through the curl, then ramps
0→10.1–10.6° over the last ~1.2 s. A persistent fold rocks the flat
curl too (unlike every tape) and one sign saturates at 3–5°; the
ramp-gated fold at ~18° target dose on the branch that removes the
catching foot reproduces the recorded signature (flat curl →
accelerating ramp crossing the 10° trip band near ramp end).

**Walk takeoff push (`dr.walk_push_*`, added 08-12).**
Signed half-sine roll TORQUE about the chassis's own x-axis
(`xfrc_applied`) over the first 0.8–1.5 s of walk-mode episodes,
peak 2.0–3.0 N·m. This is the takeoff-transient axis the command-side
kick could NOT deliver: the fold pulse saturates at 5–10° peak /
~10 °/s at any dose (planted opposite feet + write-profile rate limit
eat the command), far under the tapes' 13–27° / 11–46 °/s. Dose
calibrated policy-in-the-loop (tip1 walking 0.05 m/s): a parked
6-foot plant absorbs 2.6 N·m at ~0.2°, so the pulse only lands when
it overlaps a tripod swing — 2.6 N·m / 1.5 s yields the hardware
coin-flip regime (peaks 2.6→30°, some absorbed, some capsize, 5°
crossings at 0.56–0.76 s); 3.2 N·m falls 3/4 at 64–105 °/s (too
hot). Works on BOTH stacks: private-model (C) envs apply the xfrc in
`_advance`; the MJX vec envs read each shim's per-tick torque and
hand it to the batched stepper, which writes the chassis xfrc row
about that env's own x-axis (plumbed 08-12 through `mjx_backend`,
`mjx_vec_env`, `mjx_sharded_vec_env`). Mechanics pinned in
`test_task_semantics.py` WALK-PUSH bank; device-side application in
`test_mjx_parity.py::test_walk_push_xfrc_reaches_the_stepper`.

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
2. **Current/effort pricing at hold: the 08-10 "inversion" is
   RETRACTED (08-11, `probe_hold_current.py`,
   `logs/probe_hold_current/`).** The old comparison put sim
   MEAN-PER-SERVO (0.11 A) against hardware BUS-TOTAL (0.59 A) — an
   ×18 unit slip — AND compared different poses: the real 0.59 A is
   the SCRIPTED-stand hold (cmd = ideal pose the loaded joints sag
   from, servos fighting continuously), while the real walk-synced
   plant hold reads 0.106 A total (hw_session2 per-servo registers;
   the flailing-knee tick data shows register current tracks cmd−q
   error, e.g. 3.9° err → 0.685 A). Pose/unit-matched probe (free
   base, ServoProfile bus, hardware write profiles, air AND loaded
   params): sim |qfrc|×1.2 proxy vs hw register totals — crouch hold
   1.56 vs 0.106 A, ideal-cmd hold 1.56 vs 0.541, walk30 9.1 vs
   0.395, rl_stand replay quarters ≈4–16 vs 0.19–0.96. Sim
   OVERPRICES effort 3–25x in every condition (conservative: trips/
   charges earlier than hardware would) and REPRODUCES the real
   walk > plant-hold ordering — no inversion, no underpricing.
   Residual (DEFERRED, not a joystick blocker): the proxy is not
   register-accurate — real servos hold static load nearly free
   (gear friction) and pay ∝ cmd-fight, so a register-scale model
   needs a deadzone/cmd-error term. Fit it before any k_current>0
   hardware-pricing arm (quiet-gait/current-economy); `k_current=0`
   on hardware arms stands. Sim-vs-sim current comparisons in past
   verdicts stay valid (same proxy both sides).
3. **Hip/yaw loaded dynamics assumed**, not measured (table above).
4. **Takeoff/rise roll transients: DIAGNOSED 08-12
   (`rl_move/sim/replay_trace.py`, open-loop hardware-action replay;
   plots + JSON in `rl_move/hardware_traces/`).** Replaying the
   recorded action streams from ten deterministic stand failures and
   nine walks into the free-base sim plant, starting from the
   measured initial pose:
   - **Joints are NOT the gap.** Stand replays track at 0.95–1.15°
     RMSE (loaded set; air 1.4–1.7°) — the actuator/load model
     reproduces q(t) through the whole failure.
   - **Walk takeoff excursions ARE reproducible open-loop**: sim
     peaks 8.7–29.5° vs hardware 6–25° on the same tapes — same
     magnitude class, same early onset. The closed-loop policy in
     sim suppresses what the hardware actions excite (baseline
     policy-in-loop peaks 3–5.5°), i.e. sim training never VISITS
     the moving excursion, it doesn't fail to model it.
   - **The stand failure is a support-geometry gap**: every tape is
     flat through the curl then ramps 0→10.1–10.6° in the last
     ~1.2 s; the sim curl glides at ≤2.9° peak. Kinematic
     reconstruction shows hardware tips right pivoting on the L4
     pad while the left pads unload; in sim the left feet stay
     planted and the body is statically stable — CoM shifts to
     40 mm and μ→50 barely move sim roll. The unmodeled part is the
     knife-edge support set during load transfer (chassis/tucked-leg
     contact), not inertia and not friction.
   Mitigations shipped as calibrated DR (ramp-gated `rise_rock`,
   torque `walk_push` — see the DR section above); a true
   contact-geometry fix (belly/tucked-shank collision during the
   curl) remains open.
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

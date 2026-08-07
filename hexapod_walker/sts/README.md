# STS3215 walking (sim + RL + stall safe-stop)

Focused walking stack for the Feetech STS3215 tabletop hexapod. Reuses the
MuJoCo model / tripod gait from [`../prototype_sts3215/`](../prototype_sts3215/)
but trains a policy on **the same sensor channels the real bus returns**:

| Channel | Source on robot | Sim model |
|---|---|---|
| position (deg) | 12-bit encoder | quantized `qpos` + noise |
| speed (deg/s) | STS present-speed | `qvel` + noise |
| load (%) | present-load register | `|τ| / τ_stall × 100` |
| current (A) | present-current (6.5 mA/LSB) | `I_idle + (I_stall−I_idle)·|τ|/τ_stall` |
| voltage (V) | present-voltage | 12 V − bus sag |
| temperature (°C) | present-temperature | I²R + cooling |

Stall safe-stop (sim and hardware): if a joint stays near-stopped **and**
near-stall load/current for ~120 ms, freeze targets at the last pose, then
torque-off (`relax`) so the motor is not cooked against a jam.

Sit / stand: ~40% of training episodes practise transitioning between a
**legs-out-wide sprawl** (chassis low) and the normal standing stance.  The
nominal motion is a **4–7 s** cosine blend; the policy may only *slow* it
further (rate ∈ [0.4, 1.0]) and is rewarded for staying under soft current
limits:

- position-actuator torque capped at ~2.10 N·m (~71% stall)
- soft/hard per-servo current: 1.05 A / 1.80 A (truncate above hard)
- soft/hard bus current: 12 A / 18 A
- amp-second energy penalty on both walk and posture

so it learns thrifty stand-ups and gaits — the open-loop slam that hit ~29 A
was a demo discontinuity, not a trained behaviour.

## Layout

| File | Role |
|---|---|
| `sts_sensors.py` | STS electrical + encoder model |
| `stall_guard.py` | Stall FSM (NORMAL → HOLD → RELAX) |
| `posture.py` | Sit (sprawl) / stand joint targets + blend |
| `sts_env.py` | Gymnasium env (walk + posture + motor obs) |
| `train.py` | PPO training entrypoint |
| `rollout.py` | Watch a trained policy in MuJoCo |
| `controller.py` | Deploy-time policy + guard (sim or `FeetechBus`) |

## Quick start

```sh
# Watch sit → stand → walk with live total bus amps (viewer + terminal bar)
./.venv/bin/mjpython hexapod_walker/sts/demo_amps.py

# Smoke the env (walk + stand-up/sit-down currents + stall trip)
./.venv/bin/python hexapod_walker/sts/sts_env.py

# Short train (walk + sit/stand mix)
./.venv/bin/python hexapod_walker/sts/train.py --steps 5000 --n-envs 1

# Real training run
./.venv/bin/python hexapod_walker/sts/train.py --steps 300000 --n-envs 8

# Posture-only or walk-only
./.venv/bin/python hexapod_walker/sts/train.py --steps 200000 --posture-only
./.venv/bin/python hexapod_walker/sts/train.py --steps 200000 --no-posture

# Roll out walk / stand-up / sit-down
./.venv/bin/python hexapod_walker/sts/rollout.py \
  --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip \
  --vx 0.10 --headless
./.venv/bin/python hexapod_walker/sts/rollout.py \
  --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip \
  --mode posture --posture stand --headless --duration 5
./.venv/bin/python hexapod_walker/sts/rollout.py \
  --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip \
  --mode posture --posture sit --headless --duration 5

# Demo the stall guard mid-walk
./.venv/bin/python hexapod_walker/sts/rollout.py \
  --policy hexapod_walker/sts/policies/sts_ppo/sts_ppo.zip \
  --jam --headless --duration 6
```

## Observation contract

Leading dims match the residual-gait walker (joint state, chassis twist,
contacts, command, gait phase, optional gait scales / swing mask), except
joint pos/vel are **STS-encoder-derived**, not privileged MuJoCo state.
Trailing motor dims (57):

```
load_pct/100 ×18 | current/I_stall ×18 | bus_V/12 | mean_temp/100
| stall_severity | per-joint stall-accumulator ×18
```

That last block is what makes the policy robust to torque/amp feedback:
it can feel a joint climbing toward stall and back off *before* the guard
trips.  If it still trips, the episode ends with a stall penalty and the
controller freezes / relaxes.

Trailing posture dims (3): `mode` (0=walk, 1=posture), `posture_cmd`
(0=sit, 1=stand), `blend_alpha` (0→1 through the cosine transition).

## Hardware bridge

`controller.apply_to_bus(bus, result)` writes targets through
`prototype_sts3215/motor_setup/feetech_bus.py`.  On `relax=True` it
torque-offs every servo.  On `stall=True` (freeze before relax) it uses
a gentle hold profile (`speed≈250`) — Feetech `speed=0` means *max*,
not stop.  End a session with `safe_stop_walk(bus)` (soft-hold, keep
supporting torque).  Build the observation’s leading slice from IMU +
the same `read_feedback` dicts the policy already expects for the
trailing slice (`feedback_dicts_to_array`).

## Constants

Aligned with `prototype_sts3215/standup_current_sim.py` and WIRING.md:

- `τ_stall = 2.94 N·m` (30 kg·cm @ 12 V)
- `I_stall = 2.7 A`, `I_idle = 0.2 A`
- Sim actuator `forcerange` ceiling set to `2.7 N·m`

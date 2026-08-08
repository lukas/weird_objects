# MJX (MuJoCo-XLA / JAX) physics backend — port status

Goal: optionally run the sim twin's physics as batched JAX on CoreWeave
GPUs (pool nodes carry H200s), instead of one C-MuJoCo process per env.
The default CPU PPO path is untouched — nothing imports this backend
unless asked.

## Status: phase 1 done (2026-08-08)

What exists and is verified (`rl_move/tests/test_mjx_parity.py`, 4/4
passing against mujoco/mujoco-mjx 3.9.0, jax 0.11):

- `servo_model.build_model(mjx_compat=True)` — the ONLY model change
  MJX needs is hfield→plane (same z=0 surface, friction, condim).
  Newton solver, elliptic cone, condim 4/6 all convert as-is.
- `mjx_backend.py`:
  - Exact JAX port of `ServoProfile` (latency queue → fixed 8-slot
    ring, trapezoid, deadband) as a jit-able pytree. Matches numpy to
    float32 round-off (<0.012° over 40 ticks incl. dropped SyncWrites).
  - `MjxTickStepper` — one jitted+vmapped call advances B robots one
    25 Hz control tick, replicating `SimHexapodBalanceEnv._advance`:
    profile tick → firmware dead-zone → `mjx.step` → IMU specific-force
    (at the chassis-CoM-referenced mount point, `xipos`, matching
    `mj_objectVelocity(mjOBJ_BODY)`) + gyro accumulation per substep.
  - Per-env actuation DR supported now (latency/deadband/vel scales,
    IMU mount offset via `TickParams`); per-env MODEL DR not yet.
- Parity: air trajectory (no contacts) <0.5° over 1 s; contact settle
  within 10 mm chassis height / 3° joints (loose by design — float32
  contact solves differ); IMU reads 1 g static.
- `bench_mjx.py` — the go/no-go tool. Laptop CPU reference:
  C MuJoCo 1 core ≈ 1300 env-steps/s (48-core subproc ≈ 62k);
  MJX on CPU ≈ 8 env-steps/s/env — **MJX only pays off on GPU with
  large batches**. Run on a GPU pod before building more:

      kubectl apply -f rl_move/sim/coreweave_pod_gpu.yaml
      # tar/cp code, then: HEXAPOD_MJX=1 bash /workspace/setup.sh
      python -m rl_move.sim.bench_mjx --batch 256 1024 4096 8192

## Deliberate design decisions

- **Hybrid split**: physics on device, everything else (obs build,
  reward, SafetyLayer, IK, reset placement, goal logic) stays host-side
  numpy reading the small `TickOutput` arrays. Reward semantics remain
  byte-for-byte `rl_move/env.py` — no re-validation of the reward stack
  needed, only of the physics.
- Latency queue as a ring: a slot is only overwritten ≥320 ms after it
  was written, and every goal matures (latency ≤ ~110 ms at DR max)
  long before that, so semantics equal the unbounded numpy queue.
  `reset_envs` asserts this.
- Gyro computed from `cvel` + `site_xmat` (not MJX `sensordata`) so we
  don't depend on MJX sensor coverage; touch sensors ARE in
  `TickOutput.sensordata` but unvalidated.

## Phase 2 — before any MJX training run

1. **GPU benchmark** (above). If batched MJX doesn't clearly beat ~62k
   env-steps/s per pod, stop here; the [128,128] MLP itself trains
   fastest on CPU and eval/video is already off the hot path.
2. Batched VecEnv: an SB3-compatible `MjxVecEnv` that keeps
   `SimHexapodBalanceEnv`'s obs/reward/safety code but calls
   `MjxTickStepper.tick` once per step for the whole batch. Resets are
   the hard part (per-env settle phases while others run → mask limp /
   command per env; the stepper already takes per-env `limp` and
   `Command.valid`).
3. Per-env model DR: MJX supports vmapping model fields — batch
   `body_mass`, `geom_friction`, `geom_solref`, `opt.gravity`,
   actuator gains (today's `EpisodeRandomization.apply_to_model` +
   `apply_params_to_model` writes, but as stacked arrays).
4. Behavioral A/B: same policy checkpoint evaluated on C-MuJoCo vs MJX
   envs (harness eval, ≥20 episodes/mode) must agree before any MJX
   training result is trusted — same bar as the mujoco 2.3.7→3.11 A/B.
5. Current-estimate check: `qfrc_actuator`-based servo current feeds
   the SafetyLayer trip; MJX float32 contact forces shift it like the
   3.11 upgrade did (+0.14 A quiet hold) — recalibrate/verify.

Phase 3 (only if 1–2 prove out and PPO becomes the bottleneck): move
obs/reward into JAX and use a JAX-native PPO (Brax-style) to kill the
device↔host sync per tick. Big rewrite; not justified today.

## Files

- `mjx_backend.py` — backend (this port)
- `bench_mjx.py` — C vs MJX throughput
- `requirements-mjx.txt` — optional deps (mujoco-mjx pin MUST match mujoco)
- `coreweave_pod_gpu.yaml` — H200 bench pod
- `coreweave_pod_setup.sh` — `HEXAPOD_MJX=1` installs the JAX stack
- `../tests/test_mjx_parity.py` — parity suite (skips without jax)

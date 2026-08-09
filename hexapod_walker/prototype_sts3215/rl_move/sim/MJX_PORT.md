# MJX (MuJoCo-XLA / JAX) physics backend — port status

Goal: optionally run the sim twin's physics as batched JAX on CoreWeave
GPUs (pool nodes carry H200s), instead of one C-MuJoCo process per env.
The default CPU PPO path is untouched — nothing imports this backend
unless asked.

## H200 benchmark verdict (2026-08-08, hexapod-mjx-bench pod)

Workload: the full `MjxTickStepper` tick (profile + dead-zone + step +
IMU), robot in ground contact. Bar to beat: **~24k env-steps/s** =
48-env subproc C MuJoCo on a 48-core pod (measured 496/s/core there).

| config | batch | env-steps/s |
|---|---|---|
| impl=jax, iterations 8/8 | 4096 | 2,068 |
| impl=jax, iterations 8/8 | 8192 | 6,064 |
| **impl=warp, iterations 1/4** | 4096 | **60,409** |
| **impl=warp, iterations 1/4** | 8192 | **73,864** |

- The **XLA impl loses to the CPU pod by ~4x** and hit memory-thrash
  at batch 16384; a dead end for this contact-heavy model (elliptic
  cone, condim 4/6). Matches the docs: reference humanoid is 950K
  phys-steps/s on A100 via XLA vs 2.96M via Warp.
- The **Warp impl (`impl="warp"`, needs mujoco-warp) wins by ~3x per
  H200** with sub-second warm compiles. Contact buffers must be sized
  (`nacon_per_env`, `njmax`) — Warp *warns and drops contacts* on
  overflow, so watch stderr.
- **Physics parity held at iterations=1/ls=4 under Warp** (pod check
  vs production C at iterations=50): belly settle dz=0.0 mm / max
  joint diff 0.27°; stand-to-plant maneuver dz=0.2 mm / max 0.70°.
- Solver iterations 50→8 verified locally: settled pose bit-identical
  (Newton converges early; 50 is headroom). 8→1/4 verified on-pod as
  above.

Net: one H200 ≈ 3 CPU pods of raw physics, ~12x if per-tick host
work stays off the critical path. Worth phase 2 — but note PPO at
4k-8k envs is a different training regime than 48 envs (rollout batch
per update grows ~100x), so the RL hyperparameters need rework before
this translates into faster wall-clock learning.

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
  don't depend on MJX sensor coverage; touch sensors in
  `TickOutput.sensordata` validated against C in phase 2 (within 2%).

## Phase 2 — before any MJX training run

1. ~~GPU benchmark~~ DONE (verdict above: Warp impl only).
2. ~~Batched VecEnv~~ DONE 2026-08-08: `mjx_vec_env.MjxVecEnv`, a real
   `stable_baselines3` VecEnv (PPO consumes it directly, no
   DummyVecEnv wrap). Design:
   - **Shim envs**: B actual task-env instances (any
     `SimHexapodBalanceEnv` subclass — balance/goal/joint/walk all
     work) share ONE prepared MjModel and never step physics; they run
     the host halves `sim_env.py` now exposes (`_step_begin` /
     `_step_finish` / `_post_step` / `_reset_begin` /
     `_reset_finalize`) against a per-env numpy mirror of the batched
     `TickOutput`. Safety, IK, DR sampling, obs, and the WHOLE reward
     stack are byte-for-byte the C code — walk shaping included (it
     moved from a `step()` wrapper into the `_post_step` hook; C
     behavior unchanged, full suite green).
   - **Synchronized reset choreography**: place (host C-MuJoCo
     scratch) → slip-settle stiff → slip-settle limp → capture nominal
     → hold, all envs in lockstep; slip friction is a second
     pre-compiled model variant (`MjxTickStepper(slip_mu=...)`,
     `tick(slip=True)`) so no retrace when friction changes.
   - **Pooled mid-rollout resets**: terminations pop a pre-settled
     state (device qpos/qvel/profile/IMU inject + deep-copied host
     bookkeeping incl. SafetyLayer/IK objects) so SB3 gets its
     auto-reset obs instantly with zero fake transitions; dry pools
     auto-refill by snapshot → choreography → restore (wall clock
     only, live rollout untouched).
   - Verified on CPU MJX (`../tests/test_mjx_vec_env.py`): zero-action
     hold within 0.25 reward/step and 1.5° of the C twin; touch
     sensors match C within 2% (3.44 N vs 3.43-3.49 N per foot —
     "unvalidated" caveat below is CLOSED); rejected-action and
     truncation resets; PPO `learn()` end-to-end; walk task with DR.
   - Limits: no `render()` (eval/video workers build C envs anyway).
3. ~~Per-env model DR~~ DONE 2026-08-09. On-pod probe first: ALL 11
   fields the C env's per-episode model prep writes (`body_mass`,
   `body_inertia`, `body_ipos`, `body_pos` (leg lengths),
   `geom_friction`, `geom_solref`, `actuator_gainprm/biasprm/
   forcerange`, `dof_damping`, `opt.gravity`) batch cleanly as
   per-world arrays under BOTH warp and XLA impls (mujoco-mjx 3.11,
   H200) — vmap over a model-axes pytree, worlds diverge, finite.
   Implementation (`model_dr=True`, default ON when `randomize=True`):
   - `MjxTickStepper` takes the model as a runtime vmap argument
     (`MODEL_DR_FIELDS` get a leading (B,) dim); `set_model_fields`
     scatters per-episode rows; snapshot/restore covers the batched
     model so pool refills can't leak DR rows into live episodes.
   - The slip settle became an in-tick friction override (per-env
     `slip` flag) so it composes with per-world friction DR; the IMU's
     gravity now reads `model.opt.gravity` — the DR-tilted vector,
     matching `sim_env._advance` exactly (the old closure constant
     silently ignored gravity-tilt DR).
   - Host side: `mjx_host.ModelDrScratch` replays the C env's exact
     model prep (restore bases → `apply_to_model` →
     `apply_params_to_model` with kp/kv/torque draws) on a private
     scratch model and hands out the field rows; placement
     (`_place_at_plant`) runs on the DR'd scratch so leg-length draws
     shift the plant pose like in the C env. Workers ship rows to the
     parent over new shm blocks; pool entries carry `dr_row` and pops
     re-inject them.
   - Tested: sharded-vs-inprocess bit-equality (randomize=True now
     covers the whole DR path) + `test_model_dr_reaches_device_and_
     stays_stable` (device rows differ per world, chassis-mass row =
     base × the env's sampled `mass_scale`, physics finite).
   - Rough terrain unblocked too: Warp collides with hfields (probe
     compiled the `ccd_hfield` kernel; robot settles at z=0.038 m, no
     fall-through), so `build_model(mjx_compat=True,
     flat_terrain=False)` now KEEPS the hfield (warp impl only — XLA
     still can't). `prepare_shared_model(flat_terrain=...)` passes it
     through. Note today's C training is flat-terrain too, so this is
     capability parity for future terrain experiments, not a change to
     current runs.
   - Measured cost (H200, B=4096, 24 workers, walk task, identical
     random-action methodology): model DR OFF ≈ pre-DR baseline
     (32-34k vs 36.6k env-steps/s, run-to-run noise range); model DR
     ON ≈ 25.5k — **~25-30% for full per-world model DR**. Two perf
     lessons baked into the design: the model must stay a CLOSURE
     CONSTANT of the jitted tick (passing it as a runtime argument
     cost 1.65x under Warp), and only the 11 DR field arrays cross the
     jit boundary. The earlier "53.6k" phase-2 table entry was NOT
     reproducible with this methodology even on yesterday's exact code
     (36.6k measured) — treat per-run fps, not that table, as the
     yardstick.
4. ~~Behavioral A/B~~ PASSED 2026-08-08 (walk task): the cycle-18 walk
   champion `ppo_goal_cw_walk_step0_lowent.zip` (md5 923ee55c), 24
   deterministic episodes per arm on the H200 pod, training config
   (walk-only, no DR, cycle-18 reward overrides):
   return 680.7±116.7 (C) vs 683.5±122.5 (MJX/Warp, iterations 1/4);
   vel_err 0.0386 vs 0.0378; speed 0.0526 vs 0.0528 m/s; mean |roll|
   0.71° vs 0.78°; terminations 0/24 both. Deltas ≪ episode spread —
   MJX/Warp is behaviorally interchangeable for walk. Re-run per new
   task/mode before trusting MJX results there (rise/lower untested).
5. Current-estimate check: `qfrc_actuator`-based servo current feeds
   the SafetyLayer trip; MJX float32 contact forces shift it like the
   3.11 upgrade did (+0.14 A quiet hold) — recalibrate/verify.
6. ~~Trainer~~ DONE 2026-08-08 as a SEPARATE entry point,
   `train_ppo_mjx.py` (the campaign's `train_ppo_sim.py` is untouched;
   shared conventions imported from it). Large-batch defaults are
   starting points only — recipe validation still pending.

   **Measured host-overhead verdict (laptop, walk task, DR on,
   B=128):** the sequential Python halves cost **0.076 ms/env/step**
   (begin 4.0 ms + finish 5.8 ms per 128-env tick), so a single host
   thread caps end-to-end throughput at **~13k env-steps/s** on a
   laptop-class core — likely ~5-7k on pod cores — *no matter how
   fast Warp steps physics*. Projection at B=4096 on the H200 pod:
   ~310-600 ms host + 68 ms Warp per tick ≈ **6-11k env-steps/s
   end-to-end**, i.e. BELOW the 48-core CPU pod's ~24k/s. The raw 3x
   physics win does not cash out until the host halves are sharded
   across the pod's CPU cores (embarrassingly parallel per env —
   worker processes owning env shards around one device owner), which
   would flip the bound back to Warp's 60-74k/s ≈ **2.5-3x a CPU pod
   per H200 node**.
7. ~~Host sharding~~ DONE 2026-08-08: `mjx_sharded_vec_env.
   MjxShardedVecEnv` — same machine as `MjxVecEnv` but the B env
   objects live in N spawn worker processes (torch/jax-free; host
   halves split out into `mjx_host.py`), each running its shard's
   begin/finish/reset halves in parallel while the parent owns the
   stepper. Per-tick traffic (actions, servo commands, TickOutput
   mirror, obs/rew/done) rides shared-memory numpy arrays; only info
   dicts go by pipe. Full pooled-reset protocol preserved: host
   snapshots live worker-side, device arrays parent-side, pushed and
   popped in LIFO lockstep.

   **Correctness: tested BIT-IDENTICAL to the in-process reference**
   (`test_sharded_bitwise_matches_inprocess`) — obs/reward/done
   streams equal to the last bit across DR draws, a rejected-action
   pop, truncation, and a pool refill. The in-process class stays as
   the reference implementation and small-batch/debug path.

   **Measured host-phase scaling (laptop, walk task, DR on, B=128):**

   | host config | host ms/env/step | host ceiling |
   |---|---|---|
   | in-process (reference) | 0.076 | ~13k env-steps/s |
   | sharded, 1 worker | 0.089 (IPC tax ~17%) | ~11k |
   | sharded, 8 workers | **0.020** | **~50k env-steps/s** |

   4.5x at 8 workers on laptop cores; the residual is the parent's
   serial share (device_get + shm copies + info pickling), so pods
   should size `--host-workers` to the spare cores.

   **Measured END-TO-END on the H200 pod (2026-08-08, g131eec,
   hexapod-mjx-train-0: 1 H200 + 26 cores, walk task, DR on, Warp
   iterations 1/4):**

   | config | begin/tick/finish ms | env-steps/s |
   |---|---|---|
   | B=4096, in-process | 1055 total | 3,881 |
   | B=4096, 8 workers | 36 / 45 / 57 | 29,518 |
   | **B=4096, 24 workers** | 13 / 37 / 27 | **53,576** |
   | B=8192, 24 workers | 27 / 68 / 55 | 54,402 |

   **13.8x over the in-process baseline and ~2.2x the whole 48-core
   CPU pod (~24k/s), using 1 of the node's 8 GPUs and 26 of its 128
   cores.** At 24 workers host (~40 ms) and Warp tick (~37 ms) are
   balanced; the finish phase's parent share (info-dict pickling +
   obs copy) is what more workers can't shrink — the next lever if
   ever needed (lean infos / double-buffered batches), not needed now.
   Use `train_ppo_mjx --host-workers N` (0 = in-process). Keep the
   pod yaml's CPU request ≈ N + 2.

Phase 3 (only if 1–2 prove out and PPO becomes the bottleneck): move
obs/reward into JAX and use a JAX-native PPO (Brax-style) to kill the
device↔host sync per tick. Big rewrite; not justified today.

### Canary probes + regression auto-stop (2026-08-09)

`train_ppo_mjx` now arms the campaign's fixed-seed canaries exactly
like `train_ppo_sim`: on every warm start (`--init-from`, unless
`--no-canary`) it runs the parent policy over the 8 fixed-seed C-env
cases at launch, marks groups the parent passes 2/2 as protected, logs
`canary/*` per periodic eval (the probes ride the existing `_BgEval`
C-env worker), and auto-stops after `--canary-stop-after` (default 3)
consecutive full-group failures. Walk-task runs degrade gracefully
(no rise/lower goal modes → no protected groups → monitor-only).

### Eval/video + desync verification (2026-08-08, run `mjx-evalvid-check`)

The two pilot runs (`mjx-walk-scratch`, `mjx-walk-lowent-dr03`) launched
minutes BEFORE the `_BgEval` wiring and episode-desync fix landed on the
pods, so they show sawtooth charts and no evals/videos — code-age, not a
bug. W&B run `9jdhkouu` (12M steps, champ warm-start, DR 0.3, eval-every
1M / video-every 3M) verified the current stack end to end: 10 periodic
per-mode C-env evals, 6 video reels (incl. untrained + final), skip-if-busy
exercised, `env/walk_vel_err` flat where the lockstep run sawtoothed
0.020–0.036, 14.6k env-steps/s including eval overhead.

## Files

- `mjx_backend.py` — backend: tick stepper, profile port, state surgery
- `mjx_host.py` — host halves shared by both vec envs (torch/jax-free,
  importable by worker processes)
- `mjx_vec_env.py` — SB3 `MjxVecEnv` (phase 2, in-process reference)
- `mjx_sharded_vec_env.py` — `MjxShardedVecEnv`, the throughput path
  (host halves on worker processes; bit-identical to the reference)
- `bench_mjx.py` — C vs MJX throughput
- `requirements-mjx.txt` — optional deps (mujoco-mjx pin MUST match mujoco)
- `coreweave_pod_gpu.yaml` — H200 bench pod
- `coreweave_pod_setup.sh` — `HEXAPOD_MJX=1` installs the JAX stack
- `../tests/test_mjx_parity.py` — physics/profile parity (skips without jax)
- `../tests/test_mjx_vec_env.py` — vec env correctness (skips without jax)

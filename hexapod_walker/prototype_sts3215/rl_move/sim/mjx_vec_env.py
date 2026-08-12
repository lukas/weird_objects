"""MjxVecEnv — SB3 VecEnv running batched MJX physics (phase 2 of the port).

One ``MjxTickStepper`` advances all B robots per control tick on the
accelerator; B lightweight *shim* envs (real ``SimHexapodBalanceEnv``
task subclasses sharing one prepared MjModel) run every host-side half
byte-for-byte: safety, IK, DR sampling, obs, and the whole evolving
reward stack. The shims never step physics — they read a per-env numpy
mirror of the batched tick output (``mjx_host.FakeData``), via the
``_step_begin`` / ``_step_finish`` / ``_reset_begin`` /
``_reset_finalize`` halves that sim_env.py exposes for exactly this.

Resets
------
The C env's reset is a 30-tick physics choreography (place → slip-settle
stiff → slip-settle limp → capture nominal → hold). Running it for ONE
env would stall the whole batch, so:

- All-env resets (startup, pool refills) run the choreography
  SYNCHRONIZED: every env places and settles in lockstep, one batched
  tick per stage tick, with the slip-friction stages using a second
  pre-compiled low-friction model variant.
- Mid-rollout terminations pop a pre-settled state from a per-env POOL
  and inject it into the batch (qpos/qvel/profile/IMU + the env's host
  bookkeeping snapshot) — SB3 gets its auto-reset obs immediately and
  no fake "resetting" transitions pollute the rollout.
- When a pool runs dry the vec env snapshots live device+host state,
  runs one synchronized choreography to mint B fresh entries, and
  restores — costing wall clock but zero rollout transitions.

This class runs all B env halves in ONE thread (~0.076 ms/env/step),
which caps end-to-end throughput at a few k env-steps/s regardless of
device speed. ``mjx_sharded_vec_env.MjxShardedVecEnv`` is the same
machine with the host halves sharded across worker processes — use it
for real training throughput; use this one for small batches, tests,
and as the reference implementation (the sharded env is tested
bit-identical against this one).

DR coverage: actuation + sensing DR (latency, deadband, vel, IMU
mount/bias/noise, encoder/action noise, command drops, start offsets)
is per-env via TickParams; MODEL-field DR (mass, leg geometry,
friction, contact compliance, gravity tilt, kp/kv/torque) is per-world
on device (``model_dr``, on by default when ``randomize=True``) — each
reset replays the C env's exact model prep on a host scratch and
uploads the ``MODEL_DR_FIELDS`` rows.

Limits: ``render()`` is unsupported (eval/video workers build C envs
anyway).
"""
from __future__ import annotations

from typing import Any

import numpy as np
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from .mjx_backend import MODEL_DR_FIELDS, MjxTickStepper, mjx_is_available
from .mjx_host import (
    CommandStub, ModelDrScratch, foot_mu_from_cfg, make_shim_class,
    place_env, prepare_shared_model, push_output_row, restore_env,
    snap_attrs_for, terrain_from_cfg,
    snapshot_env, tp_rows,
)
from .servo_model import N_JOINTS, SimServoParams
from .sim_env import SimHexapodBalanceEnv


class MjxVecEnv(VecEnv):
    """SB3-compatible VecEnv over one batched MJX/Warp physics stepper.

    Parameters mirror the C-env constructor where they overlap;
    ``env_kwargs`` is forwarded to every per-env shim (cfg, randomize,
    dr_scale, episode_seconds, plant_deg, params ...).

    ``impl``: None (mjx default: XLA), or "warp" on CUDA pods —
    see MJX_PORT.md for why warp is the only impl worth training on.
    ``pool_per_env``: pre-settled reset states minted per env at
    startup; refills happen automatically when a pool runs dry.
    """

    def __init__(self, task_cls, n_envs: int,
                 env_kwargs: dict | None = None, *,
                 seed: int = 0, impl: str | None = None, device=None,
                 pool_per_env: int = 2, desync_episodes: bool = True,
                 model_dr: bool | None = None,
                 mjx_iterations: int = 8, mjx_ls_iterations: int = 8,
                 nacon_per_env: int = 64, njmax: int = 256):
        if not mjx_is_available():
            raise RuntimeError(
                "mujoco-mjx / jax not installed — "
                "pip install -r rl_move/sim/requirements-mjx.txt")
        import jax
        import mujoco
        self._jax = jax

        env_kwargs = dict(env_kwargs or {})
        params = env_kwargs.setdefault("params", SimServoParams.load())

        t_amp, t_seed = terrain_from_cfg(env_kwargs.get("cfg"))
        self.mj_model = prepare_shared_model(
            params, iterations=mjx_iterations,
            ls_iterations=mjx_ls_iterations,
            terrain_amp=t_amp, terrain_seed=t_seed,
            foot_mu=foot_mu_from_cfg(env_kwargs.get("cfg")))

        # Model-field DR (mass/geometry/friction/gravity/gains): per-world
        # device model rows, refreshed from each env's _ep_rand draw at
        # every reset. Default: on exactly when the envs randomize.
        self._model_dr = (bool(env_kwargs.get("randomize", False))
                          if model_dr is None else bool(model_dr))

        shim_cls = make_shim_class(task_cls)
        self.envs = [shim_cls(self.mj_model, seed=seed + i, **env_kwargs)
                     for i in range(n_envs)]
        e0 = self.envs[0]
        super().__init__(int(n_envs), e0.observation_space, e0.action_space)

        self.stepper = MjxTickStepper(
            self.mj_model, n_envs, params=params, device=device, impl=impl,
            nacon_per_env=nacon_per_env, njmax=njmax,
            slip_mu=SimHexapodBalanceEnv.SLIP_MU,
            model_dr=self._model_dr)
        # Scratch MjData lent to each shim during _place_at_plant (the
        # penetration-lift loop needs real contacts; one is enough since
        # placement is sequential and mj_resetData's inside).
        self._scratch_data = mujoco.MjData(self.mj_model)
        self._dr_scratch = (ModelDrScratch(self.mj_model, params)
                            if self._model_dr else None)

        self._snap_attrs = snap_attrs_for(task_cls)
        self._pool_per_env = int(pool_per_env)
        self._pool: list[list[dict]] = [[] for _ in range(n_envs)]
        self._tp_host: dict | None = None   # host copy of per-env TickParams
        self._dr_host: dict | None = None   # host copy of model-DR rows
        self._actions = None
        self._started = False
        # Batched resets start every env at episode-phase 0 in LOCKSTEP,
        # so all B envs truncate on the same tick forever: rollout means
        # sweep the episode phase (sawtooth W&B charts) and every PPO
        # update sees one correlated phase slice. Standard fix (Isaac
        # Gym-style): force-truncate each env's FIRST episode at a
        # random horizon; afterwards the fleet stays desynchronized.
        self._desync = bool(desync_episodes)
        self._seed0 = int(seed)
        self._ep_steps = np.zeros(n_envs, dtype=np.int64)
        self._desync_left: np.ndarray | None = None

    # ------------------------------------------------------------------
    # synchronized reset choreography (mirrors SimHexapodBalanceEnv.reset)
    # ------------------------------------------------------------------

    def _push_output(self, env, i: int, outs) -> None:
        push_output_row(env, self.stepper.adr.pad_bids, outs, i)

    def _choreography(self) -> list[dict]:
        """Run the full synchronized reset for ALL envs. Leaves every
        env + the device batch in fresh-episode state and returns one
        pool entry per env (self-contained: device arrays + host
        bookkeeping snapshot + first obs)."""
        B = self.num_envs
        st = self.stepper
        dt = self.envs[0].dt

        # Pre-physics half + placement (host, sequential, cheap).
        qpos0 = np.zeros((B, self.mj_model.nq))
        q_starts = np.zeros((B, N_JOINTS))
        tp = {k: [] for k in ("latency_s", "deadband", "vel_max", "imu_off")}
        dr_rows: dict[str, list] | None = (
            {k: [] for k in MODEL_DR_FIELDS} if self._model_dr else None)
        for i, env in enumerate(self.envs):
            q_start = env._reset_begin(None)
            q_starts[i] = q_start
            row = tp_rows(env)
            for k in tp:
                tp[k].append(row[k])
            dr_model = None
            if self._model_dr:
                rows = self._dr_scratch.rows_for(env)
                for k in dr_rows:
                    dr_rows[k].append(rows[k])
                dr_model = self._dr_scratch.model   # DR'd for placement
            qpos0[i] = place_env(env, self._scratch_data, q_start,
                                 model=dr_model)
            env._profile = CommandStub()
            env._cmd = q_start.copy()
        self._tp_host = {k: np.stack(v) for k, v in tp.items()}
        if self._model_dr:
            self._dr_host = {k: np.stack(v) for k, v in dr_rows.items()}
            st.set_model_fields(self._dr_host)   # before settle physics

        st.reset_envs(qpos0, np.zeros((B, self.mj_model.nv)), q_starts,
                      dt_ctrl=dt, tick_params=self._tp_host)

        # Settle stages — tick counts computed exactly like _settle().
        hold = st.make_command(np.zeros((B, N_JOINTS)), speed_deg_s=0.0,
                               acc_units=0.0, valid=False)
        out = None
        for _ in range(int(round(0.4 / dt))):          # stiff, slip feet
            out = st.tick(hold, slip=True)
        for _ in range(int(round(0.5 / dt))):          # limp, slip feet
            out = st.tick(hold, limp=True, slip=True)

        # Capture the passive equilibrium as nominal; hold it.
        outs = self._jax.device_get(out)
        q_nom = np.asarray(outs.q, dtype=float)        # (B, 18)
        st.reset_profiles(q_nom)
        for i, env in enumerate(self.envs):
            env._cmd = q_nom[i].copy()
        for _ in range(int(round(0.3 / dt))):          # stiff, normal feet
            out = st.tick(hold)
        outs = self._jax.device_get(out)

        entries = []
        for i, env in enumerate(self.envs):
            env._q_nom = q_nom[i].copy()
            self._push_output(env, i, outs)
            obs, info = env._reset_finalize()
            entries.append(dict(
                qpos=np.asarray(outs.qpos_all[i], dtype=float),
                qvel=np.asarray(outs.qvel_all[i], dtype=float),
                q_nom=q_nom[i].copy(),
                obs=obs, info=info,
                host=snapshot_env(env, self._snap_attrs),
                tp_row={k: self._tp_host[k][i].copy()
                        for k in self._tp_host},
                dr_row=({k: self._dr_host[k][i].copy()
                         for k in self._dr_host}
                        if self._model_dr else None),
            ))
        return entries

    def _refill_pools(self) -> None:
        """Mint one fresh pool entry per env without disturbing the live
        rollout: snapshot device + host state (incl. the batched model's
        DR rows), run one synchronized choreography, restore. Costs
        wall clock only."""
        dev = self.stepper.snapshot_state()
        host = [snapshot_env(env, self._snap_attrs) for env in self.envs]
        stubs = [env._profile for env in self.envs]
        tp_live = ({k: v.copy() for k, v in self._tp_host.items()}
                   if self._tp_host else None)
        dr_live = ({k: v.copy() for k, v in self._dr_host.items()}
                   if self._model_dr and self._dr_host else None)
        entries = self._choreography()
        for i, e in enumerate(entries):
            self._pool[i].append(e)
        for env, h, s in zip(self.envs, host, stubs):
            restore_env(env, h)
            env._profile = s
        self._tp_host = tp_live
        self._dr_host = dr_live
        self.stepper.restore_state(dev)

    def _pop_resets(self, idx: list[int]) -> list[tuple]:
        """Instant pooled reset for terminated envs ``idx``: restore host
        bookkeeping, inject device state, return (obs, info) per env."""
        for i in idx:
            if not self._pool[i]:
                self._refill_pools()
                break
        res, qpos, qvel, q_nom = [], [], [], []
        dr_fields = ({k: [] for k in MODEL_DR_FIELDS}
                     if self._model_dr else None)
        for i in idx:
            e = self._pool[i].pop()
            env = self.envs[i]
            restore_env(env, e["host"])
            env._profile = CommandStub()
            env._episode += 1
            for k, v in e["tp_row"].items():
                self._tp_host[k][i] = v
            if self._model_dr:
                for k, v in e["dr_row"].items():
                    self._dr_host[k][i] = v
                    dr_fields[k].append(v)
            qpos.append(e["qpos"])
            qvel.append(e["qvel"])
            q_nom.append(e["q_nom"])
            res.append((e["obs"], e["info"]))
        self.stepper.set_tick_params(self._tp_host, dt_ctrl=self.envs[0].dt)
        if self._model_dr:
            self.stepper.set_model_fields(
                {k: np.stack(v) for k, v in dr_fields.items()}, idx=idx)
        self.stepper.inject_env_states(np.asarray(idx, dtype=int),
                                       np.asarray(qpos), np.asarray(qvel),
                                       np.asarray(q_nom))
        return res

    # ------------------------------------------------------------------
    # SB3 VecEnv API
    # ------------------------------------------------------------------

    def reset(self):
        for _ in range(self._pool_per_env):
            entries = self._choreography()
            for i, e in enumerate(entries):
                self._pool[i].append(e)
        entries = self._choreography()     # live episode state stays live
        self._started = True
        self._ep_steps[:] = 0
        if self._desync:
            n = self.envs[0].episode_steps
            rng = np.random.default_rng(self._seed0 ^ 0xD35C)
            self._desync_left = rng.integers(1, n + 1, size=self.num_envs)
        self.reset_infos = [e["info"] for e in entries]
        return np.stack([e["obs"] for e in entries])

    def step_async(self, actions) -> None:
        self._actions = np.asarray(actions)

    def step_wait(self):
        assert self._started, "call reset() first"
        B = self.num_envs
        results: list[Any] = [None] * B
        ctxs: list[Any] = [None] * B
        cmd_q = np.zeros((B, N_JOINTS), dtype=np.float32)
        speed = np.zeros((B, 1), dtype=np.float32)
        acc = np.zeros((B, 1), dtype=np.float32)
        valid = np.zeros(B, dtype=bool)
        push = np.zeros(B, dtype=np.float32)

        for i, env in enumerate(self.envs):
            stub = env._profile
            stub.pending = None
            early, ctx = env._step_begin(self._actions[i])
            if early is not None:
                # Rejected action: done WITHOUT physics (C semantics).
                # The batch still ticks; this env's slot is about to be
                # overwritten by its pooled reset, so the extra tick on
                # stale state is unobserved.
                results[i] = env._post_step(early)
            else:
                ctxs[i] = ctx
                if stub.pending is not None:
                    q, s, a = stub.pending
                    cmd_q[i], speed[i], acc[i] = q, s, a
                    valid[i] = True
                # dr.walk_push_* takeoff torque: the shim computes the
                # per-tick half-sine (same pre-_step_finish clock as the
                # C env's _advance); the stepper applies it as xfrc.
                push[i] = env._walk_push_torque_nm()

        out = self.stepper.tick(self.stepper.make_command(
            cmd_q, speed_deg_s=speed, acc_units=acc, valid=valid),
            push_nm=push)
        outs = self._jax.device_get(out)

        for i, env in enumerate(self.envs):
            if results[i] is None:
                self._push_output(env, i, outs)
                results[i] = env._post_step(env._step_finish(ctxs[i]))

        self._ep_steps += 1
        if self._desync_left is not None:
            for i in range(B):
                r = results[i]
                if not (r[2] or r[3]) \
                        and self._ep_steps[i] >= self._desync_left[i]:
                    results[i] = (r[0], r[1], r[2], True, r[4])

        obs = [r[0] for r in results]
        rews = np.array([r[1] for r in results], dtype=np.float32)
        dones = np.array([r[2] or r[3] for r in results], dtype=bool)
        infos = [dict(r[4]) for r in results]

        done_idx = [i for i in range(B) if dones[i]]
        if done_idx:
            popped = self._pop_resets(done_idx)
            for i, (obs_new, info_new) in zip(done_idx, popped):
                term, trunc = results[i][2], results[i][3]
                infos[i]["terminal_observation"] = obs[i]
                infos[i]["TimeLimit.truncated"] = bool(trunc and not term)
                self.reset_infos[i] = info_new
                obs[i] = obs_new
                self._ep_steps[i] = 0
                if self._desync_left is not None:
                    # One staggered episode is enough; the env's own
                    # horizon keeps the offset from here on.
                    self._desync_left[i] = np.iinfo(np.int64).max
        return np.stack(obs), rews, dones, infos

    def close(self) -> None:
        for env in self.envs:
            env.close()

    # -- attribute plumbing (train_ppo_sim callbacks use these) ---------

    def _target_envs(self, indices):
        if indices is None:
            return self.envs
        if isinstance(indices, int):
            indices = [indices]
        return [self.envs[i] for i in indices]

    def get_attr(self, attr_name, indices=None):
        return [getattr(env, attr_name) for env in self._target_envs(indices)]

    def set_attr(self, attr_name, value, indices=None):
        for env in self._target_envs(indices):
            setattr(env, attr_name, value)

    def env_method(self, method_name, *args, indices=None, **kwargs):
        return [getattr(env, method_name)(*args, **kwargs)
                for env in self._target_envs(indices)]

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False for _ in self._target_envs(indices)]

    def seed(self, seed=None):
        for i, env in enumerate(self.envs):
            env.rng = np.random.default_rng(
                None if seed is None else seed + i)
        return [seed] * self.num_envs

    def get_images(self):
        raise NotImplementedError("MjxVecEnv does not render")

    @property
    def unwrapped(self):
        return self

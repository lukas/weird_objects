"""MjxShardedVecEnv — MjxVecEnv with the host halves on worker processes.

The measured bottleneck of the hybrid MJX design is NOT the GPU: Warp
steps 4096 robots in ~68 ms while the per-env Python halves (safety,
IK, reward, obs) cost ~0.076 ms/env/step and run sequentially — one
host thread caps end-to-end throughput at a few k env-steps/s with the
H200 ~80% idle (MJX_PORT.md phase-2 item 6).

This class is the same machine as ``mjx_vec_env.MjxVecEnv`` — same shim
envs, same synchronized reset choreography, same pooled mid-rollout
resets — but the B env objects live in N worker PROCESSES (spawn), each
running its shard's halves in parallel while the parent owns the
``MjxTickStepper``. Per-tick traffic flows through shared-memory numpy
arrays (actions, commands, tick outputs, obs/rew/done); only the info
dicts travel by pipe. The host bound becomes ~0.076 ms × B / N, so on a
pod with W spare cores the Warp throughput (60-74k env-steps/s) is the
binding constraint again.

Correctness: tested BIT-IDENTICAL to the in-process ``MjxVecEnv``
(obs/reward/done streams, including pooled-reset pops) in
``tests/test_mjx_vec_env.py`` — the in-process class is the reference
implementation, this one is the throughput path.

Split of responsibilities:

- worker: `_reset_begin` + placement, `_reset_finalize`, `_step_begin`,
  `_step_finish`/`_post_step`, host-side pool snapshots (attr dicts),
  get/set_attr + env_method plumbing.
- parent: device ticks, settle stages, device-side pool entries
  (qpos/qvel/q_nom/TickParams rows), state injection, SB3 API.
"""
from __future__ import annotations

import multiprocessing as mp
import traceback
from dataclasses import dataclass
from multiprocessing import shared_memory
from types import SimpleNamespace

import numpy as np
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from .mjx_backend import (
    MODEL_DR_FIELDS, MjxTickStepper, _model_addrs, mjx_is_available,
)
from .servo_model import N_JOINTS, SimServoParams
from .sim_env import SimHexapodBalanceEnv

_TP_KEYS = ("latency_s", "deadband", "vel_max", "imu_off")


def _dr_field_shapes(model) -> dict[str, tuple]:
    """Per-world row shape of every MODEL_DR_FIELDS entry."""
    out = {}
    for f in MODEL_DR_FIELDS:
        v = model.opt.gravity if f == "opt.gravity" else getattr(model, f)
        out[f] = tuple(np.asarray(v).shape)
    return out


# ---------------------------------------------------------------------------
# shared-memory layout
# ---------------------------------------------------------------------------


@dataclass
class _ShmSpec:
    name: str
    shape: tuple
    dtype: str


def _shm_layout(B: int, n_act: int, n_obs: int, nq: int, nv: int,
                ns: int, tag: str,
                dr_shapes: dict[str, tuple] | None = None
                ) -> dict[str, _ShmSpec]:
    def s(key, shape, dtype):
        return _ShmSpec(f"hexmjx-{tag}-{key}", shape, dtype)
    layout = {
        "actions": s("act", (B, n_act), "float32"),
        "cmd_q": s("cmdq", (B, N_JOINTS), "float32"),
        "cmd_sp": s("cmdsp", (B, 1), "float32"),
        "cmd_ac": s("cmdac", (B, 1), "float32"),
        "cmd_valid": s("cmdok", (B,), "bool"),
        "obs": s("obs", (B, n_obs), "float32"),
        "rew": s("rew", (B,), "float32"),
        "term": s("term", (B,), "bool"),
        "trunc": s("trunc", (B,), "bool"),
        "qpos0": s("qpos0", (B, nq), "float64"),
        "q_start": s("qstart", (B, N_JOINTS), "float64"),
        "q_nom": s("qnom", (B, N_JOINTS), "float64"),
        "tp_latency_s": s("tplat", (B, N_JOINTS), "float64"),
        "tp_deadband": s("tpdbd", (B, N_JOINTS), "float64"),
        "tp_vel_max": s("tpvel", (B, N_JOINTS), "float64"),
        "tp_imu_off": s("tpimu", (B, 3), "float64"),
        # TickOutput mirror (device dtypes = float32, matching what the
        # in-process class sees from jax.device_get).
        "o_qpos_all": s("oqpos", (B, nq), "float32"),
        "o_qvel_all": s("oqvel", (B, nv), "float32"),
        "o_qfrc_actuator": s("oqfrc", (B, N_JOINTS), "float32"),
        "o_chassis_xpos": s("ocxp", (B, 3), "float32"),
        "o_chassis_xmat": s("ocxm", (B, 3, 3), "float32"),
        "o_subtree_com": s("ocom", (B, 3), "float32"),
        "o_pad_xpos": s("opad", (B, 6, 3), "float32"),
        "o_f_imu": s("ofimu", (B, 3), "float32"),
        "o_gyro": s("ogyro", (B, 3), "float32"),
        "o_sensordata": s("osens", (B, ns), "float32"),
        "o_time": s("otime", (B,), "float32"),
    }
    if dr_shapes:
        # Model-DR rows (worker computes → parent uploads to device).
        for i, (field, shape) in enumerate(sorted(dr_shapes.items())):
            layout[f"dr_{field}"] = s(f"dr{i}", (B,) + shape, "float64")
    return layout


class _ShmArrays:
    """Attach/create the numpy views over the shared blocks."""

    def __init__(self, layout: dict[str, _ShmSpec], create: bool):
        self._blocks = {}
        self.arr: dict[str, np.ndarray] = {}
        for key, spec in layout.items():
            nbytes = int(np.prod(spec.shape)) * np.dtype(spec.dtype).itemsize
            if create:
                try:  # stale block from a crashed run
                    old = shared_memory.SharedMemory(name=spec.name)
                    old.close()
                    old.unlink()
                except FileNotFoundError:
                    pass
                blk = shared_memory.SharedMemory(
                    name=spec.name, create=True, size=nbytes)
            else:
                blk = shared_memory.SharedMemory(name=spec.name)
            self._blocks[key] = blk
            self.arr[key] = np.ndarray(spec.shape, dtype=spec.dtype,
                                       buffer=blk.buf)
        self._owner = create

    def __getitem__(self, key) -> np.ndarray:
        return self.arr[key]

    def outs_view(self) -> SimpleNamespace:
        """Attribute view matching TickOutput field names (global rows)."""
        return SimpleNamespace(**{
            k[2:]: self.arr[k] for k in self.arr if k.startswith("o_")})

    def close(self):
        for blk in self._blocks.values():
            blk.close()
            if self._owner:
                try:
                    blk.unlink()
                except FileNotFoundError:
                    pass


# ---------------------------------------------------------------------------
# worker process
# ---------------------------------------------------------------------------


def _worker_main(conn, layout, task_cls, env_kwargs, lo, hi, seed,
                 mjx_iterations, mjx_ls_iterations, model_dr=False):
    """Owns envs [lo, hi); reads/writes GLOBAL rows of the shm arrays."""
    try:
        from .mjx_host import (
            CommandStub, ModelDrScratch, make_shim_class, place_env,
            prepare_shared_model, push_output_row, restore_env,
            snap_attrs_for, snapshot_env, tp_rows,
        )
        import mujoco

        env_kwargs = dict(env_kwargs)
        params = env_kwargs.setdefault("params", SimServoParams.load())
        model = prepare_shared_model(params, iterations=mjx_iterations,
                                     ls_iterations=mjx_ls_iterations)
        pad_bids = _model_addrs(model).pad_bids
        scratch = mujoco.MjData(model)
        dr_scratch = ModelDrScratch(model, params) if model_dr else None
        shim_cls = make_shim_class(task_cls)
        envs = [shim_cls(model, seed=seed + g, **env_kwargs)
                for g in range(lo, hi)]
        attrs = snap_attrs_for(task_cls)

        shm = _ShmArrays(layout, create=False)
        outs = shm.outs_view()
        pool: list[list[dict]] = [[] for _ in envs]
        early: dict[int, tuple] = {}
        ctxs: dict[int, tuple] = {}
        saved = None
        conn.send(("ready", None))

        while True:
            cmd, *args = conn.recv()
            if cmd == "close":
                for env in envs:
                    env.close()
                conn.send(("ok", None))
                break

            elif cmd == "reset_begin":
                for k, env in enumerate(envs):
                    g = lo + k
                    q_start = env._reset_begin(None)
                    shm["q_start"][g] = q_start
                    row = tp_rows(env)
                    for key in _TP_KEYS:
                        shm[f"tp_{key}"][g] = row[key]
                    dr_model = None
                    if model_dr:
                        rows = dr_scratch.rows_for(env)
                        for f, v in rows.items():
                            shm[f"dr_{f}"][g] = v
                        dr_model = dr_scratch.model
                    shm["qpos0"][g] = place_env(env, scratch, q_start,
                                                model=dr_model)
                    env._profile = CommandStub()
                    env._cmd = q_start.copy()
                conn.send(("ok", None))

            elif cmd == "reset_mid":
                for k, env in enumerate(envs):
                    env._cmd = shm["q_nom"][lo + k].copy()
                conn.send(("ok", None))

            elif cmd == "reset_finalize":
                mode = args[0]
                infos = []
                for k, env in enumerate(envs):
                    g = lo + k
                    env._q_nom = shm["q_nom"][g].copy()
                    push_output_row(env, pad_bids, outs, g)
                    obs, info = env._reset_finalize()
                    shm["obs"][g] = obs
                    if mode == "pool":
                        pool[k].append(dict(
                            host=snapshot_env(env, attrs),
                            obs=obs.copy(), info=info))
                    else:
                        infos.append(info)
                conn.send(("ok", infos))

            elif cmd == "host_save":
                saved = [(snapshot_env(env, attrs), env._profile)
                         for env in envs]
                conn.send(("ok", None))

            elif cmd == "host_restore":
                for env, (snap, stub) in zip(envs, saved):
                    restore_env(env, snap)
                    env._profile = stub
                saved = None
                conn.send(("ok", None))

            elif cmd == "step_begin":
                early.clear()
                ctxs.clear()
                for k, env in enumerate(envs):
                    g = lo + k
                    stub = env._profile
                    stub.pending = None
                    e, ctx = env._step_begin(shm["actions"][g])
                    shm["cmd_valid"][g] = False
                    if e is not None:
                        early[k] = env._post_step(e)
                    else:
                        ctxs[k] = ctx
                        if stub.pending is not None:
                            q, s, a = stub.pending
                            shm["cmd_q"][g] = q
                            shm["cmd_sp"][g] = s
                            shm["cmd_ac"][g] = a
                            shm["cmd_valid"][g] = True
                conn.send(("ok", None))

            elif cmd == "step_finish":
                infos = []
                for k, env in enumerate(envs):
                    g = lo + k
                    if k in early:
                        res = early[k]
                    else:
                        push_output_row(env, pad_bids, outs, g)
                        res = env._post_step(env._step_finish(ctxs[k]))
                    obs, rew, term, trunc, info = res
                    shm["obs"][g] = obs
                    shm["rew"][g] = rew
                    shm["term"][g] = term
                    shm["trunc"][g] = trunc
                    infos.append(dict(info))
                conn.send(("ok", infos))

            elif cmd == "pop":
                local_idx = args[0]
                res = []
                for k in local_idx:
                    e = pool[k].pop()
                    env = envs[k]
                    restore_env(env, e["host"])
                    env._profile = CommandStub()
                    env._episode += 1
                    shm["obs"][lo + k] = e["obs"]
                    res.append(e["info"])
                conn.send(("ok", res))

            elif cmd == "get_attr":
                name, idx = args
                conn.send(("ok", [getattr(envs[k], name)
                                  for k in (idx if idx is not None
                                            else range(len(envs)))]))

            elif cmd == "set_attr":
                name, value, idx = args
                for k in (idx if idx is not None else range(len(envs))):
                    setattr(envs[k], name, value)
                conn.send(("ok", None))

            elif cmd == "env_method":
                name, margs, mkw, idx = args
                conn.send(("ok", [getattr(envs[k], name)(*margs, **mkw)
                                  for k in (idx if idx is not None
                                            else range(len(envs)))]))

            elif cmd == "seed":
                base = args[0]
                for k, env in enumerate(envs):
                    env.rng = np.random.default_rng(
                        None if base is None else base + lo + k)
                conn.send(("ok", None))

            else:
                raise RuntimeError(f"unknown command {cmd!r}")
        shm.close()
    except Exception:
        try:
            conn.send(("error", traceback.format_exc()))
        except Exception:
            pass


# ---------------------------------------------------------------------------
# parent
# ---------------------------------------------------------------------------


class MjxShardedVecEnv(VecEnv):
    """Sharded-host twin of ``MjxVecEnv`` — same constructor plus
    ``host_workers`` (worker process count; size it to the pod's SPARE
    cores, e.g. 24-48 on a dedicated node, 4-8 next to the campaign)."""

    def __init__(self, task_cls, n_envs: int,
                 env_kwargs: dict | None = None, *,
                 host_workers: int = 8,
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
        from .mjx_host import prepare_shared_model
        self._jax = jax

        env_kwargs = dict(env_kwargs or {})
        params = env_kwargs.setdefault("params", SimServoParams.load())
        self.mj_model = prepare_shared_model(
            params, iterations=mjx_iterations,
            ls_iterations=mjx_ls_iterations)
        # Model-field DR — same default rule as MjxVecEnv.
        self._model_dr = (bool(env_kwargs.get("randomize", False))
                          if model_dr is None else bool(model_dr))
        self._dr_shapes = (_dr_field_shapes(self.mj_model)
                           if self._model_dr else None)

        # One throwaway probe env (parent-side) for spaces + dt; the
        # real envs live in the workers.
        probe = task_cls(model=self.mj_model, seed=seed, **env_kwargs)
        obs_space, act_space = probe.observation_space, probe.action_space
        self._dt = probe.dt
        self._episode_steps = probe.episode_steps
        probe.close()

        B = int(n_envs)
        n_workers = max(1, min(int(host_workers), B))
        layout = _shm_layout(
            B, act_space.shape[0], obs_space.shape[0], self.mj_model.nq,
            self.mj_model.nv, self.mj_model.nsensordata,
            tag=f"{seed}-{np.random.randint(1 << 30)}",
            dr_shapes=self._dr_shapes)
        self._shm = _ShmArrays(layout, create=True)

        ctx = mp.get_context("spawn")
        bounds = np.linspace(0, B, n_workers + 1).astype(int)
        self._ranges = [(int(bounds[w]), int(bounds[w + 1]))
                        for w in range(n_workers)]
        self._conns, self._procs = [], []
        for lo, hi in self._ranges:
            pc, cc = ctx.Pipe()
            proc = ctx.Process(
                target=_worker_main,
                args=(cc, layout, task_cls, env_kwargs, lo, hi, seed,
                      mjx_iterations, mjx_ls_iterations, self._model_dr),
                daemon=True)
            proc.start()
            cc.close()
            self._conns.append(pc)
            self._procs.append(proc)
        for pc in self._conns:
            self._expect(pc, "ready")

        super().__init__(B, obs_space, act_space)

        self.stepper = MjxTickStepper(
            self.mj_model, B, params=params, device=device, impl=impl,
            nacon_per_env=nacon_per_env, njmax=njmax,
            slip_mu=SimHexapodBalanceEnv.SLIP_MU,
            model_dr=self._model_dr)
        self._pool_per_env = int(pool_per_env)
        # Device half of the pool entries (host half lives worker-side,
        # pushed/popped in lockstep — both LIFO).
        self._pool_dev: list[list[dict]] = [[] for _ in range(B)]
        self._started = False
        # First-episode phase stagger — see MjxVecEnv (same seed formula
        # so the two classes stay bit-identical).
        self._desync = bool(desync_episodes)
        self._seed0 = int(seed)
        self._ep_steps = np.zeros(B, dtype=np.int64)
        self._desync_left: np.ndarray | None = None

    # -- plumbing ------------------------------------------------------

    @staticmethod
    def _expect(conn, want="ok"):
        kind, payload = conn.recv()
        if kind == "error":
            raise RuntimeError(f"MjxShardedVecEnv worker died:\n{payload}")
        if kind != want:
            raise RuntimeError(f"protocol error: {kind!r} != {want!r}")
        return payload

    def _broadcast(self, *msg) -> list:
        for pc in self._conns:
            pc.send(msg)
        return [self._expect(pc) for pc in self._conns]

    def _tp_dict(self) -> dict:
        return {k: self._shm[f"tp_{k}"].copy() for k in _TP_KEYS}

    def _copy_outs(self, out) -> None:
        outs = self._jax.device_get(out)
        for field in ("qpos_all", "qvel_all", "qfrc_actuator",
                      "chassis_xpos", "chassis_xmat", "subtree_com",
                      "pad_xpos", "f_imu", "gyro", "sensordata", "time"):
            np.copyto(self._shm[f"o_{field}"], getattr(outs, field))

    # -- choreography ----------------------------------------------------

    def _dr_dict(self) -> dict:
        return {f: self._shm[f"dr_{f}"].copy() for f in MODEL_DR_FIELDS}

    def _choreography(self, mode: str) -> list:
        st = self.stepper
        B, dt = self.num_envs, self._dt
        self._broadcast("reset_begin")
        if self._model_dr:
            st.set_model_fields(self._dr_dict())   # before settle physics
        st.reset_envs(self._shm["qpos0"].copy(),
                      np.zeros((B, self.mj_model.nv)),
                      self._shm["q_start"].copy(), dt_ctrl=dt,
                      tick_params=self._tp_dict())
        hold = st.make_command(np.zeros((B, N_JOINTS)), speed_deg_s=0.0,
                               acc_units=0.0, valid=False)
        out = None
        for _ in range(int(round(0.4 / dt))):          # stiff, slip feet
            out = st.tick(hold, slip=True)
        for _ in range(int(round(0.5 / dt))):          # limp, slip feet
            out = st.tick(hold, limp=True, slip=True)
        q_nom = np.asarray(self._jax.device_get(out).q, dtype=float)
        np.copyto(self._shm["q_nom"], q_nom)
        st.reset_profiles(q_nom)
        self._broadcast("reset_mid")
        for _ in range(int(round(0.3 / dt))):          # stiff, normal feet
            out = st.tick(hold)
        self._copy_outs(out)
        infos = self._broadcast("reset_finalize", mode)
        if mode == "pool":
            for i in range(B):
                self._pool_dev[i].append(dict(
                    qpos=self._shm["o_qpos_all"][i].astype(float),
                    qvel=self._shm["o_qvel_all"][i].astype(float),
                    q_nom=q_nom[i].copy(),
                    tp_row={k: self._shm[f"tp_{k}"][i].copy()
                            for k in _TP_KEYS},
                    dr_row=({f: self._shm[f"dr_{f}"][i].copy()
                             for f in MODEL_DR_FIELDS}
                            if self._model_dr else None)))
            return []
        return [info for chunk in infos for info in chunk]

    def _refill_pools(self) -> None:
        dev = self.stepper.snapshot_state()
        tp_live = self._tp_dict()
        dr_live = self._dr_dict() if self._model_dr else None
        self._broadcast("host_save")
        self._choreography("pool")
        self._broadcast("host_restore")
        for k in _TP_KEYS:
            np.copyto(self._shm[f"tp_{k}"], tp_live[k])
        if self._model_dr:
            for f in MODEL_DR_FIELDS:
                np.copyto(self._shm[f"dr_{f}"], dr_live[f])
        self.stepper.restore_state(dev)

    def _pop_resets(self, done_idx: list[int]) -> None:
        """Pooled resets for ``done_idx``: workers restore host state and
        write the reset obs rows; parent injects device state."""
        if any(not self._pool_dev[i] for i in done_idx):
            self._refill_pools()
        qpos, qvel, q_nom = [], [], []
        dr_fields = ({f: [] for f in MODEL_DR_FIELDS}
                     if self._model_dr else None)
        for i in done_idx:
            e = self._pool_dev[i].pop()
            for k, v in e["tp_row"].items():
                self._shm[f"tp_{k}"][i] = v
            if self._model_dr:
                for f, v in e["dr_row"].items():
                    self._shm[f"dr_{f}"][i] = v
                    dr_fields[f].append(v)
            qpos.append(e["qpos"])
            qvel.append(e["qvel"])
            q_nom.append(e["q_nom"])
        for w, (pc, (lo, hi)) in enumerate(zip(self._conns, self._ranges)):
            local = [i - lo for i in done_idx if lo <= i < hi]
            if local:
                pc.send(("pop", local))
        for pc, (lo, hi) in zip(self._conns, self._ranges):
            local = [i for i in done_idx if lo <= i < hi]
            if local:
                for i, info in zip(local, self._expect(pc)):
                    self.reset_infos[i] = info
        self.stepper.set_tick_params(self._tp_dict(), dt_ctrl=self._dt)
        if self._model_dr:
            self.stepper.set_model_fields(
                {f: np.stack(v) for f, v in dr_fields.items()},
                idx=done_idx)
        self.stepper.inject_env_states(
            np.asarray(done_idx, dtype=int), np.asarray(qpos),
            np.asarray(qvel), np.asarray(q_nom))

    # -- SB3 VecEnv API --------------------------------------------------

    def reset(self):
        for _ in range(self._pool_per_env):
            self._choreography("pool")
        self.reset_infos = self._choreography("live")
        self._started = True
        self._ep_steps[:] = 0
        if self._desync:
            rng = np.random.default_rng(self._seed0 ^ 0xD35C)
            self._desync_left = rng.integers(
                1, self._episode_steps + 1, size=self.num_envs)
        return self._shm["obs"].copy()

    def step_async(self, actions) -> None:
        np.copyto(self._shm["actions"],
                  np.asarray(actions, dtype=np.float32))

    def step_wait(self):
        assert self._started, "call reset() first"
        self._broadcast("step_begin")
        out = self.stepper.tick(self.stepper.make_command(
            self._shm["cmd_q"].copy(),
            speed_deg_s=self._shm["cmd_sp"].copy(),
            acc_units=self._shm["cmd_ac"].copy(),
            valid=self._shm["cmd_valid"].copy()))
        self._copy_outs(out)
        info_chunks = self._broadcast("step_finish")
        infos = [dict(info) for chunk in info_chunks for info in chunk]

        obs = self._shm["obs"].copy()
        rews = self._shm["rew"].copy()
        term = self._shm["term"].copy()
        trunc = self._shm["trunc"].copy()
        dones = term | trunc

        self._ep_steps += 1
        if self._desync_left is not None:
            force = (~dones) & (self._ep_steps >= self._desync_left)
            trunc |= force
            dones |= force

        done_idx = [i for i in range(self.num_envs) if dones[i]]
        if done_idx:
            self._pop_resets(done_idx)   # workers rewrote those obs rows
            for i in done_idx:
                infos[i]["terminal_observation"] = obs[i].copy()
                infos[i]["TimeLimit.truncated"] = bool(
                    trunc[i] and not term[i])
                obs[i] = self._shm["obs"][i]
                self._ep_steps[i] = 0
                if self._desync_left is not None:
                    self._desync_left[i] = np.iinfo(np.int64).max
        return obs, rews, dones, infos

    def close(self) -> None:
        try:
            self._broadcast("close")
        except Exception:
            pass
        for proc in self._procs:
            proc.join(timeout=10)
            if proc.is_alive():
                proc.terminate()
        self._shm.close()

    # -- attribute plumbing ---------------------------------------------

    def _scatter(self, indices):
        """Global indices → per-worker local index lists (None = all)."""
        if indices is None:
            return [None] * len(self._conns)
        if isinstance(indices, int):
            indices = [indices]
        return [[i - lo for i in indices if lo <= i < hi]
                for lo, hi in self._ranges]

    def _gather(self, cmd, *args, indices=None):
        per_w = self._scatter(indices)
        for pc, loc in zip(self._conns, per_w):
            if loc is None or loc:
                pc.send((cmd, *args, loc))
        out = []
        for pc, loc in zip(self._conns, per_w):
            if loc is None or loc:
                r = self._expect(pc)
                if r is not None:
                    out.extend(r)
        return out

    def get_attr(self, attr_name, indices=None):
        return self._gather("get_attr", attr_name, indices=indices)

    def set_attr(self, attr_name, value, indices=None):
        self._gather("set_attr", attr_name, value, indices=indices)

    def env_method(self, method_name, *args, indices=None, **kwargs):
        return self._gather("env_method", method_name, args, kwargs,
                            indices=indices)

    def env_is_wrapped(self, wrapper_class, indices=None):
        n = self.num_envs if indices is None else (
            1 if isinstance(indices, int) else len(indices))
        return [False] * n

    def seed(self, seed=None):
        self._broadcast("seed", seed)
        return [seed] * self.num_envs

    def get_images(self):
        raise NotImplementedError("MjxShardedVecEnv does not render")

    @property
    def unwrapped(self):
        return self

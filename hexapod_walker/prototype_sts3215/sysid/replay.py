"""Suspended (fixed-base) MuJoCo replay of sysid protocols and traces.

Phase 1/8 of the sysid plan: the robot is on a stand with feet off the
ground, so the sim twin is the fixed-base model with ALL contacts
disabled — gravity loads the hanging legs exactly as on the bench.
Free-base ground replay of full episodes already exists in
``rl_move/sim/replay_trace.py``; this module deliberately covers only
the suspended regime, where actuator/timing effects are isolated from
contact modeling.

Two entry points:

- ``replay_trace(tr, params)`` — feed a HARDWARE trace's recorded
  absolute command stream through ServoProfile + MuJoCo (honoring the
  recorded per-tick durations) and return the sim joint stream aligned
  row-for-row with the hardware samples.
- ``replay_protocol(protocol, params)`` — materialize a protocol the
  same way the on-robot runner does (per-segment home = present pose,
  axis-limit clamping) and produce a SYNTHETIC trace in the identical
  schema. Used to preview experiments and for round-trip tests of the
  fitting pipeline.

CLI::

    uv run python -m sysid.replay --csv sysid/datasets/<dir>/<trace>.csv \
        --servo-params loaded --plot
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from . import PROTO_DIR  # noqa: F401  (bootstraps sys.path)
from sysid_protocol import (  # noqa: E402
    AXIS_LIMITS_DEG, DEFAULT_WRITE_ACC, DEFAULT_WRITE_SPEED, N_JOINTS,
    axis_of, materialize, protocol_hash, start_pose,
)
from rl_move.sim.servo_model import (  # noqa: E402
    LOADED_MODEL_PATH, ServoProfile, SimServoParams, apply_params_to_model,
    build_model, joint_qpos_addrs, position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
COUNTS_TO_DEG_S = 360.0 / 4096.0


def load_params(sel: str) -> SimServoParams:
    """'air' | 'loaded' | 'defaults' | explicit json path."""
    if sel == "air":
        return SimServoParams.load()
    if sel == "loaded":
        return SimServoParams.load(LOADED_MODEL_PATH)
    if sel == "defaults":
        return SimServoParams.defaults()
    p = Path(sel)
    if not p.is_file():
        raise FileNotFoundError(f"servo params {sel!r} not found")
    return SimServoParams.load(p)


class SuspendedSim:
    """Fixed-base hexapod with all contacts disabled (robot on stand)."""

    def __init__(self, params: SimServoParams):
        import mujoco
        self._mujoco = mujoco
        self.model = build_model(fixed_base=True, flat_terrain=True,
                                 mesh_visuals=False)
        # Feet are off the ground on the stand: no contact anywhere.
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0
        apply_params_to_model(self.model, params)
        self.params = params
        self.data = mujoco.MjData(self.model)
        self._qadr = joint_qpos_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self.dt = float(self.model.opt.timestep)

    def set_params(self, params: SimServoParams) -> None:
        apply_params_to_model(self.model, params)
        self.params = params

    def reset(self, q0_rad: np.ndarray, *, settle_s: float = 0.4) -> None:
        mujoco = self._mujoco
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self._qadr] = q0_rad
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self._pos_act] = q0_rad
        mujoco.mj_forward(self.model, self.data)
        for _ in range(int(settle_s / self.dt)):
            self.data.ctrl[self._pos_act] = q0_rad
            mujoco.mj_step(self.model, self.data)

    def q_deg(self) -> np.ndarray:
        return self.data.qpos[self._qadr] * RAD2DEG

    def run_cmds(self, cmd_deg: np.ndarray, tick_dts: np.ndarray, *,
                 speed_deg_s: float, acc_units: float) -> np.ndarray:
        """Stream absolute commands; return q sampled after each tick."""
        mujoco = self._mujoco
        profile = ServoProfile(self.params,
                               self.data.qpos[self._qadr].copy())
        out = np.empty_like(cmd_deg)
        for k in range(len(cmd_deg)):
            profile.command(cmd_deg[k] * DEG2RAD, speed_deg_s=speed_deg_s,
                            acc_units=acc_units)
            per = max(1, int(round(
                min(max(float(tick_dts[k]), 0.02), 0.12) / self.dt)))
            for _ in range(per):
                self.data.ctrl[self._pos_act] = profile.tick(self.dt)
                mujoco.mj_step(self.model, self.data)
            out[k] = self.q_deg()
        return out


def _profile_of(doc: dict) -> tuple[float, float]:
    speed = float(doc.get("write_speed", DEFAULT_WRITE_SPEED))
    acc = float(doc.get("write_acc", DEFAULT_WRITE_ACC))
    return speed * COUNTS_TO_DEG_S, acc


def replay_trace(tr: dict, params: SimServoParams,
                 sim: SuspendedSim | None = None) -> np.ndarray:
    """Sim joint stream (deg) aligned with a hardware trace's rows."""
    sim = sim or SuspendedSim(params)
    sim.set_params(params)
    sim.reset(tr["q"][0] * DEG2RAD)
    t = tr["t_send"]
    dts = np.append(np.diff(t), 1.0 / float(
        (tr.get("protocol") or {}).get("hz", 25.0)))
    speed_deg_s, acc = _profile_of(tr.get("protocol") or {})
    return sim.run_cmds(tr["cmd"], dts, speed_deg_s=speed_deg_s,
                        acc_units=acc)


def replay_protocol(protocol: dict, params: SimServoParams, *,
                    q0_deg: np.ndarray | None = None,
                    noise_deg: float = 0.0,
                    seed: int = 0,
                    sim: SuspendedSim | None = None) -> dict:
    """Run a protocol in sim, mirroring the on-robot runner semantics.

    Per-segment home = the sim's present pose at segment start; relative
    commands are clamped to axis limits, exactly like the runner. The
    result is a synthetic trace dict (``sysid.trace`` schema) — with
    ``noise_deg`` > 0 encoder noise is added, which makes it a stand-in
    hardware trace for pipeline round-trip tests.
    """
    mat = materialize(protocol)
    hz = mat["hz"]
    dt = 1.0 / hz
    ticks = mat["ticks"]
    sim = sim or SuspendedSim(params)
    sim.set_params(params)
    if q0_deg is None:
        # Mirror the runner: it glides to the protocol's start pose
        # before the first segment, so the sim starts settled there.
        sp = start_pose(protocol)
        q0 = (np.asarray(sp, dtype=float) if sp is not None
              else np.zeros(N_JOINTS))
    else:
        q0 = np.asarray(q0_deg, dtype=float)
    sim.reset(q0 * DEG2RAD)
    speed_deg_s, acc = _profile_of(protocol)
    rng = np.random.default_rng(seed)

    profile = ServoProfile(params, sim.data.qpos[sim._qadr].copy())
    mujoco = sim._mujoco
    per = max(1, int(round(dt / sim.dt)))

    n = len(ticks)
    q_out = np.empty((n, N_JOINTS))
    cmd_out = np.empty((n, N_JOINTS))
    seg_out = np.empty(n, dtype=int)
    joint_out = np.empty(n, dtype=int)
    phase_out: list[str] = []
    cur_seg = -1
    home = sim.q_deg().copy()
    for k, tick in enumerate(ticks):
        if tick["seg"] != cur_seg:
            cur_seg = tick["seg"]
            home = sim.q_deg().copy()
            if noise_deg > 0.0:
                home = home + rng.normal(0.0, noise_deg, N_JOINTS)
        cmd_abs = home.copy()
        for j in tick["active"]:
            v = (tick["cmd"][j] if tick["mode"] == "abs"
                 else home[j] + tick["cmd"][j])
            lo, hi = AXIS_LIMITS_DEG[axis_of(j)]
            cmd_abs[j] = min(hi, max(lo, v))
        profile.command(cmd_abs * DEG2RAD, speed_deg_s=speed_deg_s,
                        acc_units=acc)
        for _ in range(per):
            sim.data.ctrl[sim._pos_act] = profile.tick(sim.dt)
            mujoco.mj_step(sim.model, sim.data)
        q = sim.q_deg()
        if noise_deg > 0.0:
            q = q + rng.normal(0.0, noise_deg, N_JOINTS)
        q_out[k] = q
        cmd_out[k] = cmd_abs
        seg_out[k] = cur_seg
        joint_out[k] = (tick["active"][0] if len(tick["active"]) == 1
                        else -1)
        phase_out.append(tick["phase"])

    t = np.arange(n) * dt
    return {
        "name": f"sim_{protocol.get('name', 'protocol')}",
        "t": t, "tick": np.arange(n), "seg": seg_out, "phase": phase_out,
        "joint": joint_out, "t_send": t, "t_recv": t + 0.004,
        "overrun": np.zeros(n, dtype=int),
        "q": q_out, "cmd": cmd_out,
        "cur_a": np.full(n, np.nan),
        "summary": {"ok": True, "mode": "sysid-sim",
                    "protocol_hash": protocol_hash(protocol),
                    "protocol": protocol},
        "protocol": protocol,
    }


def main(argv: list[str] | None = None) -> int:
    from . import trace as trace_mod
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--csv", type=Path, help="hardware sysid trace")
    ap.add_argument("--protocol", type=Path,
                    help="protocol json (sim-only preview)")
    ap.add_argument("--servo-params", default="loaded",
                    help="'air' | 'loaded' | 'defaults' | path")
    ap.add_argument("--out", type=Path, default=None,
                    help="write the sim trace CSV here")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args(argv)
    params = load_params(args.servo_params)

    if args.csv:
        tr = trace_mod.load(args.csv)
        q_sim = replay_trace(tr, params)
        from .metrics import compare_joint_streams
        cmp_ = compare_joint_streams(tr["t"], tr["q"], q_sim)
        print(f"[{tr['name']}] vs sim ({args.servo_params}): {cmp_}")
        if args.plot:
            from .plots import overlay_trace
            out = args.csv.with_suffix(".replay.png")
            overlay_trace(tr, {args.servo_params: q_sim}, out)
    elif args.protocol:
        doc = json.loads(args.protocol.read_text())
        tr = replay_protocol(doc, params)
        print(f"simulated {doc['name']}: {len(tr['t'])} ticks")
        if args.out:
            trace_mod.write(args.out, t=tr["t"], tick=tr["tick"],
                            seg=tr["seg"], phase=tr["phase"],
                            joint=tr["joint"], t_send=tr["t_send"],
                            t_recv=tr["t_recv"], q=tr["q"],
                            cmd=tr["cmd"], summary=tr["summary"])
            print(f"wrote {args.out}")
        if args.plot:
            from .plots import overlay_trace
            out = (args.out or args.protocol).with_suffix(".sim.png")
            overlay_trace(tr, {}, out)
    else:
        ap.error("need --csv or --protocol")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

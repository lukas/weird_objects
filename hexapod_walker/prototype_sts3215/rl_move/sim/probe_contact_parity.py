"""probe_contact_parity.py — warp-vs-C loaded-foot contact-creep parity.

WHY (arch track, 08-13 — STATUS.md WAITING-ON): the noslipphase1 line
measured the SAME checkpoint at walk_loadslip_factor 0.085 under the
MJX/warp training physics vs 0.31 under the trusted C physics, and the
C-env income replay charges that drift ~-280/ep that warp never bills.
Suspect: training runs warp at solver iterations=1/ls=4 (train_ppo_mjx
default; the C twin runs the XML's Newton iterations=50), and the
recorded parity checks (MJX_PORT.md) compared settle poses and episode
RETURNS — never per-tick loaded-foot tangential creep, which is exactly
the quantity the no-slip pricing gates on. If warp under-resolves
friction, every MJX-trained walking line is trained in a world where
skating is cheaper than it really is.

WHAT IT DOES: drives the IDENTICAL open-loop command stream (the
tape-proven TripodGait at plant height, or a plant-hold freeze) through
the IDENTICAL servo-profile pipeline in a matrix of physics configs,
from ONE shared settled start state, and measures physics-level foot
slip directly from the touch sensors + pad body positions:

  c50     C MuJoCo, production XML solver (iterations=50)   <- truth
  c14     C MuJoCo, iterations=1/ls=4 (isolates the knob in C)
  warp14  MJX warp,  iterations=1/ls=4 (the TRAINING physics)
  warp88  MJX warp,  iterations=8/ls=8 (the candidate fix)
  jax88   MJX XLA,   iterations=8/ls=8 (impl cross-check, optional)

No env / reward stack is involved — this is a pure physics A/B. The
loaded-slip definition matches the harness/walk_task: a foot's XY
displacement on tick k counts while its touch force was > 0.5 N on
tick k-1; slip_per_m divides by along-command chassis progress.

    python -m rl_move.sim.probe_contact_parity \
        --configs c50,c14,warp14,warp88 --streams gait,freeze \
        --seconds 12 --out logs/probe_contact_parity/parity.json

Interpretation: if warp14 >> c50 on per-tick loaded creep and warp88
closes most of the gap, the lever is raising training iterations (cost:
bench_mjx --impl warp --mjx-iterations 8). If warp88 does NOT close it,
the gap is the warp implementation itself, not the iteration count.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.servo_model import (  # noqa: E402
    ACC_UNIT_DEG_S2, N_JOINTS, ServoProfile, SimServoParams,
    apply_params_to_model, build_model, joint_qpos_addrs,
    position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
DT_CTRL = 0.04
WALK_PLANT = (20.0, 80.0)
PLANT_RAD = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
SPEED_DEG_S = 400 * 360.0 / 4096.0   # scripted-gait bus write (bench_mjx)
ACC_UNITS = 20.0
TOUCH_ON_N = 0.5                     # walk_task loaded threshold
CMD_V = 0.055                        # trained walk band mid
MEASURE_FROM_S = 2.0                 # hold 1s + ramp 1s, measure after

CONFIGS = {
    # label: (backend, iterations, ls_iterations)
    "c50": ("c", 0, 0),          # 0 = keep the XML's production solver
    "c14": ("c", 1, 4),
    "warp14": ("warp", 1, 4),
    "warp24": ("warp", 2, 4),
    "warp48": ("warp", 4, 8),
    "warp88": ("warp", 8, 8),
    "jax88": ("jax", 8, 8),
}


def _model_for(label: str, params: SimServoParams):
    backend, iters, ls = CONFIGS[label]
    model = build_model(fixed_base=False, flat_terrain=True,
                        mesh_visuals=False,
                        mjx_compat=(backend != "c"))
    apply_params_to_model(model, params)
    if iters > 0:
        model.opt.iterations = int(iters)
        model.opt.ls_iterations = int(ls)
    return model


def _addrs(model):
    import mujoco
    touch = []
    for i in range(6):
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                f"L{i}_foot_t")
        touch.append(model.sensor_adr[sid])
    pads = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"L{i}_pad")
            for i in range(6)]
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    return np.array(touch), np.array(pads), chassis


def settle_start(params: SimServoParams):
    """Shared start state: plant stance settled under the PRODUCTION
    C solver. Every config replays from these exact qpos/qvel."""
    import mujoco
    model = _model_for("c50", params)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[qadr] = PLANT_RAD
    data.qpos[2] = 0.085
    data.ctrl[pos_act] = PLANT_RAD
    mujoco.mj_forward(model, data)
    for _ in range(int(1.2 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    data.qvel[:] = 0.0
    for _ in range(int(0.3 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return data.qpos.copy(), data.qvel.copy(), data.qpos[qadr].copy()


def build_stream(stream: str, ticks: int, vx: float) -> np.ndarray:
    """(T,18) joint goal commands in rad — identical for every config."""
    if stream == "freeze":
        return np.tile(PLANT_RAD, (ticks, 1))
    from tripod_gait import TripodGait
    gait = TripodGait(vx=0.0)
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()
    hold_n = ramp_n = int(round(1.0 / DT_CTRL))
    cmds = np.zeros((ticks, N_JOINTS))
    for k in range(ticks):
        t = k * DT_CTRL
        if k < hold_n:
            v = 0.0
        elif k < hold_n + ramp_n:
            v = vx * (k - hold_n) / ramp_n
        else:
            v = vx
        gait.set_velocity(vx=float(v), vy=0.0, omega=0.0)
        cmds[k] = np.asarray(gait.desired_deg(t)) * DEG2RAD
    return cmds


class TickTrace:
    """Per-tick foot/chassis record + the loaded-slip bookkeeping."""

    def __init__(self, ticks: int):
        self.pad_xy = np.zeros((ticks, 6, 2))
        self.touch = np.zeros((ticks, 6))
        self.chassis = np.zeros((ticks, 3))
        self.rollpitch = np.zeros((ticks, 2))

    def record(self, k, pad_xy, touch, chassis_xpos, xmat):
        self.pad_xy[k] = pad_xy
        self.touch[k] = touch
        self.chassis[k] = chassis_xpos
        # roll/pitch from the body z-axis in world frame
        zx, zy, zz = xmat[0, 2], xmat[1, 2], xmat[2, 2]
        self.rollpitch[k] = (math.degrees(math.atan2(zy, zz)),
                             math.degrees(math.atan2(-zx, math.hypot(zy, zz))))

    def metrics(self, measure_from_s: float) -> dict:
        k0 = max(1, int(round(measure_from_s / DT_CTRL)))
        T = self.pad_xy.shape[0]
        creep_mm, loaded_n = [], 0
        slip_m = 0.0
        for k in range(k0, T):
            loaded_prev = self.touch[k - 1] > TOUCH_ON_N
            d = np.linalg.norm(self.pad_xy[k] - self.pad_xy[k - 1], axis=1)
            for f in range(6):
                if loaded_prev[f]:
                    creep_mm.append(d[f] * 1000.0)
                    slip_m += d[f]
            loaded_n += int(loaded_prev.sum())
        prog = self.chassis[T - 1] - self.chassis[k0]
        prog_x = float(prog[0])
        creep = np.array(creep_mm) if creep_mm else np.zeros(1)
        return {
            "loaded_slip_m": round(slip_m, 4),
            "progress_x_m": round(prog_x, 4),
            "slip_per_m": round(slip_m / max(abs(prog_x), 0.05), 3),
            "creep_mm_per_tick_med": round(float(np.median(creep)), 4),
            "creep_mm_per_tick_p90": round(float(np.percentile(creep, 90)), 4),
            "creep_mm_per_tick_max": round(float(creep.max()), 3),
            "mean_loaded_feet": round(loaded_n / max(T - k0, 1), 2),
            "final_height_m": round(float(self.chassis[T - 1, 2]), 4),
            "max_abs_rollpitch_deg": round(
                float(np.abs(self.rollpitch[k0:]).max()), 1),
            "fell": bool(np.abs(self.rollpitch[k0:]).max() > 45.0
                         or self.chassis[T - 1, 2] < 0.045),
        }


def run_c(label: str, params, qpos0, qvel0, q0, cmds) -> TickTrace:
    import mujoco
    model = _model_for(label, params)
    touch_adr, pad_bids, cb = _addrs(model)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    data = mujoco.MjData(model)
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    mujoco.mj_forward(model, data)
    prof = ServoProfile(params, q0)
    substeps = int(round(DT_CTRL / model.opt.timestep))
    h = model.opt.timestep
    ticks = cmds.shape[0]
    tr = TickTrace(ticks)
    for k in range(ticks):
        prof.command(cmds[k], speed_deg_s=SPEED_DEG_S, acc_units=ACC_UNITS)
        for _ in range(substeps):
            target = prof.tick(h)
            q = data.qpos[qadr]
            err = target - q
            eff = q + np.sign(err) * np.maximum(
                np.abs(err) - prof.deadband_rad, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)
        tr.record(k, data.xpos[pad_bids, :2],
                  data.sensordata[touch_adr],
                  data.xpos[cb].copy(),
                  data.xmat[cb].reshape(3, 3))
    return tr


def run_mjx(label: str, params, qpos0, qvel0, q0, cmds) -> TickTrace:
    from rl_move.sim.mjx_backend import MjxTickStepper
    backend = CONFIGS[label][0]
    model = _model_for(label, params)
    touch_adr, pad_bids, cb = _addrs(model)
    st = MjxTickStepper(model, 1, params=params,
                        impl=(backend if backend != "jax" else "jax"))
    st.reset_envs(qpos0[None], qvel0[None], q0[None], dt_ctrl=DT_CTRL)
    ticks = cmds.shape[0]
    tr = TickTrace(ticks)
    for k in range(ticks):
        cmd = st.make_command(cmds[k][None], speed_deg_s=SPEED_DEG_S,
                              acc_units=ACC_UNITS)
        out = st.tick(cmd)
        sens = np.asarray(out.sensordata)[0]
        tr.record(k, np.asarray(out.pad_xpos)[0, :, :2],
                  sens[touch_adr],
                  np.asarray(out.chassis_xpos)[0],
                  np.asarray(out.chassis_xmat)[0])
    return tr


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--configs", default="c50,c14,warp14,warp88")
    ap.add_argument("--streams", default="gait,freeze")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--vx", type=float, default=CMD_V)
    ap.add_argument("--out", default="logs/probe_contact_parity/parity.json")
    args = ap.parse_args()

    params = SimServoParams.from_cfg(None)
    ticks = int(round(args.seconds / DT_CTRL))
    qpos0, qvel0, q0 = settle_start(params)
    print(f"[parity] params={Path(params.source).name} ticks={ticks} "
          f"settle height={qpos0[2]:.4f} m")

    results: dict = {"params": Path(params.source).name,
                     "seconds": args.seconds, "vx": args.vx,
                     "measure_from_s": MEASURE_FROM_S, "runs": {}}
    for stream in args.streams.split(","):
        cmds = build_stream(stream, ticks, args.vx)
        for label in args.configs.split(","):
            backend = CONFIGS[label][0]
            t0 = time.perf_counter()
            try:
                tr = (run_c if backend == "c" else run_mjx)(
                    label, params, qpos0, qvel0, q0, cmds)
            except Exception as e:  # warp/jax missing on CPU boxes
                print(f"[parity] {stream}/{label}: SKIPPED ({e})")
                continue
            m = tr.metrics(MEASURE_FROM_S if stream == "gait" else 1.0)
            m["wall_s"] = round(time.perf_counter() - t0, 1)
            results["runs"][f"{stream}/{label}"] = m
            print(f"[parity] {stream:6s}/{label:7s} "
                  f"slip {m['loaded_slip_m']:7.4f} m  "
                  f"prog {m['progress_x_m']:7.4f} m  "
                  f"slip/m {m['slip_per_m']:7.3f}  "
                  f"creep med/p90 {m['creep_mm_per_tick_med']:.4f}/"
                  f"{m['creep_mm_per_tick_p90']:.4f} mm  "
                  f"loaded {m['mean_loaded_feet']:.2f}  "
                  f"h {m['final_height_m']:.3f}  "
                  f"rp {m['max_abs_rollpitch_deg']:.1f}deg"
                  f"{'  FELL' if m['fell'] else ''}")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=1))
    print(f"[parity] wrote {out}")


if __name__ == "__main__":
    main()

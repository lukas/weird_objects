"""MuJoCo smoke test for the onboard loaded-vs-hover slip probe.

This mirrors the robot-side command pattern used by
``BenchAPI._run_leg_slip_probe``:

* settle at the learned/default plant stance,
* for each leg, sweep yaw while the foot is lifted,
* sweep the same yaw path while that foot is lightly pressed into the floor
  and the other five feet support the body,
* repeat at several foot-ground slide friction values.

It does not pretend to be an onboard-current model.  It verifies that the
probe is physically meaningful in MuJoCo: loaded sweeps make contact, the
body does not fall, and the loaded foot/chassis response changes when
friction changes.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.sim.servo_model import (  # noqa: E402
    ACC_UNIT_DEG_S2, COUNTS_PER_DEG, N_JOINTS, ServoProfile,
    SimServoParams, apply_params_to_model, build_model, joint_qpos_addrs,
    position_actuator_ids,
)
from rl_move.sim.sim_env import set_foot_ground_friction  # noqa: E402

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
PLANT = np.array([0.0, 20.0, 80.0] * 6, dtype=float)
DT_CTRL = 0.04
SPEED_COUNTS_S = 120.0
ACC_UNITS = 12.0
AMP_DEG = 7.0


def _ids(model):
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


def _roll_pitch_deg(xmat: np.ndarray) -> tuple[float, float]:
    m = xmat.reshape(3, 3)
    zx, zy, zz = m[0, 2], m[1, 2], m[2, 2]
    return (
        math.degrees(math.atan2(zy, zz)),
        math.degrees(math.atan2(-zx, math.hypot(zy, zz))),
    )


def _pose(leg: int, yaw: float, mode: str) -> np.ndarray:
    q = PLANT.copy()
    j = leg * 3
    q[j] = yaw
    if mode == "hover":
        q[j + 1] -= 6.0
        q[j + 2] += 14.0
    elif mode == "loaded":
        q[j + 1] += 3.0
    return q


def _run_segment(model, data, params, q_goal_deg: np.ndarray,
                 seconds: float, *, qadr, pos_act, touch_adr, pad_bids,
                 chassis_bid, leg: int) -> dict:
    import mujoco

    q0 = data.qpos[qadr].copy()
    profile = ServoProfile(params, q0)
    profile.command(q_goal_deg * DEG2RAD,
                    speed_deg_s=SPEED_COUNTS_S / COUNTS_PER_DEG,
                    acc_units=ACC_UNITS)
    substeps = int(round(DT_CTRL / model.opt.timestep))
    ticks = int(round(seconds / DT_CTRL))
    pad0 = data.xpos[pad_bids[leg], :2].copy()
    chassis0 = data.xpos[chassis_bid, :2].copy()
    touches = []
    tilts = []
    qerr = []
    for _ in range(ticks):
        for _sub in range(substeps):
            target = profile.tick(model.opt.timestep)
            q = data.qpos[qadr]
            err = target - q
            eff = q + np.sign(err) * np.maximum(
                np.abs(err) - profile.deadband_rad, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)
        touches.append(float(data.sensordata[touch_adr[leg]]))
        tilts.append(max(abs(x) for x in _roll_pitch_deg(
            data.xmat[chassis_bid])))
        qerr.append(abs(float(q_goal_deg[leg * 3] * DEG2RAD
                              - data.qpos[qadr[leg * 3]])) * RAD2DEG)
    pad = data.xpos[pad_bids[leg], :2].copy()
    chassis = data.xpos[chassis_bid, :2].copy()
    return {
        "pad_travel_mm": float(np.linalg.norm(pad - pad0)) * 1000.0,
        "chassis_shift_mm": float(np.linalg.norm(chassis - chassis0)) * 1000.0,
        "mean_touch_n": float(np.mean(touches)) if touches else 0.0,
        "max_tilt_deg": float(max(tilts or [0.0])),
        "max_yaw_err_deg": float(max(qerr or [0.0])),
    }


def run(mu: float, *, legs: list[int]) -> dict:
    import mujoco

    params = SimServoParams.from_cfg(None)
    model = build_model(fixed_base=False, flat_terrain=True,
                        mesh_visuals=False)
    apply_params_to_model(model, params)
    if mu > 0:
        set_foot_ground_friction(model, mu)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    touch_adr, pad_bids, chassis_bid = _ids(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = 0.16
    data.qpos[qadr] = PLANT * DEG2RAD
    data.ctrl[pos_act] = PLANT * DEG2RAD
    mujoco.mj_forward(model, data)
    for _ in range(int(round(1.5 / model.opt.timestep))):
        mujoco.mj_step(model, data)

    rows = []
    for leg in legs:
        for mode in ("hover", "loaded"):
            center = _pose(leg, 0.0, mode)
            for yaw in (0.0, -AMP_DEG, AMP_DEG, -AMP_DEG, 0.0):
                q = center.copy()
                q[leg * 3] = yaw
                r = _run_segment(
                    model, data, params, q, 0.36,
                    qadr=qadr, pos_act=pos_act, touch_adr=touch_adr,
                    pad_bids=pad_bids, chassis_bid=chassis_bid, leg=leg)
                rows.append({"leg": leg, "mode": mode, "yaw_deg": yaw, **r})
    loaded = [r for r in rows if r["mode"] == "loaded"]
    hover = [r for r in rows if r["mode"] == "hover"]
    mean = lambda key, xs: float(np.mean([r[key] for r in xs])) if xs else 0.0
    out = {
        "mu": mu,
        "legs": legs,
        "mean_loaded_touch_n": round(mean("mean_touch_n", loaded), 3),
        "mean_hover_touch_n": round(mean("mean_touch_n", hover), 3),
        "loaded_pad_travel_mm": round(mean("pad_travel_mm", loaded), 3),
        "hover_pad_travel_mm": round(mean("pad_travel_mm", hover), 3),
        "loaded_chassis_shift_mm": round(mean("chassis_shift_mm", loaded), 3),
        "hover_chassis_shift_mm": round(mean("chassis_shift_mm", hover), 3),
        "max_tilt_deg": round(max([r["max_tilt_deg"] for r in rows] or [0.0]), 2),
        "max_yaw_err_deg": round(max([r["max_yaw_err_deg"] for r in rows] or [0.0]), 2),
        "rows": rows,
    }
    out["coupling_loaded_over_hover"] = round(
        out["loaded_chassis_shift_mm"] / max(out["hover_chassis_shift_mm"], 1e-3),
        3)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mus", default="0.3,0.8,1.5")
    ap.add_argument("--legs", default="0,2,4",
                    help="comma list; default uses alternating legs for speed")
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    legs = [int(x) for x in args.legs.split(",") if x.strip()]
    results = [run(float(mu), legs=legs)
               for mu in args.mus.split(",") if mu.strip()]
    for r in results:
        print(
            f"mu={r['mu']:.2f} touch loaded/hover "
            f"{r['mean_loaded_touch_n']:.2f}/{r['mean_hover_touch_n']:.2f} N, "
            f"pad travel loaded/hover "
            f"{r['loaded_pad_travel_mm']:.2f}/"
            f"{r['hover_pad_travel_mm']:.2f} mm, "
            f"chassis shift loaded/hover "
            f"{r['loaded_chassis_shift_mm']:.2f}/"
            f"{r['hover_chassis_shift_mm']:.2f} mm, "
            f"coupling x{r['coupling_loaded_over_hover']:.2f}, "
            f"max tilt {r['max_tilt_deg']:.1f} deg")
    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(results, indent=1) + "\n")
        print(f"wrote {path}")


if __name__ == "__main__":
    main()

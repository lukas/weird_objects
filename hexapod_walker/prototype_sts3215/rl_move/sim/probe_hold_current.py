"""Hold-current probe: pose/command-matched sim-vs-hardware current table.

SIM.md gap 2 says sim hold current (0.11 A) is ~5x UNDER the real
robot's standing 0.59 A and needs a load-dependent holding-current
model. Re-reading the raw traces (hw_session2_20260810.csv) shows the
hardware itself draws 0.106 A total holding the WALK-SYNCED plant
(command == settled stance) and 0.541 A total holding the SCRIPTED
stand pose (command == ideal plant the loaded joints sag away from):
per-servo register current tracks the cmd-q fight, not the pose per
se. The 08-10 comparison put sim's crouch hold (in MEAN-PER-SERVO
units) against the real RAISED/ideal hold (in BUS-TOTAL units) — pose
AND units both mismatched.

This probe measures sim, in register-comparable per-servo/total units,
at the matched conditions:

    hold_sync   cmd := settled stance   (hw anchor: 0.106 A total)
    hold_ideal  cmd := ideal plant      (hw anchor: 0.541 A total)
    walk30      scripted tripod 30 mm/s (hw anchor: 0.395 A total mean)
    rl_stand    replay of the deployed stand-attempt cmd stream
                rl_stand_20260810_024744.csv (hw: per-tick per-servo
                currents in the CSV; quarter totals 0.23/0.19/0.20/0.91)

and reports, per condition x servo axis: the legacy synthesized current
(|qfrc_actuator| * 1.2 A/Nm, 0.1 s lowpass — what sim_env feeds the
reward, PLANT_SPEC current_ok and the 2.5 A safety trip), raw
|qfrc_actuator| (load torque), and profile-target / commanded sag
errors in deg — the candidate predictors for a fitted register-current
model.

Run on a train pod (mujoco not on the controller):
    python3 -m rl_move.sim.probe_hold_current [--params loaded] [--json out]
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO), str(_PROTO / "linux_control"),
          str(_PROTO / "linux_control" / "urt2_setup")):
    if p not in sys.path:
        sys.path.insert(0, p)

from .servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, joint_qvel_addrs, position_actuator_ids,
)

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
N = 18
TRACES = _RL / "hardware_traces"
PLANT = np.array([0.0, 20.0, 80.0] * 6) * DEG2RAD
# Hardware bus write profiles (fit_loaded_actuator.py provenance).
GAIT_SPEED, GAIT_ACC = 1500.0 * 360.0 / 4096.0, 30.0   # scripted gait path
RL_SPEED, RL_ACC = 400.0 * 360.0 / 4096.0, 20.0        # deployed RL path

HW_ANCHORS = {
    "hold_sync": "hw walk-zerocmd total 0.106 A (mean/servo 0.006)",
    "hold_ideal": "hw 'standing' total 0.541 A (mean/servo 0.030)",
    "walk30": "hw walk 30mm/s total mean 0.395 A",
    "rl_stand": "hw quarter totals 0.23/0.19/0.20/0.91 A, peak servo 2.76",
}


class _Rig:
    """Free-base sim on flat ground, ServoProfile bus, 25 Hz sampling."""

    def __init__(self, params: SimServoParams):
        import mujoco
        from .sim_env import soften_contacts
        self.mujoco = mujoco
        self.model = build_model(fixed_base=False, flat_terrain=True)
        soften_contacts(self.model)
        apply_params_to_model(self.model, params)
        self.data = mujoco.MjData(self.model)
        self.qadr = joint_qpos_addrs(self.model)
        self.vadr = joint_qvel_addrs(self.model)
        self.pos_act = position_actuator_ids(self.model)
        self.params = params
        self.dt = self.model.opt.timestep
        self._cur = None

    def place(self, q_rad: np.ndarray) -> None:
        import mujoco_prototype as MP
        from rl_move.body_ik import fk_all_feet
        mj = self.mujoco
        feet = fk_all_feet(q_rad)
        base_z = MP.YAW_OUTPUT_HEIGHT - float(np.min(feet[:, 2])) \
            + MP.FOOT_R + 0.002
        mj.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, base_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self.qadr] = q_rad
        self.data.ctrl[self.pos_act] = q_rad
        mj.mj_forward(self.model, self.data)
        for _ in range(40):
            worst = min((float(self.data.contact[ci].dist)
                         for ci in range(self.data.ncon)), default=0.0)
            if worst > -1e-4:
                break
            self.data.qpos[2] += -worst + 0.001
            mj.mj_forward(self.model, self.data)
        self._cur = None

    def settle(self, q_cmd: np.ndarray, seconds: float) -> None:
        for _ in range(int(seconds / self.dt)):
            self.data.ctrl[self.pos_act] = q_cmd
            self.mujoco.mj_step(self.model, self.data)
            self._lowpass()

    def _lowpass(self) -> np.ndarray:
        # Same synthesis as sim_env.read_state: |qfrc|*1.2 capped 3 A,
        # 0.1 s lowpass.
        raw = np.minimum(np.abs(self.data.qfrc_actuator[self.vadr]) * 1.2,
                         3.0)
        if self._cur is None:
            self._cur = raw
        else:
            a = self.dt / (self.dt + 0.1)
            self._cur = (1.0 - a) * self._cur + a * raw
        return self._cur

    def run(self, cmd_fn, seconds: float, *, speed: float, acc: float,
            record_from: float = 0.0):
        """cmd_fn(t)->q_rad target at 25 Hz; returns per-tick arrays."""
        profile = ServoProfile(self.params,
                               self.data.qpos[self.qadr].copy())
        per = int(round(0.04 / self.dt))
        ticks = int(round(seconds / 0.04))
        out = {k: [] for k in ("cur", "qfrc", "err_prof_deg",
                               "err_cmd_deg", "q_deg", "xy")}
        for k in range(ticks):
            t = k * 0.04
            cmd = cmd_fn(t)
            profile.command(cmd, speed_deg_s=speed, acc_units=acc)
            for _ in range(per):
                self.data.ctrl[self.pos_act] = profile.tick(self.dt)
                self.mujoco.mj_step(self.model, self.data)
                cur = self._lowpass()
            if t < record_from:
                continue
            q = self.data.qpos[self.qadr]
            out["cur"].append(cur.copy())
            out["qfrc"].append(
                np.abs(self.data.qfrc_actuator[self.vadr]).copy())
            out["err_prof_deg"].append(
                np.abs(self.data.ctrl[self.pos_act] - q) * RAD2DEG)
            out["err_cmd_deg"].append(np.abs(cmd - q) * RAD2DEG)
            out["q_deg"].append(q * RAD2DEG)
            out["xy"].append(self.data.qpos[:2].copy())
        return {k: np.asarray(v) for k, v in out.items()}


def _axis_means(arr: np.ndarray) -> dict:
    return {ax: [round(float(x), 4) for x in
                 arr[:, [j for j in range(N) if j % 3 == k]].mean(0)]
            for k, ax in enumerate(("yaw", "hip", "knee"))}


def summarize(name: str, rec: dict) -> dict:
    cur = rec["cur"]
    s = {
        "condition": name,
        "hw_anchor": HW_ANCHORS.get(name, ""),
        "sim_cur_total_mean_a": round(float(cur.sum(1).mean()), 3),
        "sim_cur_mean_per_servo_a": round(float(cur.mean()), 4),
        "sim_cur_max_servo_a": round(float(cur.max()), 3),
        "sim_cur_per_joint_mean": _axis_means(cur),
        "qfrc_per_joint_mean_nm": _axis_means(rec["qfrc"]),
        "err_prof_deg_per_joint_mean": _axis_means(rec["err_prof_deg"]),
        "err_cmd_deg_per_joint_mean": _axis_means(rec["err_cmd_deg"]),
    }
    print(f"== {name}  [{s['hw_anchor']}]")
    print(f"   sim legacy current: total {s['sim_cur_total_mean_a']} A, "
          f"mean/servo {s['sim_cur_mean_per_servo_a']} A, "
          f"max servo {s['sim_cur_max_servo_a']} A")
    for ax in ("yaw", "hip", "knee"):
        print(f"   {ax:4s} cur {s['sim_cur_per_joint_mean'][ax]}")
        print(f"        |tau| {s['qfrc_per_joint_mean_nm'][ax]}")
        print(f"        err_cmd_deg {s['err_cmd_deg_per_joint_mean'][ax]}")
    return s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--params", default="air", choices=("air", "loaded"))
    ap.add_argument("--json", type=Path, default=None)
    ap.add_argument("--stand-csv", type=Path,
                    default=TRACES / "rl_stand_20260810_024744.csv")
    args = ap.parse_args(argv)

    if args.params == "loaded":
        from .servo_model import LOADED_MODEL_PATH
        params = SimServoParams.load(LOADED_MODEL_PATH)
    else:
        params = SimServoParams.load()
    print(f"[params] {args.params} ({params.source})")
    results = []

    # A/B: holds. Settle at ideal plant first (1.5 s, gait profile).
    for name in ("hold_sync", "hold_ideal"):
        rig = _Rig(params)
        rig.place(PLANT)
        rig.settle(PLANT, 1.5)
        cmd = (rig.data.qpos[rig.qadr].copy() if name == "hold_sync"
               else PLANT.copy())
        rec = rig.run(lambda t, c=cmd: c, 5.0, speed=GAIT_SPEED,
                      acc=GAIT_ACC, record_from=2.0)
        results.append(summarize(name, rec))

    # C: scripted walk 30 mm/s, hardware gait write profile.
    try:
        from sim_gait_compat import TripodGait
        gait = TripodGait(vx=0.03)
        gait.sync_plant_stance(20.0, 80.0)
        gait.set_lift_mm(25.0)
        gait.reset_phase()
        rig = _Rig(params)
        rig.place(PLANT)
        rig.settle(PLANT, 1.5)
        rec = rig.run(
            lambda t: np.asarray(gait.desired_deg(t)) * DEG2RAD,
            10.0, speed=GAIT_SPEED, acc=GAIT_ACC, record_from=1.0)
        results.append(summarize("walk30", rec))
    except ImportError as e:
        print(f"[walk30] SKIPPED: {e}")

    # D: rl_stand deployed cmd-stream replay, per-tick hw comparison.
    if args.stand_csv.exists():
        import csv as _csv
        rows = list(_csv.DictReader(open(args.stand_csv)))
        cmd = np.array([[float(r[f"cmd{j}_deg"]) for j in range(N)]
                        for r in rows]) * DEG2RAD
        hw_cur = np.array([[float(r[f"cur{j}_a"]) for j in range(N)]
                           for r in rows])
        q0 = np.array([[float(r[f"q{j}_deg"]) for j in range(N)]
                       for r in rows])[0] * DEG2RAD
        rig = _Rig(params)
        rig.place(q0)
        rig.settle(q0, 0.4)
        rec = rig.run(
            lambda t: cmd[min(int(round(t / 0.04)), len(cmd) - 1)],
            len(cmd) * 0.04, speed=RL_SPEED, acc=RL_ACC)
        s = summarize("rl_stand", rec)
        nt = min(len(hw_cur), len(rec["cur"]))
        sim_t, hw_t = rec["cur"][:nt].sum(1), hw_cur[:nt].sum(1)
        qs = [slice(i * nt // 4, (i + 1) * nt // 4) for i in range(4)]
        s["quarters_total_a"] = {
            "sim": [round(float(sim_t[sl].mean()), 3) for sl in qs],
            "hw": [round(float(hw_t[sl].mean()), 3) for sl in qs]}
        s["per_tick_corr"] = round(float(np.corrcoef(
            rec["cur"][:nt].ravel(), hw_cur[:nt].ravel())[0, 1]), 3)
        s["hw_peak_servo_a"] = round(float(hw_cur.max()), 3)
        print(f"   quarters sim {s['quarters_total_a']['sim']} vs hw "
              f"{s['quarters_total_a']['hw']}; per-tick per-servo corr "
              f"{s['per_tick_corr']}; hw peak {s['hw_peak_servo_a']}")
        results.append(s)
    else:
        print(f"[rl_stand] SKIPPED: {args.stand_csv} missing")

    if args.json:
        args.json.write_text(json.dumps(
            {"params": args.params, "results": results}, indent=1))
        print(f"[saved] {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

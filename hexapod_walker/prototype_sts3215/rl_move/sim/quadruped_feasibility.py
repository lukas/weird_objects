"""Quadruped-mode feasibility sweep — GEOMETRY BEFORE RL.

Readiness review 2026-08-09 §4: before any four-leg RL arm, freeze the
two FRONT legs (L0 az=+30 deg, L5 az=-30 deg; forward = +X, the walk
task's heading-0 direction) in raised "claw" poses and sweep body
x/y shift, body height, body pitch and a middle-leg yaw splay while
the four support feet (L1 middle-left, L2 rear-left, L3 rear-right,
L4 middle-right) stay planted. For each static configuration, settle
the REAL sim model (fitted servo params, softened contacts — exactly
what training uses) under position-actuator hold and measure:

  - projected total CoM vs the 4-foot support polygon (signed margin, mm)
  - front-foot contact force (must be ~0 = unloaded/clear)
  - support-foot contact count (must be 4 planted)
  - per-joint holding torque -> servo current (env model: |tau|*1.2 A,
    trip 2.5 A, stall ~3 A)
  - attitude error vs commanded pitch, height sag
  - joint-limit margin of the support legs
  - a forward-push perturbation retest (worst direction: toward the
    polygon's front edge) for configs that pass the static screen

If NO region of this grid holds a comfortable margin without hot
servos, PPO will not rescue the morphology (review: "geometry, not
RL") and the quadruped line stops here. Usage:

    python3 -m rl_move.sim.quadruped_feasibility \
        [--quick] [--out logs/experiments/quadruped-feasibility]
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
for p in (str(_PROTO),):
    if p not in sys.path:
        sys.path.insert(0, p)

import mujoco  # noqa: E402
import mujoco_prototype as MP  # noqa: E402

from rl_move.body_ik import (  # noqa: E402
    N_JOINTS, fk_all_feet, ik_leg_from_foot_body, leg_azimuths,
)
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.safety import AXIS_LIMITS_DEG  # noqa: E402
from rl_move.sim.servo_model import (  # noqa: E402
    SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, position_actuator_ids,
)
from rl_move.sim.sim_env import (  # noqa: E402
    _default_plant_deg, soften_contacts, support_margin_m,
)

FRONT_LEGS = (0, 5)
SUPPORT_LEGS = (1, 2, 3, 4)
MID_LEGS = (1, 4)          # az 90 / 270 deg: their feet define the
                           # support polygon's FRONT edge (x ~ 0)
CURRENT_PER_NM = 1.2       # sim_env: servo current ~ |torque| * 1.2
CURRENT_CAP_A = 3.0
TRIP_A = 2.5

# Raised front-leg joint presets (yaw, hip, knee) rad. Limits:
# yaw +-0.61, hip -1.40..0.52, knee -0.35..2.62.
FRONT_POSES = {
    # tucked claw: hip high up, knee folded — CoM of the leg pulled in
    "tuck": (0.0, -1.10, 2.40),
    # forward reach: raised but extended ahead (manipulation-ish pose,
    # worst case for CoM: leg mass forward)
    "reach": (0.0, -0.60, 1.20),
}


def _grid(quick: bool):
    dxs = [0.0, -0.02, -0.04, -0.06, -0.08]
    dzs = [0.0, -0.015]
    pitches = [0.0, -0.08, 0.08]          # rad; -0.08 = nose down
    mid_yaws = [0.0, 0.30, 0.55]          # rad toward +X (fwd splay)
    poses = list(FRONT_POSES)
    if quick:
        dxs = [0.0, -0.04, -0.08]
        dzs = [0.0]
        pitches = [0.0]
        mid_yaws = [0.0, 0.55]
    return list(itertools.product(dxs, dzs, pitches, mid_yaws, poses))


def _quat_pitch(theta: float) -> tuple[float, float, float, float]:
    return (math.cos(theta / 2), 0.0, math.sin(theta / 2), 0.0)


class Sweep:
    def __init__(self):
        params = SimServoParams.load()
        self.model = build_model(flat_terrain=True, mesh_visuals=False)
        apply_params_to_model(self.model, params)
        soften_contacts(self.model)
        self.data = mujoco.MjData(self.model)
        self.qadr = joint_qpos_addrs(self.model)
        self.pos_act = position_actuator_ids(self.model)
        self.chassis = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        self.foot_gid = [mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, f"L{i}_foot")
            for i in range(6)]
        self.dt = self.model.opt.timestep
        # dof addresses of the 18 hinge joints (for qfrc_actuator)
        self.vadr = np.array([
            self.model.jnt_dofadr[mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{ax}")]
            for i in range(6) for ax in ("yaw", "pitch", "knee")])
        # nominal plant geometry
        self.plant_q = _default_plant_deg() * DEG2RAD
        feet = fk_all_feet(self.plant_q)
        self.foot_drop = float(np.min(feet[:, 2]))
        self.base_z0 = (MP.YAW_OUTPUT_HEIGHT - self.foot_drop
                        + MP.FOOT_R + 0.002)
        self.k_fk = np.array([0.0, 0.0, MP.YAW_OUTPUT_HEIGHT])
        # world foot anchors at the nominal plant (base at origin):
        # p_world = t + R @ (p_fk - k); nominal: t=(0,0,base_z0), R=I
        self.feet_world_nom = (feet - self.k_fk
                               + np.array([0, 0, self.base_z0]))
        self.az = leg_azimuths()

    # -- target joint vector for one config ------------------------------
    def solve_q(self, dx, dz, pitch, mid_yaw, pose):
        q = np.zeros(N_JOINTS)
        for leg in FRONT_LEGS:
            q[3 * leg: 3 * leg + 3] = FRONT_POSES[pose]
        t = np.array([dx, 0.0, self.base_z0 + dz])
        R = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                      [0, 1, 0],
                      [-math.sin(pitch), 0, math.cos(pitch)]])
        for leg in SUPPORT_LEGS:
            W = self.feet_world_nom[leg].copy()
            if leg in MID_LEGS and mid_yaw:
                # splay the middle foot toward +X by rotating it about
                # its yaw-servo origin (world, nominal base placement)
                a = self.az[leg]
                o = np.array([math.cos(a), math.sin(a), 0.0]) \
                    * __import__("rl_move.body_ik", fromlist=["LEG_RADIAL"]
                                 ).LEG_RADIAL
                r = W[:2] - o[:2]
                sgn = -1.0 if leg == 1 else 1.0   # toward +X for both
                c, s = math.cos(sgn * mid_yaw), math.sin(sgn * mid_yaw)
                W[:2] = o[:2] + np.array([c * r[0] - s * r[1],
                                          s * r[0] + c * r[1]])
            p_fk = R.T @ (W - t) + self.k_fk
            sol = ik_leg_from_foot_body(p_fk, self.az[leg])
            if sol is None:
                return None, f"L{leg} unreachable"
            yaw, hip, knee = sol
            lim = [AXIS_LIMITS_DEG[j] for j in range(3)]
            for val, (lo, hi), nm in zip(
                    (yaw, hip, knee), lim, ("yaw", "hip", "knee")):
                if not (lo * DEG2RAD - 1e-6 <= val <= hi * DEG2RAD + 1e-6):
                    return None, f"L{leg} {nm} limit ({math.degrees(val):.0f}deg)"
            q[3 * leg: 3 * leg + 3] = (yaw, hip, knee)
        return q, ""

    # -- physics ----------------------------------------------------------
    def _place(self, q, dx, dz, pitch):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (dx, 0.0, self.base_z0 + dz)
        self.data.qpos[3:7] = _quat_pitch(pitch)
        self.data.qpos[self.qadr] = q
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self.pos_act] = q
        mujoco.mj_forward(self.model, self.data)
        for _ in range(40):
            worst = min((self.data.contact[ci].dist
                         for ci in range(self.data.ncon)), default=0.0)
            if worst > -1e-4:
                break
            self.data.qpos[2] += -worst + 0.001
            mujoco.mj_forward(self.model, self.data)

    def _step_s(self, seconds):
        for _ in range(int(round(seconds / self.dt))):
            mujoco.mj_step(self.model, self.data)

    def _foot_forces(self):
        f = np.zeros(6)
        buf = np.zeros(6)
        for ci in range(self.data.ncon):
            con = self.data.contact[ci]
            for g in (con.geom1, con.geom2):
                for leg, gid in enumerate(self.foot_gid):
                    if g == gid:
                        mujoco.mj_contactForce(
                            self.model, self.data, ci, buf)
                        f[leg] += abs(buf[0])
        return f

    def measure(self, q, dx, dz, pitch):
        """Settle 1.5 s, then average metrics over the last 0.25 s."""
        self._place(q, dx, dz, pitch)
        self._step_s(1.5)
        n, taus, forces, coms, feetw = 0, [], [], [], None
        for _ in range(int(round(0.25 / self.dt))):
            mujoco.mj_step(self.model, self.data)
            taus.append(np.abs(self.data.qfrc_actuator[self.vadr]))
            forces.append(self._foot_forces())
            coms.append(self.data.subtree_com[0][:2].copy())
            n += 1
        tau = np.mean(taus, axis=0)
        cur = np.minimum(tau * CURRENT_PER_NM, CURRENT_CAP_A)
        ff = np.mean(forces, axis=0)
        com_xy = np.mean(coms, axis=0)
        # support feet world XY (live, post-settle)
        feet_xy = np.array([
            self.data.geom_xpos[self.foot_gid[leg]][:2]
            for leg in SUPPORT_LEGS])
        margin = support_margin_m(feet_xy, com_xy)
        Rm = self.data.xmat[self.chassis].reshape(3, 3)
        pitch_now = math.atan2(-Rm[2, 0], math.hypot(Rm[0, 0], Rm[1, 0]))
        roll_now = math.atan2(Rm[2, 1], Rm[2, 2])
        return {
            "margin_mm": round(margin * 1000, 1),
            "front_load_N": round(float(ff[list(FRONT_LEGS)].sum()), 2),
            "support_contacts": int((ff[list(SUPPORT_LEGS)] > 0.3).sum()),
            "cur_max_A": round(float(cur.max()), 2),
            "cur_mean_A": round(float(cur.mean()), 2),
            "hot_joint": int(np.argmax(cur)),
            "pitch_err_deg": round(math.degrees(pitch_now - (-pitch)), 1),
            "roll_deg": round(math.degrees(roll_now), 1),
            "base_z_mm": round(float(self.data.qpos[2]) * 1000, 1),
            "fell": bool(self.data.qpos[2] < 0.05
                         or abs(math.degrees(roll_now)) > 20),
        }

    def perturb(self, q, dx, dz, pitch, push_n=6.0):
        """Re-settle, push the chassis toward +X (front edge) 0.15 s,
        then 0.8 s free; report if still a valid 4-foot stance."""
        self._place(q, dx, dz, pitch)
        self._step_s(1.2)
        self.data.xfrc_applied[self.chassis, 0] = push_n
        self._step_s(0.15)
        self.data.xfrc_applied[self.chassis, 0] = 0.0
        self._step_s(0.8)
        ff = self._foot_forces()
        Rm = self.data.xmat[self.chassis].reshape(3, 3)
        roll_now = math.degrees(math.atan2(Rm[2, 1], Rm[2, 2]))
        ok = (self.data.qpos[2] > 0.05 and abs(roll_now) < 20
              and (ff[list(SUPPORT_LEGS)] > 0.3).sum() == 4
              and ff[list(FRONT_LEGS)].sum() < 1.0)
        return bool(ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out", default="logs/experiments/quadruped-feasibility")
    ap.add_argument("--push-n", type=float, default=6.0)
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    sw = Sweep()
    rows, t0 = [], time.time()
    grid = _grid(args.quick)
    print(f"sweep: {len(grid)} configs")
    for i, (dx, dz, pitch, my, pose) in enumerate(grid):
        cfg = {"dx_mm": round(dx * 1000), "dz_mm": round(dz * 1000),
               "pitch_deg": round(math.degrees(pitch), 1),
               "mid_yaw_deg": round(math.degrees(my), 1), "front": pose}
        q, why = sw.solve_q(dx, dz, pitch, my, pose)
        if q is None:
            rows.append({**cfg, "ik": why})
            continue
        m = sw.measure(q, dx, dz, pitch)
        row = {**cfg, "ik": "ok", **m}
        # static screen: clear fronts, 4 planted, inside polygon, cool
        row["static_pass"] = bool(
            not m["fell"] and m["front_load_N"] < 1.0
            and m["support_contacts"] == 4 and m["margin_mm"] >= 10.0
            and m["cur_max_A"] < 2.0 and abs(m["pitch_err_deg"]) < 6.0)
        if row["static_pass"]:
            row["push_ok"] = sw.perturb(q, dx, dz, pitch, args.push_n)
        rows.append(row)
        if (i + 1) % 20 == 0:
            print(f"  {i+1}/{len(grid)} ({time.time()-t0:.0f}s)")

    (out / "sweep.json").write_text(json.dumps(rows, indent=1))
    passing = [r for r in rows if r.get("static_pass")]
    robust = [r for r in passing if r.get("push_ok")]
    print(f"\n{len(rows)} configs | ik-ok "
          f"{sum(1 for r in rows if r.get('ik') == 'ok')} | static-pass "
          f"{len(passing)} | push-robust {len(robust)}")
    cols = ("dx_mm dz_mm pitch_deg mid_yaw_deg front margin_mm "
            "front_load_N cur_max_A cur_mean_A pitch_err_deg push_ok").split()
    best = sorted(passing, key=lambda r: -r["margin_mm"])[:12]
    if best:
        print("\ntop static passes by CoM margin:")
        print(" | ".join(cols))
        for r in best:
            print(" | ".join(str(r.get(c, "")) for c in cols))
    else:
        print("\nNO static passes — dump of closest margins:")
        near = sorted((r for r in rows if r.get("ik") == "ok"),
                      key=lambda r: -(r.get("margin_mm") or -999))[:12]
        print(" | ".join(cols))
        for r in near:
            print(" | ".join(str(r.get(c, "")) for c in cols))
    print(f"\nwrote {out/'sweep.json'} ({time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()

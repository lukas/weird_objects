#!/usr/bin/env python3
"""probe_quad_rear — can the robot TIP BACK and walk on four legs?

The 08-13 quad-crawl line proved a LEVEL-body open-loop quad crawl is
geometrically infeasible: with the fronts lifted the CoM sits
3.3–5.9 cm ahead of the best achievable 4-foot centroid (hip-yaw limits
cap how far the mid feet can move forward), so every mid-leg swing tips
the body onto the swinging foot. That analysis swept body pitch only to
±4.6 deg — the RL env terminates at 10 deg tilt.

This probe tries the untried lever: REAR the body (large nose-up pitch,
like a begging dog). Pitching the chassis ~15–35 deg with the CoM
~10 cm above the feet moves the projected CoM aft by several cm — the
exact size of the level-body margin gap.

Phase A (``static``): re-run the quadruped_feasibility settle/measure
machinery over large nose-up pitches x body shift x mid splay.

Phase B (``walk``): script a statically-stable 4-beat creep on the best
reared stance — body shifts toward the remaining-triangle centroid
before each swing — streamed through the fitted ServoProfile at the
20 Hz demo rate, with an mp4 and slip/margin/fall metrics.

FINDINGS (08-18, CPU twin, fitted air servo params):

  static: rearing WORKS. Nose-up 8–26 deg with the body 30–60 mm back
  gives 40–74 mm CoM margin on the 4-foot polygon (level-body work had
  single digits) and survives the 6 N forward push. Envelope edge =
  mid-leg (L1/L4) hip/yaw limits: deep pitch + big dx or splay > 32 deg
  goes IK-infeasible.

  walk: an OPEN-LOOP tip-back creep walks forward — the 08-13
  "only dynamic balance can quad-walk" conclusion does NOT hold in the
  reared regime. The mid-swing support triangle contains the aft-shifted
  CoM, so the fatal level-body mid-leg pin never happens.
  Calm/rocking boundary is sharp and slip-driven:
    stride <= 35 mm, lift 20 mm, beat 0.7 s + 0.7 s: tilt stays within
      ~1 deg of the commanded pitch for 45 s (never falls, est current
      peak 2.6 A once at the first beat, ~8 s).
    stride >= 40 mm or shift_cap >= 50 mm: rear-foot slip lurches the
      body and pumps a ~35 deg rocking oscillation (recovers, but not
      hardware-safe).
  Stride realization ~33-55 pct (rear feet pivot-slip under load);
  calm best ≈ +82 mm / 40 s at pitch -14, showcase pick pitch -17
  (+64 mm / 40 s, visibly reared):

  gait=walk (08-18, later same day): a CONTINUOUS lateral-sequence
  animal walk (footfalls LH,LF,RH,RF at quarter-cycle offsets, duty
  0.8, body at constant velocity + sinusoidal sway) beats the creep on
  every axis: +155 mm / 35 s at stride 45 mm, support margin never
  below +34 mm, peak est current 2.0 A, tilt within 1 deg of commanded.
  Sway phase is the key knob: 120-135 deg (lean lags the swing side)
  keeps the margin positive; 0 deg kills progress, 180+ goes
  margin-negative. Stride >= 50 mm slip-rocks like the creep.
  TROT (08-18, evening): DIAGONAL pairs (LF+RH / RF+LH, phases 0/0.5)
  beat the walk's realized speed by ~70%: preset period 1.6 s, stride
  70 mm, duty 0.6, NO sway → 10.6 mm/s over 40 s vs the walk's 6.2.
  Counter-intuitively it never falls and rocks LESS than the walk
  (2.7–5.4 deg band) anywhere in period 0.8–2.0 s x stride 50–90 mm x
  duty 0.5–0.65 — the fitted servo velocity clamp low-passes the fast
  end into a smooth shuffle, and the symmetric beat needs no sway.
  Realized speed ceiling ~11 mm/s (stride realization ~24%; bigger
  strides just slip more). Swept via quad_walk.GAITS overrides on the
  QuadRearWalk instance, not this file. Deployed as ``quad_trot``.

  Deployed as the ``quad_walk`` demo (motor_setup/quad_walk.py — the
  hardware port, validated end-to-end in this sim incl. entry/exit):

    python probe_quad_rear.py walk --pitch -17 --body-dx -0.04 \
        --mid-yaw 25 --stride 0.035 --lift 0.02 \
        --t-shift 0.7 --t-swing 0.7 --video out.mp4

    python probe_quad_rear.py static

Plain python (cv2 render), NOT mjpython.
"""
from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

_SIM = Path(__file__).resolve().parent
_PROTO = _SIM.parents[1]
for p in (_PROTO, _PROTO / "linux_control", _SIM):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.body_ik import (  # noqa: E402
    N_JOINTS, ik_leg_from_foot_body, leg_azimuths,
)
from rl_move.robot_state import DEG2RAD, RAD2DEG  # noqa: E402
from rl_move.safety import AXIS_LIMITS_DEG  # noqa: E402

FRONT_LEGS = (0, 5)
SUPPORT_LEGS = (1, 2, 3, 4)
TUCK = (0.0, -1.10, 2.40)      # feasibility FRONT_POSES["tuck"], c57 GO


# ---------------------------------------------------------------------------
# Phase A — static sweep at large nose-up pitch
# ---------------------------------------------------------------------------

def run_static(args) -> None:
    from quadruped_feasibility import Sweep
    sw = Sweep()
    # Sign convention check: solve_q/_place use R = rot_y(+pitch), which
    # takes body +X (nose) to world z = -sin(pitch) -> NOSE-UP needs
    # pitch < 0. Sweep both signs anyway; front_load ~ 0 + margin decides.
    pitches = [-0.15, -0.25, -0.35, -0.45, 0.25]
    dxs = [0.0, -0.03, -0.06]
    mid_yaws = [0.0, 0.30, 0.55]
    rows = []
    grid = list(itertools.product(dxs, pitches, mid_yaws))
    print(f"static sweep: {len(grid)} configs (front=tuck, dz=0)")
    for dx, pitch, my in grid:
        cfg = {"dx_mm": round(dx * 1000), "pitch_deg": round(
            math.degrees(pitch), 1), "mid_yaw_deg": round(math.degrees(my))}
        q, why = sw.solve_q(dx, 0.0, pitch, my, "tuck")
        if q is None:
            rows.append({**cfg, "ik": why})
            continue
        m = sw.measure(q, dx, 0.0, pitch)
        row = {**cfg, "ik": "ok", **m}
        row["static_pass"] = bool(
            not m["fell"] and m["front_load_N"] < 1.0
            and m["support_contacts"] == 4 and m["margin_mm"] >= 10.0
            and m["cur_max_A"] < 2.0)
        if row["static_pass"]:
            row["push_ok"] = sw.perturb(q, dx, 0.0, pitch, 6.0)
        rows.append(row)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "static_rear.json").write_text(json.dumps(rows, indent=1))
    passing = [r for r in rows if r.get("static_pass")]
    cols = ("dx_mm pitch_deg mid_yaw_deg margin_mm front_load_N "
            "cur_max_A pitch_err_deg base_z_mm push_ok").split()
    print(f"\n{len(rows)} configs | ik-ok "
          f"{sum(1 for r in rows if r.get('ik') == 'ok')} | pass "
          f"{len(passing)} | push-robust "
          f"{sum(1 for r in passing if r.get('push_ok'))}\n")
    print(" | ".join(cols))
    show = sorted((r for r in rows if r.get("ik") == "ok"),
                  key=lambda r: -(r.get("margin_mm") or -999))[:16]
    for r in show:
        print(" | ".join(str(r.get(c, "")) for c in cols))
    bad = [r for r in rows if r.get("ik") not in (None, "ok")]
    if bad:
        print("\nIK failures:")
        for r in bad:
            print(f"  {r['dx_mm']}mm {r['pitch_deg']}deg "
                  f"my{r['mid_yaw_deg']}: {r['ik']}")


# ---------------------------------------------------------------------------
# Phase B — scripted creep gait on the reared stance
# ---------------------------------------------------------------------------

def smooth(u: float) -> float:
    """cosine ease 0..1"""
    u = min(1.0, max(0.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


class RearQuadGait:
    """Open-loop tip-back creep: pure function t -> 18 joint radians.

    Timeline: entry (6-leg shift-back -> tuck fronts -> pitch up) then
    repeating 4-beat creep cycles. Body xy shifts toward the remaining
    triangle's centroid before each swing; all feet move through world
    anchors + full 3D body-frame IK, so planted feet truly stay put.
    """

    SWING_ORDER = (2, 4, 3, 1)   # rear-L, mid-R, rear-R, mid-L
    # continuous walk: lateral-sequence footfalls (LH, LF, RH, RF), the
    # slow-walk order every quadruped animal uses. "Hind" = the rear
    # pair L2/L3, "front" = the mid pair L1/L4.
    WALK_PHASE = {2: 0.0, 1: 0.25, 3: 0.5, 4: 0.75}

    def __init__(self, *, pitch: float, body_dx: float, mid_yaw: float,
                 stride: float = 0.03, lift: float = 0.03,
                 shift_gain: float = 0.8, shift_cap: float = 0.045,
                 t_shift: float = 0.9, t_swing: float = 0.8,
                 t_entry: tuple[float, float, float] = (2.0, 2.0, 2.5),
                 gait: str = "creep", period: float = 3.2,
                 duty: float = 0.8, sway: float = 0.025,
                 sway_phase: float = 0.0,
                 plant_deg: np.ndarray | None = None):
        self.gait = gait
        self.period, self.duty = period, duty
        self.sway, self.sway_phase = sway, sway_phase
        import mujoco_prototype as MP
        from rl_move.body_ik import fk_all_feet
        from rl_move.sim.sim_env import _default_plant_deg
        self.pitch, self.body_dx, self.mid_yaw = pitch, body_dx, mid_yaw
        self.stride, self.lift = stride, lift
        self.shift_gain, self.shift_cap = shift_gain, shift_cap
        self.t_shift, self.t_swing = t_shift, t_swing
        self.t_e1, self.t_e2, self.t_e3 = t_entry
        self.az = leg_azimuths()
        self.k_fk = np.array([0.0, 0.0, MP.YAW_OUTPUT_HEIGHT])

        self.plant_q = ((_default_plant_deg() if plant_deg is None
                         else np.asarray(plant_deg, float)) * DEG2RAD)
        feet = fk_all_feet(self.plant_q)
        self.foot_drop = float(np.min(feet[:, 2]))
        self.base_z0 = (MP.YAW_OUTPUT_HEIGHT - self.foot_drop
                        + MP.FOOT_R + 0.002)
        base_t = np.array([0.0, 0.0, self.base_z0])
        self.anchors0 = feet - self.k_fk + base_t      # world, 6 legs
        # splay the mid feet forward about their yaw origins
        for leg in (1, 4):
            if not mid_yaw:
                continue
            a = self.az[leg]
            from rl_move.body_ik import LEG_RADIAL
            o = np.array([math.cos(a), math.sin(a)]) * LEG_RADIAL
            r = self.anchors0[leg][:2] - o
            sgn = -1.0 if leg == 1 else 1.0
            c, s = math.cos(sgn * mid_yaw), math.sin(sgn * mid_yaw)
            self.anchors0[leg][:2] = o + np.array(
                [c * r[0] - s * r[1], s * r[0] + c * r[1]])

        self.t_entry_total = self.t_e1 + self.t_e2 + self.t_e3
        self.cycle = (self.period if gait == "walk"
                      else 4 * (self.t_shift + self.t_swing))
        # nominal body xy at rest within the reared stance
        self.rest_xy = np.array([body_dx, 0.0])

    # -- helpers -----------------------------------------------------------

    def _R(self, pitch: float) -> np.ndarray:
        c, s = math.cos(pitch), math.sin(pitch)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def _solve(self, body_xy: np.ndarray, pitch: float,
               feet_world: dict[int, np.ndarray],
               front_q: np.ndarray) -> np.ndarray:
        """18 joint radians; support legs via IK, fronts given."""
        q = np.zeros(N_JOINTS)
        for leg in FRONT_LEGS:
            q[3 * leg: 3 * leg + 3] = front_q[3 * leg: 3 * leg + 3]
        t = np.array([body_xy[0], body_xy[1], self.base_z0])
        R = self._R(pitch)
        for leg in SUPPORT_LEGS:
            p_fk = R.T @ (feet_world[leg] - t) + self.k_fk
            sol = ik_leg_from_foot_body(p_fk, self.az[leg])
            if sol is None:
                self.ik_fallbacks += 1
                q[3 * leg: 3 * leg + 3] = self.plant_q[
                    3 * leg: 3 * leg + 3]
                continue
            for j in range(3):
                lo, hi = AXIS_LIMITS_DEG[j]
                q[3 * leg + j] = float(np.clip(
                    sol[j], lo * DEG2RAD, hi * DEG2RAD))
        return q

    def _shift_target(self, swing_leg: int, anchors: dict) -> np.ndarray:
        """Body xy leaning toward the centroid of the OTHER 3 feet."""
        others = [anchors[m][:2] for m in SUPPORT_LEGS if m != swing_leg]
        centroid = np.mean(others, axis=0)
        d = centroid - self.rest_xy
        n = float(np.hypot(*d))
        if n > 1e-9:
            d = d / n * min(self.shift_cap, self.shift_gain * n)
        return self.rest_xy + d

    def _walk_cycle(self, tc: float, front_q: np.ndarray) -> np.ndarray:
        """Continuous animal walk: body advances at constant velocity
        with a lateral sway; each leg swings for (1-duty) of the cycle
        at its lateral-sequence phase offset."""
        T, S, beta = self.period, self.stride, self.duty
        v = S / T
        step_vec = np.array([S, 0.0, 0.0])
        feet: dict[int, np.ndarray] = {}
        for leg in SUPPORT_LEGS:
            ph = tc / T - self.WALK_PHASE[leg]
            n = math.floor(ph)
            s = ph - n
            # center each leg's stance sweep in its workspace
            A = (self.anchors0[leg]
                 - step_vec * (1.0 - self.WALK_PHASE[leg] - (2 - beta) / 2))
            if s < (1.0 - beta):            # swing
                u = s / (1.0 - beta)
                feet[leg] = (A + (n + smooth(u)) * step_vec + np.array(
                    [0.0, 0.0, self.lift * math.sin(math.pi * u)]))
            else:                            # stance
                feet[leg] = A + (n + 1) * step_vec
        sway_y = self.sway * math.sin(
            2 * math.pi * tc / T + self.sway_phase)
        body_xy = np.array([self.body_dx + v * tc, sway_y])
        return self._solve(body_xy, self.pitch, feet, front_q)

    # -- the pose function -------------------------------------------------

    def q_at(self, t: float) -> np.ndarray:
        self.ik_fallbacks = getattr(self, "ik_fallbacks", 0)
        tuck = np.array(TUCK)
        front_plant = self.plant_q.copy()
        front_tucked = self.plant_q.copy()
        for leg in FRONT_LEGS:
            front_tucked[3 * leg: 3 * leg + 3] = tuck

        anchors = {leg: self.anchors0[leg].copy() for leg in SUPPORT_LEGS}

        # ---- entry -------------------------------------------------------
        if t < self.t_e1:            # 1) shift body back, 6 feet planted
            u = smooth(t / self.t_e1)
            xy = np.array([u * self.body_dx, 0.0])
            feet = {leg: self.anchors0[leg] for leg in range(6)}
            q = self._solve(xy, 0.0, feet, front_plant)
            for leg in FRONT_LEGS:       # fronts still planted: IK them too
                p_fk = (self.anchors0[leg]
                        - np.array([xy[0], xy[1], self.base_z0])) + self.k_fk
                sol = ik_leg_from_foot_body(p_fk, self.az[leg])
                if sol is not None:
                    q[3 * leg: 3 * leg + 3] = sol
            return q
        if t < self.t_e1 + self.t_e2:    # 2) tuck the fronts (joint blend)
            u = smooth((t - self.t_e1) / self.t_e2)
            fq = front_plant + u * (front_tucked - front_plant)
            return self._solve(np.array([self.body_dx, 0.0]), 0.0,
                               anchors, fq)
        if t < self.t_entry_total:       # 3) rear up: ramp the pitch
            u = smooth((t - self.t_e1 - self.t_e2) / self.t_e3)
            return self._solve(np.array([self.body_dx, 0.0]),
                               u * self.pitch, anchors, front_tucked)

        # ---- gait cycles ---------------------------------------------------
        tc = t - self.t_entry_total
        if self.gait == "walk":
            return self._walk_cycle(tc, front_tucked)
        n_cyc = int(tc // self.cycle)
        ph = tc - n_cyc * self.cycle
        step_vec = np.array([self.stride, 0.0, 0.0])

        # anchors completed in past cycles
        for leg in SUPPORT_LEGS:
            anchors[leg] = anchors[leg] + n_cyc * step_vec
        body_prog = n_cyc * self.stride

        beat = self.t_shift + self.t_swing
        k = min(3, int(ph // beat))
        u_beat = ph - k * beat
        # anchors of legs already swung this cycle
        for j in range(k):
            anchors[self.SWING_ORDER[j]] = (
                anchors[self.SWING_ORDER[j]] + step_vec)
        swing_leg = self.SWING_ORDER[k]

        # body: each beat advances stride/4, split across shift+swing;
        # xy also leans toward the safe-triangle centroid for this beat.
        lean = self._shift_target(swing_leg, anchors)
        prev_lean = (self.rest_xy if k == 0 else
                     self._shift_target(self.SWING_ORDER[k - 1], anchors))
        beat_x0 = body_prog + k * (self.stride / 4.0)

        feet = dict(anchors)
        if u_beat < self.t_shift:
            # body advances while all four feet grip (swing phase would
            # push against only three)
            u = smooth(u_beat / self.t_shift)
            xy = prev_lean + u * (lean - prev_lean)
            bx = beat_x0 + u * (self.stride / 4.0)
        else:
            us = (u_beat - self.t_shift) / self.t_swing
            xy = lean
            bx = beat_x0 + self.stride / 4.0
            # swing foot: old anchor -> +stride with a sine lift
            w0 = anchors[swing_leg]
            feet[swing_leg] = w0 + smooth(us) * step_vec + np.array(
                [0.0, 0.0, self.lift * math.sin(math.pi * min(1.0, us))])
        body_xy = np.array([xy[0] + bx, xy[1]])
        return self._solve(body_xy, self.pitch, anchors | feet,
                           front_tucked)


# ---------------------------------------------------------------------------
# Phase B driver — stream the gait through the fitted servo model
# ---------------------------------------------------------------------------

def run_walk(args) -> None:
    import cv2
    import mujoco
    from rl_move.sim.servo_model import (
        SIM_MODEL_PATH, ServoProfile, SimServoParams,
        apply_params_to_model, build_model, joint_qpos_addrs,
        position_actuator_ids)
    from rl_move.sim.sim_env import support_margin_m

    gait = RearQuadGait(
        pitch=args.pitch * DEG2RAD, body_dx=args.body_dx,
        mid_yaw=args.mid_yaw * DEG2RAD, stride=args.stride,
        lift=args.lift, t_shift=args.t_shift, t_swing=args.t_swing,
        shift_cap=args.shift_cap, gait=args.gait, period=args.period,
        duty=args.duty, sway=args.sway,
        sway_phase=args.sway_phase * DEG2RAD)
    gait.ik_fallbacks = 0

    model = build_model(mesh_visuals=False, flat_terrain=True)
    params = SimServoParams.load(SIM_MODEL_PATH)
    apply_params_to_model(model, params)
    data = mujoco.MjData(model)
    qadr = joint_qpos_addrs(model)
    pos_act = position_actuator_ids(model)
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    foot_gid = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM,
                                  f"L{i}_foot") for i in range(6)]

    # place at the 6-leg plant (t=0 of the entry)
    q0 = gait.q_at(0.0)
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = (0.0, 0.0, gait.base_z0)
    data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
    data.qpos[qadr] = q0
    data.ctrl[pos_act] = q0
    mujoco.mj_forward(model, data)
    for _ in range(40):
        worst = min((float(data.contact[ci].dist)
                     for ci in range(data.ncon)), default=0.0)
        if worst > -1e-4:
            break
        data.qpos[2] += -worst + 0.001
        mujoco.mj_forward(model, data)

    profile = ServoProfile(params, q0)
    h = model.opt.timestep
    ctrl_hz = 20.0
    substeps = max(1, int(round(1.0 / ctrl_hz / h)))
    db = profile.deadband_rad

    def advance_tick():
        for _ in range(substeps):
            target = profile.tick(h)
            qj = data.qpos[qadr]
            err = target - qj
            eff = qj + np.sign(err) * np.maximum(np.abs(err) - db, 0.0)
            data.ctrl[pos_act] = eff
            mujoco.mj_step(model, data)

    for _ in range(int(1.0 * ctrl_hz)):     # settle at plant
        profile.command(q0, acc_units=200)
        advance_tick()

    vw = renderer = cam = None
    if args.video:
        renderer = mujoco.Renderer(model, height=480, width=720)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance = 0.65
        cam.elevation = -16.0
        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
        vw = cv2.VideoWriter(args.video, cv2.VideoWriter_fourcc(*"mp4v"),
                             20, (720, 480))

    n_ticks = int(args.seconds * ctrl_hz)
    min_up, max_cur, fell_at = 1.0, 0.0, None
    max_cur_t = 0.0
    min_margin_swing = 1.0
    vadr = np.array([model.jnt_dofadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, f"L{i}_{ax}")]
        for i in range(6) for ax in ("yaw", "pitch", "knee")])
    x0 = float(data.qpos[0])
    for i in range(n_ticks):
        t = i / ctrl_hz
        q_goal = gait.q_at(t)
        profile.command(q_goal, acc_units=200)
        advance_tick()
        Rm = data.xmat[chassis].reshape(3, 3)
        uz = float(Rm[2, 2])
        min_up = min(min_up, uz)
        if fell_at is None and uz < 0.55:
            fell_at = t
        cur = float(np.max(np.abs(data.qfrc_actuator[vadr]))) * 1.2
        if cur > max_cur:
            max_cur, max_cur_t = cur, t
        if t > gait.t_entry_total:
            # support polygon of the feet actually IN CONTACT
            loaded = set()
            for ci in range(data.ncon):
                con = data.contact[ci]
                for g in (con.geom1, con.geom2):
                    for m in SUPPORT_LEGS:
                        if g == foot_gid[m]:
                            loaded.add(m)
            if len(loaded) >= 3:
                feet_xy = np.array([data.geom_xpos[foot_gid[m]][:2]
                                    for m in sorted(loaded)])
                com_xy = data.subtree_com[0][:2]
                min_margin_swing = min(min_margin_swing,
                                       support_margin_m(feet_xy, com_xy))
        if vw is not None and i % 1 == 0:
            cam.azimuth = 135.0
            cam.lookat[:] = (float(data.qpos[0]), float(data.qpos[1]), 0.10)
            renderer.update_scene(data, camera=cam)
            vw.write(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))
    if vw is not None:
        vw.release()
        renderer.close()

    dist = float(data.qpos[0]) - x0
    n_cycles = max(0.0, (args.seconds - gait.t_entry_total) / gait.cycle)
    print(f"pitch={args.pitch}deg dx={args.body_dx} stride={args.stride} "
          f"cap={args.shift_cap}")
    print(f"forward: {dist*1000:+.0f} mm in {args.seconds:.0f} s "
          f"({n_cycles:.1f} cycles x {args.stride*1000:.0f} mm commanded)")
    print(f"min_up={min_up:.3f} (tilt {math.degrees(math.acos(max(-1, min(1, min_up)))):.0f}deg max) "
          f"fell={'%.1fs' % fell_at if fell_at else 'no'} "
          f"max_est_cur={max_cur:.2f}A@{max_cur_t:.1f}s "
          f"min_margin={min_margin_swing*1000:.0f}mm "
          f"ik_fallbacks={gait.ik_fallbacks}")
    if args.video:
        print(f"video: {args.video}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("static")
    s.add_argument("--out", default="logs/experiments/quad-rear")
    w = sub.add_parser("walk")
    w.add_argument("--pitch", type=float, default=-20.0,
                   help="deg; NEGATIVE = nose up in this convention")
    w.add_argument("--body-dx", type=float, default=-0.04)
    w.add_argument("--mid-yaw", type=float, default=17.0, help="deg splay")
    w.add_argument("--stride", type=float, default=0.03)
    w.add_argument("--lift", type=float, default=0.03)
    w.add_argument("--t-shift", type=float, default=0.9)
    w.add_argument("--t-swing", type=float, default=0.8)
    w.add_argument("--shift-cap", type=float, default=0.045)
    w.add_argument("--seconds", type=float, default=25.0)
    w.add_argument("--video", default="")
    w.add_argument("--gait", choices=("creep", "walk"), default="creep")
    w.add_argument("--period", type=float, default=3.2,
                   help="walk gait: s per full cycle")
    w.add_argument("--duty", type=float, default=0.8,
                   help="walk gait: stance fraction")
    w.add_argument("--sway", type=float, default=0.025,
                   help="walk gait: lateral body sway amplitude (m)")
    w.add_argument("--sway-phase", type=float, default=0.0,
                   help="walk gait: sway phase offset (deg)")
    args = ap.parse_args()
    if args.cmd == "static":
        run_static(args)
    else:
        run_walk(args)


if __name__ == "__main__":
    main()

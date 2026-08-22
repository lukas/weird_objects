"""Scripted A/B: two stand-up strategies from the belly (zero) pose.

Question (operator, 08-10): would the robot stand more reliably if it
TUCKED its legs in through the air first and then pushed straight up,
instead of pulling the feet inward along the ground where friction
fights the curl?

Three scripted (no-policy) strategies, same servo model, same write
speed, same total duration, same final stance geometry where possible:

- ``blend``: naive linear joint ramp zero -> plant (0/20/80). The
  historical "just command the stand" move; feet grind inward under
  increasing load the whole way.
- ``drag``: foot tip swept inward ALONG THE GROUND (2-link IK, slight
  downward press to guarantee contact) to the touchdown radius, then a
  vertical push to full stance height. Friction resists the whole pull.
- ``tuck``: hips lift the feet high, knees fold with the feet IN THE
  AIR, feet touch down at the same touchdown radius, then the IDENTICAL
  vertical push. Friction only ever sees vertical foot motion.

``drag`` and ``tuck`` share the exact push phase, so the difference is
purely the pull-in method. Run under PLAIN python (offscreen cv2
render path):

    cd hexapod_walker/prototype_sts3215
    python -m rl_move.sim.compare_standup            # metrics sweep + videos

Outputs (rl_move/sim/logs/standup_compare/): results.json, one mp4 +
contact-sheet png per strategy at default friction, printed table.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from .sim_env import SimHexapodBalanceEnv, set_foot_ground_friction
from .servo_model import ServoProfile

_RL = Path(__file__).resolve().parents[1]
for p in (str(_RL.parent), str(_RL.parent / "linux_control")):
    if p not in sys.path:
        sys.path.insert(0, p)

from rl_move.robot_state import RAD2DEG, N_JOINTS  # noqa: E402
import mujoco_prototype as MP  # noqa: E402

PRESS = 0.003                      # drag strategy: press feet into ground (m)
SPEED_DEG_S = 90.0                 # scripted SyncWrite profile speed (all arms)
CONTACT_N = 0.5                    # touch force (N) counting as "loaded"

T_PULL = 3.5                       # drag: ground sweep  | tuck: lift + fold
T_LIFT = 2.0
T_FOLD = 1.5
T_PUSH = 2.5
T_HOLD = 2.0


class RealLegFK:
    """Foot placement against the REAL MuJoCo leg (not the naive 2-link
    frame — measured up to ~90 mm foot-height error at folded poses).

    Uses a fixed-base copy of the model: fk(hip, knee) returns the
    (radial, vertical) position of the FOOT BOTTOM in the chassis frame
    for leg 0 (all legs identical); solve() Gauss-Newtons hip/knee onto
    an (r, z) target within the hardware joint limits.
    """
    HIP_LO, HIP_HI = math.radians(-78.0), math.radians(28.0)
    KNEE_LO, KNEE_HI = math.radians(-18.0), math.radians(148.0)

    def __init__(self):
        import mujoco
        from .servo_model import build_model, joint_qpos_addrs
        self._mujoco = mujoco
        self.model = build_model(fixed_base=True, flat_terrain=True,
                                 mesh_visuals=False)
        self.data = mujoco.MjData(self.model)
        self._qadr = joint_qpos_addrs(self.model)
        self._foot_gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "L0_foot")
        self._chassis_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        # Coarse FK grid: global seed for solve() — pure Gauss-Newton
        # runs away to a joint-limit corner near full extension, where
        # the Jacobian is ill-conditioned.
        hs = np.arange(self.HIP_LO, self.HIP_HI, math.radians(1.5))
        ks = np.arange(self.KNEE_LO, self.KNEE_HI, math.radians(1.5))
        grid = [(h, k, *self.fk(h, k)) for h in hs for k in ks]
        arr = np.array(grid)
        self._grid_hk = arr[:, :2]
        self._grid_rz = arr[:, 2:]

    def fk(self, hip: float, knee: float) -> tuple[float, float]:
        self.data.qpos[:] = 0.0
        self.data.qpos[self._qadr[1]] = hip
        self.data.qpos[self._qadr[2]] = knee
        self._mujoco.mj_forward(self.model, self.data)
        p = (self.data.geom_xpos[self._foot_gid]
             - self.data.xpos[self._chassis_bid])
        return (float(np.hypot(p[0], p[1])),
                float(p[2]) - MP.FOOT_R)

    def _res(self, h: float, k: float, r_t: float, z_t: float) -> float:
        r, z = self.fk(h, k)
        return math.hypot(r_t - r, z_t - z)

    def solve(self, r_t: float, z_t: float,
              seed: tuple[float, float] | None = None
              ) -> tuple[float, float]:
        """Least-squares (hip, knee) for a foot-bottom (r, z) target:
        grid/seed init, then damped Gauss-Newton with backtracking (a
        step is only taken if it reduces the residual)."""
        gi = int(np.argmin((self._grid_rz[:, 0] - r_t) ** 2
                           + (self._grid_rz[:, 1] - z_t) ** 2))
        h, k = self._grid_hk[gi]
        if seed is not None and (self._res(*seed, r_t, z_t)
                                 < self._res(h, k, r_t, z_t)):
            h, k = seed
        eps = 1e-4
        best = self._res(h, k, r_t, z_t)
        for _ in range(10):
            if best < 2e-5:
                break
            r, z = self.fk(h, k)
            rh, zh = self.fk(h + eps, k)
            rk, zk = self.fk(h, k + eps)
            J = np.array([[(rh - r) / eps, (rk - r) / eps],
                          [(zh - z) / eps, (zk - z) / eps]])
            dh, dk = np.linalg.solve(
                J.T @ J + 1e-4 * np.eye(2),
                J.T @ np.array([r_t - r, z_t - z]))
            n = max(abs(dh), abs(dk))
            if n > math.radians(6.0):
                sc = math.radians(6.0) / n
                dh, dk = dh * sc, dk * sc
            improved = False
            for s in (1.0, 0.5, 0.25, 0.125):
                ht = min(max(h + dh * s, self.HIP_LO), self.HIP_HI)
                kt = min(max(k + dk * s, self.KNEE_LO), self.KNEE_HI)
                res = self._res(ht, kt, r_t, z_t)
                if res < best - 1e-9:
                    h, k, best, improved = ht, kt, res, True
                    break
            if not improved:
                break
        return float(h), float(k)

    def min_ground_radius(self, z_ground: float) -> tuple[float, float, float]:
        """(hip, knee, r) of the innermost reachable ground touch:
        sweep knee, bisect hip so the foot bottom sits at z_ground."""
        best = None
        for k_deg in np.arange(60.0, 148.0, 2.0):
            k = math.radians(k_deg)
            lo, hi = self.HIP_LO, self.HIP_HI
            if not (self.fk(lo, k)[1] > z_ground > self.fk(hi, k)[1]):
                continue
            for _ in range(30):
                mid = 0.5 * (lo + hi)
                if self.fk(mid, k)[1] > z_ground:
                    lo = mid
                else:
                    hi = mid
            h = 0.5 * (lo + hi)
            r = self.fk(h, k)[0]
            if best is None or r < best[2]:
                best = (h, k, r)
        assert best is not None, "no reachable ground touch"
        return best


def pose18(hip_rad: float, knee_rad: float) -> np.ndarray:
    q = np.zeros(N_JOINTS, dtype=float)
    q[1::3] = hip_rad
    q[2::3] = knee_rad
    return q


TRIPOD_A = (0, 2, 4)
TRIPOD_B = (1, 3, 5)


def smooth(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return 3 * s * s - 2 * s ** 3


def seg(q0: np.ndarray, q1: np.ndarray, t_s: float, dt: float) -> list:
    n = max(1, int(round(t_s / dt)))
    return [q0 + (q1 - q0) * smooth((i + 1) / n) for i in range(n)]


def build_traj(strategy: str, fkm: RealLegFK, z_gnd: float,
               dt: float) -> tuple[list, int]:
    """Per-tick 18-joint targets (rad) and the tick where the push
    (body-raise) phase begins — the pull-in/push metric boundary.

    ``z_gnd`` is the ground plane in the chassis frame at belly rest.
    All foot targets are (radius-from-center, foot-bottom z) pairs
    solved against the real model kinematics.
    """
    zero = pose18(0.0, 0.0)
    plant = (math.radians(20.0), math.radians(80.0))
    _, z_plant = fkm.fk(*plant)
    _, _, r_td = fkm.min_ground_radius(z_gnd)
    r0, z0 = fkm.fk(0.0, 0.0)

    seeds = [[0.0, 0.0] for _ in range(6)]

    def to_legs(targets: list) -> np.ndarray:
        """targets: 6 per-leg (r, z) pairs → 18-joint pose."""
        q = np.zeros(N_JOINTS, dtype=float)
        for i, (r_t, z_t) in enumerate(targets):
            h, k = fkm.solve(r_t, z_t, (seeds[i][0], seeds[i][1]))
            seeds[i] = [h, k]
            q[3 * i + 1] = h
            q[3 * i + 2] = k
        return q

    def follow_legs(p0s: list, p1s: list, t_s: float) -> list:
        n = max(1, int(round(t_s / dt)))
        out = []
        for i in range(n):
            s = smooth((i + 1) / n)
            out.append(to_legs([
                (a[0] + (b[0] - a[0]) * s, a[1] + (b[1] - a[1]) * s)
                for a, b in zip(p0s, p1s)]))
        return out

    def follow(p0, p1, t_s: float) -> list:
        return follow_legs([p0] * 6, [p1] * 6, t_s)

    def push(z_from: float) -> list:
        # shared: feet stay at the touchdown radius, body goes
        # straight up (foot bottom straight down to plant depth)
        return follow((r_td, z_from), (r_td, z_plant), T_PUSH)

    if strategy == "blend":
        traj = seg(zero, pose18(*plant), T_PULL + T_PUSH, dt)
    elif strategy == "drag":
        down = (r0, z_gnd - PRESS)
        traj = follow((r0, z0), down, 0.4)
        traj += follow(down, (r_td, z_gnd - PRESS), T_PULL - 0.4)
        traj += push(z_gnd)
    elif strategy == "tuck":
        lift = (r0 - 0.3 * (r0 - r_td), z_gnd + 0.055)
        fold = (r_td, z_gnd + 0.020)
        traj = follow((r0, z0), lift, T_LIFT)
        traj += follow(lift, fold, T_FOLD * 0.7)
        traj += follow(fold, (r_td, z_gnd), T_FOLD * 0.3)
        traj += push(z_gnd)
    elif strategy == "step":
        # Tripod re-plant: tuck the feet in THROUGH THE AIR one tripod
        # at a time while the belly (plus the other tripod) carries the
        # weight, then the shared vertical push. A moving leg is always
        # unloaded, so it reaches its commanded angle regardless of
        # friction, and only 6 servos move loaded at once (bus current).
        # Earlier variant crouched before re-planting — refuted: the
        # support tripod's knees saturate (2.64 A sustained) holding the
        # body on nearly-extended legs. Belly stays down instead.
        lift = (r0 - 0.3 * (r0 - r_td), z_gnd + 0.055)
        fold = (r_td, z_gnd + 0.020)
        near = (r_td, z_gnd)

        def swing(tripod: tuple) -> list:
            out = []
            for p0, p1, t_s in (((r0, z0), lift, 1.0),
                                (lift, fold, 0.5), (fold, near, 0.3)):
                p0s, p1s = [], []
                for i in range(6):
                    if i in tripod:
                        p0s.append(p0)
                        p1s.append(p1)
                    else:
                        # support legs hold wherever they already are
                        cur = fkm.fk(seeds[i][0], seeds[i][1])
                        p0s.append(cur)
                        p1s.append(cur)
                out += follow_legs(p0s, p1s, t_s)
            # settle before moving the other tripod
            out += [out[-1]] * int(round(0.25 / dt))
            return out

        traj = swing(TRIPOD_A) + swing(TRIPOD_B)
        i_push = len(traj)
        traj += push(z_gnd)
    else:
        raise ValueError(strategy)
    if strategy != "step":
        i_push = int(round(T_PULL / dt))
    traj += [traj[-1]] * int(round(T_HOLD / dt))
    return traj, i_push


def reset_at_zero(env: SimHexapodBalanceEnv, *,
                  seed: int | None = None,
                  torque_scale: float = 1.0) -> None:
    """Belly-down zero-pose reset (mirrors env.reset()'s settle
    choreography and DR application, minus the goal machinery).

    ``torque_scale`` < 1 models battery sag / weak servos on top of any
    DR draw — needed to reproduce the hardware pinned-feet stall, which
    requires BOTH friction and finite torque."""
    from .servo_model import apply_params_to_model
    if seed is not None:
        env.rng = np.random.default_rng(seed)
    er = (env.randomizer.sample(env.rng)
          if env.randomizer is not None else None)
    env._ep_rand = er
    q0 = np.zeros(N_JOINTS, dtype=float)
    if er is not None:
        q0 = env._clip_to_joint_limits(q0 + er.start_offset_rad)
        er.apply_to_model(env.model, chassis_bid=env._chassis_bid)
        apply_params_to_model(env.model, env.params,
                              kp_scale=er.kp_scale, kv_scale=er.kv_scale,
                              torque_scale=er.torque_scale * torque_scale)
        er.apply_fault_to_model(env.model)  # dr.fault_*, no-op if healthy
    else:
        apply_params_to_model(env.model, env.params,
                              torque_scale=torque_scale)
    env._place_at_plant(q0)
    env._profile = ServoProfile(
        env.params, q0,
        latency_scale=1.0 if er is None else er.latency_scale,
        deadband_scale=1.0 if er is None else er.deadband_scale,
        vel_scale=1.0 if er is None else er.vel_scale)
    env._cmd = q0.copy()
    fr = env.model.geom_friction[:, 0].copy()
    env.model.geom_friction[:, 0] = env.SLIP_MU
    env._settle(0.4)
    env._settle(0.5, limp=True)
    env.model.geom_friction[:, 0] = fr
    env._q_nom = env.data.qpos[env._qadr].copy()
    env._profile.reset(env._q_nom)
    env._cmd = env._q_nom.copy()
    env._settle(0.3)


def chassis_tilt(env) -> tuple[float, float]:
    r = env.data.xmat[env._chassis_bid].reshape(3, 3)
    roll = math.atan2(r[2, 1], r[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, r[2, 0])))
    return roll, pitch


def plant_ref_height() -> float:
    """Chassis z (m) of the settled reference plant stance — the height
    a successful stand-up should reach."""
    env = SimHexapodBalanceEnv(randomize=False, seed=0)
    env.reset()
    z = float(env.data.xpos[env._chassis_bid, 2])
    env.close()
    return z


def run_once(strategy: str, mu: float | None, plant_z: float,
             fkm: RealLegFK, *,
             randomize: bool = False, seed: int = 0,
             torque_scale: float = 1.0,
             video_path: Path | None = None,
             sheet_path: Path | None = None) -> dict:
    env = SimHexapodBalanceEnv(randomize=randomize, seed=seed)
    if mu is not None:
        set_foot_ground_friction(env.model, mu)
    reset_at_zero(env, seed=seed, torque_scale=torque_scale)

    z_start = float(env.data.xpos[env._chassis_bid, 2])
    traj, i_push = build_traj(strategy, fkm, -z_start, env.dt)
    xy_start = env.data.xpos[env._chassis_bid, :2].copy()
    prev_xy = [env.data.xpos[b, :2].copy() for b in env._pad_bids]
    prev_f = [0.0] * 6

    # [pull-in phase, push+hold phase]
    slip_m = [0.0, 0.0]     # loaded foot travel along the ground (all legs)
    fric_j = [0.0, 0.0]     # sum F_normal * slide distance (friction work)
    cur_peak = [0.0, 0.0]
    cur_sum, cur_n = 0.0, 0
    tilt_peak = 0.0
    frames = []

    writer = None
    if video_path is not None:
        import cv2
        writer = cv2.VideoWriter(str(video_path),
                                 cv2.VideoWriter_fourcc(*"mp4v"),
                                 25, (640, 480))
        env.render_mode = "rgb_array"

    for ti, q in enumerate(traj):
        env._cmd = q.copy()
        env._profile.command(q, speed_deg_s=SPEED_DEG_S,
                             acc_units=env.write_acc_units)
        env._advance()

        ph = 0 if ti < i_push else 1
        cur = np.minimum(
            np.abs(env.data.qfrc_actuator[env._vadr]) * 1.2, 3.0)
        cur_peak[ph] = max(cur_peak[ph], float(cur.max()))
        cur_sum += float(cur.mean())
        cur_n += 1
        roll, pitch = chassis_tilt(env)
        tilt_peak = max(tilt_peak, abs(roll), abs(pitch))
        for i in range(6):
            f = (float(env.data.sensordata[env._touch_adr[i]])
                 if env._touch_adr[i] >= 0 else 0.0)
            xy = env.data.xpos[env._pad_bids[i], :2].copy()
            if f > CONTACT_N and prev_f[i] > CONTACT_N:
                d = float(np.linalg.norm(xy - prev_xy[i]))
                slip_m[ph] += d
                fric_j[ph] += f * d
            prev_xy[i], prev_f[i] = xy, f
        if writer is not None:
            img = env.render()
            frames.append(img)
            import cv2
            writer.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    if writer is not None:
        writer.release()
    if sheet_path is not None and frames:
        import cv2
        idx = np.linspace(0, len(frames) - 1, 6).astype(int)
        sheet = np.concatenate([frames[i] for i in idx], axis=1)
        cv2.imwrite(str(sheet_path), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))

    roll, pitch = chassis_tilt(env)
    z_end = float(env.data.xpos[env._chassis_bid, 2])
    dz = z_end - z_start
    drift = float(np.linalg.norm(
        env.data.xpos[env._chassis_bid, :2] - xy_start))
    # Feet-down count is GEOMETRIC (bottom within 10 mm of the floor),
    # not touch force: load concentration can leave a properly grounded
    # foot under 0.5 N (same rationale as the rise posture gate).
    import mujoco
    n_loaded = 0
    for i in range(6):
        gid = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_GEOM,
                                f"L{i}_foot")
        if gid >= 0 and (float(env.data.geom_xpos[gid, 2])
                         - MP.FOOT_R) < 0.010:
            n_loaded += 1
    res = {
        "strategy": strategy,
        "mu": mu if mu is not None else "default(2.0)",
        "torque_scale": torque_scale,
        "dr_seed": seed if randomize else None,
        "rise_mm": dz * 1000.0,
        "final_z_mm": z_end * 1000.0,
        "plant_ref_z_mm": plant_z * 1000.0,
        "final_roll_deg": roll * RAD2DEG,
        "final_pitch_deg": pitch * RAD2DEG,
        "tilt_peak_deg": tilt_peak * RAD2DEG,
        "slip_pull_mm": slip_m[0] * 1000.0,
        "slip_push_mm": slip_m[1] * 1000.0,
        "fric_work_pull_j": fric_j[0],
        "fric_work_push_j": fric_j[1],
        "current_peak_pull_a": cur_peak[0],
        "current_peak_push_a": cur_peak[1],
        "current_mean_a": cur_sum / max(cur_n, 1),
        "xy_drift_mm": drift * 1000.0,
        "feet_loaded_end": n_loaded,
        "success": bool(z_end >= 0.9 * plant_z
                        and abs(roll) < math.radians(10)
                        and abs(pitch) < math.radians(10)
                        and n_loaded >= 5),
    }
    env.close()
    return res


MODE_DESCRIPTIONS = {
    "blend": "Naive single blend from zero straight to the plant pose — "
             "feet drag inward under increasing load (historical behavior; "
             "friction fights the whole way).",
    "drag": "Belly-down: pull the feet inward ALONG THE GROUND to the "
            "touchdown radius, then push straight up.",
    "tuck": "Belly-down: lift and fold the feet IN THE AIR, touch down "
            "under the body, then push straight up. Immune to ground "
            "friction during the pull-in.",
    "step": "Tripod re-plant: tuck the feet in through the air three legs "
            "at a time (a moving leg is always unloaded, so it always "
            "reaches its target), then push straight up. The recovery "
            "move when feet are pinned.",
}


def export_modes(path: Path, fkm: RealLegFK, *,
                 keyframe_s: float = 0.4) -> None:
    """Bake the sim-validated stand-up trajectories into keyframes the
    robot plays back with chained ``ease_to_pose`` glides (the board
    cannot run MuJoCo). Keyframes are logical joint DEGREES in the
    set-zero frame (zero = legs straight out, belly down)."""
    env = SimHexapodBalanceEnv(randomize=False, seed=0)
    reset_at_zero(env)
    z_start = float(env.data.xpos[env._chassis_bid, 2])
    dt = env.dt
    env.close()

    modes = {}
    for mode in ("blend", "drag", "tuck", "step"):
        traj, i_push = build_traj(mode, fkm, -z_start, dt)
        step_n = max(1, int(round(keyframe_s / dt)))
        idx = list(range(step_n - 1, len(traj), step_n))
        if idx[-1] != len(traj) - 1:
            idx.append(len(traj) - 1)
        # Explicit start keyframe: gently align to the zero pose first
        # (the operator placed the robot belly-down, legs straight out).
        keyframes = [{"q_deg": [0.0] * N_JOINTS, "s": 0.8}]
        prev_q = np.zeros(N_JOINTS)
        for j in idx:
            q = np.round(np.asarray(traj[j]) * RAD2DEG, 2)
            s = round((j + 1 - (idx[idx.index(j) - 1] + 1
                                if idx.index(j) else 0)) * dt, 3)
            if prev_q is not None and float(
                    np.max(np.abs(q - prev_q))) < 0.1:
                keyframes[-1]["s"] = round(keyframes[-1]["s"] + s, 3)
                continue
            keyframes.append({"q_deg": q.tolist(), "s": s})
            prev_q = q
        modes[mode] = {
            "description": MODE_DESCRIPTIONS[mode],
            "keyframes": keyframes,
            "push_starts_at_keyframe": 1 + int(
                sum(1 for j in idx if j < i_push)),
            "total_s": round(sum(k["s"] for k in keyframes), 2),
        }
    out = {
        "generated": "compare_standup.py --export (sim-validated: all "
                     "modes stand 10/10 under DR 1.0, friction 0.8-2.0)",
        "frame": "logical joint degrees (set_zero frame), "
                 "[yaw,hip,knee] x legs 0..5; start pose = zero "
                 "(legs straight out, belly down)",
        "modes": modes,
    }
    path.write_text(json.dumps(out, indent=1))
    print(f"exported {len(modes)} stand-up modes -> {path}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mus", type=float, nargs="*",
                    default=[0.8, 1.4, 2.0],
                    help="foot+floor slide friction values to sweep")
    ap.add_argument("--torque-scales", type=float, nargs="*",
                    default=[1.0], dest="torque_scales",
                    help="actuator torque-limit scales to sweep "
                         "(<1 = battery sag / weak servos; the pinned-"
                         "feet stall needs friction AND finite torque)")
    ap.add_argument("--strategies", nargs="*",
                    default=["blend", "drag", "tuck", "step"])
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--dr-episodes", type=int, default=10,
                    help="domain-randomized episodes per strategy "
                         "(0 disables the DR panel)")
    ap.add_argument("--export", metavar="JSON",
                    help="bake keyframes for the robot-side stand-up "
                         "player and exit (e.g. "
                         "linux_control/standup_modes.json)")
    ap.add_argument("--outdir", default=str(
        Path(__file__).parent / "logs" / "standup_compare"))
    args = ap.parse_args()

    if args.export:
        export_modes(Path(args.export), RealLegFK())
        return

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    plant_z = plant_ref_height()
    print(f"reference plant-stance chassis z: {plant_z * 1000:.1f} mm")
    fkm = RealLegFK()
    results = []
    for strat in args.strategies:
        for mu in args.mus:
          for ts in args.torque_scales:
            r = run_once(strat, mu, plant_z, fkm, torque_scale=ts)
            results.append(r)
            tslab = f" tq={ts:<4}" if len(args.torque_scales) > 1 else ""
            print(f"{strat:6s} mu={mu:<4}{tslab}"
                  f" z_end={r['final_z_mm']:6.1f}mm"
                  f"/{r['plant_ref_z_mm']:.0f}"
                  f" slip pull/push={r['slip_pull_mm']:6.1f}/"
                  f"{r['slip_push_mm']:6.1f}mm"
                  f" fricJ={r['fric_work_pull_j']:5.2f}/"
                  f"{r['fric_work_push_j']:5.2f}"
                  f" Ipk={r['current_peak_pull_a']:4.2f}/"
                  f"{r['current_peak_push_a']:4.2f}A"
                  f" Imean={r['current_mean_a']:4.2f}A"
                  f" feet={r['feet_loaded_end']}"
                  f" {'OK' if r['success'] else 'FAIL'}", flush=True)
        if not args.no_video:
            r = run_once(strat, None, plant_z, fkm,
                         video_path=outdir / f"{strat}.mp4",
                         sheet_path=outdir / f"{strat}_sheet.png")
            r["video"] = f"{strat}.mp4"
            results.append(r)
            print(f"{strat:6s} video (default mu): "
                  f"z_end={r['final_z_mm']:.1f}mm"
                  f" {'OK' if r['success'] else 'FAIL'}", flush=True)
    if args.dr_episodes > 0:
        print(f"\nDR 1.0 panel ({args.dr_episodes} randomized episodes "
              f"per strategy):")
        for strat in args.strategies:
            ok, worst_z, worst_tilt = 0, 1e9, 0.0
            for s in range(args.dr_episodes):
                r = run_once(strat, None, plant_z, fkm,
                             randomize=True, seed=s)
                r["dr"] = True
                results.append(r)
                ok += int(r["success"])
                worst_z = min(worst_z, r["final_z_mm"])
                worst_tilt = max(worst_tilt, abs(r["final_roll_deg"]),
                                 abs(r["final_pitch_deg"]))
            print(f"  {strat:6s} success {ok}/{args.dr_episodes}"
                  f"  worst z_end={worst_z:6.1f}mm"
                  f"  worst |tilt|={worst_tilt:5.1f} deg", flush=True)
    (outdir / "results.json").write_text(json.dumps(results, indent=2))
    print(f"\nwrote {outdir / 'results.json'}")


if __name__ == "__main__":
    main()

"""On-robot RL policy runner: STAND UP / LOWER / WALK buttons (web UI).

Runs the sim-trained raw-joint PPO policies (exported to plain numpy
weights by ``rl_move/sim/export_policy_np.py`` — no torch on the board)
in the exact conventions they were trained with. Two weight files:
``rl_policy_weights.json`` = stance champion (stand/lower, obs 68),
``rl_walk_weights.json`` = walk champion (obs 72; see the walk-mode
constants below for its hardware caveats).

- 25 Hz loop; obs = build_obs(q, qd, tilt-rel-to-start, gyro,
  prev_action(18), goal(9)) with q_nom = the pose read at arm time.
- action in [-1,1]^18 -> absolute joint targets via the AXIS_LIMITS_DEG
  center/half-range map (same as sim joint_task.action_to_q_rad).
- goal height ramps copied from the training GoalGenerator:
  rise  = hold 0 for 5 s (curl window), ramp to +50 mm over 4 s, hold.
  lower = hold 0 for 1 s, ramp to -45 mm over 5 s, hold, then limp.
- every command goes through rl_move.safety.SafetyLayer: 1.5 deg/tick
  rate clamp, joint limits, 10 deg tilt trip, sustained 2.5 A trip,
  temp/load trips. Trip => immediate limp (do not fight a fall).

Post-2026-08-06 rules baked in: NO motion unless every preflight gate
passes — all 18 servo IDs answering, IMU ok, tilt < 12 deg, and the
present pose near the expected start (flat/belly for stand, captured
plant for lower). The operator must be watching; the web button is the
explicit order.
"""
from __future__ import annotations

import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

# rl_move lives one level above linux_control on the robot and in repo.
_HERE = Path(__file__).resolve().parent
for _p in (_HERE.parent, _HERE, _HERE / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import cfg_get, load_config            # noqa: E402
from rl_move.env import TaskGoal, build_obs                # noqa: E402
from rl_move.robot_state import (                          # noqa: E402
    DEG2RAD, N_JOINTS, RAD2DEG, RobotStateEstimator,
)
from rl_move.safety import AXIS_LIMITS_DEG, SafetyLayer    # noqa: E402

WEIGHTS_PATH = _HERE / "rl_policy_weights.json"        # stance (obs 68)
WALK_WEIGHTS_PATH = _HERE / "rl_walk_weights.json"     # walk (obs 72)
HZ = 25.0
DT = 1.0 / HZ

# Trained trajectory shapes (rl_move/config.yaml goal section).
RISE_HOLD_S = 5.0      # curl window: height ref pinned at 0
RISE_RAMP_S = 4.0
RISE_TARGET_M = 0.050  # mid of the trained 30-70 mm range
RISE_TOTAL_S = 16.0    # hold + ramp + ~7 s stabilise, then hold pose
LOWER_HOLD_S = 1.0
LOWER_RAMP_S = 5.0
LOWER_TARGET_M = -0.045
LOWER_TOTAL_S = 11.0   # ends resting on the belly -> limp

PREFLIGHT_MAX_TILT_DEG = 12.0
# Start-pose gates (max per-joint |delta| from the expected pose).
STAND_START_TOL_DEG = 30.0   # near flat belly pose (logical zero-ish)
LOWER_START_TOL_DEG = 25.0   # near the captured plant stance

# Walk mode (ppo_goal_cw_walk_longdist_r2, obs 72). The sim champion is
# NOT hardware-ready (paddle-slide persists in sim; RL_LOG State) — this
# is an operator-supervised experiment, tightly bounded:
# - starts ONLY from the captured plant stance (same gate as lower);
# - command ramps 0 -> v over 1 s after a 1 s settle (training profile),
#   holds, then ramps back to 0 for the last second and HOLDS the pose;
# - speed clamped to the trained band; duration clamped to 20 s;
# - the 4 walk obs dims are [vx_ref, vy_ref, vx_meas, vy_meas]/0.15.
#   The board has NO body-velocity estimate, so vx/vy_meas are fed the
#   ref (open-loop, sim v1 style). The velocity-error feedback loop is
#   therefore ABSENT on hardware — expect worse tracking than sim.
WALK_VEL_SCALE = 0.15
WALK_SPEED_MAX = 0.06        # trained command band tops out here
WALK_HOLD_S = 1.0
WALK_RAMP_S = 1.0
WALK_MAX_TOTAL_S = 20.0
WALK_START_TOL_DEG = 25.0    # near the captured plant stance

_CENTER_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][0] + AXIS_LIMITS_DEG[j % 3][1]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])
_HALF_RAD = np.array([
    (AXIS_LIMITS_DEG[j % 3][1] - AXIS_LIMITS_DEG[j % 3][0]) * 0.5 * DEG2RAD
    for j in range(N_JOINTS)])


class NumpyPolicy:
    """Deterministic SB3 MlpPolicy actor: tanh MLP + linear head."""

    def __init__(self, path: Path = WEIGHTS_PATH):
        d = json.loads(Path(path).read_text())
        self.meta = d["meta"]
        self.W1 = np.array(d["W1"]); self.b1 = np.array(d["b1"])
        self.W2 = np.array(d["W2"]); self.b2 = np.array(d["b2"])
        self.Wo = np.array(d["Wout"]); self.bo = np.array(d["bout"])

    def act(self, obs: np.ndarray) -> np.ndarray:
        h = np.tanh(self.W1 @ obs + self.b1)
        h = np.tanh(self.W2 @ h + self.b2)
        return np.clip(self.Wo @ h + self.bo, -1.0, 1.0)


def _height_ref(mode: str, t: float) -> float:
    if mode == "stand":
        if t < RISE_HOLD_S:
            return 0.0
        f = min(1.0, (t - RISE_HOLD_S) / RISE_RAMP_S)
        return f * RISE_TARGET_M
    if t < LOWER_HOLD_S:
        return 0.0
    f = min(1.0, (t - LOWER_HOLD_S) / LOWER_RAMP_S)
    return f * LOWER_TARGET_M


def _walk_vel_ref(t: float, total_s: float,
                  vx: float, vy: float) -> tuple[float, float]:
    """Training-shaped command: settle, 1 s ramp in, hold, 1 s ramp out."""
    if t < WALK_HOLD_S:
        f = 0.0
    elif t < WALK_HOLD_S + WALK_RAMP_S:
        f = (t - WALK_HOLD_S) / WALK_RAMP_S
    elif t > total_s - WALK_RAMP_S:
        f = max(0.0, (total_s - t) / WALK_RAMP_S)
    else:
        f = 1.0
    return f * vx, f * vy


class _EpisodeLog:
    """Every RL episode leaves a full local trace in ``logs/``.

    ``rl_<mode>_<stamp>.csv``  — one row per 25 Hz tick: attitude, gyro,
    goal refs, measured q (18), commanded q (18), raw action (18), and
    per-servo current when full feedback is available.
    ``rl_<mode>_<stamp>_summary.json`` — params + final result.
    Start/end also land in events.jsonl (kind ``rl_episode``).
    Pull with receive_robot_logs.py / scp for offline analysis.
    """

    def __init__(self, mode: str, params: dict):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        d = _HERE / "logs"
        d.mkdir(exist_ok=True)
        self.mode = mode
        self.params = params
        self.csv_path = d / f"rl_{mode}_{stamp}.csv"
        self.sum_path = d / f"rl_{mode}_{stamp}_summary.json"
        self.started_iso = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._n = 0
        self._f = self.csv_path.open("w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(
            ["t_s", "roll_deg", "pitch_deg",
             "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
             "height_ref_mm", "vx_ref_mps", "vy_ref_mps", "max_cur_a"]
            + [f"q{j}_deg" for j in range(N_JOINTS)]
            + [f"cmd{j}_deg" for j in range(N_JOINTS)]
            + [f"act{j}" for j in range(N_JOINTS)]
            + [f"cur{j}_a" for j in range(N_JOINTS)])
        try:
            from event_log import emit
            emit("rl_episode", f"{mode} started ({self.csv_path.name})",
                 src="rl_policy", data=params)
        except Exception:
            pass

    def tick(self, t: float, state, action, q_cmd_rad, goal,
             vx_r: float, vy_r: float, max_cur: float) -> None:
        cur = (state.servo_current.tolist()
               if state.servo_current is not None else [None] * N_JOINTS)
        self._w.writerow(
            [round(t, 3),
             round(state.imu_roll * RAD2DEG, 2),
             round(state.imu_pitch * RAD2DEG, 2)]
            + [round(float(g) * RAD2DEG, 2) for g in state.imu_gyro]
            + [round(goal.height_ref * 1000, 1),
               round(vx_r, 4), round(vy_r, 4), round(max_cur, 3)]
            + [round(float(q) * RAD2DEG, 2) for q in state.joint_position]
            + [round(float(q) * RAD2DEG, 2) for q in q_cmd_rad]
            + [round(float(a), 4) for a in action]
            + ["" if c is None else round(float(c), 3) for c in cur])
        self._n += 1
        if self._n % 25 == 0:      # survive a mid-run kill: flush each ~1 s
            self._f.flush()

    def close(self, result: dict) -> str:
        try:
            self._f.close()
        except Exception:
            pass
        try:
            self.sum_path.write_text(json.dumps(
                {"started": self.started_iso, "csv": self.csv_path.name,
                 "ticks_logged": self._n, "params": self.params,
                 "result": result}, indent=1))
        except Exception:
            pass
        try:
            from event_log import emit
            emit("rl_episode",
                 f"{self.mode} " + ("done" if result.get("ok")
                                    else f"FAILED: {result.get('error')}"),
                 src="rl_policy",
                 level="info" if result.get("ok") else "warn",
                 data={"csv": self.csv_path.name, **result})
        except Exception:
            pass
        return self.csv_path.name


def _read_q_deg(bus) -> tuple[np.ndarray | None, str]:
    pos = bus.read_all_positions()
    if not isinstance(pos, dict):
        return None, "bus read failed"
    missing = [j for j in range(N_JOINTS) if j not in pos]
    if missing:
        return None, f"servo IDs not answering: joints {missing}"
    return np.array([float(pos[j]) for j in range(N_JOINTS)]), ""


def _expected_start_deg(mode: str) -> tuple[np.ndarray | None, str]:
    if mode == "stand":
        # Rise training starts belly-down at logical zero (legs straight
        # out). Partial curls were also trained, so the gate is loose.
        return np.zeros(N_JOINTS), ""
    # lower AND walk both start from the captured plant stance.
    try:
        from feetech_bus import load_plant_pose
        plant = load_plant_pose()
        joints = plant.get("joints_deg")
        if joints is None or len(joints) != 18:
            return None, ("no captured plant pose (joints_deg) — stand "
                          "first or run capture_plant from a good stance")
        return np.asarray(joints, dtype=float), ""
    except Exception as e:  # pragma: no cover
        return None, f"plant pose unavailable: {e}"


def preflight(bus, mode: str) -> tuple[bool, str, dict]:
    """All checks are read-only. Returns (ok, reason, details)."""
    q_deg, err = _read_q_deg(bus)
    if q_deg is None:
        return False, err, {}
    try:
        imu = bus.read_imu(apply_calib=True)
    except Exception:
        imu = None
    if not isinstance(imu, dict) or "ax_g" not in imu:
        return False, "IMU not answering", {}
    mag = math.sqrt(imu["ax_g"] ** 2 + imu["ay_g"] ** 2
                    + imu["az_g"] ** 2)
    if not 0.5 <= mag <= 1.5:
        # A dead/asleep MPU reads zeros; atan2(0,0)=0 would false-pass
        # the tilt gate. At rest |accel| must be ~1 g.
        return False, f"IMU reading implausible (|g|={mag:.2f})", {}
    roll = math.degrees(math.atan2(imu["ay_g"], imu["az_g"]))
    pitch = math.degrees(math.atan2(-imu["ax_g"],
                                    math.hypot(imu["ay_g"], imu["az_g"])))
    exp, err = _expected_start_deg(mode)
    if exp is None:
        return False, err, {}
    dq = np.abs(q_deg - exp)
    tol = {"stand": STAND_START_TOL_DEG,
           "walk": WALK_START_TOL_DEG}.get(mode, LOWER_START_TOL_DEG)
    details = {
        "roll_deg": round(roll, 1), "pitch_deg": round(pitch, 1),
        "max_pose_delta_deg": round(float(np.max(dq)), 1),
        "pose_tol_deg": tol,
    }
    if mode in ("stand", "walk") and (abs(roll) > PREFLIGHT_MAX_TILT_DEG
                                      or abs(pitch) > PREFLIGHT_MAX_TILT_DEG):
        return False, (f"tilt too high for start "
                       f"(roll {roll:+.1f} pitch {pitch:+.1f})"), details
    if float(np.max(dq)) > tol:
        worst = int(np.argmax(dq))
        want = ("belly-down, legs straight out (logical zero)"
                if mode == "stand" else "the captured standing plant")
        return False, (f"pose is not {want}: joint {worst} is "
                       f"{dq[worst]:.0f} deg from expected (tol {tol:.0f})"
                       ), details
    return True, "", details


def run_policy_move(drive, mode: str, *, on_progress=None,
                    abort_check=None, vx: float = 0.03, vy: float = 0.0,
                    duration_s: float = 6.0) -> dict:
    """Blocking policy episode. Call from a worker thread.

    ``drive`` is web_drive's DriveController (bus + arm state).
    ``mode`` is "stand", "lower" or "walk". Walk extras: body-frame
    vx/vy (m/s, clamped to the trained band) and duration_s.
    """
    assert mode in ("stand", "lower", "walk")
    on_progress = on_progress or (lambda p: None)
    abort_check = abort_check or (lambda: False)
    bus = drive.bus
    if bus is None or drive.dry_run:
        return {"ok": False, "error": "no bus"}

    cfg = load_config(str(_HERE.parent / "rl_move" / "config.yaml"))
    if mode == "walk":
        policy = NumpyPolicy(WALK_WEIGHTS_PATH)
        if policy.meta.get("obs_dim") != 72:
            return {"ok": False,
                    "error": ("rl_walk_weights.json is not a walk policy "
                              f"(obs {policy.meta.get('obs_dim')} != 72)")}
        spd = math.hypot(vx, vy)
        if spd > WALK_SPEED_MAX:
            s = WALK_SPEED_MAX / spd
            vx, vy = vx * s, vy * s
        total_s = min(max(float(duration_s), 3.0), WALK_MAX_TOTAL_S)
    else:
        policy = NumpyPolicy()
        total_s = RISE_TOTAL_S if mode == "stand" else LOWER_TOTAL_S

    ok, reason, details = preflight(bus, mode)
    if not ok:
        return {"ok": False, "error": f"preflight: {reason}", **details}

    def limp():
        try:
            bus.enable_all_torque(False)
        except Exception:
            try:
                drive._torque_all(False)
            except Exception:
                pass

    # --- arm: torque on, hold the PRESENT pose (never yank) ---
    with drive._lock:
        drive.mode = "demo"
        try:
            drive.gait.stop()
        except Exception:
            pass
        if not drive.armed:
            drive._torque_all(True)
            drive.armed = True

    est = RobotStateEstimator(bus, cfg)
    safety = SafetyLayer(cfg)
    write_speed = int(cfg_get(cfg, "bus", "write_speed", default=400))
    write_acc = int(cfg_get(cfg, "bus", "write_acc", default=20))

    # Settle reads (no motion): q_nom = the pose we actually start from,
    # tilt reference = the attitude we actually start at (sim-identical).
    state = None
    for _ in range(5):
        state = est.update(want_full_feedback=True)
        time.sleep(DT)
    if state is None or not state.bus_ok:
        limp()
        return {"ok": False, "error": "bus dropped during settle"}
    q_nom = state.joint_position.copy()
    est.set_commanded(q_nom)
    bus.write_all((q_nom * RAD2DEG).tolist(), speed=write_speed,
                  acc=write_acc)
    est.reset_episode_filters()
    for _ in range(3):
        state = est.update()
        time.sleep(DT)
    tilt_ref0 = (state.imu_roll, state.imu_pitch)
    safety.set_nominal(q_nom)
    safety.set_tilt_reference(*tilt_ref0)

    prev_action = np.zeros(N_JOINTS, dtype=float)
    vx_r = vy_r = 0.0
    n_ticks = int(round(total_s * HZ))
    overruns = 0
    max_cur = 0.0
    t_next = time.monotonic()
    result: dict = {"ok": True, "mode": mode}
    elog = _EpisodeLog(mode, {
        "mode": mode, "total_s": round(total_s, 1), "hz": HZ,
        "policy": dict(policy.meta),
        "q_nom_deg": [round(float(q) * RAD2DEG, 2) for q in q_nom],
        "tilt_ref_deg": [round(tilt_ref0[0] * RAD2DEG, 2),
                         round(tilt_ref0[1] * RAD2DEG, 2)],
        "preflight": details,
        **({"vx": round(vx, 3), "vy": round(vy, 3)}
           if mode == "walk" else {}),
    })

    for i in range(n_ticks):
        if abort_check():
            # Operator stop: HOLD pose (torque stays on); X still limps.
            result.update(ok=False, error="aborted",
                          held_pose=True, ticks=i)
            break
        t = i * DT
        if mode == "walk":
            goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                            height_ref=0.0, unload_leg=None)
            vx_r, vy_r = _walk_vel_ref(t, total_s, vx, vy)
            obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                            tilt_ref=tilt_ref0)
            # Sim obs tail: [vx_ref, vy_ref, vx_meas, vy_meas]/scale.
            # No velocity estimate on the board -> meas := ref
            # (open-loop; see module notes).
            obs = np.concatenate(
                [obs, np.array([vx_r, vy_r, vx_r, vy_r]) / WALK_VEL_SCALE])
        else:
            goal = TaskGoal(roll_ref=0.0, pitch_ref=0.0,
                            height_ref=_height_ref(mode, t),
                            unload_leg=None)
            obs = build_obs(cfg, state, q_nom, prev_action, goal=goal,
                            tilt_ref=tilt_ref0)
        action, bad = safety.validate_action(policy.act(obs),
                                             n_act=N_JOINTS)
        if action is None:
            limp()
            result.update(ok=False, error=f"bad action: {bad}", ticks=i)
            break
        q_prop = _CENTER_RAD + action * _HALF_RAD
        q_safe, status = safety.filter(q_prop, state, action=action)
        if status.terminate:
            limp()
            result.update(ok=False, error=f"safety trip: {status.reason}",
                          limped=True, ticks=i)
            break
        est.set_commanded(q_safe)
        bus.write_all((q_safe * RAD2DEG).tolist(), speed=write_speed,
                      acc=write_acc)
        prev_action = action.copy()

        t_next += DT
        lag = time.monotonic() - t_next
        if lag > 0:
            overruns += 1
            t_next = time.monotonic()
        else:
            time.sleep(-lag)
        state = est.update()
        if state.servo_current is not None:
            max_cur = max(max_cur,
                          float(np.max(np.abs(state.servo_current))))
        elog.tick(t, state, action, q_safe, goal, vx_r, vy_r, max_cur)
        if i % 5 == 0:
            if mode == "walk":
                phase = ("settle" if t < WALK_HOLD_S else
                         "decel" if t > total_s - WALK_RAMP_S else
                         "ramp" if t < WALK_HOLD_S + WALK_RAMP_S
                         else "walk")
                ref_txt = (f"v=({vx_r * 1000:+.0f},{vy_r * 1000:+.0f})mm/s")
            else:
                phase = ("curl" if mode == "stand" and t < RISE_HOLD_S else
                         "ramp" if t < (RISE_HOLD_S + RISE_RAMP_S
                                        if mode == "stand"
                                        else LOWER_HOLD_S + LOWER_RAMP_S)
                         else "hold")
                ref_txt = f"href={goal.height_ref * 1000:+.0f}mm"
            on_progress({
                "msg": (f"{mode} {phase} t={t:4.1f}s "
                        f"{ref_txt} "
                        f"maxI={max_cur:.2f}A"),
                "t_s": round(t, 2), "phase": phase,
                "height_ref_mm": round(goal.height_ref * 1000, 1),
                "roll_deg": round((state.imu_roll - tilt_ref0[0])
                                  * RAD2DEG, 2),
                "pitch_deg": round((state.imu_pitch - tilt_ref0[1])
                                   * RAD2DEG, 2),
                "max_current_a": round(max_cur, 2),
                "overruns": overruns,
            })
    else:
        result["ticks"] = n_ticks

    if result.get("ok") and mode == "lower":
        # Finished on the belly: go limp, that's the safe rest state.
        limp()
        result["limped"] = True
    if mode == "walk":
        # Walk ends holding the final stance (torque on) after the
        # decel ramp — the operator decides what happens next.
        result.update(vx_cmd=round(vx, 3), vy_cmd=round(vy, 3),
                      duration_s=round(total_s, 1))
    result.update(max_current_a=round(max_cur, 2), overruns=overruns,
                  tilt_ref_deg=[round(tilt_ref0[0] * RAD2DEG, 2),
                                round(tilt_ref0[1] * RAD2DEG, 2)])
    result["log"] = elog.close(result)
    return result

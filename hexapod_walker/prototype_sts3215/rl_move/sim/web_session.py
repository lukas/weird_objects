"""MuJoCo session object controlled by the hexapod web UI API."""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from rl_move.env import TaskGoal, build_obs

from .joint_task import q_rad_to_action
from rl_move.joint_frame import (
    RAD2DEG,
    robot_abs_rad_to_sim_rad,
    sim_rad_to_robot_abs_deg,
)
from .play import (
    _CRUISE,
    _DESC,
    _HIST_K,
    _LEGACY_PROFILE,
    _N_MODE,
    _NOSLIP_CLEAN,
    _NOSLIP_RIPPLE,
    _NOSLIP_WAVE,
    _ROLE_OBS,
    _SCRIPTED_ROWS,
    _SCRIPTED_TRIPOD,
    make_noslip_gait,
    _SPEED_MAX,
    _load_profiles,
    _obs_width,
    _sim_only_obs,
    _PlayEnv,
    scan_policies,
)

QUAD_VARIANTS = {
    "_safe": ("rear_safe", "walk_safe", "trot_safe", "safe"),
    "": ("rear", "walk", "trot", "cool"),
    "_pitch": ("rear_pitch", "walk_pitch", "trot_pitch", "pitched"),
    "_aft": ("rear_aft", "walk_aft", "trot_aft", "aft-shift"),
    "_high": ("rear_high", "walk_high", "trot_high", "high-body"),
    "_step": ("rear_step", "walk_step", "trot_step", "high-step"),
    "_aggressive": (
        "rear_aggressive", "walk_aggressive", "trot_aggressive",
        "aggressive"),
}


def _quad_name(action: str, suffix: str) -> str:
    return f"quad_{action}{suffix}"


QUAD_REAR_DEMOS = tuple(
    _quad_name("rear", suffix) for suffix in QUAD_VARIANTS)
QUAD_DOWN_DEMOS = tuple(
    _quad_name("down", suffix) for suffix in QUAD_VARIANTS)
QUAD_REARED_END_DEMOS = tuple(
    _quad_name(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("rear", "hold", "walk", "walk_back",
                   "trot", "trot_back"))
QUAD_REQUIRES_REAR = tuple(
    _quad_name(action, suffix)
    for suffix in QUAD_VARIANTS
    for action in ("hold", "walk", "walk_back", "trot",
                   "trot_back", "down"))
QUAD_STREAM_DEMOS = (*QUAD_REARED_END_DEMOS, *QUAD_DOWN_DEMOS)
QUAD_DEMO_GAITS = {}
for _quad_suffix, (_rear_gait, _walk_gait, _trot_gait, _label) in (
        QUAD_VARIANTS.items()):
    QUAD_DEMO_GAITS[_quad_name("rear", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("hold", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("down", _quad_suffix)] = _rear_gait
    QUAD_DEMO_GAITS[_quad_name("walk", _quad_suffix)] = _walk_gait
    QUAD_DEMO_GAITS[_quad_name("walk_back", _quad_suffix)] = _walk_gait
    QUAD_DEMO_GAITS[_quad_name("trot", _quad_suffix)] = _trot_gait
    QUAD_DEMO_GAITS[_quad_name("trot_back", _quad_suffix)] = _trot_gait


def _quad_action(name: str) -> str:
    for suffix in sorted(QUAD_VARIANTS, key=len, reverse=True):
        if suffix and name.endswith(suffix):
            return name[:-len(suffix)].removeprefix("quad_")
    return name.removeprefix("quad_")


@dataclass
class SimWebConfig:
    policy_dir: Path
    stance: Path
    walk: Path
    recover: Path
    log_dir: Path
    realtime: float = 1.0
    viewer: bool = False
    web_frames: bool = True
    phase_obs: bool = False
    phase_hz: float = 0.1666667
    all_models: bool = False


class SimWebSession:
    """Route-compatible controller for one local MuJoCo hexapod."""

    def __init__(self, cfg: SimWebConfig):
        self.cfg = cfg
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.msg = "starting MuJoCo"
        self.armed = False
        self.mode = "hold"
        self.auto: list | None = None
        self.downed = False
        self.sitting = False
        self.drive_active = False
        self.last_drive_cmd_at = 0.0
        self.timed_walk_until: float | None = None
        self.job_kind: str | None = None
        self.job_result: dict[str, Any] = {"ok": True, "ended": "idle"}
        self.sim_t = 0.0
        self.gait = None
        self.gait_t = 0.0
        self.om_cmd = 0.0
        self.hist: list[np.ndarray] | None = None
        self.gru = {"state": None, "start": np.ones((1,), dtype=bool)}
        self.push_ticks = 0
        self.push_force = np.zeros(3, dtype=float)
        self.demo_name: str | None = None
        self.demo_status = "idle"
        self.demo_params: dict[str, Any] = {}
        self.demo_speed_live = 1.0
        self.demo_pose_fn = None
        self.demo_t = 0.0
        self.demo_duration = 0.0
        self.demo_started_sim_t = 0.0
        self.demo_telemetry: dict[str, Any] | None = None
        # Dance scripts (dances-as-data, motor_setup/dance_script.py):
        # notes = [(t, msg)] surfaced live; cap = tightest per-act speed cap.
        self.demo_notes: list[tuple[float, str]] = []
        self.demo_note: str = ""
        self.demo_speed_cap: float | None = None
        self.demo_is_script = False
        self.demo_end_home = ""
        self.demo_direct_profile = False
        self.demo_pose_frame = "model"
        self.demo_write_speed_deg_s: float | None = None
        self.demo_write_acc_units: float | None = None
        self.demo_last_target_deg: list[float] | None = None
        self.quad_reared = False
        self.pose_hold_q: np.ndarray | None = None
        self.command_log: list[tuple[float, str, str | None]] = []
        self.last_command = ""
        self.log_dir = cfg.log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_fp = None
        self._log_writer = None
        self._log_name = ""
        self._last_log_row_t = -1.0
        self.roles: dict[str, str] = {}
        self.role_models: dict[str, tuple[Any, Path, int] | str] = {}
        self._frame_ready = threading.Event()
        self._frame_jpeg: bytes | None = None
        self._frame_error = ""
        self._frame_interval_s = 1.0 / 8.0
        self._last_frame_at = 0.0
        self.thread: threading.Thread | None = None
        self.cv2 = None

        self._load_runtime()
        if not cfg.viewer:
            self.thread = threading.Thread(target=self._run,
                                           name="sim-web-tick",
                                           daemon=True)
            self.thread.start()

    def _load_runtime(self) -> None:
        import mujoco
        from stable_baselines3 import PPO

        from .servo_model import SimServoParams
        from ..config import load_config

        root = Path(__file__).resolve().parents[2]
        self._proto_root = root
        for p in (root / "linux_control", root / "motor_setup"):
            if str(p) not in sys.path:
                sys.path.insert(0, str(p))
        from sim_gait_compat import NoSlipGait
        from sim_gait_compat import TripodGait

        self.mujoco = mujoco
        self.PPO = PPO
        if self.cfg.web_frames:
            import cv2
            self.cv2 = cv2
        self.load_checkpoint_auto = self._load_checkpoint_auto
        self.NoSlipGait = NoSlipGait
        self.TripodGait = TripodGait

        cfg = load_config()
        cfg.setdefault("obs", {})["mode_onehot"] = 1.0
        cfg["obs"]["mode_onehot_cmd"] = 1.0
        self.walk_widths: tuple[int, ...] = (72, 78, 1152)
        if self.cfg.phase_obs:
            cfg.setdefault("goal", {})["walk_phase_obs"] = 1.0
            cfg["goal"]["walk_phase_hz"] = self.cfg.phase_hz
            _ROLE_OBS[74] = "walk"
            self.walk_widths = (72, 74, 78, 1152)
        render_mode = "rgb_array" if self.cfg.web_frames else None
        self.env = _PlayEnv(params=SimServoParams.from_cfg(cfg),
                            randomize=False, episode_seconds=3600.0,
                            render_mode=render_mode, cfg=cfg)
        self.traj = self.env.traj
        self.chassis_bid = self.env.model.body("chassis").id
        self.profiles = _load_profiles()

        cats = scan_policies(self.cfg.policy_dir,
                             all_models=self.cfg.all_models)
        self.stance_list = cats["stance"]
        self.walk_list = cats["walk"]
        self.si = self._ensure_listed(self.stance_list, self.cfg.stance, (68,))
        self.wi = self._ensure_listed(self.walk_list, self.cfg.walk,
                                      self.walk_widths)
        self.walk_list.extend([_NOSLIP_CLEAN, _NOSLIP_RIPPLE, _NOSLIP_WAVE])
        self.walk_list.extend(_SCRIPTED_TRIPOD)
        self.policy_index = self._build_policy_index()
        self._register_uploaded_policies()

        self.stance = self.PPO.load(self.stance_list[self.si], device="cpu")
        self.walk = self.load_checkpoint_auto(self.walk_list[self.wi],
                                              device="cpu")
        self.n_stance = int(self.stance.observation_space.shape[0])
        self.n_walk = int(self.walk.observation_space.shape[0])
        self.walk_kind = self._walk_kind_of(self.n_walk)
        self.n_env = int(self.env.observation_space.shape[0])
        if self.walk_kind == "plain" and self.n_walk > self.n_env:
            raise ValueError(f"{self.walk_list[self.wi]} needs --phase-obs")
        self.recover = (self.load_checkpoint_auto(self.cfg.recover,
                                                  device="cpu")
                        if self.cfg.recover.exists() else None)

        self._regime_base: dict[str, Any] = {}
        self.servo_fit_counts = float(
            getattr(SimServoParams.load(), "speed_counts_s", 350.0))
        self._apply_vel_contract(self.walk_list[self.wi].stem)

        self.traj.start_at = "plant"
        self.obs, _ = self.env.reset()
        self.q_plant = self._q_now()
        self.z_plant = self._chassis_z()
        self.q_sit = self.q_plant.copy()
        self.msg = "ready"

    def _load_checkpoint_auto(self, path: Path, device: str = "cpu"):
        """Load plain PPO checkpoints without requiring sb3-contrib.

        Only 78-obs recurrent GRU checkpoints need ``gru_policy`` and its
        sb3-contrib dependency; the default web-sim stance/walk pair is
        plain PPO and should start in a lean local venv.
        """
        if _obs_width(path) == 78:
            from .gru_policy import load_checkpoint_auto
            return load_checkpoint_auto(path, device=device)
        return self.PPO.load(path, device=device)

    # -- uploaded numpy policies (policies as data, rl_move/np_policy) ---
    # The robot's export_policy_np.py JSON is an uploadable artifact:
    # POST /api/rl/policies lands in ~/.hexapod_policies, the picker
    # lists it next to the checkpoint zips, and select/role/run all
    # work — the same file drives the sim and any robot.

    @staticmethod
    def _policy_obs_width(p: Path) -> int | None:
        if p.suffix == ".json":
            from ..np_policy import np_policy_obs_width
            return np_policy_obs_width(p)
        return _obs_width(p)

    def _load_model(self, p: Path, device: str = "cpu"):
        if p.suffix == ".json":
            from ..np_policy import load_np_policy
            m = load_np_policy(p)
            prof = m.meta.get("profile")
            if isinstance(prof, dict):
                # Trained goal ramps travel with the file — same
                # contract as the robot runner.
                self.profiles[p.stem] = prof
            return m
        return self._load_checkpoint_auto(p, device=device)

    def _register_uploaded_policies(self) -> None:
        from ..np_policy import UPLOAD_DIR
        try:
            paths = sorted(UPLOAD_DIR.glob("*.json"))
        except OSError:
            paths = []
        for p in paths:
            w = self._policy_obs_width(p)
            p = p.resolve()
            if w == 68 and p not in self.stance_list:
                self.stance_list.append(p)
            elif w in (72, 74) and p not in self.walk_list:
                self.walk_list.append(p)
        self.policy_index = self._build_policy_index()

    def save_rl_policy(self, obj, *, name: str = "") -> dict[str, Any]:
        from ..np_policy import (UPLOAD_DIR, safe_policy_name,
                                 validate_np_policy)
        errs, info = validate_np_policy(obj)
        if errs:
            return {"ok": False, "error": "; ".join(errs[:5])}
        stem = safe_policy_name(name or info.get("name") or "")
        if stem is None:
            return {"ok": False,
                    "error": "need a name ([A-Za-z0-9._-]{1,64}) — "
                             "?name=... or meta.name"}
        try:
            UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
            p = UPLOAD_DIR / f"{stem}.json"
            tmp = p.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(obj))
            tmp.replace(p)
        except OSError as e:
            return {"ok": False, "error": f"save failed: {e}"}
        with self.lock:
            self._record_command(f"/api/rl/policies upload {stem}")
            self._register_uploaded_policies()
        return {"ok": True, "file": p.name, "obs_dim": info["obs_dim"],
                "slot": "stance" if info["obs_dim"] == 68 else "walk",
                "hidden": info.get("hidden"), "bytes": p.stat().st_size}

    def get_rl_policy(self, file: str) -> str | None:
        name = Path(str(file)).name
        if not name.endswith(".json"):
            name += ".json"
        from ..np_policy import UPLOAD_DIR
        p = UPLOAD_DIR / name
        try:
            return p.read_text() if p.is_file() else None
        except OSError:
            return None

    def delete_rl_policy(self, file: str) -> dict[str, Any]:
        from ..np_policy import UPLOAD_DIR
        name = Path(str(file)).name
        if not name.endswith(".json"):
            name += ".json"
        p = (UPLOAD_DIR / name).resolve()
        if not p.is_file():
            return {"ok": False, "error": f"no uploaded policy {name!r}"}
        with self.lock:
            active = {self.stance_list[self.si], self.walk_list[self.wi]}
            in_role = any(isinstance(e, tuple) and e[1] == p
                          for e in self.role_models.values())
            if p in active or in_role:
                return {"ok": False,
                        "error": f"{name} is selected/role-assigned — "
                                 f"switch away first"}
            try:
                p.unlink()
            except OSError as e:
                return {"ok": False, "error": str(e)}
            keep_s = self.stance_list[self.si]
            keep_w = self.walk_list[self.wi]
            if p in self.stance_list:
                self.stance_list.remove(p)
            if p in self.walk_list:
                self.walk_list.remove(p)
            self.si = self.stance_list.index(keep_s)
            self.wi = self.walk_list.index(keep_w)
            self.policy_index = self._build_policy_index()
        return {"ok": True, "deleted": name}

    def _ensure_listed(self, lst: list[Path], p: Path,
                       want: tuple[int, ...]) -> int:
        if not p.exists():
            raise FileNotFoundError(
                f"{p} not found; pass --policy-dir or pull checkpoints first")
        w = _obs_width(p)
        if w not in want:
            raise ValueError(f"{p}: obs width {w}, need one of {want}")
        p = p.resolve()
        if p not in lst:
            lst.insert(0, p)
        return lst.index(p)

    def _build_policy_index(self) -> dict[str, Path]:
        out: dict[str, Path] = {}
        for p in self.stance_list + self.walk_list:
            out[p.name] = p
            out[p.stem] = p
        for p in _SCRIPTED_ROWS:
            out[f"scripted:{p.name}"] = p
            out[p.name] = p
        return out

    @staticmethod
    def _ckpt_regime(stem: str) -> dict[str, float] | None:
        regimes = (
            (("fasttrack1", "steer6", "fastnoslip"),
             dict(speed=1500.0, acc=80.0, clamp_deg=5.0,
                  cruise=0.08, vmax=0.10)),
            (("middose", "midnoslip"),
             dict(speed=750.0, acc=40.0, clamp_deg=3.0,
                  cruise=0.08, vmax=0.10)),
        )
        for tokens, regime in regimes:
            if any(t in stem for t in tokens):
                return regime
        return None

    @staticmethod
    def _walk_kind_of(width: int) -> str:
        return {1152: "hist", 78: "gru"}.get(width, "plain")

    def _apply_vel_contract(self, stem: str) -> None:
        if self._ckpt_regime(stem) is not None:
            mode = 3.0
        else:
            mode = 1.0 if _sim_only_obs("walk", stem) else 2.0
        self.env.cfg.setdefault("goal", {})["walk_obs_body_vel"] = mode

    def _reset_memories(self, hard: bool) -> None:
        self.hist = None
        if hard:
            self.gru["state"] = None
            self.gru["start"] = np.ones((1,), dtype=bool)

    def _walk_predict(self) -> np.ndarray:
        if self.walk_kind == "hist":
            frame = self.obs[:72].copy()
            if self.hist is None:
                self.hist = [frame.copy() for _ in range(_HIST_K)]
            else:
                self.hist.pop()
                self.hist.insert(0, frame)
            a, _ = self.walk.predict(np.concatenate(self.hist),
                                     deterministic=True)
            return a
        if self.walk_kind == "gru":
            o = np.concatenate([self.obs[:72], self.obs[-_N_MODE:]])
            a, self.gru["state"] = self.walk.policy.predict(
                o, state=self.gru["state"],
                episode_start=self.gru["start"], deterministic=True)
            self.gru["start"] = np.zeros((1,), dtype=bool)
            return a
        a, _ = self.walk.predict(self.obs[:self.n_walk], deterministic=True)
        return a

    def _chassis_z(self) -> float:
        return float(self.env.data.xpos[self.chassis_bid, 2])

    def _q_now(self) -> np.ndarray:
        return self.env.data.qpos[7:25].copy()

    def _roll_pitch_deg(self) -> tuple[float, float]:
        qw, qx, qy, qz = self.env.data.qpos[3:7]
        roll = math.atan2(2 * (qw * qx + qy * qz),
                          1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        return round(math.degrees(roll), 1), round(math.degrees(pitch), 1)

    def _body_vel(self) -> tuple[float, float]:
        try:
            v = self.env._body_vel_xy()
            return float(v[0]), float(v[1])
        except Exception:
            return 0.0, 0.0

    def _do_reset(self, start: str, h_goal: float, note: str) -> None:
        self._stop_demo_locked(status="idle", clear_name=True)
        self.quad_reared = False
        self.pose_hold_q = None
        self.auto = None
        self.downed = False
        self.sitting = False
        self.drive_active = False
        self.timed_walk_until = None
        self.gait = None
        self.gait_t = 0.0
        self.om_cmd = 0.0
        self.traj.start_at = start
        self.traj.goal = TaskGoal()
        self.traj.goal.height_ref = h_goal
        self.traj.vx = self.traj.vy = 0.0
        self.traj.mode = "hold"
        self.traj.reset_published()
        self._reset_memories(hard=True)
        self.obs, _ = self.env.reset()
        if start == "plant":
            self.q_plant = self._q_now()
            self.z_plant = self._chassis_z()
        self.msg = note
        self._finish_job(note)

    def _demo_running(self) -> bool:
        return self.demo_pose_fn is not None

    def _set_demo_safety(self, wide: bool) -> None:
        """Widen the tilt trip while an open-loop demo plays.

        The RL episode terminates at 10 deg body tilt — correct for
        policies, but scripted choreography legitimately exceeds it
        (quad rear is -20 deg pitch by design; the robot guards these
        acts at 45 deg). 60 deg still catches a genuine tip-over.
        """
        s = self.env.safety
        if "tilt" not in self._regime_base:
            self._regime_base["tilt"] = (s.max_roll, s.max_pitch)
        if wide:
            s.max_roll = s.max_pitch = math.radians(60.0)
        else:
            s.max_roll, s.max_pitch = self._regime_base["tilt"]

    def _stop_demo_locked(self, status: str = "aborted",
                          clear_name: bool = False) -> None:
        if self._regime_base.get("tilt"):
            self._set_demo_safety(False)
        self.demo_pose_fn = None
        self.demo_t = 0.0
        self.demo_duration = 0.0
        self.demo_status = status
        self.demo_params = {}
        self.demo_notes = []
        self.demo_note = ""
        self.demo_speed_cap = None
        self.demo_is_script = False
        self.demo_end_home = ""
        self.demo_direct_profile = False
        self.demo_pose_frame = "model"
        self.demo_write_speed_deg_s = None
        self.demo_write_acc_units = None
        self.demo_last_target_deg = None
        if clear_name:
            self.demo_name = None
            self.demo_telemetry = None
        self._close_log()

    def _demo_speed_eff_locked(self) -> float:
        speed = float(self.demo_speed_live)
        if self.demo_name in QUAD_DEMO_GAITS:
            try:
                import quad_walk as QW
                gait = QUAD_DEMO_GAITS[self.demo_name]
                cap = QW.GAITS.get(gait, {}).get("speed_cap")
                if cap is not None:
                    speed = min(speed, float(cap))
            except Exception:
                pass
        if self.demo_speed_cap is not None:
            speed = min(speed, float(self.demo_speed_cap))
        hi = 10.0 if (self.demo_name or "").startswith("standup_") else 3.0
        return max(0.25, min(hi, speed))

    def _record_command(self, text: str, key: str | None = None) -> None:
        clean = " ".join(str(text).split())
        if len(clean) > 80:
            clean = clean[:77] + "..."
        self.last_command = clean
        row = (float(self.sim_t), clean, key)
        if key and self.command_log and self.command_log[-1][2] == key:
            self.command_log[-1] = row
        else:
            self.command_log.append(row)
            del self.command_log[:-6]

    def _new_gait(self):
        # sim_gait_compat gait classes accept MuJoCo/model-relative
        # hip/knee inputs here. Robot-absolute knees are only for the
        # hardware demo stack; using them here corrupts the neutral stance.
        plant_deg = [float(v) * RAD2DEG for v in self.q_plant]
        kw = _SCRIPTED_TRIPOD.get(self.walk_list[self.wi])
        if kw is not None:
            g = self.TripodGait(period=kw["period"],
                                lift=kw["lift_mm"] * 0.001, ramp=0.4)
            g.sync_plant_stance(plant_deg[1], plant_deg[2])
            g.set_lift_mm(kw["lift_mm"])
            g.reset_phase(t=0.0)
            return g
        g = make_noslip_gait(self.walk_list[self.wi], self.NoSlipGait)
        g.sync_plant_stance(plant_deg[1], plant_deg[2])
        return g

    def _blend_ticks(self) -> int:
        gap = max(self.z_plant - self._chassis_z(), 0.0)
        return int(round(min(max(gap / 0.020, 0.5), 4.0) / self.env.dt))

    def _stance_profile(self, kind: str) -> dict[str, float]:
        path = self._role_path(kind) or self.stance_list[self.si]
        prof = self.profiles.get(path.stem, {})
        return {**_LEGACY_PROFILE[kind], **prof.get(kind, {})}

    def _apply_ramp(self, kind: str) -> dict[str, float]:
        prof = self._stance_profile(kind)
        self.traj.HEIGHT_RATE = abs(prof["target_m"]) / max(prof["ramp_s"], 0.1)
        self.traj.BELLY_HOLD_S = float(prof["hold_s"]) if kind == "stand" else 0.0
        return prof

    def _restore_phys(self, keep_q: np.ndarray, keep_v: np.ndarray) -> None:
        self.env.data.qpos[:] = keep_q
        self.env.data.qvel[:] = keep_v
        self.mujoco.mj_forward(self.env.model, self.env.data)
        self.env._profile.reset(self._q_now())
        self.env.safety.set_nominal(self._q_now())

    def _re_anchor_plant(self) -> None:
        keep_q = self.env.data.qpos.copy()
        keep_v = self.env.data.qvel.copy()
        self.traj.start_at = "plant"
        self.traj.goal = TaskGoal()
        self.traj.vx = self.traj.vy = 0.0
        self.traj.reset_published()
        self._reset_memories(hard=False)
        self.obs, _ = self.env.reset()
        self._restore_phys(keep_q, keep_v)

    def _re_anchor_belly(self) -> None:
        keep_q = self.env.data.qpos.copy()
        keep_v = self.env.data.qvel.copy()
        self.traj.start_at = "zero"
        self.traj.goal = TaskGoal()
        self.traj.vx = self.traj.vy = 0.0
        self.traj.reset_published()
        self._reset_memories(hard=False)
        self.obs, _ = self.env.reset()
        self._restore_phys(keep_q, keep_v)

    def _role_path(self, role: str) -> Path | None:
        entry = self.role_models.get(role)
        if isinstance(entry, tuple):
            return entry[1]
        return None

    def _role_model(self, role: str):
        entry = self.role_models.get(role)
        if isinstance(entry, tuple):
            return entry[0], entry[2]
        if role == "hold" and entry == "walk":
            return self.walk, self.n_walk
        return self.stance, self.n_stance

    def _stance_action(self, role: str) -> np.ndarray:
        model, n = self._role_model(role)
        a, _ = model.predict(self.obs[:n], deterministic=True)
        return a

    def _do_stand(self) -> None:
        self.pose_hold_q = None
        if self.auto is not None:
            if self.auto[0] == "lower":
                self.auto = None
            elif self.auto[0] == "recover":
                self.msg = "recovering - wait for the stand"
                return
            else:
                self.msg = "rise already running"
                return
        self.sitting = False
        prof = self._apply_ramp("stand")
        if (not self.downed and self.traj.start_at == "plant"
                and self._chassis_z() > 0.09):
            self.traj.goal.height_ref = 0.0
            self.msg = "rising back up (in place)"
            return
        keep_q = self.env.data.qpos.copy()
        keep_v = self.env.data.qvel.copy()
        self.downed = False
        self.gait = None
        self.om_cmd = 0.0
        self.traj.start_at = "zero"
        self.traj.goal = TaskGoal()
        self.traj.goal.height_ref = float(prof["target_m"])
        self.traj.vx = self.traj.vy = 0.0
        self.traj.reset_published()
        self._reset_memories(hard=False)
        self.obs, _ = self.env.reset()
        self._restore_phys(keep_q, keep_v)
        rise_total = float(prof["hold_s"]) + float(prof["ramp_s"]) + 1.5
        self.auto = ["rise", 0, rise_total]
        self.msg = "RISE (in place)"

    def _do_sit(self) -> None:
        self.pose_hold_q = None
        if self.downed:
            self.msg = "robot is down - reset or recover first"
            return
        if self.sitting:
            self.msg = "already lowered"
            return
        if self.auto is not None and self.auto[0] in ("blend", "fold", "fell"):
            self.msg = "scripted transition in progress"
            return
        self.auto = None
        self.traj.vx = self.traj.vy = 0.0
        self.om_cmd = 0.0
        prof = self._apply_ramp("lower")
        if self.traj.start_at in ("zero", "belly"):
            self.auto = ["fold", 0, int(6.0 / self.env.dt), self._chassis_z()]
            self.msg = "LOWER: settling to the ground"
            return
        self.traj.goal.height_ref = float(prof["target_m"])
        total = float(prof["hold_s"]) + float(prof["ramp_s"]) + 1.5
        self.auto = ["lower", 0, total]
        self.msg = "LOWER: crouch, then settle"

    def _upright(self) -> bool:
        roll, pitch = self._roll_pitch_deg()
        return (self._chassis_z() > 0.10 and abs(roll) < 17.0
                and abs(pitch) < 17.0)

    def _direct_profile_step_locked(self, q_rad: np.ndarray) -> None:
        """Issue a bus-like joint target directly to MuJoCo's servo model."""
        if self.demo_pose_frame == "robot_abs":
            q_model = robot_abs_rad_to_sim_rad(q_rad)
            self.env._profile.command_robot_abs(
                q_rad,
                speed_deg_s=self.demo_write_speed_deg_s,
                acc_units=self.demo_write_acc_units)
        else:
            q_model = np.asarray(q_rad, dtype=float).copy()
            self.env._profile.command(
                q_model,
                speed_deg_s=self.demo_write_speed_deg_s,
                acc_units=self.demo_write_acc_units)
        self.env._cmd = q_model.copy()
        self.env._advance()
        self.env._state = self.env._read_state()
        self.env._step_i += 1
        goal = self.env._current_goal()
        self.obs = self.env._final_obs(
            build_obs(self.env.cfg, self.env._state, self.env._q_nom,
                      self.env._prev_action, goal=goal,
                      tilt_ref=self.env._tilt_ref0),
            reset=False)

    def _demo_end_live_locked(self) -> dict[str, Any]:
        roll, pitch = self._roll_pitch_deg()
        out: dict[str, Any] = {
            "height_mm": round(self._chassis_z() * 1000.0, 1),
            "roll_deg": roll,
            "pitch_deg": pitch,
        }
        if self.demo_last_target_deg is not None:
            if self.demo_pose_frame == "robot_abs":
                actual = sim_rad_to_robot_abs_deg(self._q_now())
            else:
                actual = [math.degrees(float(v)) for v in self._q_now()]
            out["max_lag_deg"] = round(max(
                abs(a - b) for a, b in zip(actual, self.demo_last_target_deg)
            ), 2)
        return out

    def _finish_demo_locked(self) -> None:
        name = self.demo_name or "demo"
        end_home = self.demo_end_home
        self.demo_pose_fn = None
        self._set_demo_safety(False)
        live = self._demo_end_live_locked()
        max_lag = live.get("max_lag_deg")
        ok = True
        if self.demo_is_script or end_home == "sit":
            ok = (
                self._chassis_z() < 0.09
                and (max_lag is None or float(max_lag) <= 12.0))
            self.sitting = bool(ok)
            self.downed = not ok
            self.quad_reared = False
            self.q_sit = self._q_now()
            self.pose_hold_q = self.q_sit.copy()
            self.msg = (f"{name} done - sitting" if ok
                        else f"{name} did not reach sit cleanly")
        elif end_home == "stand":
            ok = self._upright()
            if ok:
                self.sitting = False
                self.downed = False
                self.quad_reared = False
                self.q_plant = self._q_now()
                self.z_plant = self._chassis_z()
                self.pose_hold_q = self.q_plant.copy()
                self.traj.start_at = "plant"
                self.msg = f"{name} done - standing"
            else:
                self.sitting = False
                self.downed = True
                self.pose_hold_q = self._q_now().copy()
                self.traj.start_at = "zero"
                self.msg = (
                    f"{name} command ended low/tilted - did not stand")
        else:
            self.sitting = False
            self.downed = False
            self.quad_reared = name in QUAD_REARED_END_DEMOS
            self.pose_hold_q = self._q_now().copy()
            self.msg = f"{name} done - holding"
        self.demo_status = "done" if ok else "failed"
        self.demo_end_home = ""
        self.demo_direct_profile = False
        self.demo_pose_frame = "model"
        self.demo_write_speed_deg_s = None
        self.demo_write_acc_units = None
        self.demo_telemetry = {
            "ok": ok,
            "ended": end_home or "hold",
            **live,
            "sim_t_s": round(self.sim_t - self.demo_started_sim_t, 2),
            "demo_time_s": round(self.demo_t, 2),
            "log_name": self._log_name or None,
            "log": str(self.log_dir / self._log_name)
            if self._log_name else None,
        }
        self.demo_last_target_deg = None
        self._close_log()

    def _do_fall(self) -> None:
        roll = 0.4
        pitch = 0.3
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        self.traj.vx = self.traj.vy = 0.0
        self.env.data.qpos[2] = 0.20
        self.env.data.qpos[3:7] = [cr * cp, sr * cp, cr * sp, sr * sp]
        lo, hi = self.env.model.jnt_range[1:, 0], self.env.model.jnt_range[1:, 1]
        self.env.data.qpos[7:25] = np.random.uniform(lo, hi)
        self.env.data.qvel[:] = 0.0
        self.mujoco.mj_forward(self.env.model, self.env.data)
        self._reset_memories(hard=True)
        self.sitting = False
        self.downed = False
        self.auto = ["fell", 0, int(4.0 / self.env.dt), self._chassis_z()]
        self.msg = "FALLING into sprawled pose"

    def _do_recover(self) -> None:
        self.pose_hold_q = None
        if self.recover is None:
            self.msg = f"no recovery checkpoint ({self.cfg.recover.name})"
            return
        if self.auto is not None and self.auto[0] == "fell":
            self.msg = "still tumbling - recover after it lands"
            return
        self.traj.vx = self.traj.vy = 0.0
        self.om_cmd = 0.0
        self._re_anchor_belly()
        self.downed = False
        self.sitting = False
        self.upright_ticks = 0
        self.env.cfg.setdefault("goal", {})["walk_obs_body_vel"] = 1.0
        self.auto = ["recover", 0, int(20.0 / self.env.dt)]
        self.msg = "RECOVER: policy getting up"

    def _engage_walk(self) -> bool:
        if self.auto is not None:
            self.msg = "auto transition in progress"
            return False
        if self.downed:
            self.msg = "robot is down - reset or recover first"
            return False
        if self.sitting:
            self.msg = "lowered - stand first"
            return False
        if self._chassis_z() < 0.09:
            self.msg = "too low to walk - stand first"
            return False
        self.traj.goal.roll_ref = self.traj.goal.pitch_ref = 0.0
        self.traj.goal.height_ref = 0.0
        self.traj._pub.roll_ref = self.traj._pub.pitch_ref = 0.0
        self.traj._pub.height_ref = 0.0
        self.pose_hold_q = None
        if self.walk is not None:
            self._apply_vel_contract(self.walk_list[self.wi].stem)
        return True

    def _drive_band(self) -> tuple[float, float]:
        kw = _SCRIPTED_TRIPOD.get(self.walk_list[self.wi])
        if kw is not None:
            return kw["cruise"], kw["cruise"]
        reg = None if self.walk is None else self._ckpt_regime(self.walk_list[self.wi].stem)
        if reg is not None:
            return reg["cruise"], reg["vmax"]
        return _CRUISE, _SPEED_MAX

    def _apply_servo_regime(self) -> None:
        prof = self.env._profile
        if prof is None:
            return
        if not self._regime_base:
            self._regime_base["vel"] = prof._vel_default.copy()
            self._regime_base["speed"] = self.env.write_speed_deg_s
            self._regime_base["acc"] = self.env.write_acc_units
            self._regime_base["dq"] = self.env.safety.max_dq
        tripod_live = self.walk_list[self.wi] in _SCRIPTED_TRIPOD
        reg = None if tripod_live or self.walk is None else self._ckpt_regime(
            self.walk_list[self.wi].stem)
        if tripod_live:
            s = 1500.0 / max(self.servo_fit_counts, 1.0)
            prof._vel_default[:] = self._regime_base["vel"] * s
            self.env.write_speed_deg_s = 1500.0 * 360.0 / 4096.0
            self.env.write_acc_units = 80.0
            self.env.safety.max_dq = self._regime_base["dq"]
        elif reg is not None:
            s = reg["speed"] / max(self.servo_fit_counts, 1.0)
            prof._vel_default[:] = self._regime_base["vel"] * s
            self.env.write_speed_deg_s = reg["speed"] * 360.0 / 4096.0
            self.env.write_acc_units = reg["acc"]
            self.env.safety.max_dq = math.radians(reg["clamp_deg"])
        else:
            prof._vel_default[:] = self._regime_base["vel"]
            self.env.write_speed_deg_s = self._regime_base["speed"]
            self.env.write_acc_units = self._regime_base["acc"]
            self.env.safety.max_dq = self._regime_base["dq"]

    def _tick_locked(self) -> None:
        self._apply_servo_regime()
        now = time.monotonic()
        if self.drive_active and now - self.last_drive_cmd_at > 0.6:
            self.traj.vx = self.traj.vy = 0.0
        if self.timed_walk_until is not None and self.sim_t >= self.timed_walk_until:
            self.traj.vx = self.traj.vy = 0.0
            self.timed_walk_until = None
            self._finish_job("timed walk complete")

        demo_running = self._demo_running()
        cmd_speed = float(np.hypot(self.traj.vx, self.traj.vy))
        scripted = self.walk is None
        walking = ((cmd_speed > 1e-3 or (scripted and abs(self.om_cmd) > 1e-3))
                   and self.auto is None and not self.downed and not self.sitting
                   and not demo_running)
        if not walking:
            self.gait = None
            self.hist = None
        self.mode = ("demo" if demo_running
                     else "rise" if self.auto is not None and self.auto[0] in
                     ("rise", "blend", "recover")
                     else "lower" if self.auto is not None and self.auto[0] in
                     ("lower", "fold", "fell")
                     else "walk" if walking else "hold")
        self.traj.mode = "hold" if self.mode == "demo" else self.mode

        action = None
        if self.push_ticks > 0:
            self.env.data.xfrc_applied[self.chassis_bid, :3] = self.push_force
            self.push_ticks -= 1
        else:
            self.env.data.xfrc_applied[self.chassis_bid, :3] = 0.0

        if self.downed:
            action = q_rad_to_action(self._q_now())
        elif self.auto is not None and self.auto[0] == "rise":
            action = self._stance_action("stand")
            self.auto[1] += 1
            if self.auto[1] * self.env.dt >= self.auto[2]:
                if self._chassis_z() > 0.06:
                    self.q_blend_from = self._q_now()
                    self.auto = ["blend", 0, self._blend_ticks()]
                    self.msg = "aligning to walk stance"
                else:
                    self.auto = None
                    self._finish_job("rise failed", ok=False)
        elif self.auto is not None and self.auto[0] == "blend":
            self.auto[1] += 1
            s = min(self.auto[1] / max(self.auto[2], 1), 1.0)
            action = q_rad_to_action((1.0 - s) * self.q_blend_from
                                     + s * self.q_plant)
            if self.auto[1] >= self.auto[2]:
                self._re_anchor_plant()
                self.auto = None
                self._finish_job("up at walk stance")
        elif self.auto is not None and self.auto[0] == "lower":
            action = self._stance_action("lower")
            self.auto[1] += 1
            if self.auto[1] * self.env.dt >= self.auto[2]:
                self.auto = ["fold", 0, int(6.0 / self.env.dt),
                             self._chassis_z()]
                self.msg = "settling to ground"
        elif self.auto is not None and self.auto[0] == "fold":
            self.env._advance(limp=True)
            self.auto[1] += 1
            z = self._chassis_z()
            settled = self.auto[1] * self.env.dt > 1.0 and abs(z - self.auto[3]) < 2e-5
            self.auto[3] = z
            if settled or self.auto[1] >= self.auto[2]:
                self._re_anchor_belly()
                self.auto = None
                self.sitting = True
                self.q_sit = self._q_now()
                self._finish_job("lowered, parked on ground")
        elif self.auto is not None and self.auto[0] == "fell":
            self.env._advance(limp=True)
            self.auto[1] += 1
            z = self._chassis_z()
            settled = self.auto[1] * self.env.dt > 1.0 and abs(z - self.auto[3]) < 2e-5
            self.auto[3] = z
            if settled or self.auto[1] >= self.auto[2]:
                self._re_anchor_belly()
                self.auto = None
                self.downed = True
                self.msg = "FALLEN - recover or reset"
        elif self.auto is not None and self.auto[0] == "recover":
            action, _ = self.recover.predict(self.obs[:72], deterministic=True)
            self.auto[1] += 1
            self.upright_ticks = self.upright_ticks + 1 if self._upright() else 0
            if self.upright_ticks >= int(1.0 / self.env.dt):
                self._re_anchor_plant()
                self.auto = None
                self._finish_job("recovered - standing")
            elif self.auto[1] >= self.auto[2]:
                self.auto = None
                self._finish_job("recovery timed out", ok=False)
        elif demo_running:
            pose_deg = self.demo_pose_fn(self.demo_t)
            self.demo_last_target_deg = [float(v) for v in pose_deg]
            pose_rad = np.radians(pose_deg)
            if self.demo_direct_profile:
                self._direct_profile_step_locked(pose_rad)
            else:
                if self.demo_pose_frame == "robot_abs":
                    pose_rad = robot_abs_rad_to_sim_rad(pose_rad)
                action = q_rad_to_action(pose_rad)
            self.demo_t += self.env.dt * self._demo_speed_eff_locked()
            while self.demo_notes and self.demo_notes[0][0] <= self.demo_t:
                self.demo_note = self.demo_notes.pop(0)[1]
            self.demo_status = (
                f"{self.demo_note} · x{self.demo_speed_live:.2f}"
                if self.demo_note
                else f"running @ {self.demo_speed_live:.2f}x")
            if self.demo_t >= self.demo_duration:
                self._finish_demo_locked()
        elif self.sitting:
            action = q_rad_to_action(self.q_sit)
        elif self.pose_hold_q is not None:
            action = q_rad_to_action(self.pose_hold_q)
        elif walking and scripted:
            if self.gait is None:
                self.gait = self._new_gait()
                self.gait_t = 0.0
            self.gait.set_velocity(vx=self.traj.vx, vy=self.traj.vy,
                                   omega=self.om_cmd)
            # _new_gait() builds the sim_gait_compat variants, whose
            # desired_deg() already returns MuJoCo/model-relative knees.
            # Converting again folds the knee by one hip angle and makes
            # the scripted rows look broken in the web UI.
            q_model = np.radians(self.gait.desired_deg(self.gait_t))
            action = q_rad_to_action(q_model)
            self.gait_t += self.env.dt
        elif walking:
            action = self._walk_predict()
        else:
            action = self._stance_action("hold")

        if action is not None:
            self.obs, _r, term, trunc, info = self.env.step(action)
            if term or trunc:
                if self._demo_running():
                    self._stop_demo_locked(
                        status=(info.get("termination_reason")
                                or "episode end") + "; DOWN")
                self.downed = True
                self.auto = None
                self.drive_active = False
                self.timed_walk_until = None
                self.traj.vx = self.traj.vy = 0.0
                reason = info.get("termination_reason") or "episode end"
                self._finish_job(f"{reason}; DOWN", ok=False)
        self.sim_t += self.env.dt
        self._write_log_row()

    def _run(self) -> None:
        while not self.stop_event.is_set():
            t0 = time.monotonic()
            with self.lock:
                self._tick_locked()
                now = time.monotonic()
                if (self.cfg.web_frames
                        and now - self._last_frame_at >= self._frame_interval_s):
                    self._last_frame_at = now
                    self._render_frame_locked()
            if self.cfg.realtime > 0:
                delay = self.env.dt / self.cfg.realtime - (time.monotonic() - t0)
                if delay > 0:
                    self.stop_event.wait(delay)

    def run_native_viewer(self, web_url: str = "") -> None:
        """Run physics and a native MuJoCo viewer on this process thread.

        Closing the window only detaches the viewer: the sim keeps
        stepping headless and the web server stays up (operator 08-22 —
        the old close-to-stop coupling kept killing the web UI by
        accident). Ctrl-C in the terminal stops the whole server.
        """
        import mujoco
        import mujoco.viewer

        print("MuJoCo viewer: closing the window detaches the viewer; "
              "the sim + web server keep running (Ctrl-C stops them)",
              flush=True)
        with mujoco.viewer.launch_passive(self.env.model,
                                          self.env.data) as viewer:
            while viewer.is_running() and not self.stop_event.is_set():
                t0 = time.monotonic()
                with self.lock:
                    self._tick_locked()
                    self._update_viewer_hud_locked(viewer, mujoco, web_url)
                    now = time.monotonic()
                    if (self.cfg.web_frames
                            and now - self._last_frame_at >= self._frame_interval_s):
                        self._last_frame_at = now
                        self._render_frame_locked()
                    viewer.sync()
                if self.cfg.realtime > 0:
                    delay = (self.env.dt / self.cfg.realtime
                             - (time.monotonic() - t0))
                    if delay > 0:
                        self.stop_event.wait(delay)
        if self.stop_event.is_set():
            return
        print("MuJoCo viewer closed — sim continues headless; web UI "
              f"still at {web_url or 'the same URL'}", flush=True)
        self._run()

    def _update_viewer_hud_locked(self, viewer: Any, mujoco_mod: Any,
                                  web_url: str) -> None:
        set_texts = getattr(viewer, "set_texts", None)
        if set_texts is None:
            return
        live = self._live()
        width = 54

        def cell(text: str) -> str:
            text = str(text)
            if len(text) > width:
                text = text[:width - 3] + "..."
            return text.ljust(width)

        commands = [
            f"{t:7.1f}s  {cmd}" for t, cmd, _key in self.command_log[-4:]
        ]
        while len(commands) < 4:
            commands.append("")

        labels = [
            "Hexapod sim",
            "status",
            "mode",
            "cmd ref",
            "body vel",
            "tilt",
            "url",
            "last cmd",
            "commands",
            "",
            "",
            "",
        ]
        values = [
            "MuJoCo web control",
            live["status"],
            f"{live['mode']}  h {live['height_mm']:>6.1f} mm",
            f"{live['vx_ref']:+.3f},{live['vy_ref']:+.3f} m/s",
            f"{live['vx_body']:+.3f},{live['vy_body']:+.3f} m/s",
            f"{live['roll_deg']:+.1f},{live['pitch_deg']:+.1f} deg",
            web_url or "",
            self.last_command,
            *commands,
        ]
        title = "\n".join(label.ljust(12) for label in labels)
        detail = "\n".join(cell(value) for value in values)
        set_texts([(mujoco_mod.mjtFontScale.mjFONTSCALE_100,
                    mujoco_mod.mjtGridPos.mjGRID_TOPLEFT,
                    title, detail)])

    def _render_frame_locked(self) -> None:
        try:
            if self.cv2 is None:
                raise RuntimeError("browser frames disabled")
            frame = self.env.render()
            if frame is None:
                raise RuntimeError("browser frames disabled")
            img = self.cv2.cvtColor(frame, self.cv2.COLOR_RGB2BGR)
            ok, data = self.cv2.imencode(
                ".jpg", img, [int(self.cv2.IMWRITE_JPEG_QUALITY), 86])
            if not ok:
                raise RuntimeError("could not encode sim frame")
            self._frame_jpeg = data.tobytes()
            self._frame_error = ""
        except Exception as e:
            self._frame_error = str(e)
        finally:
            self._frame_ready.set()

    def _finish_job(self, ended: str, ok: bool = True) -> None:
        self.msg = ended
        if self.job_kind:
            self.job_result = {
                "ok": ok,
                "ended": ended,
                "mode": self.job_kind,
                "sim_t_s": round(self.sim_t, 2),
                "log": self._log_name or None,
            }
            self.job_kind = None
            self._close_log()
        elif ended:
            self.job_result = {"ok": ok, "ended": ended,
                               "sim_t_s": round(self.sim_t, 2)}

    def _open_log(self, kind: str) -> None:
        self._close_log()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self._log_name = f"sim_{kind}_{stamp}.csv"
        self._log_fp = (self.log_dir / self._log_name).open("w", newline="")
        self._log_writer = csv.DictWriter(self._log_fp, fieldnames=[
            "t_s", "mode", "height_mm", "roll_deg", "pitch_deg",
            "vx_ref_mps", "vy_ref_mps", "vx_body_mps", "vy_body_mps",
            "stance", "walk", "msg",
        ])
        self._log_writer.writeheader()
        self._last_log_row_t = -1.0

    def _write_log_row(self) -> None:
        if self._log_writer is None or self.sim_t - self._last_log_row_t < 0.04:
            return
        roll, pitch = self._roll_pitch_deg()
        vx, vy = self._body_vel()
        self._log_writer.writerow({
            "t_s": round(self.sim_t, 3),
            "mode": self.mode,
            "height_mm": round(self._chassis_z() * 1000.0, 1),
            "roll_deg": roll,
            "pitch_deg": pitch,
            "vx_ref_mps": round(float(self.traj.vx), 4),
            "vy_ref_mps": round(float(self.traj.vy), 4),
            "wz_ref_rad_s": round(float(getattr(self, "om_cmd", 0.0)), 4),
            "vx_body_mps": round(vx, 4),
            "vy_body_mps": round(vy, 4),
            "stance": self._active_stance_name(),
            "walk": self._active_walk_name(),
            "msg": self.msg,
        })
        self._last_log_row_t = self.sim_t

    def _close_log(self) -> None:
        if self._log_fp is not None:
            self._log_fp.close()
        self._log_fp = None
        self._log_writer = None

    def _active_stance_name(self) -> str:
        return self.stance_list[self.si].name

    def _active_walk_name(self) -> str:
        p = self.walk_list[self.wi]
        return f"scripted:{p.name}" if p in _SCRIPTED_ROWS else p.name

    def _live(self) -> dict[str, Any]:
        roll, pitch = self._roll_pitch_deg()
        vx, vy = self._body_vel()
        return {
            "model": self._active_walk_name(),
            "stance": self._active_stance_name(),
            "mode": self.mode,
            "status": self.msg,
            "vx_ref": round(float(self.traj.vx), 4),
            "vy_ref": round(float(self.traj.vy), 4),
            "wz_ref": round(float(getattr(self, "om_cmd", 0.0)), 4),
            "vx_body": round(vx, 4),
            "vy_body": round(vy, 4),
            "roll_deg": roll,
            "pitch_deg": pitch,
            "height_mm": round(self._chassis_z() * 1000.0, 1),
            "t_s": round(self.sim_t, 1),
        }

    # Public API methods used by web_server.py -------------------------

    def demo_state(self) -> dict[str, Any]:
        with self.lock:
            return {
                "name": self.demo_name,
                "status": self.demo_status,
                "running": self._demo_running(),
                "speed_live": self.demo_speed_live,
                "params": dict(self.demo_params),
                "progress": {"msg": self.demo_status, "live": self._live()}
                if self._demo_running() else None,
                "telemetry": dict(self.demo_telemetry)
                if self.demo_telemetry else None,
                "bus_hot": False,
            }

    def list_demos(self) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        actions = (
            ("rear", "REAR UP", "tip back on 4 legs and hold"),
            ("hold", "HOLD", "settle to reared hold"),
            ("walk", "WALK FORWARD", "animal walk while reared"),
            ("walk_back", "WALK BACKWARD", "reverse animal walk while reared"),
            ("trot", "TROT FORWARD", "diagonal pairs while reared"),
            ("trot_back", "TROT BACKWARD", "reverse diagonal pairs while reared"),
            ("down", "COME DOWN", "untuck fronts and return to stand"),
        )
        for suffix, (_rear_gait, _walk_gait, _trot_gait, label) in (
                QUAD_VARIANTS.items()):
            tag = "" if not suffix else f" {label.upper()}"
            for action, title, desc in actions:
                out.append({
                    "name": _quad_name(action, suffix),
                    "title": f"[8 quad] {title}{tag} - {desc}",
                    "air": False,
                    "group": "quad",
                    "live_speed": True,
                    "has_size": False,
                })
        for meta in self.list_dance_scripts():
            out.append({
                "name": meta["name"],
                "title": meta.get("title") or meta["name"],
                "air": True,            # scripts start AND end at sit zero
                "group": "uploaded",
                "live_speed": True,
                "has_size": False,
                "uploaded": True,
                "stands": bool(meta.get("stands")),
                "seconds": meta.get("seconds"),
            })
        return out

    def standup_modes(self) -> dict[str, Any]:
        path = self._proto_root / "linux_control" / "standup_modes.json"
        try:
            data = json.loads(path.read_text())
        except (OSError, ValueError) as e:
            return {"ok": False, "error": f"standup_modes.json: {e}"}
        return {
            "ok": True,
            "frame": data.get("frame", ""),
            "modes": [
                {"name": name,
                 "description": m.get("description", ""),
                 "keyframes": len(m.get("keyframes", [])),
                 "total_s": m.get("total_s")}
                for name, m in (data.get("modes") or {}).items()],
        }

    def pose(self) -> dict[str, Any]:
        with self.lock:
            deg = [round(float(v), 2)
                   for v in sim_rad_to_robot_abs_deg(self._q_now())]
            return {"ok": True, "sim": True, "degrees": deg, "live": 18,
                    "armed": self.armed, "mode": self.mode,
                    "ts": time.time()}

    def _standup_frames(self, mode: str,
                        direction: str) -> list[tuple[list[float], float]]:
        data = json.loads(
            (self._proto_root / "linux_control" / "standup_modes.json")
            .read_text())
        mode = (mode or "tuck").strip()
        if mode == "plant":
            kfs = data["modes"]["tuck"]["keyframes"]
            plant = sim_rad_to_robot_abs_deg(self.q_plant)
            keyframes = list(kfs[:-1]) + [{"q_deg": plant, "s": 0.5}]
        else:
            keyframes = data["modes"][mode]["keyframes"]
        frames = [([float(v) for v in kf["q_deg"]], float(kf["s"]))
                  for kf in keyframes]
        if (direction or "up").strip().lower() in {"down", "lower", "sit"}:
            qs = [q for q, _ in frames]
            ss = [s for _, s in frames]
            frames = [(qs[-1], 0.8)] + [
                (qs[i], ss[i + 1]) for i in range(len(qs) - 2, -1, -1)]
        return frames

    @staticmethod
    def _pose_fn_from_frames(
            start_deg: list[float],
            frames: list[tuple[list[float], float]]):
        import bisect
        segs: list[tuple[float, float, Any]] = []
        bounds: list[float] = []
        t = 0.0
        cur = [float(v) for v in start_deg]
        for target, seconds in frames:
            a = list(cur)
            b = [float(v) for v in target]
            dur = max(0.05, float(seconds))

            def fn(u: float, a=a, b=b) -> list[float]:
                w = 0.5 - 0.5 * math.cos(math.pi * u)
                return [x + (y - x) * w for x, y in zip(a, b)]

            segs.append((t, t + dur, fn))
            t += dur
            bounds.append(t)
            cur = b

        def pose_at(tt: float) -> list[float]:
            if not segs:
                return list(start_deg)
            i = min(bisect.bisect_right(bounds, max(0.0, tt)),
                    len(segs) - 1)
            t0, t1, fn = segs[i]
            u = (max(0.0, tt) - t0) / max(1e-9, t1 - t0)
            return fn(min(1.0, max(0.0, u)))

        return pose_at, t

    def sim_standup(self, *, mode: str = "tuck", speed: float = 1.0,
                    direction: str = "up") -> dict[str, Any]:
        with self.lock:
            try:
                frames = self._standup_frames(mode, direction)
            except (OSError, ValueError, KeyError) as e:
                return {"ok": False,
                        "error": f"unknown stand-up mode: {e}"}
            speed = self._clamp_float(speed, 1.0, 0.25, 10.0)
            direction = (direction or "up").strip().lower()
            down = direction in {"down", "lower", "sit"}
            home = "sit" if down else "stand"
            name = f"standup_{mode}" + ("_down" if down else "")
            switched_from = self.demo_name if self._demo_running() else None
            self._record_command(
                f"/api/standup mode={mode} direction={direction} "
                f"speed={speed:.2f}")
            self._do_reset("plant" if down else "zero", 0.0,
                           f"{name}: command playback")
            start_deg = sim_rad_to_robot_abs_deg(self._q_now())
            pose_fn, dur = self._pose_fn_from_frames(start_deg, frames)
            self._set_demo_safety(True)
            self.demo_pose_fn = pose_fn
            self.demo_t = 0.0
            self.demo_duration = dur
            self.demo_notes = []
            self.demo_note = ""
            self.demo_speed_cap = 10.0
            self.demo_is_script = False
            self.demo_end_home = home
            self.demo_direct_profile = True
            self.demo_pose_frame = "robot_abs"
            # compare_standup.py validated these stand-up paths with a
            # 90 deg/s bus profile; the ServoProfile still applies its
            # fitted per-axis velocity ceiling, latency, deadband, and
            # torque/friction limits.
            self.demo_write_speed_deg_s = 90.0
            self.demo_write_acc_units = self.env.write_acc_units
            self.demo_last_target_deg = None
            self.demo_started_sim_t = self.sim_t
            self.demo_name = name
            self.demo_status = f"running @ {speed:.2f}x"
            self.demo_speed_live = speed
            self.demo_telemetry = None
            self.demo_params = {"mode": mode, "speed": speed,
                                "direction": direction,
                                "home": home,
                                "seconds": round(dur / speed, 2)}
            if switched_from:
                self.demo_params["switched_from"] = switched_from
            self.drive_active = False
            self.timed_walk_until = None
            self.auto = None
            self.gait = None
            self.om_cmd = 0.0
            self.armed = True
            self.sitting = False
            self.pose_hold_q = None
            self.msg = f"{name} running"
            self._open_log(name)
            return {"ok": True, "params": dict(self.demo_params),
                    "home": home, "switched": bool(switched_from),
                    "switched_from": switched_from,
                    "demo": self.demo_state(),
                    "robot": self.robot_state()}

    # -- dance scripts (dances as data — same API shape as the robot) --------
    # Sources: the repo's baked library (dances/) is always available;
    # uploads via POST /api/dances land in ~/.hexapod_dances (upload wins
    # on a name clash, mirroring "robot-local state beats the repo").

    @property
    def _dance_upload_dir(self) -> Path:
        return Path.home() / ".hexapod_dances"

    def _dance_sources(self) -> list[Path]:
        return [self._dance_upload_dir, self._proto_root / "dances"]

    def _dance_file(self, name: str) -> Path | None:
        import dance_script as DS
        if not isinstance(name, str) or not DS.NAME_RE.match(name):
            return None
        for d in self._dance_sources():
            p = d / f"{name}.json"
            if p.is_file():
                return p
        return None

    def list_dance_scripts(self) -> list[dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for d in reversed(self._dance_sources()):   # uploads override repo
            try:
                paths = sorted(d.glob("*.json"))
            except OSError:
                continue
            for p in paths:
                try:
                    s = json.loads(p.read_text())
                    out[s["name"]] = {
                        "name": s["name"],
                        "title": s.get("title") or s["name"],
                        "stands": bool(s.get("stands")),
                        "seconds": s.get("seconds"),
                        "acts": len(s.get("acts") or []),
                        "bytes": p.stat().st_size,
                        "baked_from": s.get("baked_from"),
                    }
                except (OSError, ValueError, KeyError):
                    continue
        return sorted(out.values(), key=lambda m: m["name"])

    def get_dance_script(self, name: str) -> dict[str, Any] | None:
        p = self._dance_file(name)
        if p is None:
            return None
        try:
            return json.loads(p.read_text())
        except (OSError, ValueError):
            return None

    def save_dance_script(self, script: Any) -> dict[str, Any]:
        import dance_script as DS
        errs, stats = DS.validate_script(script)
        if errs:
            return {"ok": False, "error": "; ".join(errs[:5])}
        name = script["name"]
        if name in QUAD_STREAM_DEMOS:
            return {"ok": False, "error": f"{name!r} is a built-in demo name"}
        try:
            self._dance_upload_dir.mkdir(parents=True, exist_ok=True)
            p = self._dance_upload_dir / f"{name}.json"
            tmp = p.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(script))
            tmp.replace(p)
        except OSError as e:
            return {"ok": False, "error": f"save failed: {e}"}
        return {"ok": True, "name": name, "stats": stats,
                "bytes": p.stat().st_size}

    def delete_dance_script(self, name: str) -> dict[str, Any]:
        p = self._dance_file(name)
        if p is None or p.parent != self._dance_upload_dir:
            return {"ok": False,
                    "error": f"no uploaded dance {name!r} (repo-baked "
                             f"scripts can't be deleted here)"}
        try:
            p.unlink()
        except OSError as e:
            return {"ok": False, "error": str(e)}
        return {"ok": True, "deleted": name}

    def _run_dance_script(self, name: str, script: dict[str, Any],
                          speed: float) -> dict[str, Any]:
        """Compile a dance script to a sim timeline and start it."""
        import dance_script as DS
        try:
            kfs = DS.load_standup_keyframes(
                self._proto_root / "linux_control" / "standup_modes.json")
        except (OSError, ValueError):
            kfs = {}
        try:
            pose_fn, dur, notes, cap = DS.compile_script_timeline(
                script, standup_keyframes=kfs)
        except ValueError as e:
            return {"ok": False, "error": str(e)}
        switched_from = self.demo_name if self._demo_running() else None
        self._record_command(f"/api/demo name={name} speed={speed:.2f}")
        # Scripts start at sit zero (belly down, legs straight out).
        self._do_reset("zero", 0.0, f"sit zero -> {name}")
        self._set_demo_safety(True)
        self.demo_pose_fn = pose_fn
        self.demo_t = 0.0
        self.demo_duration = dur
        self.demo_notes = list(notes)
        self.demo_note = ""
        self.demo_speed_cap = cap
        self.demo_is_script = True
        self.demo_direct_profile = True
        self.demo_pose_frame = "robot_abs"
        self.demo_write_speed_deg_s = self.env.write_speed_deg_s
        self.demo_write_acc_units = self.env.write_acc_units
        self.demo_started_sim_t = self.sim_t
        self.demo_name = name
        self.demo_status = f"running @ {speed:.2f}x"
        self.demo_speed_live = speed
        self.demo_telemetry = None
        params: dict[str, Any] = {"speed": speed, "home": "sit",
                                  "seconds": round(dur, 1)}
        if switched_from:
            params["switched_from"] = switched_from
        self.demo_params = params
        self.drive_active = False
        self.timed_walk_until = None
        self.auto = None
        self.gait = None
        self.om_cmd = 0.0
        self.armed = True
        self.msg = f"{name} running"
        self._open_log(f"demo_{name}")
        return {
            "ok": True,
            "params": dict(params),
            "home": "sit",
            "switched": bool(switched_from),
            "switched_from": switched_from,
            "demo": self.demo_state(),
            "robot": self.robot_state(),
        }

    @staticmethod
    def _clamp_float(value: Any, default: float,
                     lo: float, hi: float) -> float:
        try:
            x = float(value)
        except (TypeError, ValueError):
            x = default
        return max(lo, min(hi, x))

    def set_demo_speed(self, speed: Any) -> dict[str, Any]:
        v = self._clamp_float(speed, 1.0, 0.25, 3.0)
        with self.lock:
            self._record_command(f"/api/demo/speed speed={v:.2f}",
                                 key="demo-speed")
            self.demo_speed_live = v
            if self.demo_params:
                self.demo_params = {**self.demo_params, "speed_live": v}
            if self._demo_running():
                self.demo_status = f"running @ {v:.2f}x"
            return {"ok": True, "speed": v,
                    "running": self._demo_running(),
                    "demo": self.demo_state()}

    def run_demo(self, name: str, *, speed: float = 1.0,
                 size: float = 1.0, rate: float | None = None,
                 torque: int | None = None, softness: float = 1.0,
                 seconds: float | None = None) -> dict[str, Any]:
        name = (name or "").strip()
        if name not in QUAD_STREAM_DEMOS:
            script = self.get_dance_script(name)
            if script is None:
                return {"ok": False,
                        "error": f"demo {name!r} is not simulated yet",
                        "demos": [d["name"] for d in self.list_demos()]}
            with self.lock:
                return self._run_dance_script(
                    name, script,
                    self._clamp_float(speed, 1.0, 0.25, 3.0))
        with self.lock:
            try:
                import quad_walk as QW
            except Exception as e:
                return {"ok": False, "error": f"quad_walk missing: {e}"}

            speed = self._clamp_float(speed, 1.0, 0.25, 3.0)
            action = _quad_action(name)
            if name in QUAD_DOWN_DEMOS:
                dur = float(QW.EXIT_TOTAL_S)
            else:
                default_dur = 300.0 if name in QUAD_REQUIRES_REAR else 40.0
                dur = self._clamp_float(seconds, default_dur, 2.0, 300.0)
                if name in QUAD_REAR_DEMOS:
                    dur = max(dur, float(QW.ENTRY_TOTAL_S) + 0.5)
            switched_from = self.demo_name if self._demo_running() else None
            quad_current = self._demo_running() and self.demo_name in QUAD_STREAM_DEMOS
            if name in QUAD_REQUIRES_REAR and not (self.quad_reared or quad_current):
                return {
                    "ok": False,
                    "error": "quad: rear up first, then walk/trot/down",
                    "demo": self.demo_state(),
                    "robot": self.robot_state(),
                }
            self._record_command(
                f"/api/demo name={name} speed={speed:.2f} seconds={dur:.1f}")

            if name in QUAD_REAR_DEMOS:
                self._do_reset("plant", 0.0, f"stand zero -> {name}")
            else:
                if quad_current:
                    self._stop_demo_locked(status="aborted")
                self.pose_hold_q = None
                self.drive_active = False
                self.timed_walk_until = None
                self.traj.vx = self.traj.vy = 0.0
                self.auto = None
                self.gait = None
                self.om_cmd = 0.0
            self._set_demo_safety(True)
            base_deg = sim_rad_to_robot_abs_deg(self.q_plant)
            gait = QUAD_DEMO_GAITS[name]
            phase = (
                "rear" if action == "rear"
                else "hold" if action == "hold"
                else "down" if action == "down"
                else "walk")
            direction = -1.0 if action in ("walk_back", "trot_back") else 1.0
            self.demo_pose_fn = QW.make_quad_walk_pose_fn(
                base_deg, dur, gait=gait, direction=direction, phase=phase)
            self.demo_t = 0.0
            self.demo_duration = dur
            self.demo_started_sim_t = self.sim_t
            self.demo_end_home = "stand" if name in QUAD_DOWN_DEMOS else ""
            self.demo_direct_profile = True
            self.demo_pose_frame = "robot_abs"
            self.demo_write_speed_deg_s = self.env.write_speed_deg_s
            self.demo_write_acc_units = 254.0
            self.demo_name = name
            self.demo_status = f"running @ {speed:.2f}x"
            self.demo_speed_live = speed
            self.demo_telemetry = None
            params: dict[str, Any] = {
                "speed": speed,
                "home": "stand" if name in QUAD_REAR_DEMOS else "quad",
                "seconds": dur,
            }
            if switched_from:
                params["switched_from"] = switched_from
            self.demo_params = params
            self.drive_active = False
            self.timed_walk_until = None
            self.auto = None
            self.gait = None
            self.om_cmd = 0.0
            self.armed = True
            self.msg = f"{name} running"
            self._open_log(f"demo_{name}")
            return {
                "ok": True,
                "params": dict(params),
                "home": params["home"],
                "switched": bool(switched_from),
                "switched_from": switched_from,
                "demo": self.demo_state(),
                "robot": self.robot_state(),
            }

    def stop_demo(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/demo/stop")
            was_running = self._demo_running()
            prev = self.demo_name or ""
            self._stop_demo_locked(status="aborted" if was_running else "idle")
            if was_running and prev in QUAD_STREAM_DEMOS:
                self.quad_reared = False
                self.pose_hold_q = self._q_now().copy()
            self.drive_active = False
            self.timed_walk_until = None
            self.traj.vx = self.traj.vy = 0.0
            self.mode = "hold"
            self.traj.mode = "hold"
            self.msg = "demo stopped - holding"
            return {"ok": True, "demo": self.demo_state(),
                    "robot": self.robot_state()}

    def go_zero(self, pose: str = "sit", *, force: bool = False) -> dict[str, Any]:
        pose = (pose or "sit").strip().lower()
        pose = "stand" if pose in {"stand", "standing", "plant"} else "sit"
        with self.lock:
            self._record_command(f"/api/zero pose={pose}")
            if pose == "stand":
                self._do_reset("plant", 0.0, "at stand zero")
                self.armed = True
            else:
                self._do_reset("zero", 0.0, "at sit zero")
                self.sitting = True
                self.q_sit = self._q_now()
                self.armed = True
            self.demo_name = f"{pose}_zero"
            self.demo_status = "done"
            self.demo_params = {"home": pose, "force": bool(force)}
            return {"ok": True, "pose": pose, "demo": self.demo_state(),
                    "robot": self.robot_state()}

    def safe_zero(self, *, dry_run: bool = False) -> dict[str, Any]:
        return self.go_zero("sit")

    def set_zero_here(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/set_zero")
            return {"ok": True, "sim": True, "ok_n": 18, "count": 18,
                    "message": "sim logical zero unchanged"}

    def ping(self) -> dict[str, Any]:
        return {"ok": True, "service": "hexapod-sim",
                "kind": "sim", "mode": self.mode,
                "viewer": self.cfg.viewer,
                "frames": self.cfg.web_frames}

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {"ok": True, "sim": True, "motors": [],
                    "live": self._live()}

    def robot_state(self) -> dict[str, Any]:
        with self.lock:
            act = "armed" if self.armed else "limp"
            if self.drive_active:
                act = "driving"
            elif self._demo_running():
                act = "demo"
            elif self.auto is not None or self.job_kind:
                act = "rl"
            return {"ok": True, "activity": act, "detail": self.msg,
                    "mode": self.mode, "armed": self.armed,
                    "sim": True, "live": self._live(),
                    "demo": self.demo_state()}

    def operation_state(self) -> dict[str, Any]:
        with self.lock:
            running = bool(self.job_kind)
            return {"ok": True, "running": running,
                    "name": "rl_policy_sim" if running else "",
                    "progress": {"msg": self.msg, "live": self._live()},
                    "result": self.job_result}

    def calibration_report(self) -> dict[str, Any]:
        with self.lock:
            try:
                import tripod_gait as TG
            except Exception:
                TG = None
            plant_deg = [round(math.degrees(float(v)), 3)
                         for v in self.q_plant]

            def foot_from(hip_deg: float, knee_deg: float) -> dict[str, float]:
                if TG is None:
                    return {"radial_mm": 0.0, "z_mm": 0.0}
                hip = math.radians(float(hip_deg))
                knee = math.radians(float(knee_deg))
                reach = (TG.COXA_MM + TG.FEMUR_MM * math.cos(hip)
                         + TG.TIBIA_MM * math.cos(hip + knee))
                z = (-TG.FEMUR_MM * math.sin(hip)
                     - TG.TIBIA_MM * math.sin(hip + knee))
                return {"radial_mm": round(reach, 2), "z_mm": round(z, 2)}

            per_leg = []
            for leg in range(6):
                yaw, hip, knee = plant_deg[leg * 3:leg * 3 + 3]
                per_leg.append({
                    "leg": leg,
                    "yaw_deg": yaw,
                    "hip_deg": hip,
                    "knee_deg": knee,
                    **foot_from(hip, knee),
                })
            z_vals = [float(r["z_mm"]) for r in per_leg]
            radial_vals = [float(r["radial_mm"]) for r in per_leg]
            params = getattr(self.env, "params", None)
            servo_params = None
            if params is not None:
                try:
                    servo_params = {
                        "source": getattr(params, "source", ""),
                        "timestamp": getattr(params, "timestamp", ""),
                        "speed_counts_s": getattr(params, "speed_counts_s", None),
                        "axes": {
                            ax: asdict(p)
                            for ax, p in getattr(params, "axes", {}).items()
                        },
                        "spread": getattr(params, "spread", {}),
                    }
                except Exception:
                    servo_params = {"source": str(getattr(params, "source", ""))}
            report = {
                "ok": True,
                "mode": "calibration_report",
                "sim": True,
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "phases": [{
                    "name": "sim_snapshot",
                    "ok": True,
                    "summary": "MuJoCo state and active servo contract",
                }],
                "geometry": {
                    "ok": True,
                    "nominal_mm": ({
                        "coxa": TG.COXA_MM,
                        "femur": TG.FEMUR_MM,
                        "tibia": TG.TIBIA_MM,
                        "chassis_flat_to_flat": TG.CHASSIS_FLAT_TO_FLAT_MM,
                    } if TG is not None else {}),
                    "plant_joint_deg": plant_deg,
                    "per_leg": per_leg,
                    "summary": {
                        "mean_foot_z_mm": round(sum(z_vals) / len(z_vals), 2),
                        "foot_z_spread_mm": round(max(z_vals) - min(z_vals), 2),
                        "mean_radial_mm": round(
                            sum(radial_vals) / len(radial_vals), 2),
                        "radial_spread_mm": round(
                            max(radial_vals) - min(radial_vals), 2),
                    },
                    "mujoco_hint": {
                        "plant_joint_deg": plant_deg,
                        "neutral_foot_z_m": round(
                            (sum(z_vals) / len(z_vals)) * 0.001, 5),
                    },
                },
                "imu": {
                    "ok": True,
                    "sim": True,
                    "body_calibrated": True,
                    "body_frame": {
                        "pitch_axis": "pitch",
                        "pitch_axis_roll": 0.0,
                        "pitch_axis_pitch": 1.0,
                        "pitch_sign": 1.0,
                        "source": "mujoco_body_frame",
                    },
                },
                "actuators": {
                    "ok": True,
                    "sim": True,
                    "learned_model": servo_params,
                    "snapshot": None,
                },
                "live": self._live(),
            }
            stamp = time.strftime("%Y%m%d_%H%M%S")
            path = self.log_dir / f"calibration_report_{stamp}.json"
            latest = self.log_dir / "calibration_report_latest.json"
            path.write_text(json.dumps(report, indent=2) + "\n")
            latest.write_text(json.dumps(report, indent=2) + "\n")
            report["path"] = str(path)
            report["log_name"] = path.name
            report["latest"] = str(latest)
            return report

    def run_calibrate(self, *, mode: str = "checkup",
                      clearance_mm: float = 40.0, **_kw) -> dict[str, Any]:
        mode = (mode or "checkup").strip().lower()
        with self.lock:
            if mode not in {"checkup", "calibration", "auto", "all",
                            "geometry", "imu"}:
                self.job_result = {
                    "ok": False,
                    "mode": mode,
                    "error": f"sim calibration mode {mode!r} is report-only",
                }
            else:
                report = self.calibration_report()
                phases = list(report.get("phases") or [])
                if mode in {"checkup", "calibration", "auto", "all"}:
                    phases = [
                        {"name": "imu_rest", "ok": True,
                         "summary": "sim IMU is body-frame exact"},
                        {"name": "geometry_plant", "ok": True,
                         "summary": "sim plant pose captured"},
                        {"name": "actuator_contract", "ok": True,
                         "summary": "active servo params captured"},
                    ]
                self.job_result = {
                    "ok": True,
                    "mode": "checkup" if mode in {"calibration", "auto",
                                                 "all"} else mode,
                    "phases": phases,
                    "report": report,
                    "geometry": report.get("geometry"),
                    "imu": report.get("imu"),
                    "actuators": report.get("actuators"),
                    "path": report.get("path"),
                    "log_name": report.get("log_name"),
                    "latest": report.get("latest"),
                    "clearance_mm": float(clearance_mm),
                    "msg": "sim calibration report saved",
                }
            self.msg = self.job_result.get("msg") or self.job_result.get(
                "error") or "sim calibration"
            self.job_kind = None
            return {"ok": bool(self.job_result.get("ok")),
                    "calibrate": self.operation_state(),
                    "result": dict(self.job_result)}

    def sim_state(self) -> dict[str, Any]:
        with self.lock:
            return {"ok": True, "active": self.drive_active,
                    "auto": self.auto[0] if self.auto else None,
                    "downed": self.downed, "sitting": self.sitting,
                    "viewer": self.cfg.viewer,
                    "frames": self.cfg.web_frames,
                    "live": self._live()}

    def rl_preflight(self, mode: str = "stand") -> dict[str, Any]:
        with self.lock:
            roll, pitch = self._roll_pitch_deg()
            if self.auto is not None:
                return {"ok": False, "error": "auto transition running",
                        "roll_deg": roll, "pitch_deg": pitch}
            if mode == "walk" and (self.downed or self.sitting
                                   or self._chassis_z() < 0.09):
                return {"ok": False, "error": "stand before walking",
                        "roll_deg": roll, "pitch_deg": pitch}
            return {"ok": True, "sim": True, "mode": mode,
                    "roll_deg": roll, "pitch_deg": pitch,
                    "max_pose_delta_deg": 0.0, "pose_tol_deg": 180.0}

    def rl_policy_info(self) -> dict[str, Any]:
        with self.lock:
            return {"ok": True, **self._model_info(self.stance,
                                                   self.stance_list[self.si]),
                    "walk": self._model_info(self.walk, self.walk_list[self.wi])
                    if self.walk is not None else self._scripted_info()}

    def _model_info(self, model: Any, path: Path) -> dict[str, Any]:
        if hasattr(model, "meta"):      # uploaded numpy policy
            return {"source": str(path),
                    "obs_dim": int(model.observation_space.shape[0]),
                    "act_dim": int(model.action_space.shape[0]),
                    "hidden": list(model.hidden),
                    "activation": model.meta.get("activation", "tanh"),
                    "uploaded": True}
        return {"source": str(path), "obs_dim": int(model.observation_space.shape[0]),
                "act_dim": int(model.action_space.shape[0]),
                "hidden": self._hidden_layers(model),
                "activation": getattr(getattr(model.policy, "activation_fn", None),
                                      "__name__", model.policy.__class__.__name__)}

    @staticmethod
    def _hidden_layers(model: Any) -> list[int]:
        try:
            import torch.nn as nn
            return [m.out_features for m in model.policy.mlp_extractor.policy_net
                    if isinstance(m, nn.Linear)]
        except Exception:
            return []

    def _scripted_info(self) -> dict[str, Any]:
        return {"source": self._active_walk_name(), "obs_dim": 72,
                "act_dim": 18, "hidden": [], "activation": "scripted"}

    def rl_policies(self) -> dict[str, Any]:
        with self.lock:
            rows = []
            for p in self.stance_list:
                rows.append(self._policy_row(p, "stance", 68,
                                             p == self.stance_list[self.si]))
            for p in self.walk_list:
                if p in _SCRIPTED_ROWS:
                    rows.append(self._policy_row(p, "walk", 72,
                                                 p == self.walk_list[self.wi],
                                                 scripted=True))
                else:
                    rows.append(self._policy_row(
                        p, "walk", self._policy_obs_width(p) or 0,
                        p == self.walk_list[self.wi]))
            return {"ok": True, "dir": str(self.cfg.policy_dir),
                    "policies": rows}

    def _policy_row(self, p: Path, slot: str, obs_dim: int, active: bool,
                    scripted: bool = False) -> dict[str, Any]:
        file = f"scripted:{p.name}" if scripted else p.name
        row = {"file": file, "name": p.stem, "slot": slot,
               "obs_dim": obs_dim, "active": active,
               "notes": _DESC.get(p.stem, "scripted gait" if scripted else "")}
        if p.suffix == ".json":
            row["uploaded"] = True
            try:
                meta = json.loads(p.read_text())["meta"]
                row["name"] = meta.get("name") or p.stem
                row["notes"] = meta.get("notes") or "uploaded policy"
            except (OSError, ValueError, KeyError):
                row["notes"] = "uploaded policy"
        return row

    def rl_roles(self) -> dict[str, Any]:
        with self.lock:
            roles = {}
            for role in ("walk", "hold", "stand", "lower"):
                roles[role] = {"file": self.roles.get(role, ""),
                               "resolved": self._role_resolved(role)}
            return {"ok": True,
                    "allowed_obs": {"walk": list(self.walk_widths),
                                    "hold": [68, *self.walk_widths],
                                    "stand": [68], "lower": [68]},
                    "roles": roles}

    def _role_resolved(self, role: str) -> str:
        if role == "walk":
            return self._active_walk_name()
        entry = self.role_models.get(role)
        if isinstance(entry, tuple):
            return entry[1].name
        if entry == "walk":
            return "walk policy @ zero command"
        return self._active_stance_name()

    def rl_role_set(self, role: str, file: str) -> dict[str, Any]:
        role = role.strip().lower()
        if role not in {"walk", "hold", "stand", "lower"}:
            return {"ok": False, "error": f"bad role {role!r}"}
        with self.lock:
            self._record_command(
                f"/api/rl/roles {role}={file or 'default'}")
            if not file:
                self.roles.pop(role, None)
                self.role_models.pop(role, None)
                return self.rl_roles()
            if role == "hold" and file == "walk":
                self.roles[role] = "walk"
                self.role_models[role] = "walk"
                return self.rl_roles()
            p = self.policy_index.get(file)
            if p is None or p in _SCRIPTED_ROWS:
                return {"ok": False, "error": f"unknown policy {file!r}"}
            w = self._policy_obs_width(p)
            if role in {"stand", "lower"} and w != 68:
                return {"ok": False, "error": f"{role} needs obs 68"}
            if role == "hold" and w not in (68, *self.walk_widths):
                return {"ok": False, "error": "hold needs stance/walk obs"}
            model = self._load_model(p)
            self.roles[role] = p.name
            self.role_models[role] = (model, p, int(w or 0))
            return self.rl_roles()

    def rl_policy_select(self, file: str) -> dict[str, Any]:
        with self.lock:
            self._record_command(f"/api/rl/policy_select file={file}")
            p = self.policy_index.get(file)
            if p is None:
                return {"ok": False, "error": f"unknown policy {file!r}"}
            if p in _SCRIPTED_ROWS:
                self._set_walk_path(p)
                return {"ok": True, "name": p.stem, "slot": "walk"}
            w = self._policy_obs_width(p)
            if w == 68:
                self._set_stance_path(p)
                return {"ok": True, "name": p.stem, "slot": "stance"}
            if w in self.walk_widths or (p.suffix == ".json"
                                         and w in (72, 74)):
                self._set_walk_path(p)
                return {"ok": True, "name": p.stem, "slot": "walk"}
            return {"ok": False, "error": f"unsupported obs width {w}"}

    def _set_stance_path(self, p: Path) -> None:
        self.si = self.stance_list.index(p.resolve())
        self.stance = self._load_model(p)
        self.n_stance = int(self.stance.observation_space.shape[0])
        self.msg = f"stance model -> {p.stem}"

    def _set_walk_path(self, p: Path) -> None:
        self.wi = self.walk_list.index(p)
        self.gait = None
        if p in _SCRIPTED_ROWS:
            self.walk = None
            self.n_walk = 72
            self.walk_kind = "plain"
            self.msg = f"walk driver -> scripted {p.name}"
            return
        self.walk = self._load_model(p)
        self.n_walk = int(self.walk.observation_space.shape[0])
        self.walk_kind = self._walk_kind_of(self.n_walk)
        if self.walk_kind == "plain" and self.n_walk > self.n_env:
            raise ValueError(f"{p.stem} needs --phase-obs")
        self._reset_memories(hard=True)
        self._apply_vel_contract(p.stem)
        self.msg = f"walk model -> {p.stem}"

    def rl_capture_plant(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/rl/capture_plant")
            self.q_plant = self._q_now()
            self.z_plant = self._chassis_z()
            return {"ok": True, "sim": True,
                    "hip_deg": round(math.degrees(self.q_plant[1]), 1),
                    "knee_deg": round(math.degrees(self.q_plant[2]), 1)}

    def rl_policy_move(self, mode: str, vx: float = 0.03, vy: float = 0.0,
                       duration_s: float = 6.0) -> dict[str, Any]:
        with self.lock:
            if mode == "walk":
                self._record_command(
                    f"/api/rl/walk vx={vx:+.3f} vy={vy:+.3f} "
                    f"duration={duration_s:.1f}")
            else:
                self._record_command(f"/api/rl/{mode}")
            self.drive_active = False
            self.timed_walk_until = None
            self._open_log(mode)
            self.job_kind = mode
            if mode == "stand":
                self._do_stand()
                if self.auto is None:
                    self._finish_job(self.msg)
            elif mode == "lower":
                self._do_sit()
            elif mode == "walk":
                if not self._engage_walk():
                    self._finish_job(self.msg, ok=False)
                    return {"ok": False, "error": self.msg}
                _, vmax = self._drive_band()
                mag = float(np.hypot(vx, vy))
                scale = min(vmax / mag, 1.0) if mag > 1e-9 else 0.0
                self.traj.vx = float(vx * scale)
                self.traj.vy = float(vy * scale)
                self.timed_walk_until = self.sim_t + max(0.1, duration_s)
                self.msg = "timed walk running"
            else:
                self._finish_job(f"bad mode {mode}", ok=False)
                return {"ok": False, "error": f"bad mode {mode}"}
            return {"ok": True, "status": self.msg,
                    "active": self.drive_active, "live": self._live()}

    def rl_stop(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/rl/stop")
            self.drive_active = False
            self.timed_walk_until = None
            self.traj.vx = self.traj.vy = 0.0
            self.om_cmd = 0.0
            self.auto = None
            self._finish_job("stopped - holding")
            return {"ok": True, "status": self.msg, "live": self._live()}

    def rl_drive_start(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/rl/drive/start")
            if self.auto is None and (self.sitting or self._chassis_z() < 0.09):
                self._do_stand()
            self.drive_active = True
            self.last_drive_cmd_at = time.monotonic()
            self.traj.vx = self.traj.vy = 0.0
            self.om_cmd = 0.0
            self._open_log("drive")
            self.msg = "drive session active"
            return {"ok": True, "active": True, "status": self.msg,
                    "live": self._live()}

    def rl_drive_cmd(self, vx: float, vy: float,
                     wz: float = 0.0) -> dict[str, Any]:
        with self.lock:
            wz = max(-0.5, min(0.5, float(wz)))
            self._record_command(
                f"/api/rl/drive/cmd vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f}",
                key="drive-cmd")
            if not self.drive_active:
                return {"ok": True, "active": False, "status": "not active",
                        "result": self.job_result}
            self.last_drive_cmd_at = time.monotonic()
            if self._engage_walk():
                _, vmax = self._drive_band()
                mag = float(np.hypot(vx, vy))
                scale = min(vmax / mag, 1.0) if mag > 1e-9 else 0.0
                self.traj.vx = float(vx * scale)
                self.traj.vy = float(vy * scale)
                self.om_cmd = wz
                twz = getattr(self.traj, "wz", None)
                if twz is not None:
                    try:
                        twz[:] = wz
                    except (TypeError, ValueError):
                        self.traj.wz = wz
            return {"ok": True, "active": self.drive_active,
                    "status": self.msg, "live": self._live()}

    def rl_drive_stop(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/rl/drive/stop")
            self.drive_active = False
            self.traj.vx = self.traj.vy = 0.0
            self.om_cmd = 0.0
            self._close_log()
            self.job_result = {"ok": True, "ended": "drive stopped",
                               "sim_t_s": round(self.sim_t, 2),
                               "log": self._log_name or None}
            self.msg = "drive stopped - holding"
            return {"ok": True, "active": False, "result": self.job_result,
                    "live": self._live()}

    def rl_drive_state(self) -> dict[str, Any]:
        with self.lock:
            return {"ok": True, "active": self.drive_active,
                    "status": self.msg, "result": self.job_result,
                    "live": self._live()}

    def cmd(self, line: str) -> dict[str, Any]:
        parts = line.strip().split()
        head = parts[0].upper() if parts else ""
        with self.lock:
            self._record_command(f"/cmd {line.strip()}",
                                 key="cmd-j" if head == "J" else None)
            if head == "ARM":
                self.armed = True
                self.msg = "sim armed"
            elif head in {"P", "STAND"}:
                self._do_reset("plant", 0.0, "reset plant")
            elif head in {"X", "DISARM", "RELAX"}:
                self.armed = False
                self.quad_reared = False
                self.rl_stop()
                self.msg = "sim stopped"
            elif head == "SETTLE":
                self.armed = False
                self._do_sit()
            elif head == "HOLD":
                self.traj.vx = self.traj.vy = 0.0
                self.msg = "holding"
            elif head == "J" and len(parts) >= 4:
                if self._engage_walk():
                    self.traj.vx = float(parts[1]) / 1000.0
                    self.traj.vy = float(parts[2]) / 1000.0
                    self.om_cmd = float(parts[3])
                    self.msg = "J command routed to sim"
            return {"ok": True, "status": self.msg}

    def sim_reset(self, start: str = "plant") -> dict[str, Any]:
        with self.lock:
            if start not in {"plant", "zero", "belly"}:
                start = "plant"
            self._record_command(f"/api/sim/reset start={start}")
            h = 0.0
            self._do_reset("zero" if start in {"zero", "belly"} else "plant",
                           h, f"reset {start}")
            return {"ok": True, "status": self.msg, "live": self._live()}

    def sim_pose(self, degrees: Any, source: str = "robot") -> dict[str, Any]:
        try:
            if not isinstance(degrees, (list, tuple)) or len(degrees) != 18:
                raise ValueError("expected 18 joint degrees")
            q_deg = np.array([float(v) for v in degrees], dtype=float)
            if not np.all(np.isfinite(q_deg)):
                raise ValueError("joint degrees must be finite numbers")
        except Exception as e:
            return {"ok": False, "error": str(e)}

        q_rad = np.radians(q_deg)
        with self.lock:
            self._record_command(f"/api/sim/pose source={source}")
            self._stop_demo_locked(status="idle", clear_name=True)
            self.auto = None
            self.downed = False
            self.sitting = False
            self.drive_active = False
            self.timed_walk_until = None
            self.gait = None
            self.gait_t = 0.0
            self.om_cmd = 0.0
            self.traj.start_at = "plant"
            self.traj.goal = TaskGoal()
            self.traj.vx = self.traj.vy = 0.0
            self.traj.mode = "hold"
            self.traj.reset_published()
            self._reset_memories(hard=True)
            if hasattr(self.env, "_place_at_plant"):
                self.env._place_at_plant(q_rad)
            else:
                qpos = self.env.data.qpos.copy()
                qpos[7:25] = q_rad
                qvel = np.zeros_like(self.env.data.qvel)
                self._restore_phys(qpos, qvel)
            self.pose_hold_q = q_rad.copy()
            self.q_plant = q_rad.copy()
            self.z_plant = self._chassis_z()
            self.env._profile.reset(self._q_now())
            self.env.safety.set_nominal(self._q_now())
            self._finish_job(f"synced {source} pose")
            return {
                "ok": True,
                "status": self.msg,
                "source": source,
                "live_joints": 18,
                "degrees": [round(float(v), 2) for v in q_deg],
                "live": self._live(),
            }

    def sim_fall(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/sim/fall")
            self._do_fall()
            return {"ok": True, "status": self.msg, "live": self._live()}

    def sim_recover(self) -> dict[str, Any]:
        with self.lock:
            self._record_command("/api/sim/recover")
            self.job_kind = "recover"
            self._open_log("recover")
            self._do_recover()
            return {"ok": self.auto is not None and self.auto[0] == "recover",
                    "status": self.msg, "live": self._live()}

    def sim_push(self, x: float = 4.0, y: float = 0.0) -> dict[str, Any]:
        with self.lock:
            self._record_command(f"/api/sim/push x={x:+.1f} y={y:+.1f}")
            self.push_force[:] = [x, y, 0.0]
            self.push_ticks = int(0.20 / self.env.dt)
            self.msg = f"push {x:+.1f},{y:+.1f} N"
            return {"ok": True, "status": self.msg, "live": self._live()}

    def frame_jpeg(self) -> bytes:
        if not self.cfg.web_frames:
            raise RuntimeError("browser frames disabled; use native MuJoCo viewer")
        if not self._frame_ready.wait(2.0):
            raise RuntimeError("sim frame not ready")
        with self.lock:
            if self._frame_jpeg is not None:
                return self._frame_jpeg
            err = self._frame_error or "sim frame unavailable"
        raise RuntimeError(err)

    def logs(self) -> dict[str, Any]:
        files = []
        self.log_dir.mkdir(parents=True, exist_ok=True)
        for f in sorted(self.log_dir.iterdir()):
            if f.is_file():
                st = f.stat()
                files.append({"name": f.name, "bytes": st.st_size,
                              "mtime_unix": round(st.st_mtime, 1)})
        files.sort(key=lambda x: -x["mtime_unix"])
        return {"ok": True, "dir": str(self.log_dir), "files": files}

    def log_file(self, name: str, request_path: str = "") -> tuple[bytes, str]:
        f = self.log_dir / Path(name).name
        if not f.is_file():
            raise FileNotFoundError(f"no such log: {name!r}")
        tail = 0
        if "tail=" in request_path:
            try:
                tail = int(request_path.split("tail=", 1)[1].split("&", 1)[0])
            except ValueError:
                tail = 0
        data = f.read_bytes()
        if tail > 0:
            lines = data.splitlines()[-tail:]
            data = b"\n".join(lines) + (b"\n" if lines else b"")
        return data, "text/csv; charset=utf-8"

    def close(self) -> None:
        self.stop_event.set()
        if getattr(self, "thread", None):
            self.thread.join(timeout=2.0)
        self._close_log()

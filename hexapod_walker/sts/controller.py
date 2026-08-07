"""Inference controller: residual RL policy + STS stall safe-stop.

Used by ``rollout.py`` (sim) and eventually by the Uno Q drive loop.
The observation builder consumes either:

  * simulated ``StsSensorBank`` feedback, or
  * real ``FeetechBus.read_feedback`` dicts (same field names)

so the trained policy never sees privileged MuJoCo state at deploy time.

Safe-stop behaviour (mirrors ``StallGuard``):
  1. detect near-stall on load ∧ current ∧ low speed
  2. freeze joint targets at the last measured pose
  3. after a short hold, request torque-off (``relax``)
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

from stall_guard import GuardState, StallConfig, StallGuard, stall_severity
from sts_sensors import I_STALL_A, V_NOMINAL


@dataclass
class ControllerConfig:
    residual_scale: float = 0.055
    gait_period: float = 0.82
    action_filter_tau: float = 0.14
    gait_action: bool = True
    per_leg_lift: bool = True
    period_scale_range: tuple = (0.88, 1.38)
    lift_scale_range: tuple = (0.65, 1.75)
    stride_scale_range: tuple = (0.52, 1.18)
    gait_action_filter_tau: float = 0.38
    control_hz: int = 50
    stall: StallConfig | None = None


def load_env_cfg(policy_dir: str | Path) -> dict:
    path = Path(policy_dir) / "env_cfg.json"
    if path.is_file():
        return json.loads(path.read_text())
    return {}


def feedback_dicts_to_array(fbs: list[dict]) -> np.ndarray:
    """Pack FeetechBus-style dicts into the (18, 6) array StallGuard expects."""
    out = np.zeros((18, 6), dtype=np.float64)
    for fb in fbs:
        j = int(fb["joint"])
        out[j, 0] = float(fb["deg"])
        out[j, 1] = float(fb["load_pct"])
        out[j, 2] = float(fb.get("volt", V_NOMINAL))
        out[j, 3] = float(fb.get("temp_c", 30.0))
        out[j, 4] = float(fb["current_a"])
        out[j, 5] = float(fb.get("speed_deg_s", 0.0))
    return out


def motor_obs_from_fb(
    fb: np.ndarray,
    guard: StallGuard,
    stall_cfg: StallConfig,
) -> np.ndarray:
    """The trailing MOTOR_OBS_DIM channels appended by StsWalkerEnv._obs."""
    load_n = (fb[:, 1] / 100.0).astype(np.float32)
    cur_n = (fb[:, 4] / I_STALL_A).astype(np.float32)
    volt_n = np.array([fb[:, 2].mean() / V_NOMINAL], dtype=np.float32)
    temp_n = np.array([fb[:, 3].mean() / 100.0], dtype=np.float32)
    sev = np.array([stall_severity(fb)], dtype=np.float32)
    accum = (guard._accum_s / max(stall_cfg.persist_s, 1e-6)).astype(np.float32)
    return np.concatenate([load_n, cur_n, volt_n, temp_n, sev, accum])


class WalkingController:
    """Policy + stall guard.  ``predict`` returns joint targets in radians.

    Parameters
    ----------
    policy:
        Object with ``.predict(obs, deterministic=True) -> (action, _)``.
        Pass a loaded SB3 PPO model, or any duck-typed equivalent.
    gait_fn:
        ``gait_fn(t) -> (yaws, pitches, knees)`` each length-6 in radians.
        Typically ``TripodGait.desired``.
    build_base_obs:
        Callable that builds the leading (non-motor) observation slice from
        whatever proprioception the platform has (sim env or real IMU+encoders).
        Signature: ``build_base_obs(fb_array) -> np.ndarray``.
    """

    def __init__(
        self,
        policy,
        *,
        gait_fn: Callable[[float], tuple],
        build_base_obs: Callable[[np.ndarray], np.ndarray],
        cfg: ControllerConfig | None = None,
        vec_normalize=None,
    ):
        self.policy = policy
        self.gait_fn = gait_fn
        self.build_base_obs = build_base_obs
        self.cfg = cfg or ControllerConfig()
        self.vec_normalize = vec_normalize
        self.guard = StallGuard(self.cfg.stall or StallConfig())
        self._filtered = np.zeros(26, dtype=np.float32)  # max act dim
        self._t0 = 0.0
        self.act_dim = 26 if self.cfg.per_leg_lift else (
            21 if self.cfg.gait_action else 18
        )

    def reset(self, t: float = 0.0):
        self.guard.reset()
        self._filtered[:] = 0.0
        self._t0 = t

    @property
    def state(self) -> GuardState:
        return self.guard.state

    def predict(
        self,
        fb: np.ndarray | list[dict],
        *,
        t: float,
        deterministic: bool = True,
    ) -> dict:
        """One control cycle.

        Returns dict with:
          targets_rad (18,), relax (bool), stall (bool), action, info
        """
        if isinstance(fb, list):
            fb = feedback_dicts_to_array(fb)
        fb = np.asarray(fb, dtype=np.float64)
        dt = 1.0 / self.cfg.control_hz
        self.guard.update(fb, dt)

        if self.guard.is_safe_stopped():
            return {
                "targets_rad": self.guard.freeze_targets_rad(),
                "relax": self.guard.relax_requested,
                "stall": True,
                "action": np.zeros(self.act_dim, dtype=np.float32),
                "tripped_joints": self.guard.tripped_joints.copy(),
            }

        base = self.build_base_obs(fb)
        motor = motor_obs_from_fb(fb, self.guard, self.guard.cfg)
        obs = np.concatenate([base, motor]).astype(np.float32)
        if self.vec_normalize is not None:
            obs = self.vec_normalize.normalize_obs(obs)
        action, _ = self.policy.predict(obs, deterministic=deterministic)
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        action = np.clip(action, -1.0, 1.0)

        # LPF residual (same as env).
        alpha = dt / (self.cfg.action_filter_tau + dt)
        self._filtered[: self.act_dim] += alpha * (
            action[: self.act_dim] - self._filtered[: self.act_dim]
        )
        residual = self._filtered[:18] * self.cfg.residual_scale

        yaws, pitches, knees = self.gait_fn(max(0.0, t - self._t0))
        targets = np.empty(18, dtype=np.float64)
        for i in range(6):
            targets[3 * i + 0] = float(yaws[i]) + residual[3 * i + 0]
            targets[3 * i + 1] = float(pitches[i]) + residual[3 * i + 1]
            targets[3 * i + 2] = float(knees[i]) + residual[3 * i + 2]

        return {
            "targets_rad": targets,
            "relax": False,
            "stall": False,
            "action": action,
            "tripped_joints": self.guard.tripped_joints.copy(),
        }


# Match prototype_sts3215/motor_setup/feetech_bus.py profiles.
# Feetech WritePosEx: speed==0 means MAX, not stop — never use 0 for hold.
_WALK_SPEED = 1500
_WALK_ACC = 30
_HOLD_SPEED = 250
_HOLD_ACC = 40


def _coerce_bus_speed(speed: int | None, default: int) -> int:
    if speed is None:
        return default
    # Feetech footgun: 0 = max.  Treat accidental 0 as a soft hold.
    if int(speed) == 0:
        return _HOLD_SPEED
    return int(speed)


def apply_to_bus(bus, result: dict, *,
                 speed: int | None = None, acc: int | None = None,
                 hold_speed: int | None = None, hold_acc: int | None = None):
    """Write controller output to a live ``FeetechBus``.

    On ``relax=True`` torques every servo off.  On ``stall=True`` (freeze,
    not yet limp) re-commands targets with a *gentle* hold profile —
    never Feetech ``speed=0`` (that is max speed).  Normal walk uses
    ``_WALK_SPEED`` / ``_WALK_ACC``.
    """
    if result.get("relax"):
        if hasattr(bus, "enable_all_torque"):
            bus.enable_all_torque(False)
        else:
            for j in range(18):
                bus.torque(j + 2, False)  # servo IDs 2..19
        return
    degs = np.degrees(result["targets_rad"]).tolist()
    if result.get("stall"):
        sp = _coerce_bus_speed(hold_speed, _HOLD_SPEED)
        ac = _HOLD_ACC if hold_acc is None else int(hold_acc)
    else:
        sp = _coerce_bus_speed(speed, _WALK_SPEED)
        ac = _WALK_ACC if acc is None else int(acc)
    bus.write_all(degs, speed=sp, acc=ac)


def safe_stop_walk(bus, *, limp: bool = False) -> None:
    """End a walk session: soft-hold current pose (supporting torque on).

    Under body weight leave ``limp=False``.  Bench/air demos may limp.
    Prefer ``bus.safe_stop`` / ``hold_current_pose`` when available.
    """
    if hasattr(bus, "safe_stop"):
        bus.safe_stop(limp=limp)
        return
    if hasattr(bus, "hold_current_pose"):
        bus.hold_current_pose(speed=_HOLD_SPEED, acc=_HOLD_ACC)
        if limp and hasattr(bus, "enable_all_torque"):
            bus.enable_all_torque(False)
        return
    # Minimal fallback: re-read isn't available — just enable/disable torque.
    if limp:
        if hasattr(bus, "enable_all_torque"):
            bus.enable_all_torque(False)
        else:
            for j in range(18):
                bus.torque(j + 2, False)

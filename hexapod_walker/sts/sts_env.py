"""Gymnasium env for STS3215 walking + gentle sit/stand.

Builds on the tabletop MuJoCo model + residual tripod gait from
``prototype_sts3215``, but replaces the privileged joint state the old
env exposed with channels that match the real STS bus feedback:

    quantized encoder position / speed
    present-load %
    bus voltage
    winding temperature
    current (A)

Episodes mix:
  * walking (residual on TripodGait), and
  * posture (sit ↔ stand from a legs-out-wide sprawl)

Posture episodes use a slow cosine blend as the nominal trajectory, a
reduced actuator torque cap, and hard/soft current penalties so the
policy learns to stand up and sit down without blasting the motors.

A stall-guard FSM freezes targets (then relaxes torque) when a joint
is jammed.
"""

from __future__ import annotations

import importlib
import math
import os
import sys

import numpy as np
import mujoco
from gymnasium import spaces

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.join(os.path.dirname(THIS_DIR), "prototype_sts3215")
FULLSIZE_DIR = os.path.join(os.path.dirname(THIS_DIR), "fullsize_v1")
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, PROTO_DIR)
sys.path.insert(0, FULLSIZE_DIR)

import mujoco_prototype as MP  # noqa: E402

# Use STS stall torque as the sim actuator ceiling (was hobby-servo 2.2).
MP.TORQUE_LIMIT = 2.70

sys.modules["mujoco_walker"] = MP
_base = importlib.import_module("hexapod_env")

from sts_sensors import (  # noqa: E402
    I_STALL_A,
    SensorNoiseConfig,
    StsSensorBank,
    V_NOMINAL,
)
from stall_guard import StallConfig, StallGuard, stall_severity  # noqa: E402
from posture import (  # noqa: E402
    I_HARD_A,
    I_SOFT_A,
    I_TRUNK_HARD_A,
    I_TRUNK_SOFT_A,
    POSTURE_TORQUE_CAP_NM,
    blend_joints,
    posture_target_z,
    set_joint_qpos,
    sit_joints,
    stand_joints,
)

# Module-level default so train.py can flip posture training without
# forking fullsize_v1/train_walker.make_env.
DEFAULT_POSTURE_TASK_PROB = 0.40


class StsWalkerEnv(_base.HexapodWalkerEnv):
    """Residual-gait walker with STS sensor obs, sit/stand, stall stop."""

    MOTOR_OBS_DIM = 18 + 18 + 1 + 1 + 1 + 18  # = 57
    POSTURE_OBS_DIM = 3  # mode, posture_cmd, blend_alpha

    def __init__(
        self,
        *,
        sensor_noise: SensorNoiseConfig | None = None,
        stall_cfg: StallConfig | None = None,
        stall_penalty: float = 3.0,
        load_penalty_w: float = 0.15,
        current_penalty_w: float = 0.22,
        peak_current_w: float = 0.45,
        energy_w: float = 0.08,
        jam_event_prob: float = 0.04,
        jam_hold_seconds: float = 0.35,
        use_sts_joint_state: bool = True,
        terminate_on_stall: bool = True,
        posture_task_prob: float | None = None,
        # Slow blends keep stand/sit inside the soft current band; the
        # policy may slow further (never speed up) via rate modulation.
        posture_seconds: tuple = (4.0, 7.0),
        posture_residual_scale: float = 0.030,
        posture_current_w: float = 0.90,
        posture_peak_w: float = 2.20,
        posture_energy_w: float = 0.35,
        posture_height_w: float = 1.8,
        posture_pose_w: float = 0.8,
        posture_flip_prob: float = 0.45,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.sensor_noise = sensor_noise or SensorNoiseConfig(
            pos_std_rad=0.008,
            vel_std_rad_s=0.05,
            load_std_pct=3.0,
            current_std_a=0.04,
            volt_std_v=0.08,
            temp_std_c=0.5,
            dropout_prob=0.01,
        )
        self.sensors = StsSensorBank(self.sensor_noise)
        self.stall_cfg = stall_cfg or StallConfig()
        self.guard = StallGuard(self.stall_cfg)
        self.stall_penalty = float(stall_penalty)
        self.load_penalty_w = float(load_penalty_w)
        self.current_penalty_w = float(current_penalty_w)
        self.peak_current_w = float(peak_current_w)
        self.energy_w = float(energy_w)
        self.jam_event_prob = float(jam_event_prob)
        self.jam_hold_seconds = float(jam_hold_seconds)
        self.use_sts_joint_state = bool(use_sts_joint_state)
        self.terminate_on_stall = bool(terminate_on_stall)

        self.posture_task_prob = (
            DEFAULT_POSTURE_TASK_PROB
            if posture_task_prob is None
            else float(posture_task_prob)
        )
        self.posture_seconds = tuple(posture_seconds)
        self.posture_residual_scale = float(posture_residual_scale)
        self.posture_current_w = float(posture_current_w)
        self.posture_peak_w = float(posture_peak_w)
        self.posture_energy_w = float(posture_energy_w)
        self.posture_height_w = float(posture_height_w)
        self.posture_pose_w = float(posture_pose_w)
        self.posture_flip_prob = float(posture_flip_prob)

        # Expand observation space.
        self.OBS_DIM = self.OBS_DIM + self.MOTOR_OBS_DIM + self.POSTURE_OBS_DIM
        high = np.full(self.OBS_DIM, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high, dtype=np.float32)

        self._fb = np.zeros((18, 6), dtype=np.float64)
        self._jam_joint: int | None = None
        self._jam_steps_left = 0
        self._stall_terminated = False

        # Posture episode state.
        self._mode = "walk"            # "walk" | "posture"
        self._posture_cmd = 1.0        # 0 = sit, 1 = stand
        self._posture_start = stand_joints()
        self._posture_goal = stand_joints()
        self._posture_T = 5.0
        self._posture_alpha = 0.0
        self._posture_rate = 1.0          # 0.4..1.0 — policy may only slow
        self._posture_flip_step = 10**9
        self._energy_As = 0.0            # ∫ bus_current dt (amp-seconds)
        self._orig_forcerange = self.model.actuator_forcerange.copy()

        # Cache dof / position-actuator addresses for torque reads.
        # Each leg has 6 actuators (pos×3 + vel-damper×3); STS present-load
        # tracks the position servo effort only, not the sim dampers.
        self._joint_dof = self.idx.joint_qvel.copy()
        self._pos_actuator = np.array(
            [leg * 6 + axis for leg in range(6) for axis in range(3)],
            dtype=np.int32,
        )

    # ---- prototype-scale fall / stuck thresholds ------------------------

    def _check_done(self):
        z = float(self.data.body(self.idx.chassis_body).xpos[2])
        qw, qx, qy, qz = self.data.qpos[3:7]
        up_z = 1 - 2 * (qx * qx + qy * qy)
        # During sit the chassis is intentionally low; don't treat that as a fall.
        z_floor = 0.006 if self._mode == "posture" else 0.010
        fell = (z < z_floor) or (up_z < 0.5)
        if fell and self.terminate_on_fall:
            return True, False, True

        if self.terminate_on_stuck_seconds > 0.0 and self._mode == "walk":
            window = max(1, int(round(self.terminate_on_stuck_seconds * self.control_hz)))
            if self._step_count - self._stuck_check_step >= window:
                cur_xy = self.data.body(self.idx.chassis_body).xpos[:2]
                disp = float(((cur_xy - self._stuck_check_xy) ** 2).sum() ** 0.5)
                if disp < 0.015:
                    return False, True, False
                self._stuck_check_xy[:] = cur_xy
                self._stuck_check_step = self._step_count

        if self._step_count >= self._max_steps:
            return False, True, False
        return False, False, False

    # ---- torque cap helpers ---------------------------------------------

    def _set_posture_torque_cap(self, enabled: bool):
        """Clamp position-actuator forcerange during sit/stand episodes."""
        self.model.actuator_forcerange[:] = self._orig_forcerange
        if not enabled:
            return
        for aid in self._pos_actuator:
            self.model.actuator_forcerange[aid, 0] = -POSTURE_TORQUE_CAP_NM
            self.model.actuator_forcerange[aid, 1] = POSTURE_TORQUE_CAP_NM

    # ---- reset / step ---------------------------------------------------

    def reset(self, *, seed=None, options=None):
        opts = dict(options or {})
        force_mode = opts.pop("mode", None)  # "walk" | "posture" | None
        force_posture = opts.pop("posture", None)  # "sit" | "stand" | None

        obs, info = super().reset(seed=seed, options=opts or None)
        r = self.np_random

        # Re-roll sensor noise magnitudes lightly each episode.
        self.sensors.noise = SensorNoiseConfig(
            pos_std_rad=float(r.uniform(0.004, 0.014)),
            vel_std_rad_s=float(r.uniform(0.02, 0.10)),
            load_std_pct=float(r.uniform(1.5, 5.0)),
            current_std_a=float(r.uniform(0.02, 0.08)),
            volt_std_v=float(r.uniform(0.04, 0.15)),
            temp_std_c=float(r.uniform(0.3, 1.0)),
            load_bias_pct=float(r.uniform(-4.0, 4.0)),
            current_bias_a=float(r.uniform(-0.05, 0.05)),
            dropout_prob=float(r.uniform(0.0, 0.03)),
        )
        self.sensors.reset(rng=np.random.default_rng(int(r.integers(0, 2**31 - 1))))
        self.guard.reset()
        self._jam_joint = None
        self._jam_steps_left = 0
        self._stall_terminated = False
        self._posture_alpha = 0.0
        self._posture_rate = 1.0
        self._posture_flip_step = 10**9
        self._energy_As = 0.0

        # Choose episode mode.
        if force_mode is not None:
            self._mode = force_mode
        else:
            self._mode = (
                "posture" if r.random() < self.posture_task_prob else "walk"
            )

        if self._mode == "posture":
            self._setup_posture_episode(r, force_posture=force_posture)
            self._set_posture_torque_cap(True)
            # No walk command / no jam events during posture work.
            self._cmd[:] = 0.0
            self._gait.set_velocity(vx=0.0, vy=0.0, omega=0.0)
            self._jam_joint = None
        else:
            self._set_posture_torque_cap(False)
            self._posture_cmd = 1.0
            self._posture_start = stand_joints()
            self._posture_goal = stand_joints()
            if self.jam_event_prob > 0 and r.random() < self.jam_event_prob:
                delay = int(r.integers(int(0.8 * self.control_hz),
                                       max(int(0.8 * self.control_hz) + 1,
                                           self._max_steps // 2)))
                self._jam_joint = int(r.integers(0, 18))
                self._jam_steps_left = -delay

        self._refresh_sensors(dt=1.0 / self.control_hz)
        info = dict(info)
        info["mode"] = self._mode
        info["posture_cmd"] = float(self._posture_cmd)
        info["posture_alpha"] = float(self._posture_alpha)
        info["chassis_z"] = float(self.data.body(self.idx.chassis_body).xpos[2])
        info["chassis_xy"] = self.data.body(self.idx.chassis_body).xpos[:2].copy()
        info["stall"] = False
        return self._obs(), info

    def _setup_posture_episode(self, r, *, force_posture=None):
        """Spawn in sit or stand and command the opposite (or forced) posture."""
        sit = sit_joints()
        stand = stand_joints()
        if force_posture == "stand":
            start_sit = True
        elif force_posture == "sit":
            start_sit = False
        else:
            start_sit = bool(r.random() < 0.5)

        if start_sit:
            # Stand up from legs-out-wide.
            self._posture_start = sit
            self._posture_goal = stand
            self._posture_cmd = 1.0
            set_joint_qpos(self.model, self.data, sit, chassis_z=0.012)
        else:
            # Sit down from standing.
            self._posture_start = stand
            self._posture_goal = sit
            self._posture_cmd = 0.0
            set_joint_qpos(self.model, self.data, stand,
                           chassis_z=float(MP.stance_chassis_height()))

        lo, hi = self.posture_seconds
        self._posture_T = float(r.uniform(lo, hi))
        # Optionally flip sit↔stand mid-episode so one rollout practices both.
        if r.random() < self.posture_flip_prob:
            # Flip after the first transition + a short hold.
            flip_at = self._posture_T + float(r.uniform(0.6, 1.4))
            self._posture_flip_step = int(round(flip_at * self.control_hz))
        self._last_chassis_xy[:] = self.data.body(self.idx.chassis_body).xpos[:2]
        self._stuck_check_xy[:] = self._last_chassis_xy

    def _maybe_flip_posture(self):
        if self._mode != "posture":
            return
        if self._step_count < self._posture_flip_step:
            return
        # Reverse the command; re-anchor blend from the current pose.
        q = np.asarray(self.data.qpos[self.idx.joint_qpos], dtype=np.float64)
        self._posture_start = q.copy()
        if self._posture_cmd >= 0.5:
            self._posture_cmd = 0.0
            self._posture_goal = sit_joints()
        else:
            self._posture_cmd = 1.0
            self._posture_goal = stand_joints()
        self._t_offset = float(self.data.time)
        self._posture_alpha = 0.0
        # Only one flip per episode.
        self._posture_flip_step = 10**9

    def _refresh_sensors(self, dt: float):
        idx = self.idx
        q = np.asarray(self.data.qpos[idx.joint_qpos], dtype=np.float64)
        qd = np.asarray(self.data.qvel[idx.joint_qvel], dtype=np.float64)
        tau = np.asarray(
            self.data.actuator_force[self._pos_actuator], dtype=np.float64
        )
        if self._jam_active():
            j = self._jam_joint
            tau = tau.copy()
            tau[j] = math.copysign(MP.TORQUE_LIMIT * 0.98, tau[j] + 1e-9)
            qd = qd.copy()
            qd[j] = 0.0
        self._fb = self.sensors.sample(q, qd, tau, dt=dt)

    def _advance_jam_schedule(self):
        if self._jam_joint is None:
            return
        if self._jam_steps_left < 0:
            self._jam_steps_left += 1
            if self._jam_steps_left == 0:
                self._jam_steps_left = max(
                    1, int(round(self.jam_hold_seconds * self.control_hz))
                )

    def _jam_active(self) -> bool:
        return self._jam_joint is not None and self._jam_steps_left > 0

    def _consume_jam_frame(self):
        if not self._jam_active():
            return
        self._jam_steps_left -= 1
        if self._jam_steps_left <= 0:
            self._jam_joint = None

    def step(self, action):
        if self.guard.is_safe_stopped():
            return self._step_safe_stop()

        if self._mode == "posture":
            self._maybe_flip_posture()
            return self._step_posture(action)

        return self._step_walk(action)

    def _step_safe_stop(self):
        freeze = self.guard.freeze_targets_rad()
        for i in range(6):
            base = i * 6
            self.data.ctrl[base + 0] = freeze[3 * i + 0]
            self.data.ctrl[base + 1] = freeze[3 * i + 1]
            self.data.ctrl[base + 2] = freeze[3 * i + 2]
        for _ in range(self._steps_per_ctrl):
            mujoco.mj_step(self.model, self.data)
            self._refresh_sensors(self._sim_dt)
            self.guard.update(self._fb, self._sim_dt)
        self._step_count += 1
        self._stall_terminated = True
        terminated = bool(self.terminate_on_stall)
        truncated = not terminated
        obs = self._obs()
        reward = -self.stall_penalty
        info = self._base_info(fell=False, stall=True)
        return obs, float(reward), terminated, truncated, info

    def _step_walk(self, action):
        self._advance_jam_schedule()
        obs, reward, terminated, truncated, info = super().step(action)

        if self._jam_active():
            self.data.qvel[int(self._joint_dof[self._jam_joint])] = 0.0

        self._refresh_sensors(1.0 / self.control_hz)
        self.guard.update(self._fb, 1.0 / self.control_hz)
        self._consume_jam_frame()

        load_pen = float(np.mean((self._fb[:, 1] / 100.0) ** 2))
        cur = self._fb[:, 4]
        peak_i = float(cur.max())
        bus_i = float(cur.sum())
        cur_pen = float(np.mean((cur / I_STALL_A) ** 2))
        peak_pen = max(0.0, peak_i - I_SOFT_A) ** 2
        bus_pen = max(0.0, bus_i - I_TRUNK_SOFT_A) ** 2
        self._energy_As += bus_i * (1.0 / self.control_hz)
        reward = float(
            reward
            - self.load_penalty_w * load_pen
            - self.current_penalty_w * cur_pen
            - self.peak_current_w * peak_pen
            - 0.15 * bus_pen
            - self.energy_w * (bus_i / 18.0)   # mean amps — prefer thrifty gait
            - 0.25 * stall_severity(self._fb)
        )

        if self.guard.is_safe_stopped():
            self._stall_terminated = True
            reward -= self.stall_penalty
            if self.terminate_on_stall:
                terminated = True
            else:
                truncated = True
            info["stall"] = True
            info["stall_joints"] = self.guard.tripped_joints.copy()
            info["relax"] = self.guard.relax_requested
        else:
            info["stall"] = False
            # Walk hard ceiling is looser than posture (tripod push-off
            # legitimately spikes above the sit/stand soft band) but still
            # well under STS stall (2.7 A) / a 24 A pack budget.
            walk_motor_hard = 2.20
            walk_bus_hard = 20.0
            if (not self._jam_active()
                    and (peak_i >= walk_motor_hard or bus_i >= walk_bus_hard)):
                reward -= 2.0
                truncated = True

        info.update(self._motor_info())
        info["mode"] = "walk"
        info["peak_current_a"] = peak_i
        info["trunk_peak_a"] = bus_i
        info["energy_As"] = self._energy_As
        obs = self._obs()
        return obs, float(reward), terminated, truncated, info

    def _step_posture(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(self.ACT_DIM)
        action = np.clip(action, -1.0, 1.0)

        # LPF residual (joints only).
        ctrl_dt = 1.0 / self.control_hz
        joint_action = action[: self.BASE_ACT_DIM]
        if self.action_filter_tau > 0:
            alpha = ctrl_dt / (self.action_filter_tau + ctrl_dt)
            self._filtered_action[: self.BASE_ACT_DIM] += alpha * (
                joint_action - self._filtered_action[: self.BASE_ACT_DIM]
            )
        else:
            self._filtered_action[: self.BASE_ACT_DIM] = joint_action

        residual = (
            self._filtered_action[: self.BASE_ACT_DIM] * self.posture_residual_scale
        )

        # Rate modulation: first trailing action dim (gait-period slot when
        # gait_action=True) maps to rate ∈ [0.4, 1.0].  The policy can only
        # SLOW the blend — never slam faster than the nominal cosine.
        if self.ACT_DIM > self.BASE_ACT_DIM:
            raw_rate = float(action[self.BASE_ACT_DIM])
            target_rate = 0.4 + 0.6 * (0.5 * (raw_rate + 1.0))  # [-1,1]→[0.4,1]
            self._posture_rate += 0.15 * (target_rate - self._posture_rate)
        else:
            self._posture_rate = 1.0
        self._posture_rate = float(np.clip(self._posture_rate, 0.4, 1.0))

        # Advance blend clock by rate-scaled dt (slower rate → longer stand-up).
        self._posture_alpha = float(np.clip(
            self._posture_alpha + self._posture_rate * ctrl_dt / max(self._posture_T, 1e-3),
            0.0, 1.0,
        ))
        base = blend_joints(self._posture_start, self._posture_goal, self._posture_alpha)
        target = base + residual

        peak_i = 0.0
        trunk_i = 0.0
        energy_step = 0.0
        for _ in range(self._steps_per_ctrl):
            for i in range(6):
                b = i * 6
                self.data.ctrl[b + 0] = target[3 * i + 0]
                self.data.ctrl[b + 1] = target[3 * i + 1]
                self.data.ctrl[b + 2] = target[3 * i + 2]
            mujoco.mj_step(self.model, self.data)
            self._refresh_sensors(self._sim_dt)
            peak_i = max(peak_i, float(self._fb[:, 4].max()))
            bus = float(self._fb[:, 4].sum())
            trunk_i = max(trunk_i, bus)
            energy_step += bus * self._sim_dt
            if self._viewer is not None:
                self._viewer.sync()

        self._energy_As += energy_step
        self._step_count += 1
        self.guard.update(self._fb, ctrl_dt)

        terminated, truncated, fall_term = self._check_done()
        reward = self._posture_reward(action, fall_term, peak_i, trunk_i, energy_step)

        if peak_i >= I_HARD_A or trunk_i >= I_TRUNK_HARD_A:
            reward -= 2.5
            truncated = True
            terminated = False

        if self.guard.is_safe_stopped():
            self._stall_terminated = True
            reward -= self.stall_penalty
            if self.terminate_on_stall:
                terminated = True
            else:
                truncated = True

        self._last_action[:] = action
        info = self._base_info(fell=fall_term, stall=self.guard.is_safe_stopped())
        info.update(self._motor_info())
        info["mode"] = "posture"
        info["posture_cmd"] = float(self._posture_cmd)
        info["posture_alpha"] = float(self._posture_alpha)
        info["posture_rate"] = float(self._posture_rate)
        info["peak_current_a"] = peak_i
        info["trunk_peak_a"] = trunk_i
        info["energy_As"] = self._energy_As
        return self._obs(), float(reward), terminated, truncated, info

    def _posture_reward(self, action, fell, peak_i, trunk_i, energy_step) -> float:
        z = float(self.data.body(self.idx.chassis_body).xpos[2])
        z_tgt = posture_target_z(self._posture_cmd)
        z_err = abs(z - z_tgt)
        height_r = math.exp(-40.0 * z_err)

        q = np.asarray(self.data.qpos[self.idx.joint_qpos], dtype=np.float64)
        pose_err = float(np.mean((q - self._posture_goal) ** 2))
        pose_r = self._posture_alpha * math.exp(-8.0 * pose_err)

        qw, qx, qy, qz = self.data.qpos[3:7]
        up_z = 1 - 2 * (qx * qx + qy * qy)

        # Energy-aware shaping: punish mean current, peaks above soft, and
        # amp-seconds.  Staying under soft limits is strongly rewarded.
        cur = self._fb[:, 4]
        load = self._fb[:, 1] / 100.0
        soft = np.maximum(0.0, cur - I_SOFT_A)
        cur_pen = float(np.mean((cur / I_STALL_A) ** 2)) + 2.0 * float(np.mean(soft ** 2))
        trunk_pen = max(0.0, trunk_i - I_TRUNK_SOFT_A) ** 2
        peak_pen = max(0.0, peak_i - I_SOFT_A) ** 2
        load_pen = float(np.mean(load ** 2))
        # Prefer finishing under the soft band.
        under_soft = 0.25 if (peak_i < I_SOFT_A and trunk_i < I_TRUNK_SOFT_A) else 0.0

        residual_pen = float(np.mean(action[: self.BASE_ACT_DIM] ** 2))
        action_delta = float(np.mean((action - self._last_action) ** 2))
        fall_pen = self.fall_w if fell else 0.0

        done_bonus = 0.0
        if self._posture_alpha >= 1.0 and z_err < 0.008 and up_z > 0.9:
            # Extra bonus if the whole transition stayed gentle.
            done_bonus = 0.5 + (0.4 if under_soft else 0.0)

        return (
            self.posture_height_w * height_r
            + self.posture_pose_w * pose_r
            + 0.35 * up_z
            + done_bonus
            + under_soft
            - self.posture_current_w * cur_pen
            - self.posture_peak_w * peak_pen
            - 0.45 * trunk_pen
            - self.posture_energy_w * (energy_step * self.control_hz / 18.0)
            - self.load_penalty_w * load_pen
            - self.action_w * residual_pen
            - self.delta_w * action_delta
            - fall_pen
            + self.alive_w
        )

    def _base_info(self, *, fell, stall):
        return {
            "command": self._cmd.copy(),
            "chassis_xy": self.data.body(self.idx.chassis_body).xpos[:2].copy(),
            "chassis_z": float(self.data.body(self.idx.chassis_body).xpos[2]),
            "fell": fell,
            "stall": stall,
            "stall_joints": self.guard.tripped_joints.copy(),
            "relax": self.guard.relax_requested,
        }

    def _motor_info(self):
        return {
            "sts_feedback": self.sensors.as_dicts(self._fb),
            "bus_current_a": float(self._fb[:, 4].sum()),
            "bus_volt": float(self._fb[:, 2].mean()),
        }

    def _obs(self):
        idx = self.idx
        qpos = np.asarray(self.data.qpos[idx.joint_qpos], dtype=np.float32)
        qvel = np.asarray(self.data.qvel[idx.joint_qvel], dtype=np.float32)

        if self.use_sts_joint_state and self._fb is not None:
            qpos = np.radians(self._fb[:, 0]).astype(np.float32)
            qvel = np.radians(self._fb[:, 5]).astype(np.float32)

        quat = np.asarray(self.data.qpos[3:7], dtype=np.float32)
        cvel_world_lin = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        cvel_world_ang = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        v_body = _base._world_to_body(*quat, *cvel_world_lin)
        w_body = _base._world_to_body(*quat, *cvel_world_ang)
        contacts = np.asarray(
            [self.data.sensordata[a] for a in idx.foot_touch],
            dtype=np.float32,
        )
        cmd = self._cmd.astype(np.float32)
        phase = float(self._gait._phase)
        parts = [
            qpos, qvel, quat,
            v_body.astype(np.float32), w_body.astype(np.float32),
            contacts, cmd,
            np.array([math.sin(phase), math.cos(phase)], dtype=np.float32),
        ]
        if self.gait_action:
            parts.append(
                self._filtered_gait_scales[: self._n_gait].astype(np.float32)
            )
        if self.per_leg_lift:
            parts.append(self._gait.swing_mask().astype(np.float32))

        # --- STS motor channels ---
        load_n = (self._fb[:, 1] / 100.0).astype(np.float32)
        cur_n = (self._fb[:, 4] / I_STALL_A).astype(np.float32)
        volt_n = np.array([self._fb[:, 2].mean() / V_NOMINAL], dtype=np.float32)
        temp_n = np.array([self._fb[:, 3].mean() / 100.0], dtype=np.float32)
        sev = np.array([stall_severity(self._fb)], dtype=np.float32)
        accum = (self.guard._accum_s / max(self.stall_cfg.persist_s, 1e-6)).astype(
            np.float32
        )
        parts.extend([load_n, cur_n, volt_n, temp_n, sev, accum])

        # --- posture command ---
        parts.append(np.array([
            1.0 if self._mode == "posture" else 0.0,
            float(self._posture_cmd),
            float(self._posture_alpha),
        ], dtype=np.float32))
        return np.concatenate(parts)


# Alias expected by thin train/rollout wrappers that import HexapodWalkerEnv.
HexapodWalkerEnv = StsWalkerEnv


def make_env(**kwargs) -> StsWalkerEnv:
    return StsWalkerEnv(**kwargs)


def _smoke():
    print("=== Walk smoke ===")
    env = StsWalkerEnv(
        obstacle_count=4,
        randomize_command=False,
        terrain_seed=0,
        obstacle_seed=0,
        gait_action=True,
        per_leg_lift=True,
        stub_w=0.5,
        jam_event_prob=0.0,
        posture_task_prob=0.0,
    )
    print(f"observation_space = {env.observation_space.shape}")
    print(f"action_space      = {env.action_space.shape}")
    obs, info = env.reset(seed=0, options={"command": (0.10, 0.0, 0.0), "mode": "walk"})
    print(f"obs shape = {obs.shape}, mode={info['mode']}")
    total = 0.0
    for k in range(60):
        obs, reward, term, trunc, info = env.step(np.zeros(env.ACT_DIM, np.float32))
        total += reward
        if term or trunc:
            print(f"  ended at {k}: term={term} trunc={trunc} stall={info.get('stall')}")
            break
    print(f"60-step walk reward = {total:.3f}  bus≈{info.get('bus_current_a', float('nan')):.2f}A")
    env.close()

    for name, posture in (("stand-up", "stand"), ("sit-down", "sit")):
        print(f"\n=== Posture smoke: {name} (zero residual) ===")
        env = StsWalkerEnv(
            obstacle_count=0,
            terrain_enabled=False,
            randomize_command=False,
            jam_event_prob=0.0,
            posture_task_prob=1.0,
            posture_flip_prob=0.0,
            posture_seconds=(5.0, 5.0),
            episode_seconds=7.0,
            sensor_noise=SensorNoiseConfig(
                pos_std_rad=0.0, vel_std_rad_s=0.0, load_std_pct=0.0,
                current_std_a=0.0, volt_std_v=0.0, temp_std_c=0.0,
                dropout_prob=0.0,
            ),
        )
        obs, info = env.reset(seed=0, options={"mode": "posture", "posture": posture})
        env.sensors.noise = SensorNoiseConfig(
            pos_std_rad=0.0, vel_std_rad_s=0.0, load_std_pct=0.0,
            current_std_a=0.0, volt_std_v=0.0, temp_std_c=0.0,
            dropout_prob=0.0,
        )
        z0 = info["chassis_z"]
        peak = 0.0
        trunk_peak = 0.0
        total = 0.0
        for k in range(int(7.0 * env.control_hz)):
            obs, reward, term, trunc, info = env.step(np.zeros(env.ACT_DIM, np.float32))
            total += reward
            peak = max(peak, info.get("peak_current_a", 0.0))
            trunk_peak = max(trunk_peak, info.get("trunk_peak_a", 0.0))
            if term or trunc:
                break
        print(f"  z: {z0:.3f} → {info['chassis_z']:.3f}  "
              f"(target≈{posture_target_z(env._posture_cmd):.3f})")
        print(f"  peak servo {peak:.2f} A  trunk peak {trunk_peak:.2f} A  "
              f"(soft {I_SOFT_A}/{I_TRUNK_SOFT_A} A, hard {I_HARD_A}/{I_TRUNK_HARD_A} A)")
        print(f"  energy≈{info.get('energy_As', 0):.1f} A·s  "
              f"reward={total:.2f}  alpha={info.get('posture_alpha'):.2f}  "
              f"stall={info.get('stall')}")
        env.close()

    print("\n=== Forced jam / stall trip ===")
    env = StsWalkerEnv(
        obstacle_count=0,
        terrain_enabled=False,
        randomize_command=False,
        jam_event_prob=0.0,
        posture_task_prob=0.0,
        terminate_on_stall=True,
        sensor_noise=SensorNoiseConfig(
            pos_std_rad=0.0, vel_std_rad_s=0.0, load_std_pct=0.0,
            current_std_a=0.0, volt_std_v=0.0, temp_std_c=0.0,
            dropout_prob=0.0,
        ),
    )
    obs, _ = env.reset(seed=1, options={"command": (0.08, 0.0, 0.0), "mode": "walk"})
    env.sensors.noise = SensorNoiseConfig(
        pos_std_rad=0.0, vel_std_rad_s=0.0, load_std_pct=0.0,
        current_std_a=0.0, volt_std_v=0.0, temp_std_c=0.0,
        dropout_prob=0.0,
    )
    env._jam_joint = 2
    env._jam_steps_left = int(0.5 * env.control_hz)
    tripped = False
    for k in range(80):
        obs, reward, term, trunc, info = env.step(np.zeros(env.ACT_DIM, np.float32))
        if info.get("stall"):
            print(f"  stall at step {k}, joints={info['stall_joints'].nonzero()[0].tolist()}")
            tripped = True
            break
        if term or trunc:
            break
    print(f"stall tripped: {tripped}")
    env.close()


if __name__ == "__main__":
    _smoke()

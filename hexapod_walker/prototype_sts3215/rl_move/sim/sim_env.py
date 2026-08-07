"""SimHexapodBalanceEnv — MuJoCo twin of the hardware ``HexapodBalanceEnv``.

Same 46-dim observation, 5-dim body-offset action, reward terms, safety
layer and fixed-foot body IK as the real env — only the "robot" is a
MuJoCo model driven through the fitted ``ServoProfile`` (latency +
profile speed + deadband) so a policy trained here sees hardware-like
actuation, not ideal position control.

Usage
-----
    from rl_move.sim.sim_env import SimHexapodBalanceEnv
    env = SimHexapodBalanceEnv(randomize=True, seed=0)
    obs, info = env.reset()
    obs, r, term, trunc, info = env.step(env.action_space.sample())
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.body_ik import FixedFootBodyIK, fk_all_feet  # noqa: E402
from rl_move.config import cfg_get, load_config  # noqa: E402
from rl_move.env import build_obs, compute_reward  # noqa: E402
from rl_move.robot_state import (  # noqa: E402
    DEG2RAD, N_JOINTS, RAD2DEG, RobotState,
)
from rl_move.safety import SafetyLayer, action_to_body_offset  # noqa: E402

from .domain_rand import DomainRandomizer, EpisodeRandomization  # noqa: E402
from .servo_model import (  # noqa: E402
    ServoProfile, SimServoParams, apply_params_to_model, build_model,
    joint_qpos_addrs, joint_qvel_addrs, position_actuator_ids,
)

G0 = 9.80665
N_OBS = 46
N_ACT = 5

try:  # gymnasium is optional for pure scripted use
    import gymnasium as _gym
    _GymBase = _gym.Env
except Exception:  # pragma: no cover
    _gym = None
    _GymBase = object


def _default_plant_deg() -> np.ndarray:
    """Standing plant in hardware convention (learned plant or +20/+80)."""
    try:
        from feetech_bus import standing_pose_degrees
        return np.asarray(standing_pose_degrees(), dtype=float)
    except Exception:
        return np.array([0.0, 20.0, 80.0] * 6, dtype=float)


class SimHexapodBalanceEnv(_GymBase):
    """Gymnasium env; obs/action/reward identical to the hardware env."""

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, cfg: dict | None = None, *,
                 params: SimServoParams | None = None,
                 randomize: bool = False,
                 randomizer: DomainRandomizer | None = None,
                 plant_deg: np.ndarray | list[float] | None = None,
                 episode_seconds: float | None = None,
                 seed: int | None = None,
                 render_mode: str | None = None):
        import mujoco
        self._mujoco = mujoco
        self.cfg = cfg if cfg is not None else load_config()
        self.params = params if params is not None else SimServoParams.load()
        self.render_mode = render_mode
        self.rng = np.random.default_rng(seed)

        self.dt = 1.0 / float(cfg_get(self.cfg, "control", "hz", default=25))
        ep_s = (episode_seconds if episode_seconds is not None
                else float(cfg_get(self.cfg, "episode", "seconds", default=5)))
        self.episode_steps = int(round(ep_s / self.dt))
        self.write_speed_deg_s = (
            float(cfg_get(self.cfg, "bus", "write_speed", default=400))
            * 360.0 / 4096.0)
        self.write_acc_units = float(
            cfg_get(self.cfg, "bus", "write_acc", default=20))

        self.model = build_model(fixed_base=False, flat_terrain=True)
        self.data = mujoco.MjData(self.model)
        self._substeps = max(1, int(round(self.dt / self.model.opt.timestep)))
        self._qadr = joint_qpos_addrs(self.model)
        self._vadr = joint_qvel_addrs(self.model)
        self._pos_act = position_actuator_ids(self.model)
        self._chassis_bid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        gid = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "chassis_gyro")
        self._gyro_adr = self.model.sensor_adr[gid]

        # Soften the foot contacts: the CAD model's solref (0.01 s) is
        # near-rigid, so mm-scale randomized leg-length differences make
        # 2-3 "long" legs carry the whole robot (three-legged-stool) and
        # the stiff servos read 2-3 A standing still. Real rubber feet +
        # PLA leg flex compress ~1-2 mm and spread the load; ~3x softer
        # timeconst gives that. DR's contact_stiff_scale still varies it.
        for i in range(6):
            fid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, f"L{i}_foot")
            self.model.geom_solref[fid, 0] *= 3.0

        # Pristine copies for DR restore at every reset.
        self._base_body_mass = self.model.body_mass.copy()
        self._base_body_inertia = self.model.body_inertia.copy()
        self._base_body_ipos = self.model.body_ipos.copy()
        self._base_body_pos = self.model.body_pos.copy()
        self._base_geom_pos = self.model.geom_pos.copy()
        self._base_site_pos = self.model.site_pos.copy()
        self._base_geom_friction = self.model.geom_friction.copy()
        self._base_geom_solref = self.model.geom_solref.copy()
        self._base_gravity = self.model.opt.gravity.copy()

        if randomizer is not None:
            self.randomizer = randomizer
        elif randomize:
            self.randomizer = DomainRandomizer.from_params(self.params)
        else:
            self.randomizer = None
        self._ep_rand: EpisodeRandomization | None = None

        self._plant_deg = (np.asarray(plant_deg, dtype=float).reshape(N_JOINTS)
                           if plant_deg is not None else _default_plant_deg())

        self.ik = FixedFootBodyIK()
        self.safety = SafetyLayer(self.cfg)
        self._q_nom = np.zeros(N_JOINTS, dtype=float)
        self._prev_action = np.zeros(N_ACT, dtype=float)
        self._cmd = np.zeros(N_JOINTS, dtype=float)
        self._profile: ServoProfile | None = None
        self._step_i = 0
        self._episode = 0
        self._state: RobotState | None = None
        self._renderer = None
        self._goal_traj = None            # set by goal-conditioned subclass
        self._imu_prev_p: np.ndarray | None = None
        self._imu_prev_v: np.ndarray | None = None
        self._att_rp: np.ndarray | None = None

        if _gym is not None:
            self.observation_space = _gym.spaces.Box(
                -np.inf, np.inf, shape=(N_OBS,), dtype=np.float32)
            self.action_space = _gym.spaces.Box(
                -1.0, 1.0, shape=(N_ACT,), dtype=np.float32)

    # ------------------------------------------------------------------
    # state readout (sim → RobotState, with DR sensor corruption)
    # ------------------------------------------------------------------

    def _read_state(self) -> RobotState:
        mujoco = self._mujoco
        q = self.data.qpos[self._qadr].copy()
        qd = self.data.qvel[self._vadr].copy()

        # Attitude the way the hardware computes it: from the accelerometer
        # specific force f = a - g at the IMU's mounting point. Gravity may
        # be tilted (ground slope DR), the IMU frame rotated (mount
        # misalignment DR), and — because the IMU could be bolted anywhere
        # on the robot — the mounting point offset from the chassis center.
        # An off-center IMU picks up lever-arm acceleration whenever the
        # body rotates, corrupting the tilt estimate exactly during leans;
        # that corruption is the point of modeling it.
        er = self._ep_rand
        R = self.data.xmat[self._chassis_bid].reshape(3, 3).copy()
        mount = np.eye(3) if er is None else er.imu_mount_rot
        r_off = np.zeros(3) if er is None else er.imu_pos_m
        p_imu = self.data.xpos[self._chassis_bid] + R @ r_off
        a_imu = np.zeros(3)
        if self._imu_prev_p is not None:
            v_imu = (p_imu - self._imu_prev_p) / self.dt
            if self._imu_prev_v is not None:
                a_imu = (v_imu - self._imu_prev_v) / self.dt
            self._imu_prev_v = v_imu
        self._imu_prev_p = p_imu.copy()
        f_world = a_imu - np.asarray(self.model.opt.gravity, dtype=float)
        f_imu = (R @ mount).T @ f_world
        ax, ay, az = f_imu  # measures +g when level and static
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))
        gyro = self.data.sensordata[self._gyro_adr:self._gyro_adr + 3].copy()

        if er is not None:
            q = q + er.joint_zero_bias_rad
            q = q + self.rng.normal(0.0, er.encoder_noise_rad, N_JOINTS)
            roll_acc += (er.imu_bias_rad[0]
                         + self.rng.normal(0.0, er.tilt_noise_rad))
            pitch_acc += (er.imu_bias_rad[1]
                          + self.rng.normal(0.0, er.tilt_noise_rad))
            gyro = er.imu_mount_rot.T @ gyro  # gyro axes rotate with mount
            gyro = (gyro + er.gyro_bias_rad_s
                    + self.rng.normal(0.0, er.gyro_noise_rad_s, 3))

        # Complementary filter, same as the hardware estimator
        # (ComplementaryAttitude alpha=0.98): integrate gyro, drift-correct
        # slowly toward the accel tilt. Without it the raw accel tilt sees
        # the full lever-arm spikes of an off-center IMU (±20° for one
        # tick) — real firmware filters those out, so the sim must too.
        alpha = 0.98
        if self._att_rp is None:
            self._att_rp = np.array([roll_acc, pitch_acc])
        else:
            self._att_rp = np.array([
                alpha * (self._att_rp[0] + gyro[0] * self.dt)
                + (1.0 - alpha) * roll_acc,
                alpha * (self._att_rp[1] + gyro[1] * self.dt)
                + (1.0 - alpha) * pitch_acc])
        roll, pitch = float(self._att_rp[0]), float(self._att_rp[1])

        # Servo current ≈ |net actuator torque| × A/N·m. Feeds the effort
        # reward penalty AND the SafetyLayer over-current trip (2.5 A), so
        # stall-fighting a bad pose terminates in sim like it must on
        # hardware. 1.2 A/N·m puts torque saturation (2.2 N·m) just past
        # the trip — SUSTAINED saturation = episode over. Low-pass with a
        # ~0.1 s time constant so a millisecond torque spike doesn't trip:
        # hardware reads current at ~10 Hz and never sees such transients.
        torque = self.data.qfrc_actuator[self._vadr]
        raw_current = np.abs(torque) * 1.2
        if getattr(self, "_cur_filt", None) is None:
            self._cur_filt = raw_current
        else:
            alpha = self.dt / (self.dt + 0.1)
            self._cur_filt = (1.0 - alpha) * self._cur_filt + alpha * raw_current
        servo_current = self._cur_filt.copy()

        del mujoco
        return RobotState(
            timestamp=self.data.time,
            joint_position=q,
            joint_velocity=qd,
            imu_roll=float(roll),
            imu_pitch=float(pitch),
            imu_yaw=0.0,
            imu_gyro=gyro,
            imu_accel=f_imu,
            commanded_position=self._cmd.copy(),
            servo_current=servo_current,
            bus_ok=True,
            imu_ok=True,
            dt=self.dt,
        )

    # ------------------------------------------------------------------
    # physics
    # ------------------------------------------------------------------

    def _advance(self) -> None:
        assert self._profile is not None
        mujoco = self._mujoco
        for _ in range(self._substeps):
            target = self._profile.tick(self.model.opt.timestep)
            self.data.ctrl[self._pos_act] = target
            mujoco.mj_step(self.model, self.data)

    def _settle(self, seconds: float) -> None:
        n = int(round(seconds / self.dt))
        for _ in range(n):
            self._advance()

    def _start_pose_rad(self) -> np.ndarray:
        """Plant pose plus this episode's placement noise / bad-start
        offsets, clipped to the hardware joint limits."""
        from rl_move.safety import AXIS_LIMITS_DEG
        q = self._plant_deg * DEG2RAD
        if self._ep_rand is not None:
            q = q + self._ep_rand.start_offset_rad
        for j in range(N_JOINTS):
            lo, hi = AXIS_LIMITS_DEG[j % 3]
            q[j] = float(np.clip(q[j], lo * DEG2RAD, hi * DEG2RAD))
        return q

    def _place_at_plant(self, q_rad: np.ndarray) -> None:
        """Set qpos to ``q_rad`` with the chassis at foot-contact height."""
        import mujoco_prototype as MP
        mujoco = self._mujoco
        feet = fk_all_feet(q_rad)              # body frame, z at yaw plane
        foot_drop = float(np.min(feet[:, 2]))  # most negative = lowest foot
        base_z = MP.YAW_OUTPUT_HEIGHT - foot_drop + MP.FOOT_R + 0.002

        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = (0.0, 0.0, base_z)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qpos[self._qadr] = q_rad
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self._pos_act] = q_rad
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------
    # gym API
    # ------------------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        del options
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self._episode += 1
        self._step_i = 0
        self._prev_action[:] = 0.0
        self.safety.clear_estop()

        # Restore pristine model, then apply this episode's randomization.
        self.model.body_mass[:] = self._base_body_mass
        self.model.body_inertia[:] = self._base_body_inertia
        self.model.body_ipos[:] = self._base_body_ipos
        self.model.body_pos[:] = self._base_body_pos
        self.model.geom_pos[:] = self._base_geom_pos
        self.model.site_pos[:] = self._base_site_pos
        self.model.geom_friction[:] = self._base_geom_friction
        self.model.geom_solref[:] = self._base_geom_solref
        self.model.opt.gravity[:] = self._base_gravity
        if self.randomizer is not None:
            self._ep_rand = self.randomizer.sample(self.rng)
            self._ep_rand.apply_to_model(
                self.model, chassis_bid=self._chassis_bid)
            apply_params_to_model(
                self.model, self.params,
                kp_scale=self._ep_rand.kp_scale,
                kv_scale=self._ep_rand.kv_scale,
                torque_scale=self._ep_rand.torque_scale)
        else:
            self._ep_rand = None
            apply_params_to_model(self.model, self.params)

        q_start = self._start_pose_rad()
        self._place_at_plant(q_start)
        er = self._ep_rand
        self._profile = ServoProfile(
            self.params, q_start,
            latency_scale=1.0 if er is None else er.latency_scale,
            deadband_scale=1.0 if er is None else er.deadband_scale,
            vel_scale=1.0 if er is None else er.vel_scale,
        )
        self._cmd = q_start.copy()
        # Settle with slippery feet first: when a human sets the robot
        # down, feet micro-slip and the structure flexes until the stance
        # is compatible — otherwise randomized geometry + pinned contacts
        # leave the legs isometrically preloaded at 2-3 A from step 0.
        fr = self.model.geom_friction[:, 0].copy()
        self.model.geom_friction[:, 0] = 0.15
        self._settle(0.6)  # load the feet under gravity, let them slide
        self.model.geom_friction[:, 0] = fr

        # Hold-current semantics, same as the hardware env: nominal is the
        # pose the robot actually SETTLED at (however badly it was placed),
        # not the ideal plant. Feet are frozen wherever they really are.
        self._q_nom = self.data.qpos[self._qadr].copy()
        self._profile.reset(self._q_nom)
        self._cmd = self._q_nom.copy()
        # Re-anchoring the target to the settled pose zeroes the actuator
        # torque for an instant; with the stiff fitted kp the catch-up spike
        # would read as a fake over-current on step 1. Settle again so the
        # episode starts at the hold equilibrium, like hardware does.
        self._settle(0.3)
        self.ik.reset(self._q_nom)
        self.safety.set_nominal(self._q_nom)
        self._cur_filt = None
        self._imu_prev_p = None
        self._imu_prev_v = None
        self._att_rp = None
        self._goal_traj = self._sample_goal()
        self._state = self._read_state()
        self.safety.set_tilt_reference(self._state.imu_roll,
                                       self._state.imu_pitch)
        info = {
            "episode": self._episode,
            "q_nominal_deg": (self._q_nom * RAD2DEG).tolist(),
            "roll_deg": self._state.imu_roll * RAD2DEG,
            "pitch_deg": self._state.imu_pitch * RAD2DEG,
            "randomization": None if er is None else er.summary(),
        }
        goal = self._current_goal()
        if goal is not None:
            info["goal_mode"] = self._goal_traj.mode
        return build_obs(self.cfg, self._state, self._q_nom,
                         self._prev_action, goal=goal), info

    # Goal-conditioned subclass hooks (base env: no goal, 46-dim obs).
    def _sample_goal(self):
        return None

    def _current_goal(self):
        if self._goal_traj is None:
            return None
        return self._goal_traj.at(self._step_i)

    def step(self, action):
        assert self._state is not None and self._profile is not None
        clipped, bad = self.safety.validate_action(action)
        pen = float(cfg_get(self.cfg, "reward",
                            "safety_termination_penalty", default=10))
        if clipped is None:
            self._step_i += 1
            parts = {"reward_termination": -pen}
            return (build_obs(self.cfg, self._state, self._q_nom,
                              self._prev_action, goal=self._current_goal()),
                    -pen, True, False, {"termination_reason": bad, **parts})

        if self._ep_rand is not None and self._ep_rand.action_noise > 0:
            clipped = np.clip(
                clipped + self.rng.normal(0.0, self._ep_rand.action_noise,
                                          N_ACT), -1.0, 1.0)

        offset = action_to_body_offset(clipped, self.cfg)
        ik = self.ik.solve(offset)
        q_safe, status = self.safety.filter(
            ik.q_rad, self._state, ik_ok=ik.ok, ik_reason=ik.reason,
            action=clipped)

        terminated = bool(status.terminate)
        if not terminated:
            self._cmd = q_safe.copy()
            # DR: occasionally a SyncWrite is lost on the bus — the servos
            # keep chasing the previous goal for one tick.
            dropped = (self._ep_rand is not None
                       and self.rng.random() < self._ep_rand.cmd_drop_prob)
            if not dropped:
                self._profile.command(
                    q_safe, speed_deg_s=self.write_speed_deg_s,
                    acc_units=self.write_acc_units)
        self._advance()

        self._state = self._read_state()
        self._step_i += 1
        goal = self._current_goal()
        reward, parts = compute_reward(self.cfg, self._state, clipped,
                                       self._prev_action, goal=goal)
        if terminated:
            parts["reward_termination"] = -pen
            reward -= pen
        truncated = self._step_i >= self.episode_steps
        self._prev_action = clipped.copy()
        info = {"termination_reason": status.reason, **parts,
                "safety_ok": status.ok,
                "roll_deg": self._state.imu_roll * RAD2DEG,
                "pitch_deg": self._state.imu_pitch * RAD2DEG}
        if goal is not None:
            info["goal_mode"] = self._goal_traj.mode
            info["roll_ref_deg"] = goal.roll_ref * RAD2DEG
            info["pitch_ref_deg"] = goal.pitch_ref * RAD2DEG
            info["track_err_deg"] = math.hypot(
                self._state.imu_roll - goal.roll_ref,
                self._state.imu_pitch - goal.pitch_ref) * RAD2DEG
        return (build_obs(self.cfg, self._state, self._q_nom,
                          self._prev_action, goal=goal),
                float(reward), terminated, truncated, info)

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = self._mujoco.Renderer(self.model, 480, 640)
        self._renderer.update_scene(self.data, camera=-1)
        return self._renderer.render()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def make_env(**kwargs):
    """Factory for SB3 ``make_vec_env``."""
    def _thunk():
        return SimHexapodBalanceEnv(**kwargs)
    return _thunk


if __name__ == "__main__":
    env = SimHexapodBalanceEnv(randomize=True, seed=0)
    obs, info = env.reset()
    print(f"obs {obs.shape} roll={info['roll_deg']:+.2f}° "
          f"pitch={info['pitch_deg']:+.2f}°")
    t0 = time.monotonic()
    ret = 0.0
    for i in range(env.episode_steps):
        obs, r, term, trunc, info = env.step(np.zeros(N_ACT))
        ret += r
        if term or trunc:
            break
    print(f"zero-action episode: steps={i + 1} return={ret:.3f} "
          f"roll={info['roll_deg']:+.2f}° pitch={info['pitch_deg']:+.2f}° "
          f"({time.monotonic() - t0:.2f}s wall)")

"""Gymnasium environment wrapping the MuJoCo hexapod walker.

The environment is designed for **residual reinforcement learning** on top
of the existing alternating-tripod IK gait: at every control step the
underlying gait controller (mujoco_walker.TripodGait) computes the
nominal joint targets that would track a commanded body twist
(vx, vy, omega), and the policy outputs a small per-joint residual that
gets added to those nominal targets before they are written to the PD
actuators.

This works much better than learning the gait from scratch because:

* The IK gait already handles foot placement, swing/stance phasing,
  static balance and basic forward translation -- so the policy starts
  from a reasonable behaviour.
* The residual is bounded to ±0.20 rad, which means the policy can
  fine-tune timing, stride length, body lean and energy use without
  ever destabilising the walker.
* Reward shaping focuses on what the gait does badly: tracking the
  commanded twist accurately under terrain disturbances and obstacles.

Action space  : Box([-1, 1]^18)  (per-joint residual; scaled to ±0.20 rad)
                With ``gait_action=True`` the action becomes
                Box([-1, 1]^21): the trailing 3 dims modulate the
                gait-shape scales (period, lift, stride) within the
                configured ranges so the policy can dynamically tune the
                gait, not just the joint targets.
Observation   : Box(R^57) -- joint pos/vel + chassis pose / twist +
                foot contacts + commanded twist + gait phase
                (+3 dims of current gait scales when ``gait_action=True``)
Reward        : tracking - stability_penalty - effort_penalty + alive

The walker runs at 500 Hz inside MuJoCo; the policy is queried at
``control_hz`` (default 50 Hz), and the residual is held for the 10
intervening sim steps.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass

import numpy as np

import gymnasium as gym
from gymnasium import spaces
import mujoco

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)
import mujoco_walker as MW  # noqa: E402


@dataclass
class _Indices:
    chassis_body: int
    joint_qpos: np.ndarray   # (18,) qpos addresses for the 18 leg joints
    joint_qvel: np.ndarray   # (18,) qvel addresses
    joint_id:   np.ndarray   # (18,) raw joint ids
    foot_touch: np.ndarray   # (6,) touch sensor ids -> sensor_adr
    foot_touch_dim: int


def _build_indices(model: mujoco.MjModel) -> _Indices:
    chassis = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")
    qpos = []
    qvel = []
    jid  = []
    for i in range(6):
        for kind in ("yaw", "pitch", "knee"):
            j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                  f"L{i}_{kind}")
            jid.append(j)
            qpos.append(model.jnt_qposadr[j])
            qvel.append(model.jnt_dofadr[j])
    touches = []
    for i in range(6):
        s = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                              f"L{i}_foot_t")
        touches.append(model.sensor_adr[s])
    return _Indices(
        chassis_body=int(chassis),
        joint_qpos=np.asarray(qpos, dtype=np.int32),
        joint_qvel=np.asarray(qvel, dtype=np.int32),
        joint_id=np.asarray(jid, dtype=np.int32),
        foot_touch=np.asarray(touches, dtype=np.int32),
        foot_touch_dim=1,  # mujoco touch sensor is 1D
    )


def _quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
    return math.atan2(2 * (qw * qz + qx * qy),
                      1 - 2 * (qy * qy + qz * qz))


def _world_to_body(qw, qx, qy, qz, vx, vy, vz):
    """Rotate a world-frame vector into the body frame given the body's
    quaternion (w,x,y,z) -- expressed analytically without a full mjData."""
    # Inverse (conjugate) rotation: q^-1 v q.
    # Equivalent matrix form is easiest:
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qw*qz),     2*(qx*qz - qw*qy)],
        [2*(qx*qy - qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz + qw*qx)],
        [2*(qx*qz + qw*qy),     2*(qy*qz - qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)
    v_world = np.array([vx, vy, vz], dtype=np.float64)
    # body = R^T * world
    return R.T @ v_world


class HexapodWalkerEnv(gym.Env):
    """RL training environment for the hexapod walker (residual control)."""

    metadata = {"render_modes": ["human"], "render_fps": 50}

    BASE_OBS_DIM = 18 + 18 + 4 + 3 + 3 + 6 + 3 + 2  # = 57
    BASE_ACT_DIM = 18
    GAIT_PARAM_NAMES = ("period", "lift", "stride")

    # Class-level fallbacks; instance copies in __init__ may shrink these
    # when ``gait_action=False`` -- old eval scripts read env.ACT_DIM /
    # env.OBS_DIM directly so those names must keep working.
    OBS_DIM = BASE_OBS_DIM
    ACT_DIM = BASE_ACT_DIM

    def __init__(
        self,
        *,
        control_hz: int = 50,
        episode_seconds: float = 8.0,
        residual_scale: float = 0.06,           # rad -- small corrections only
        action_filter_tau: float = 0.08,        # 1st-order LPF on the residual,
                                                # in seconds (0 disables it)
        gait_period: float = 1.0,               # gait cycle in seconds
        gait_lift:   float = 0.07,
        # ---- parameterised-gait action (opt-in) ----
        gait_action: bool = False,              # adds 3 trailing action dims
                                                # for (period, lift, stride)
                                                # multipliers, plus 3 obs dims
        period_scale_range: tuple = (0.70, 1.30),
        lift_scale_range:   tuple = (0.60, 1.60),
        stride_scale_range: tuple = (0.50, 1.40),
        gait_action_filter_tau: float = 0.25,   # heavier LPF for gait params
                                                # so the agent can't oscillate
                                                # the cycle freq each step
        terrain_seed: int | None = None,        # None = randomise per reset
        obstacle_seed: int | None = None,
        obstacle_count: int = 8,
        terrain_enabled: bool = True,
        cmd_vx_range: tuple = (-0.4, 0.5),
        cmd_vy_range: tuple = (-0.3, 0.3),
        cmd_omega_range: tuple = (-0.2, 0.2),
        randomize_command: bool = True,
        cmd_speed_bias: float = 0.0,            # 0 = uniform; 1 = biased to ends
        terminate_on_fall: bool = True,
        # reward shaping (kept on the env so train + eval see the same thing)
        track_w_v: float = 1.5,
        track_w_w: float = 0.6,
        upright_w: float = 0.3,
        progress_w: float = 0.0,    # extra reward per metre of forward body motion
        action_w:  float = 0.40,    # penalty on |action|^2
        delta_w:   float = 1.50,    # penalty on |action - prev_action|^2 (smoothness)
        alive_w:   float = 0.02,
        fall_w:    float = 2.5,
        # ---- domain randomisation (all default to 0 = OFF for parity) ----
        dr_mass_pct: float = 0.0,         # chassis mass scale ±pct
        dr_friction_pct: float = 0.0,     # ground friction scale ±pct
        dr_motor_latency_ms: float = 0.0, # uniform delay on residual command
        dr_joint_bias_rad: float = 0.0,   # per-joint position offset added each step
        dr_action_noise: float = 0.0,     # gaussian std on the residual
        dr_velocity_kick: float = 0.0,    # m/s impulse magnitude at start of episode
        # ---- terrain curriculum ----
        terrain_level_max: float = 1.0,   # height-map intensity multiplier (0..1)
        terrain_level_min: float = 0.0,   # lower bound when curriculum_episodes>0
        curriculum_episodes: int = 0,     # episodes over which level ramps min->max
        render_mode: str | None = None,
    ):
        super().__init__()

        self.control_hz = int(control_hz)
        self.episode_seconds = float(episode_seconds)
        self.residual_scale = float(residual_scale)
        self.action_filter_tau = float(action_filter_tau)
        self.gait_period = float(gait_period)
        self.gait_lift = float(gait_lift)
        self.cmd_speed_bias = float(cmd_speed_bias)
        self.progress_w = float(progress_w)

        self.terrain_seed = terrain_seed
        self.obstacle_seed = obstacle_seed
        self.obstacle_count = int(obstacle_count)
        self.terrain_enabled = bool(terrain_enabled)

        self.cmd_vx_range = tuple(cmd_vx_range)
        self.cmd_vy_range = tuple(cmd_vy_range)
        self.cmd_omega_range = tuple(cmd_omega_range)
        self.randomize_command = bool(randomize_command)

        self.terminate_on_fall = bool(terminate_on_fall)
        self.render_mode = render_mode

        self.track_w_v = float(track_w_v)
        self.track_w_w = float(track_w_w)
        self.upright_w = float(upright_w)
        self.action_w  = float(action_w)
        self.delta_w   = float(delta_w)
        self.alive_w   = float(alive_w)
        self.fall_w    = float(fall_w)

        self.dr_mass_pct        = float(dr_mass_pct)
        self.dr_friction_pct    = float(dr_friction_pct)
        self.dr_motor_latency_ms = float(dr_motor_latency_ms)
        self.dr_joint_bias_rad  = float(dr_joint_bias_rad)
        self.dr_action_noise    = float(dr_action_noise)
        self.dr_velocity_kick   = float(dr_velocity_kick)
        self.terrain_level_max   = float(terrain_level_max)
        self.terrain_level_min   = float(terrain_level_min)
        self.curriculum_episodes = int(curriculum_episodes)
        self._episode_count      = 0

        # ---- parameterised gait setup -----------------------------------
        self.gait_action = bool(gait_action)
        self.period_scale_range = tuple(period_scale_range)
        self.lift_scale_range   = tuple(lift_scale_range)
        self.stride_scale_range = tuple(stride_scale_range)
        self.gait_action_filter_tau = float(gait_action_filter_tau)

        n_gait = len(self.GAIT_PARAM_NAMES) if self.gait_action else 0
        self.ACT_DIM = self.BASE_ACT_DIM + n_gait
        self.OBS_DIM = self.BASE_OBS_DIM + n_gait
        self._n_gait = n_gait

        # Build the world once; reset() will reuse the same model unless the
        # caller asks for a per-episode terrain reroll.  Per-step rebuild
        # would be wasteful (re-parsing XML each time).
        self._build_world()

        # Sim and control rate.
        self._sim_dt = float(self.model.opt.timestep)
        self._steps_per_ctrl = max(1, int(round(1.0 / (self.control_hz * self._sim_dt))))
        self._max_steps = int(round(self.episode_seconds * self.control_hz))

        # Action and observation spaces.
        self.action_space = spaces.Box(low=-1.0, high=1.0,
                                       shape=(self.ACT_DIM,),
                                       dtype=np.float32)
        # Loose bounds; SB3 handles +/-inf fine but bounded helps logging.
        high = np.full(self.OBS_DIM, np.inf, dtype=np.float32)
        self.observation_space = spaces.Box(low=-high, high=high,
                                            dtype=np.float32)

        self._gait = MW.TripodGait(period=self.gait_period,
                                    lift=self.gait_lift, ramp=0.4)
        self._t_offset = 0.0
        self._step_count = 0
        self._cmd = np.zeros(3, dtype=np.float32)
        self._last_action = np.zeros(self.ACT_DIM, dtype=np.float32)
        self._filtered_action = np.zeros(self.ACT_DIM, dtype=np.float32)
        # Filtered gait-scale outputs in their physical units, so the policy's
        # observation can include the actual scales currently applied to the
        # gait (rather than the raw [-1, 1] action that would still need
        # mapping to the configured ranges).
        self._filtered_gait_scales = np.ones(max(1, n_gait), dtype=np.float32)
        self._last_chassis_xy = np.zeros(2, dtype=np.float64)
        self._viewer = None

        # Domain-randomisation state (resampled every reset).
        self._joint_bias = np.zeros(self.BASE_ACT_DIM, dtype=np.float64)
        self._latency_buf: list = []
        self._latency_steps = 0

        # Snapshot the original mass / friction / hfield so we can restore +
        # rerandomise from a known baseline every reset.
        self._orig_body_mass = self.model.body_mass.copy()
        self._orig_geom_friction = self.model.geom_friction.copy()
        self._orig_hfield_data = self.heights.copy()

    # ---- gait-action helpers --------------------------------------------

    def _gait_range(self, name: str) -> tuple:
        if name == "period": return self.period_scale_range
        if name == "lift":   return self.lift_scale_range
        if name == "stride": return self.stride_scale_range
        raise KeyError(name)

    def _map_gait_action(self, raw: np.ndarray) -> np.ndarray:
        """Map a raw [-1, 1]^3 action vector to physical scale values."""
        out = np.empty(len(self.GAIT_PARAM_NAMES), dtype=np.float32)
        for k, name in enumerate(self.GAIT_PARAM_NAMES):
            lo, hi = self._gait_range(name)
            u = 0.5 * (float(raw[k]) + 1.0)
            out[k] = float(lo + (hi - lo) * np.clip(u, 0.0, 1.0))
        return out

    # ---- world setup -----------------------------------------------------

    def _build_world(self, terrain_seed: int | None = None,
                     obstacle_seed: int | None = None):
        ts = (self.terrain_seed if terrain_seed is None else terrain_seed)
        os_ = (self.obstacle_seed if obstacle_seed is None else obstacle_seed)
        if ts is None: ts = int(np.random.randint(0, 1_000_000))
        if os_ is None: os_ = int(np.random.randint(0, 1_000_000))

        self.model, self.data, self.heights = MW.build_world(
            terrain_seed=ts,
            terrain_enabled=self.terrain_enabled,
            obstacle_count=self.obstacle_count,
            obstacle_seed=os_,
        )
        self.idx = _build_indices(self.model)
        self._cur_terrain_seed = ts
        self._cur_obstacle_seed = os_

    # ---- gym API ---------------------------------------------------------

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)

        # If options ask us to re-roll the world, do so.  Otherwise we keep
        # the same terrain across episodes, which is far cheaper.
        opts = options or {}
        if opts.get("rebuild_world", False):
            self._build_world(terrain_seed=opts.get("terrain_seed"),
                              obstacle_seed=opts.get("obstacle_seed"))
            # Re-snapshot baselines after a fresh model is built.
            self._orig_body_mass = self.model.body_mass.copy()
            self._orig_geom_friction = self.model.geom_friction.copy()
            self._orig_hfield_data = self.heights.copy()

        # ---- Apply domain randomisation for this episode --------------------
        self._apply_domain_randomization()

        # Sample the commanded twist for this episode.
        if self.randomize_command:
            r = self.np_random
            def _bias_sample(lo, hi):
                u = r.uniform(0.0, 1.0)
                if self.cmd_speed_bias > 0.0:
                    # Pull samples toward the endpoints so the policy sees
                    # high-speed commands often.  Mix uniform with a U-shape.
                    u_edge = 0.5 + 0.5 * np.sign(u - 0.5) * (abs(2*u - 1) ** 0.4)
                    u = (1.0 - self.cmd_speed_bias) * u + self.cmd_speed_bias * u_edge
                return float(lo + (hi - lo) * u)
            self._cmd[0] = _bias_sample(*self.cmd_vx_range)
            self._cmd[1] = _bias_sample(*self.cmd_vy_range)
            self._cmd[2] = _bias_sample(*self.cmd_omega_range)
            if "command" in opts:
                self._cmd[:] = opts["command"]
        else:
            self._cmd[:] = opts.get("command", (0.3, 0.0, 0.0))

        # Reset the gait -- new TripodGait so the smoothing state resets.
        self._gait = MW.TripodGait(period=self.gait_period,
                                   lift=self.gait_lift, ramp=0.4,
                                   vx=float(self._cmd[0]),
                                   vy=float(self._cmd[1]),
                                   omega=float(self._cmd[2]))
        self._gait.reset_phase()
        # Initialise the filtered gait scales to their range midpoints so
        # the very first control step uses a neutral gait.
        if self.gait_action:
            for k, name in enumerate(self.GAIT_PARAM_NAMES):
                lo, hi = self._gait_range(name)
                self._filtered_gait_scales[k] = 0.5 * (lo + hi)
            self._gait.set_scales(
                period_scale=float(self._filtered_gait_scales[0]),
                lift_scale=float(self._filtered_gait_scales[1]),
                stride_scale=float(self._filtered_gait_scales[2]),
            )

        # Stand the walker on the spawn pad.
        MW._set_stance_qpos(self.model, self.data)
        # Add a small per-episode jitter to the spawn height so the policy
        # can't memorise a single starting condition.
        self.data.qpos[2] += self.np_random.uniform(-0.005, 0.005)
        mujoco.mj_forward(self.model, self.data)

        self._t_offset = float(self.data.time)
        self._step_count = 0
        self._last_action[:] = 0.0
        self._filtered_action[:] = 0.0
        self._latency_buf = []
        self._last_chassis_xy[:] = self.data.body(self.idx.chassis_body).xpos[:2]

        # Optional starting velocity kick for robustness (real robots wobble).
        if self.dr_velocity_kick > 0:
            kick = self.np_random.uniform(-1, 1, size=3) * self.dr_velocity_kick
            kick[2] = 0  # don't kick vertically (no chassis lift / fall)
            self.data.qvel[0:3] = kick

        return self._obs(), {}

    # ---- domain randomisation ------------------------------------------

    def _apply_domain_randomization(self):
        """Resample physics parameters and per-episode disturbances.

        Called from reset().  Restores from snapshots and re-rolls within
        the configured ranges.  All of this is gated on the corresponding
        ``dr_*`` knob being non-zero, so disabling DR fully restores
        deterministic behaviour.
        """
        r = self.np_random
        self._episode_count += 1

        # ---- chassis mass + inertia ----
        self.model.body_mass[:] = self._orig_body_mass
        if self.dr_mass_pct > 0:
            scale = float(r.uniform(1.0 - self.dr_mass_pct,
                                    1.0 + self.dr_mass_pct))
            cb = self.idx.chassis_body
            self.model.body_mass[cb] *= scale
            self.model.body_inertia[cb] *= scale  # treat as rigid scale

        # ---- ground / terrain friction ----
        self.model.geom_friction[:] = self._orig_geom_friction
        if self.dr_friction_pct > 0:
            f = float(r.uniform(1.0 - self.dr_friction_pct,
                                1.0 + self.dr_friction_pct))
            for gname in ("floor", "terrain"):
                gid = mujoco.mj_name2id(self.model,
                                        mujoco.mjtObj.mjOBJ_GEOM, gname)
                if gid >= 0:
                    self.model.geom_friction[gid, 0] *= f  # only sliding

        # ---- motor latency ----
        if self.dr_motor_latency_ms > 0:
            ms = float(r.uniform(0.0, self.dr_motor_latency_ms))
            self._latency_steps = int(round(ms * self.control_hz / 1000.0))
        else:
            self._latency_steps = 0

        # ---- per-joint position bias ----
        if self.dr_joint_bias_rad > 0:
            self._joint_bias = r.uniform(-self.dr_joint_bias_rad,
                                          self.dr_joint_bias_rad,
                                          size=self.BASE_ACT_DIM)
        else:
            self._joint_bias[:] = 0.0

        # ---- terrain heightmap intensity (curriculum) ----
        if self.terrain_enabled and (self.terrain_level_max < 1.0
                                     or self.terrain_level_min > 0.0
                                     or self.curriculum_episodes > 0):
            if self.curriculum_episodes > 0:
                progress = min(1.0, self._episode_count / self.curriculum_episodes)
                level_cap = (self.terrain_level_min
                             + (self.terrain_level_max - self.terrain_level_min)
                             * progress)
            else:
                level_cap = self.terrain_level_max
            level = float(r.uniform(self.terrain_level_min,
                                    max(self.terrain_level_min, level_cap)))
            scaled = (self._orig_hfield_data * level).astype(np.float32)
            # mujoco hfields take a flat row-major float32 array
            self.model.hfield_data[:] = scaled.reshape(-1)

    # ---- gym step ------------------------------------------------------

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(self.ACT_DIM)
        action = np.clip(action, -1.0, 1.0)

        # ---- split joint-residual dims from gait-shape dims --------------
        joint_action = action[: self.BASE_ACT_DIM]
        if self.gait_action:
            gait_action_raw = action[self.BASE_ACT_DIM:
                                     self.BASE_ACT_DIM + self._n_gait]
        else:
            gait_action_raw = None

        # First-order low-pass filter on the residual action.  This forces
        # the joint targets to evolve smoothly between control steps no
        # matter how jittery the policy output is.  We only filter / store
        # the leading 18 dims here; the gait-shape dims (if any) get a
        # separate, heavier filter below since their effect compounds over
        # the whole gait cycle.
        ctrl_dt = 1.0 / self.control_hz
        if self.action_filter_tau > 0:
            alpha = ctrl_dt / (self.action_filter_tau + ctrl_dt)
            self._filtered_action[: self.BASE_ACT_DIM] += alpha * (
                joint_action - self._filtered_action[: self.BASE_ACT_DIM]
            )
        else:
            self._filtered_action[: self.BASE_ACT_DIM] = joint_action

        # Apply the (optionally filtered) gait-shape multipliers to the gait.
        if self.gait_action:
            target_scales = self._map_gait_action(gait_action_raw)
            if self.gait_action_filter_tau > 0:
                a_g = ctrl_dt / (self.gait_action_filter_tau + ctrl_dt)
                self._filtered_gait_scales += a_g * (
                    target_scales - self._filtered_gait_scales
                )
            else:
                self._filtered_gait_scales[:] = target_scales
            self._gait.set_scales(
                period_scale=float(self._filtered_gait_scales[0]),
                lift_scale=float(self._filtered_gait_scales[1]),
                stride_scale=float(self._filtered_gait_scales[2]),
            )
            # Mirror the filtered raw action back into ``_filtered_action``
            # so observation / smoothness penalty see a consistent value.
            for k in range(self._n_gait):
                lo, hi = self._gait_range(self.GAIT_PARAM_NAMES[k])
                u = (self._filtered_gait_scales[k] - lo) / max(hi - lo, 1e-6)
                self._filtered_action[self.BASE_ACT_DIM + k] = (
                    np.clip(2.0 * u - 1.0, -1.0, 1.0)
                )

        residual = (self._filtered_action[: self.BASE_ACT_DIM]
                    * self.residual_scale)

        # Action noise: simulates motor servo error / unmodelled dynamics.
        if self.dr_action_noise > 0:
            residual = residual + self.np_random.normal(
                0.0, self.dr_action_noise * self.residual_scale,
                size=self.BASE_ACT_DIM
            )

        # Motor latency: residual command applied to the actuators is the
        # one we sent ``_latency_steps`` control-cycles ago.
        self._latency_buf.append(residual.copy())
        max_buf = max(1, self._latency_steps + 1)
        while len(self._latency_buf) > max_buf:
            self._latency_buf.pop(0)
        residual = self._latency_buf[0]

        # Per-joint position bias: simulates motor calibration error.
        residual = residual + self._joint_bias

        # Always re-pull base targets from the gait (it advances with time).
        for _ in range(self._steps_per_ctrl):
            t = float(self.data.time) - self._t_offset
            yaws, pitches, knees = self._gait.desired(max(0.0, t))
            base_targets = np.empty(self.BASE_ACT_DIM, dtype=np.float64)
            for i in range(6):
                base_targets[3 * i + 0] = yaws[i]
                base_targets[3 * i + 1] = pitches[i]
                base_targets[3 * i + 2] = knees[i]
            target = base_targets + residual

            # Write into ctrl in the same order build_xml uses (per-leg
            # interleaved: yaw_pos, pitch_pos, knee_pos, then 3 velocity
            # actuators with target zero -> joint dampers).
            for i in range(6):
                base = i * 6
                self.data.ctrl[base + 0] = target[3 * i + 0]
                self.data.ctrl[base + 1] = target[3 * i + 1]
                self.data.ctrl[base + 2] = target[3 * i + 2]
            mujoco.mj_step(self.model, self.data)

            if self._viewer is not None:
                self._viewer.sync()

        self._step_count += 1
        terminated, truncated, fall_term = self._check_done()

        obs = self._obs()
        new_chassis_xy = self.data.body(self.idx.chassis_body).xpos[:2].copy()
        # Progress reward = component of body displacement in the direction
        # we were COMMANDED to move (penalises sideways drift on a forward
        # command without re-doing all the tracking math).
        cmd_lin = np.array([self._cmd[0], self._cmd[1]], dtype=np.float64)
        n = np.linalg.norm(cmd_lin)
        delta_xy = new_chassis_xy - self._last_chassis_xy
        if n > 1e-3 and self.progress_w > 0:
            cmd_dir = cmd_lin / n
            progress = float(np.dot(delta_xy, cmd_dir))
        else:
            progress = 0.0
        reward = self._reward(action, terminated, fall_term, progress=progress)

        self._last_action[:] = action
        self._last_chassis_xy[:] = new_chassis_xy
        info = {
            "command": self._cmd.copy(),
            "chassis_xy": self.data.body(self.idx.chassis_body).xpos[:2].copy(),
            "chassis_z":  float(self.data.body(self.idx.chassis_body).xpos[2]),
            "fell": fall_term,
        }
        return obs, float(reward), terminated, truncated, info

    # ---- observation / reward -------------------------------------------

    def _obs(self):
        idx = self.idx
        qpos = np.asarray(self.data.qpos[idx.joint_qpos], dtype=np.float32)
        qvel = np.asarray(self.data.qvel[idx.joint_qvel], dtype=np.float32)
        # Chassis quaternion (qpos[3:7]) is the freejoint orientation.
        quat = np.asarray(self.data.qpos[3:7], dtype=np.float32)
        # Chassis linear + angular velocity in WORLD frame -> rotate to body.
        cvel_world_lin = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        cvel_world_ang = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        v_body = _world_to_body(*quat, *cvel_world_lin)
        w_body = _world_to_body(*quat, *cvel_world_ang)
        contacts = np.asarray(
            [self.data.sensordata[a] for a in idx.foot_touch],
            dtype=np.float32,
        )
        cmd = self._cmd.astype(np.float32)
        # gait phase (sin, cos) -- helps the policy know the swing/stance state.
        # We read the gait's stateful phase directly so the answer stays
        # correct when ``period_scale`` is being modulated mid-episode.
        phase = float(self._gait._phase)
        phase_sin = math.sin(phase)
        phase_cos = math.cos(phase)
        parts = [
            qpos, qvel, quat,
            v_body.astype(np.float32), w_body.astype(np.float32),
            contacts, cmd, np.array([phase_sin, phase_cos], dtype=np.float32),
        ]
        if self.gait_action:
            # Expose the currently-applied gait scales to the policy so it
            # can build a state-aware closed loop on top of its own choices.
            parts.append(self._filtered_gait_scales[:self._n_gait]
                         .astype(np.float32))
        return np.concatenate(parts)

    def _reward(self, action, terminated, fell, progress: float = 0.0):
        idx = self.idx
        # Realised body-frame velocity (positional + angular).
        quat = self.data.qpos[3:7]
        v_world = np.asarray(self.data.qvel[0:3], dtype=np.float64)
        w_world = np.asarray(self.data.qvel[3:6], dtype=np.float64)
        v_body = _world_to_body(*quat, *v_world)
        w_body = _world_to_body(*quat, *w_world)

        # Tracking errors.
        v_err = math.hypot(v_body[0] - self._cmd[0],
                           v_body[1] - self._cmd[1])
        w_err = abs(w_body[2] - self._cmd[2])

        # Tilt: we want the chassis upright.  Up-vector in world is the
        # third column of R; in body frame it should be (0, 0, 1).
        qw, qx, qy, qz = quat
        up_z = 1 - 2 * (qx * qx + qy * qy)
        tilt_pen = max(0.0, 1.0 - up_z)  # 0 when upright, ~1 if sideways

        # Energy-ish penalty: norm of residual + joint accelerations.
        residual_pen = float(np.mean(action * action))
        joint_acc = self.data.qacc[idx.joint_qvel]
        accel_pen = float(np.mean(joint_acc * joint_acc)) * 1e-4

        # Smoothness penalty (penalise jerky changes between consecutive
        # actions so the policy doesn't oscillate).
        action_delta = float(np.mean((action - self._last_action) ** 2))

        fall_pen = self.fall_w if fell else 0.0

        reward = (
            self.track_w_v * math.exp(-2.0 * v_err)
            + self.track_w_w * math.exp(-3.0 * w_err)
            + self.upright_w * up_z
            + self.progress_w * progress      # +1 per metre in commanded dir
            - 0.5 * tilt_pen
            - self.action_w * residual_pen
            - self.delta_w * action_delta
            - accel_pen
            - fall_pen
            + self.alive_w
        )
        return reward

    def _check_done(self):
        z = float(self.data.body(self.idx.chassis_body).xpos[2])
        qw, qx, qy, qz = self.data.qpos[3:7]
        up_z = 1 - 2 * (qx * qx + qy * qy)
        fell = (z < 0.05) or (up_z < 0.5)  # tilted >60° or chassis on ground
        if fell and self.terminate_on_fall:
            return True, False, True
        if self._step_count >= self._max_steps:
            return False, True, False
        return False, False, False

    # ---- viewer ---------------------------------------------------------

    def render(self):
        if self.render_mode != "human":
            return
        if self._viewer is None:
            import mujoco.viewer as viewer
            self._viewer = viewer.launch_passive(self.model, self.data)
        self._viewer.sync()

    def close(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    # ---- conveniences ---------------------------------------------------

    def set_command(self, vx=None, vy=None, omega=None):
        if vx is not None:    self._cmd[0] = float(vx)
        if vy is not None:    self._cmd[1] = float(vy)
        if omega is not None: self._cmd[2] = float(omega)
        self._gait.set_velocity(vx=self._cmd[0], vy=self._cmd[1],
                                omega=self._cmd[2])


def make_env(**kwargs) -> HexapodWalkerEnv:
    """Convenience factory matching the gym.make pattern."""
    return HexapodWalkerEnv(**kwargs)


# ---------------------------------------------------------------------------
# Smoke test (run directly with `./run.sh hexapod_walker/hexapod_env.py`).
# ---------------------------------------------------------------------------

def _smoke_test():
    print("=== Backward-compat (gait_action=False) ===")
    env = HexapodWalkerEnv(obstacle_count=4, randomize_command=True,
                           terrain_seed=0, obstacle_seed=0)
    print(f"observation_space = {env.observation_space.shape}")
    print(f"action_space      = {env.action_space.shape}")
    obs, info = env.reset(seed=0)
    print(f"obs shape = {obs.shape}, dtype = {obs.dtype}")
    total_reward = 0.0
    for k in range(40):
        action = np.zeros(env.ACT_DIM, dtype=np.float32)  # zero residual
        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward
        if term or trunc:
            print(f"  episode ended at step {k} (term={term}, trunc={trunc})")
            break
    print(f"40-step zero-action total reward = {total_reward:.3f}")
    print(f"final chassis pos = {info['chassis_xy']}, z={info['chassis_z']:.3f}")
    print(f"command was = {info['command']}")
    env.close()

    print("\n=== Parameterised gait (gait_action=True) ===")
    env = HexapodWalkerEnv(obstacle_count=4, randomize_command=True,
                           terrain_seed=0, obstacle_seed=0,
                           gait_action=True)
    print(f"observation_space = {env.observation_space.shape}")
    print(f"action_space      = {env.action_space.shape}")
    obs, info = env.reset(seed=0)
    print(f"obs shape = {obs.shape}")
    total_reward = 0.0
    for k in range(40):
        # Drive gait knobs to the upper end of their ranges so we can
        # confirm the scales propagate into the gait.
        action = np.zeros(env.ACT_DIM, dtype=np.float32)
        action[18:21] = +0.6   # bias toward longer/faster/higher gait
        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward
        if term or trunc:
            print(f"  episode ended at step {k} (term={term}, trunc={trunc})")
            break
    print(f"40-step total reward = {total_reward:.3f}")
    print(f"final chassis pos = {info['chassis_xy']}, z={info['chassis_z']:.3f}")
    print(f"final gait scales = {env._filtered_gait_scales}")
    env.close()


if __name__ == "__main__":
    _smoke_test()

"""Goal-conditioned lean / weight-shift task on the MuJoCo twin.

One policy, one command interface: each episode gets a goal —

- ``hold``:   zero references (plain balance; the old task is a subset)
- ``lean``:   a constant roll/pitch target, ramped in over ~0.75 s
- ``track``:  slowly moving roll/pitch references (sums of slow sines) —
              deliberately excites the IMU and forces the policy to drive
              the body smoothly through the real servo dynamics
              (150-200 ms latency, deadband, acceleration ramp)
- ``unload``: drive one leg's servo currents to zero by shifting the
              body weight off it (the atomic prerequisite of stepping)

The goal is appended to the observation (see ``TaskGoal.as_obs``), and
the shared ``compute_reward`` tracks the reference instead of zero.
References stay inside the body-IK action envelope (max_roll/pitch_deg)
so every goal is physically reachable with all six feet planted.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rl_move.config import cfg_get
from rl_move.env import GOAL_DIM, TaskGoal
from .sim_env import N_OBS, SimHexapodBalanceEnv

DEG2RAD = math.pi / 180.0

try:
    import gymnasium as _gym
except ImportError:
    _gym = None


@dataclass
class GoalTrajectory:
    """Per-step references for one episode."""
    mode: str
    roll: np.ndarray                 # (n_steps,) rad
    pitch: np.ndarray                # (n_steps,) rad
    unload_leg: int | None

    def at(self, step: int) -> TaskGoal:
        i = min(max(step, 0), len(self.roll) - 1)
        return TaskGoal(roll_ref=float(self.roll[i]),
                        pitch_ref=float(self.pitch[i]),
                        unload_leg=self.unload_leg)


class GoalGenerator:
    """Samples one episode's goal trajectory."""

    def __init__(self, cfg: dict):
        g = cfg.get("goal", {}) if isinstance(cfg, dict) else {}
        self.p_hold = float(g.get("p_hold", 0.15))
        self.p_lean = float(g.get("p_lean", 0.30))
        self.p_track = float(g.get("p_track", 0.35))
        self.p_unload = float(g.get("p_unload", 0.20))
        # References must be reachable through the body IK action limits.
        max_roll = float(cfg_get(cfg, "actions", "max_roll_deg", default=3.0))
        max_pitch = float(cfg_get(cfg, "actions", "max_pitch_deg", default=3.0))
        ref_cap = float(g.get("max_ref_deg", 2.5))
        self.max_roll = min(ref_cap, max_roll) * DEG2RAD
        self.max_pitch = min(ref_cap, max_pitch) * DEG2RAD
        period = g.get("track_period_s", [2.5, 8.0])
        self.period_s = (float(period[0]), float(period[1]))
        self.ramp_s = float(g.get("ramp_s", 0.75))

    def _ramp(self, n_steps: int, dt: float) -> np.ndarray:
        """Ease references in from 0 so episodes never start with a step
        change the servos (200 ms latency) could not possibly follow."""
        n_ramp = max(1, int(round(self.ramp_s / dt)))
        r = np.ones(n_steps)
        r[:n_ramp] = np.linspace(0.0, 1.0, n_ramp, endpoint=False)
        return r

    def _track_channel(self, rng: np.random.Generator, n_steps: int,
                       dt: float, amp_max: float) -> np.ndarray:
        t = np.arange(n_steps) * dt
        out = np.zeros(n_steps)
        for _ in range(int(rng.integers(1, 3))):
            amp = rng.uniform(0.3, 1.0) * amp_max
            period = rng.uniform(*self.period_s)
            phase = rng.uniform(0.0, 2.0 * math.pi)
            out += amp * np.sin(2.0 * math.pi * t / period + phase)
        # Sum of sines can exceed the reachable envelope: clip, then the
        # ramp (applied by caller) removes the initial discontinuity.
        return np.clip(out, -amp_max, amp_max)

    def sample(self, rng: np.random.Generator, n_steps: int,
               dt: float) -> GoalTrajectory:
        probs = np.array([self.p_hold, self.p_lean, self.p_track,
                          self.p_unload])
        mode = str(rng.choice(["hold", "lean", "track", "unload"],
                              p=probs / probs.sum()))
        roll = np.zeros(n_steps)
        pitch = np.zeros(n_steps)
        unload_leg: int | None = None
        ramp = self._ramp(n_steps, dt)
        if mode == "lean":
            roll = rng.uniform(-self.max_roll, self.max_roll) * ramp
            pitch = rng.uniform(-self.max_pitch, self.max_pitch) * ramp
        elif mode == "track":
            roll = self._track_channel(rng, n_steps, dt, self.max_roll) * ramp
            pitch = self._track_channel(rng, n_steps, dt,
                                        self.max_pitch) * ramp
        elif mode == "unload":
            unload_leg = int(rng.integers(0, 6))
        return GoalTrajectory(mode=mode, roll=roll, pitch=pitch,
                              unload_leg=unload_leg)


class SimHexapodGoalEnv(SimHexapodBalanceEnv):
    """Goal-conditioned twin: 54-dim obs (46 + 8 goal), same 5-dim action."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._goal_gen = GoalGenerator(self.cfg)
        if _gym is not None:
            self.observation_space = _gym.spaces.Box(
                -np.inf, np.inf, shape=(N_OBS + GOAL_DIM,), dtype=np.float32)

    def _sample_goal(self) -> GoalTrajectory:
        # +1: _current_goal is also read at step index == episode_steps.
        return self._goal_gen.sample(self.rng, self.episode_steps + 1,
                                     self.dt)


def make_goal_env(**kwargs):
    """Factory for SB3 ``make_vec_env``."""
    def _thunk():
        return SimHexapodGoalEnv(**kwargs)
    return _thunk

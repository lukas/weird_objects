"""MJX task shim that emits dynamics frames with every environment step.

This module deliberately has no torch or JAX imports.  Sharded MJX host
workers import the task class in separate processes, while accelerator
physics remains owned by :class:`MjxShardedVecEnv` in the parent process.
"""
from __future__ import annotations

import numpy as np

from rl_move.dynamics import frames as fr
from rl_move.sim.domain_rand import DomainRandomizer
from rl_move.sim.walk_task import SimHexapodJointWalkEnv


GOAL_PROFILES = {
    "walk": {"walk": 1.0},
    "stance": {
        "hold": 0.10, "lean": 0.15, "track": 0.15, "raise": 0.15,
        "rise": 0.30, "lower": 0.15, "unload": 0.0, "quad": 0.0,
        "walk": 0.0,
    },
    "mixed": {
        "hold": 0.10, "lean": 0.10, "track": 0.10, "raise": 0.10,
        "rise": 0.25, "lower": 0.10, "unload": 0.05, "quad": 0.0,
        "walk": 0.20,
    },
}


class DynrepCollectWalkEnv(SimHexapodJointWalkEnv):
    """Walk task with pool-safe frame/label capture for GPU collection."""

    MJX_SNAPSHOT_EXTRA = SimHexapodJointWalkEnv.MJX_SNAPSHOT_EXTRA + (
        "_dynrep_yaw0", "_dynrep_initial_frame", "_dynrep_initial_priv",
        "_dynrep_initial_mode", "_dynrep_initial_qnom",
    )

    def dynrep_configure(self, profile: str, dr_scale: float) -> None:
        """Set one row's goal distribution and DR scale before reset.

        ``MjxShardedVecEnv`` accepts uniform constructor kwargs, but each
        world still owns an independent host shim.  Replacing the shim's
        randomizer here gives the collector the original per-episode
        {0, .3, .6, 1.0} DR mixture without creating four GPU steppers.
        """
        if profile not in GOAL_PROFILES:
            raise ValueError(f"unknown dynrep goal profile {profile!r}")
        all_modes = (
            "hold", "lean", "track", "unload", "raise", "rise", "lower",
            "quad", "walk",
        )
        for mode in all_modes:
            attr = f"p_{mode}"
            if hasattr(self._goal_gen, attr):
                setattr(self._goal_gen, attr,
                        float(GOAL_PROFILES[profile].get(mode, 0.0)))

        self.randomizer = DomainRandomizer.from_params(
            self.params, scale=float(dr_scale))
        for key, value in (self.cfg.get("dr") or {}).items():
            if not hasattr(self.randomizer.ranges, key):
                raise ValueError(f"unknown DR override dr.{key}")
            if isinstance(value, str):
                parts = tuple(float(x) for x in value.split(","))
                value = parts[0] if len(parts) == 1 else parts
            setattr(self.randomizer.ranges, key, value)

    def _reset_finalize(self):
        obs, info = super()._reset_finalize()
        fr.reset_priv_episode(self)
        self._dynrep_initial_frame = fr.extract_frame(
            self, np.zeros(fr.ACTION_DIM, dtype=np.float32))
        self._dynrep_initial_priv = fr.extract_priv(self)
        self._dynrep_initial_mode = str(
            getattr(self._goal_traj, "mode", "?"))
        self._dynrep_initial_qnom = self._q_nom.astype(np.float32).copy()
        return obs, info

    def _post_step(self, result):
        obs, reward, term, trunc, info = super()._post_step(result)
        info = dict(info)
        info["dynrep_frame"] = fr.extract_frame(self, self._prev_action)
        info["dynrep_priv"] = fr.extract_priv(self)
        info["dynrep_action"] = self._prev_action.astype(np.float32).copy()
        return obs, reward, term, trunc, info

    def dynrep_initial(self):
        """Return the current pooled episode's first frame and metadata."""
        return (
            self._dynrep_initial_frame.copy(),
            self._dynrep_initial_priv.copy(),
            self._dynrep_initial_mode,
            self._dynrep_initial_qnom.copy(),
        )

    def dynrep_goal_traj(self):
        """Return the goal trajectory for scripted gait actors."""
        return self._goal_traj

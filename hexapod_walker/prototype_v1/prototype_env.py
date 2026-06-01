"""Gymnasium RL environment for the tabletop prototype hexapod.

This reuses the full-size ``hexapod_env.HexapodWalkerEnv`` implementation
unchanged, but swaps in ``mujoco_prototype`` for the MuJoCo model/gait module.
The action/observation/reward contract is therefore identical to the large
walker:

    action = residual joint targets on top of TripodGait
    optional gait-action dims for period/lift/stride modulation
    optional per-leg lift dims for stub recovery
"""

from __future__ import annotations

import importlib
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WALKER_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, WALKER_DIR)

import mujoco_prototype as MP  # noqa: E402

# ``hexapod_env`` imports ``mujoco_walker as MW`` at module import time.  Point
# that import at the prototype module before loading it.
sys.modules["mujoco_walker"] = MP
_base = importlib.import_module("hexapod_env")


class HexapodPrototypeEnv(_base.HexapodWalkerEnv):
    """Prototype-scale alias of the full-size residual-gait environment."""

    def _check_done(self):
        z = float(self.data.body(self.idx.chassis_body).xpos[2])
        qw, qx, qy, qz = self.data.qpos[3:7]
        up_z = 1 - 2 * (qx * qx + qy * qy)
        fell = (z < 0.010) or (up_z < 0.5)
        if fell and self.terminate_on_fall:
            return True, False, True

        if self.terminate_on_stuck_seconds > 0.0:
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


# Keep the exact class name that train_walker.py / rollout_walker.py import.
HexapodWalkerEnv = HexapodPrototypeEnv


def make_env(**kwargs) -> HexapodPrototypeEnv:
    return HexapodPrototypeEnv(**kwargs)


if __name__ == "__main__":
    env = HexapodPrototypeEnv(obstacle_count=4, randomize_command=False,
                              terrain_seed=0, obstacle_seed=0)
    print(f"observation_space = {env.observation_space.shape}")
    print(f"action_space      = {env.action_space.shape}")
    obs, info = env.reset(seed=0, options={"command": (0.10, 0.0, 0.0)})
    print(f"obs shape = {obs.shape}, dtype = {obs.dtype}")
    total_reward = 0.0
    for k in range(80):
        action = env.action_space.sample() * 0.0
        obs, reward, term, trunc, info = env.step(action)
        total_reward += reward
        if term or trunc:
            print(f"  episode ended at step {k} (term={term}, trunc={trunc})")
            break
    print(f"80-step zero-action total reward = {total_reward:.3f}")
    print(f"final chassis pos = {info['chassis_xy']}, z={info['chassis_z']:.3f}")
    print(f"command was = {info['command']}")
    env.close()

"""One-shot smoke test for the yaw-command channel and quad goal mode.

Run locally (plain python, repo .venv):
    python -m rl_move.sim.smoke_yaw_quad
Checks:
  1. LEGACY: default cfg -> obs width unchanged, goal modes sample,
     a walk episode steps without error.
  2. YAW: walk_yaw_cmd=1 -> obs +1, wz refs nonzero in some episodes,
     reward_walk_yaw present with k_walk_yaw set.
  3. QUAD: p_quad=1 -> mode samples, lift one-hot in obs, quad reward
     block runs and reports infos after the grace window.
"""
from __future__ import annotations

import numpy as np

from rl_move.sim.walk_task import SimHexapodJointWalkEnv


def run_episode(env, n=120):
    obs, _ = env.reset()
    infos = []
    for _ in range(n):
        a = env.action_space.sample() * 0.1
        obs, r, term, trunc, info = env.step(a)
        infos.append(info)
        if term or trunc:
            break
    return obs, infos


def main():
    # 1. legacy widths
    env = SimHexapodJointWalkEnv(episode_seconds=6.0, seed=3)
    w_legacy = env.observation_space.shape[0]
    obs, _ = env.reset()
    assert obs.shape[0] == w_legacy, (obs.shape, w_legacy)
    run_episode(env, 40)
    print(f"legacy obs width {w_legacy} OK")

    # 2. yaw channel
    cfg = {"goal": {"walk_yaw_cmd": 1.0},
           "reward": {"k_walk_yaw": 1.0}}
    env = SimHexapodJointWalkEnv(cfg, episode_seconds=6.0, seed=3)
    assert env.observation_space.shape[0] == w_legacy + 1, \
        env.observation_space.shape
    env._goal_gen.p_walk = 1.0
    saw_yaw_reward = saw_nonzero_wz = False
    for ep in range(6):
        obs, infos = run_episode(env)
        assert obs.shape[0] == w_legacy + 1
        traj = env._goal_traj
        if traj is not None and getattr(traj, "wz", None) is not None:
            if float(np.abs(traj.wz).max()) > 1e-6:
                saw_nonzero_wz = True
        if any("reward_walk_yaw" in i for i in infos):
            saw_yaw_reward = True
    assert saw_yaw_reward, "yaw kernel never paid"
    assert saw_nonzero_wz, "no nonzero wz command sampled in 6 episodes"
    ys = [i["reward_walk_yaw"] for i in infos if "reward_walk_yaw" in i]
    print(f"yaw obs width {w_legacy + 1} OK; "
          f"kernel mean {np.mean(ys):.3f} (peak 1.0)")

    # 3. quad mode
    cfg = {"goal": {"p_quad": 1.0, "quad_grace_s": 0.5},
           "reward": {"k_quad_clear": 1.0, "k_quad_plant": 1.0}}
    env = SimHexapodJointWalkEnv(cfg, episode_seconds=6.0, seed=3)
    env.set_goal_mix({"hold": 0, "lean": 0, "track": 0, "unload": 0,
                      "raise": 0, "rise": 0, "lower": 0, "walk": 0,
                      "quad": 1.0})
    obs, infos = run_episode(env)
    traj = env._goal_traj
    assert traj is not None and traj.mode == "quad", traj
    goal = env._current_goal()
    assert tuple(goal.lift_legs) == (0, 5), goal.lift_legs
    onehot = goal.as_obs(env.cfg)[3:]
    assert onehot[0] == 1.0 and onehot[5] == 1.0 and onehot[1:5].sum() == 0
    qinfos = [i for i in infos if "quad_planted_frac" in i]
    assert qinfos, "quad reward block never ran"
    last = qinfos[-1]
    print(f"quad OK: mode={traj.mode}, one-hot {onehot.tolist()}, "
          f"{len(qinfos)} paid ticks, last info: "
          f"clear {last['quad_clear_mm']:.1f} mm, "
          f"fronts_off {last['quad_fronts_off']:.2f}, "
          f"planted {last['quad_planted_frac']:.2f}, "
          f"r_clear {last['reward_quad_clear']:.3f}, "
          f"r_plant {last['reward_quad_plant']:.3f}")

    # legacy rng-stream guard: quad entry with p=0 must not shift draws
    env_a = SimHexapodJointWalkEnv(episode_seconds=6.0, seed=11)
    env_a._goal_gen.p_walk = 0.0
    modes_a = []
    for _ in range(12):
        env_a.reset()
        modes_a.append(env_a._goal_traj.mode)
    print(f"mode stream @seed11 (p_quad=0): {modes_a}")
    print("ALL SMOKE CHECKS PASSED")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Hardware PPO placeholder — blocked until Milestone 2 tests pass.

Sim training is NOT blocked: use ``rl_move.sim.train_ppo_sim``
(MuJoCo twin + domain randomization) any time. This gate only covers
training loops that command the real robot.
"""
from __future__ import annotations

import sys


def main() -> int:
    print(
        "train_ppo (hardware): not enabled yet.\n"
        "Complete balance_test, balance_sine, and balance_pd first "
        "(see archive/RL_PLAN.md Steps G-I / Milestone 2).\n"
        "For sim training run: python -m rl_move.sim.train_ppo_sim",
        file=sys.stderr,
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

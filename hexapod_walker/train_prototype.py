"""Train a residual gait policy for the tabletop prototype hexapod.

Wrapper around ``train_walker.py`` that swaps in the prototype MuJoCo model
and changes the default gait/command scales to match the small hobby-servo
robot.

Examples
--------

    # quick smoke train
    ./.venv/bin/python hexapod_walker/train_prototype.py --steps 5000 --n-envs 1

    # baseline run
    ./.venv/bin/python hexapod_walker/train_prototype.py --steps 250000 --n-envs 8

    # terrain/stub-learning run like the large walker
    ./.venv/bin/python hexapod_walker/train_prototype.py \
        --steps 1000000 --n-envs 8 --gait-action --per-leg-lift --stub-w 0.5
"""

from __future__ import annotations

import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

import mujoco_prototype as MP  # noqa: E402
import prototype_env as PE  # noqa: E402

sys.modules["mujoco_walker"] = MP
sys.modules["hexapod_env"] = PE


def _append_default(flag: str, value: str):
    if flag not in sys.argv:
        sys.argv.extend([flag, value])


def _append_bool_default(flag: str):
    if flag not in sys.argv:
        sys.argv.append(flag)


def main():
    _append_default("--tag", "prototype_ppo")
    _append_default("--out-dir", os.path.join(THIS_DIR, "policies_prototype"))
    _append_default("--gait-period", "0.65")
    _append_default("--residual-scale", "0.08")
    _append_default("--vx-max", "0.18")
    _append_default("--vy-max", "0.12")
    _append_default("--omega-max", "0.60")
    _append_default("--obstacle-count", "8")
    _append_default("--progress-w", "30.0")
    _append_default("--net-arch", "128,128")
    # Use the same richer action surface as the larger walker unless the user
    # explicitly removes it by editing this wrapper or calling train_walker.py.
    _append_bool_default("--gait-action")
    _append_bool_default("--per-leg-lift")
    _append_default("--stub-w", "0.50")
    _append_default("--period-scale-range", "0.7,1.4")
    _append_default("--lift-scale-range", "0.6,2.2")
    _append_default("--stride-scale-range", "0.5,1.5")

    import train_walker  # noqa: E402

    train_walker.main()


if __name__ == "__main__":
    main()

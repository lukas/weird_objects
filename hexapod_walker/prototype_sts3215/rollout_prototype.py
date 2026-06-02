"""Roll out a trained prototype policy in MuJoCo.

Uses the exact same rollout code as the large walker, with the module imports
aliased to the tabletop prototype model/env.

Examples
--------

    ./.venv/bin/mjpython hexapod_walker/prototype/rollout_prototype.py \
        --policy hexapod_walker/prototype/policies_prototype/prototype_ppo/prototype_ppo.zip

    ./.venv/bin/python hexapod_walker/prototype/rollout_prototype.py \
        --policy hexapod_walker/prototype/policies_prototype/prototype_ppo/prototype_ppo.zip \
        --headless --episodes 3
"""

from __future__ import annotations

import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
WALKER_DIR = os.path.dirname(THIS_DIR)
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, WALKER_DIR)

import mujoco_prototype as MP  # noqa: E402
import prototype_env as PE  # noqa: E402

sys.modules["mujoco_walker"] = MP
sys.modules["hexapod_env"] = PE


def _append_default(flag: str, value: str):
    if flag not in sys.argv:
        sys.argv.extend([flag, value])


def main():
    _append_default("--vx", "0.10")
    _append_default("--duration", "12.0")
    _append_default("--obstacles", "8")
    import rollout_walker  # noqa: E402

    rollout_walker.main()


if __name__ == "__main__":
    main()

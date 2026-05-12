"""Evaluate a trained prototype policy headlessly.

This is a small aliasing wrapper around ``eval_walker.py`` so the same
evaluation script runs against ``mujoco_prototype`` / ``prototype_env``.
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


def main():
    import eval_walker  # noqa: E402

    eval_walker.main()


if __name__ == "__main__":
    main()

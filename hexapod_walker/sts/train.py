"""Train an STS-aware residual walking + sit/stand policy (PPO).

Wraps ``fullsize_v1/train_walker.py`` but swaps in ``sts_env`` so the
policy sees STS3215 load/current/voltage channels, is penalised for
near-stall behaviour, and spends a fraction of episodes practising
gentle sit ↔ stand transitions from a legs-out-wide sprawl.

Examples
--------

    # smoke (~1 min on CPU)
    ./.venv/bin/python hexapod_walker/sts/train.py --steps 5000 --n-envs 1

    # baseline (walk + sit/stand mix)
    ./.venv/bin/python hexapod_walker/sts/train.py --steps 300000 --n-envs 8

    # walk only
    ./.venv/bin/python hexapod_walker/sts/train.py --steps 300000 --no-posture
"""

from __future__ import annotations

import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
PROTO_DIR = os.path.join(os.path.dirname(THIS_DIR), "prototype_sts3215")
FULLSIZE_DIR = os.path.join(os.path.dirname(THIS_DIR), "fullsize_v1")
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, PROTO_DIR)
sys.path.insert(0, FULLSIZE_DIR)

import mujoco_prototype as MP  # noqa: E402
import sts_env as SE  # noqa: E402

MP.TORQUE_LIMIT = 2.70
sys.modules["mujoco_walker"] = MP
sys.modules["hexapod_env"] = SE


def _append_default(flag: str, value: str):
    if flag not in sys.argv:
        sys.argv.extend([flag, value])


def _append_bool_default(flag: str):
    if flag not in sys.argv:
        sys.argv.append(flag)


def _consume_flag(flag: str) -> bool:
    if flag in sys.argv:
        sys.argv.remove(flag)
        return True
    return False


def main():
    if _consume_flag("--no-posture"):
        SE.DEFAULT_POSTURE_TASK_PROB = 0.0
    elif _consume_flag("--posture-only"):
        SE.DEFAULT_POSTURE_TASK_PROB = 1.0
    else:
        # Mix walk + sit/stand by default.
        SE.DEFAULT_POSTURE_TASK_PROB = 0.40

    _append_default("--tag", "sts_ppo")
    _append_default("--out-dir", os.path.join(THIS_DIR, "policies"))
    _append_default("--gait-period", "0.82")
    _append_default("--residual-scale", "0.055")
    _append_default("--action-filter-tau", "0.14")
    _append_default("--gait-action-filter-tau", "0.38")
    _append_default("--action-w", "0.72")
    _append_default("--delta-w", "2.85")
    _append_default("--progress-w", "12.0")
    _append_default("--cmd-speed-bias", "0.12")
    _append_default("--log-std-init", "-2.0")
    _append_default("--vx-max", "0.13")
    _append_default("--vy-max", "0.09")
    _append_default("--omega-max", "0.38")
    _append_default("--obstacle-count", "8")
    _append_default("--net-arch", "128,128,64")
    _append_default("--episode-seconds", "10.0")
    _append_bool_default("--gait-action")
    _append_bool_default("--per-leg-lift")
    _append_default("--stub-w", "0.50")
    _append_default("--period-scale-range", "0.88,1.38")
    _append_default("--lift-scale-range", "0.65,1.75")
    _append_default("--stride-scale-range", "0.52,1.18")
    # Sensor / motor domain randomisation defaults for sim→real.
    _append_default("--dr-mass-pct", "0.15")
    _append_default("--dr-friction-pct", "0.20")
    _append_default("--dr-motor-latency-ms", "12")
    _append_default("--dr-joint-bias-rad", "0.03")
    _append_default("--dr-action-noise", "0.04")

    import train_walker  # noqa: E402

    print(f"posture_task_prob = {SE.DEFAULT_POSTURE_TASK_PROB}")
    train_walker.main()


if __name__ == "__main__":
    main()

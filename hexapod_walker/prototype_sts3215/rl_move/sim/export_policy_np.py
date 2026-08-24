"""Export an SB3 PPO MlpPolicy to a plain-JSON numpy MLP for the robot.

The Uno Q has no torch. The deterministic actor is just
tanh(W1 x + b1) -> tanh(W2 h + b2) -> Wout h + bout, clipped to [-1, 1],
so we dump the five matrices as JSON lists plus metadata (dims, the
action->joint-angle mapping constants, source checkpoint) and verify
bit-for-bit-ish parity against model.predict before writing.

Usage:
    uv run python -m rl_move.sim.export_policy_np \
        --policy rl_move/sim/policies/ppo_goal_cw_lower.zip \
        --out linux_control/rl_policy_weights.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from rl_move.joint_frame import FRAME_ROBOT_ABS, normalize_joint_frame
from rl_move.config import cfg_get, load_config


def export(policy_path: str, out_path: str, *, name: str = "",
           notes: str = "", joint_frame: str = FRAME_ROBOT_ABS,
           extra_meta: dict | None = None,
           control_hz: float | None = None) -> None:
    from stable_baselines3 import PPO
    import torch

    model = PPO.load(policy_path, device="cpu")
    pol = model.policy
    assert type(pol.mlp_extractor.policy_net[1]).__name__ == "Tanh", \
        "expected tanh activations (SB3 MlpPolicy default)"

    def t2l(t: torch.Tensor) -> list:
        return t.detach().cpu().numpy().astype(np.float64).tolist()

    net = pol.mlp_extractor.policy_net
    meta = {
        "source": str(policy_path),
        # Optional display fields for the robot's policy picker
        # (linux_control/policies/ + /api/rl/policies).
        **({"name": name} if name else {}),
        **({"notes": notes} if notes else {}),
        "obs_dim": int(pol.observation_space.shape[0]),
        "act_dim": int(pol.action_space.shape[0]),
        "hidden": [int(net[0].out_features), int(net[2].out_features)],
        "activation": "tanh",
        # New/default policies speak the real robot's logical
        # absolute-tibia joint frame. Old MuJoCo-native checkpoints must
        # be exported with --joint-frame model_rel.
        "joint_frame": normalize_joint_frame(joint_frame),
        # Free-form extras (e.g. the trained goal "profile" —
        # hold/ramp/target shapes — which linux_control/rl_policy.py
        # reads so runner constants can never drift from the config
        # a checkpoint was actually trained with).
        **(extra_meta or {}),
    }
    meta["joint_frame"] = normalize_joint_frame(meta.get("joint_frame"))
    # Trained control cadence (2026-08-24 100 Hz flip,
    # fb_20260824T174619_c49b7e): the robot runner
    # (linux_control/rl_policy.py) REFUSES a policy whose control_hz
    # doesn't match its own config-derived rate, so every export must
    # carry it. Default = the CURRENT config's control.hz (what a
    # just-trained checkpoint was trained at); pass --control-hz for
    # legacy exports (e.g. 25 for pre-flip checkpoints). An explicit
    # extra_meta control_hz wins over both.
    if "control_hz" not in meta:
        if control_hz is None:
            from rl_move.config import cfg_get, load_config
            control_hz = float(cfg_get(load_config(), "control", "hz",
                                       default=25.0))
        meta["control_hz"] = float(control_hz)
    payload = {
        "meta": {
            **meta,
        },
        "W1": t2l(net[0].weight), "b1": t2l(net[0].bias),
        "W2": t2l(net[2].weight), "b2": t2l(net[2].bias),
        "Wout": t2l(pol.action_net.weight), "bout": t2l(pol.action_net.bias),
    }

    # Parity: numpy forward vs model.predict(deterministic=True).
    W1 = np.array(payload["W1"]); b1 = np.array(payload["b1"])
    W2 = np.array(payload["W2"]); b2 = np.array(payload["b2"])
    Wo = np.array(payload["Wout"]); bo = np.array(payload["bout"])
    rng = np.random.default_rng(0)
    worst = 0.0
    for _ in range(200):
        obs = rng.normal(0, 1, payload["meta"]["obs_dim"]).astype(np.float32)
        h = np.tanh(W1 @ obs + b1)
        h = np.tanh(W2 @ h + b2)
        a_np = np.clip(Wo @ h + bo, -1.0, 1.0)
        a_sb3, _ = model.predict(obs, deterministic=True)
        worst = max(worst, float(np.max(np.abs(a_np - a_sb3))))
    print(f"parity: max |a_np - a_sb3| over 200 random obs = {worst:.2e}")
    assert worst < 1e-5, "numpy forward does not match SB3 predict"

    out = Path(out_path)
    out.write_text(json.dumps(payload))
    print(f"wrote {out} ({out.stat().st_size / 1024:.0f} KiB)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="",
                    help="display name for the robot's policy picker")
    ap.add_argument("--notes", default="",
                    help="one-line operator notes shown in the picker")
    ap.add_argument("--extra-meta", default="",
                    help="JSON object merged into meta (e.g. the trained "
                         "goal 'profile' rl_policy.py reads)")
    ap.add_argument("--control-hz", type=float, default=None,
                    help="trained policy decision rate; robot runner checks "
                         "this before playback (default: config "
                         "control.hz)")
    ap.add_argument("--inner-hz", type=float, default=None,
                    help="optional robot-only servo stream rate override; "
                         "does not change the trained policy rate")
    ap.add_argument("--bus-write-speed", type=int, default=None,
                    help="optional robot bus write_speed for this policy")
    ap.add_argument("--bus-write-acc", type=int, default=None,
                    help="optional robot bus write_acc for this policy")
    ap.add_argument("--joint-frame", default=FRAME_ROBOT_ABS,
                    help=("policy joint frame: robot_abs (default, real "
                          "robot logical knee) or model_rel (legacy "
                          "MuJoCo femur-relative knee)"))
    ap.add_argument("--control-hz", type=float, default=None,
                    help=("trained control rate written to meta.control_hz "
                          "(default: current config control.hz). Legacy "
                          "pre-2026-08-24 checkpoints: pass 25."))
    args = ap.parse_args()
    extra_meta = json.loads(args.extra_meta) if args.extra_meta else {}
    if args.control_hz is None:
        args.control_hz = float(cfg_get(load_config(), "control", "hz",
                                        default=100.0))
    for key, value in (
        ("control_hz", args.control_hz),
        ("inner_hz", args.inner_hz),
        ("bus_write_speed", args.bus_write_speed),
        ("bus_write_acc", args.bus_write_acc),
    ):
        if value is not None:
            extra_meta[key] = value
    export(args.policy, args.out, name=args.name, notes=args.notes,
           joint_frame=args.joint_frame,
           extra_meta=extra_meta or None,
           control_hz=args.control_hz)

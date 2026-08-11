"""Export an SB3 PPO MlpPolicy to a plain-JSON numpy MLP for the robot.

The Uno Q has no torch. The deterministic actor is just
tanh(W1 x + b1) -> tanh(W2 h + b2) -> Wout h + bout, clipped to [-1, 1],
so we dump the five matrices as JSON lists plus metadata (dims, the
action->joint-angle mapping constants, source checkpoint) and verify
bit-for-bit-ish parity against model.predict before writing.

Usage:
    python -m rl_move.sim.export_policy_np \
        --policy rl_move/sim/policies/ppo_goal_cw_lower.zip \
        --out linux_control/rl_policy_weights.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def export(policy_path: str, out_path: str, *, name: str = "",
           notes: str = "") -> None:
    from stable_baselines3 import PPO
    import torch

    model = PPO.load(policy_path, device="cpu")
    pol = model.policy
    assert type(pol.mlp_extractor.policy_net[1]).__name__ == "Tanh", \
        "expected tanh activations (SB3 MlpPolicy default)"

    def t2l(t: torch.Tensor) -> list:
        return t.detach().cpu().numpy().astype(np.float64).tolist()

    net = pol.mlp_extractor.policy_net
    payload = {
        "meta": {
            "source": str(policy_path),
            # Optional display fields for the robot's policy picker
            # (linux_control/policies/ + /api/rl/policies).
            **({"name": name} if name else {}),
            **({"notes": notes} if notes else {}),
            "obs_dim": int(pol.observation_space.shape[0]),
            "act_dim": int(pol.action_space.shape[0]),
            "hidden": [int(net[0].out_features), int(net[2].out_features)],
            "activation": "tanh",
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
    args = ap.parse_args()
    export(args.policy, args.out, name=args.name, notes=args.notes)

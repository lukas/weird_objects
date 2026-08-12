"""bc_init_gait.py — BC-INIT from the hardware-proven scripted gait
(TALL LADDER endgame, operator session 08-11 late).

WHY (the campaign's conclusion): PPO never *finds* tall walking — it
invents paddles from scratch (10+ arms), ignores prices (6 pricing
arms flat at −72..−75 mm, kh10 pays more than its income), freezes
under a BC anchor DURING RL (gaitbc1: a moving reference is satisfied
by freezing), and dives back to the crouch even when spawned
mid-stride tall (RSI, cw-dep-tall-rsi1). The missing piece is never
states or prices — it is the POLICY'S ACTIONS from tall states.

THIS tool fixes the actions directly: supervised behavior cloning of
the scripted TripodGait (the controller that walks tall in sim AND
walked the physical robot in the 08-11 tape sessions) into a fresh
PPO policy net, saved as a normal SB3 checkpoint for --init-from.
RL fine-tuning afterwards adds ONLY what RL has proven it adds:
disturbance recovery (tipped-recovery 0.25 → 1.0 in one run,
cw-dep-tip1-takeoff25-r1). An init cannot be satisfied by freezing —
the known failure of anchor-during-RL does not apply.

Usage (from prototype root):

    ../../.venv/bin/python -m rl_move.sim.bc_init_gait \
        --out rl_move/sim/policies/ppo_goal_cw_bcgait_init.zip

Then verify with the campaign's binding metric BEFORE any RL:

    ../../.venv/bin/python -m rl_move.sim.probe_tall_wall \
        --ckpt rl_move/sim/policies/ppo_goal_cw_bcgait_init.zip \
        --ref 0 --no-gait
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.probe_walk_income import (  # noqa: E402
    STACKS, WALK_PLANT, make_env,
)


def collect(episodes: int, seed0: int,
            noise: float = 0.05) -> tuple[np.ndarray, np.ndarray, object]:
    """Roll the scripted gait under the env's OWN sampled walk goals
    (dep-contract obs, DR off) and record (obs_t, scripted_action_t).

    DART-style noise injection: the EXECUTED action carries Gaussian
    noise while the LABEL stays the clean expert action, so the
    dataset covers slightly-off-cycle states and teaches the clone to
    steer BACK to the gait. Without it the pure clone walks tall but
    collapses to the mean pose in closed loop (measured 08-11: 11 mm
    travel over a 15 s / 825 mm command)."""
    from tripod_gait import TripodGait

    rng = np.random.default_rng(seed0)
    obs_rows, act_rows = [], []
    env = None
    for ep in range(episodes):
        if env is not None:
            env.close()
        env = make_env(seed0 + ep, STACKS["vref1"])
        obs, _ = env.reset()
        traj = env._goal_traj
        n = len(traj.vx)
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()
        # A third of episodes are clean (the exact cycle must also be
        # in-distribution); the rest run noisy at two amplitudes.
        amp = (0.0, noise, 2.0 * noise)[ep % 3]
        step = 0
        while True:
            t = step * env.dt
            i = min(step, n - 1)
            gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t)) * DEG2RAD)
            obs_rows.append(np.asarray(obs, dtype=np.float32).copy())
            act_rows.append(np.asarray(act, dtype=np.float32).copy())
            act_exec = act if amp == 0.0 else np.clip(
                act + rng.normal(0.0, amp, act.shape), -1.0, 1.0)
            obs, _, term, trunc, _ = env.step(act_exec)
            step += 1
            if term or trunc:
                break
    return np.stack(obs_rows), np.stack(act_rows), env


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO

    print(f"collecting {args.episodes} scripted-gait episodes "
          f"(dep-contract obs, DR off)...")
    X, Y, env = collect(args.episodes, args.seed)
    print(f"dataset: {X.shape[0]} pairs, obs {X.shape[1]}, act {Y.shape[1]}")

    # Fresh PPO with the trainer's EXACT policy shape (train_ppo_sim:
    # MlpPolicy, net_arch [128,128], log_std_init -1.0) so --init-from
    # loads it without transplant.
    model = PPO(
        "MlpPolicy", env,
        n_steps=256, batch_size=2048,
        learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
        ent_coef=1e-3, clip_range=0.2,
        policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0),
        seed=args.seed, verbose=0, device="cpu",
    )

    pol = model.policy
    params = (list(pol.mlp_extractor.policy_net.parameters())
              + list(pol.action_net.parameters()))
    opt = torch.optim.Adam(params, lr=args.lr)
    Xt = torch.as_tensor(X)
    Yt = torch.as_tensor(Y)
    n = Xt.shape[0]
    rng = np.random.default_rng(args.seed)
    pol.train()
    for epoch in range(args.epochs):
        order = rng.permutation(n)
        tot, nb = 0.0, 0
        for i0 in range(0, n, args.batch):
            idx = torch.as_tensor(order[i0:i0 + args.batch])
            feats = pol.extract_features(
                Xt[idx], pol.features_extractor)
            latent = pol.mlp_extractor.forward_actor(feats)
            mean = pol.action_net(latent)
            loss = torch.nn.functional.mse_loss(mean, Yt[idx])
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += float(loss)
            nb += 1
        if epoch % 10 == 0 or epoch == args.epochs - 1:
            print(f"epoch {epoch:3d}  mse {tot / max(nb, 1):.6f}")
    pol.eval()

    # Sanity: deterministic replay error on a held-out slice.
    with torch.no_grad():
        feats = pol.extract_features(Xt[-2000:], pol.features_extractor)
        mean = pol.action_net(pol.mlp_extractor.forward_actor(feats))
        err = float((mean - Yt[-2000:]).abs().mean())
    print(f"holdout mean |action err|: {err:.4f} (action units)")

    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out))
    print(f"saved {out}")
    env.close()


if __name__ == "__main__":
    main()

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


def _make_teacher(teacher: str, period_scale: float = 1.0):
    """Scripted teacher by name. Both share the TripodGait call surface.

    tripod = the hardware-proven drag gait (the original BC-INIT
    teacher); noslip = the world-anchored step-then-shift gait
    (linux_control/noslip_gait.py) — quasi-static, zero commanded foot
    drag, clamps commands to its own envelope (MAX_VX 0.04);
    noslip_clean = the same gait on NoSlipGait.CLAMP_FIT_KW — the 08-12
    sweep's cleanest timing under the fitted ~31 deg/s servo clamp
    (zero true scrub IN the clamped training profile).

    period_scale (fast-cadence lever, operator order 08-20): scales the
    TripodGait clock period (default 0.75 s). 1.0 = bit-exact legacy
    behavior; 0.75 = 0.5625 s cycle (33% faster cadence, SHORTER
    strides at the same body speed — stride = v*t_eff/2 — so the servo
    excursion per swing shrinks instead of the write_speed dose
    rising). Tripod teacher only."""
    if teacher.startswith("noslip"):
        if period_scale != 1.0:
            raise SystemExit(
                "--tripod-period-scale only applies to the tripod "
                "teacher; noslip gaits own their timing")
        from sim_gait_compat import NoSlipGait
        gait = (NoSlipGait.clamp_fit() if teacher == "noslip_clean"
                else NoSlipGait())
        gait.sync_plant_stance(*WALK_PLANT)
        return gait
    from sim_gait_compat import TripodGait
    gait = TripodGait(vx=0.0, period_scale=period_scale)
    gait.sync_plant_stance(*WALK_PLANT)
    gait.reset_phase()
    return gait


def collect(episodes: int, seed0: int, noise: float = 0.05,
            teacher: str = "tripod", stack: dict | None = None,
            period_scale: float = 1.0,
            ) -> tuple[np.ndarray, np.ndarray, object]:
    """Roll the scripted gait under the env's OWN sampled walk goals
    (dep-contract obs, DR off) and record (obs_t, scripted_action_t).

    DART-style noise injection: the EXECUTED action carries Gaussian
    noise while the LABEL stays the clean expert action, so the
    dataset covers slightly-off-cycle states and teaches the clone to
    steer BACK to the gait. Without it the pure clone walks tall but
    collapses to the mean pose in closed loop (measured 08-11: 11 mm
    travel over a 15 s / 825 mm command)."""
    if stack is None:
        stack = STACKS["vref1"]
    # With goal.walk_phase_obs=1 the env appends [sin, cos] of its walk
    # phase clock at the obs tail (walk_task._augment_obs; requires
    # walk_yaw_cmd / mode_onehot OFF so the tail really is the phase).
    # The teacher's clock is then derived FROM that obs by unwrapping
    # the angle — exact alignment with the env clock by construction,
    # including the settle hold / park prefixes where the env clock
    # freezes (it only advances while a velocity is commanded). This is
    # what makes the clock-driven gait expressible for BC at all: the
    # policy sees the same phase the teacher acts on. Set
    # goal.walk_phase_hz = 1 / gait period so one clock revolution is
    # one gait cycle (the sin/cos <-> phase map must be one-to-one per
    # cycle for the feature to disambiguate).
    phase_obs = float(stack.get(("goal", "walk_phase_obs"), 0.0)) == 1.0
    if phase_obs and not teacher.startswith("noslip"):
        # The phase-obs <-> teacher-clock coupling (docstring above)
        # must also hold for a SCALED clock: fail closed on mismatch.
        _g = _make_teacher(teacher, period_scale)
        _per = _g.period * _g.period_scale
        _hz = float(stack.get(("goal", "walk_phase_hz"), 1.0))
        if abs(_hz * _per - 1.0) > 1e-3:
            raise SystemExit(
                f"goal.walk_phase_hz={_hz} mismatches teacher period "
                f"{_per:.4f}s; set goal.walk_phase_hz={1.0 / _per:.4f}")
    rng = np.random.default_rng(seed0)
    obs_rows, act_rows = [], []
    env = None
    for ep in range(episodes):
        if env is not None:
            env.close()
        env = make_env(seed0 + ep, stack)
        obs, _ = env.reset()
        traj = env._goal_traj
        n = len(traj.vx)
        gait = _make_teacher(teacher, period_scale)
        # A third of episodes are clean (the exact cycle must also be
        # in-distribution); the rest run noisy at two amplitudes.
        amp = (0.0, noise, 2.0 * noise)[ep % 3]
        step = 0
        t_gait = 0.0
        phase_prev = 0.0
        while True:
            if phase_obs:
                phase = float(np.arctan2(obs[-2], obs[-1]))
                d = phase - phase_prev
                if d < -np.pi:
                    d += 2.0 * np.pi
                # The clock never runs backward; tiny negative d would
                # be a layout bug (wrong tail indices), not physics.
                t_gait += max(d, 0.0) / (2.0 * np.pi
                                         * float(stack.get(
                                             ("goal", "walk_phase_hz"),
                                             1.0)))
                phase_prev = phase
            else:
                t_gait = step * env.dt
            i = min(step, n - 1)
            gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
            act = q_rad_to_action(
                np.asarray(gait.desired_deg(t_gait)) * DEG2RAD)
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
    ap.add_argument("--teacher",
                    choices=("tripod", "noslip", "noslip_clean"),
                    default="tripod")
    ap.add_argument("--stack", choices=list(STACKS), default="vref1")
    ap.add_argument("--set", action="append", default=[], dest="sets",
                    metavar="SEC.KEY=V",
                    help="override a stack cfg leaf so the demo env "
                         "matches the training run (e.g. "
                         "goal.walk_speed_min_m_s=0.008); repeatable")
    ap.add_argument("--tripod-period-scale", type=float, default=1.0,
                    help="scale the TripodGait clock period (default "
                         "1.0 = bit-exact legacy 0.75 s cycle; 0.75 -> "
                         "0.5625 s cycle, 33%% faster cadence with "
                         "proportionally SHORTER strides at the same "
                         "body speed). Tripod teacher only; with "
                         "goal.walk_phase_obs=1 collect() enforces "
                         "goal.walk_phase_hz = 1/(0.75*scale)")
    ap.add_argument("--episodes", type=int, default=60)
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--batch", type=int, default=1024)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--log-std", type=float, default=-1.0,
                    help="exploration log_std BAKED INTO the saved "
                         "checkpoint (a plain --init-from warm start "
                         "keeps it; the trainer's --log-std-init only "
                         "applies to fresh models). The quasi-static "
                         "no-slip gait dies under the default sigma "
                         "0.37 (measured 08-12: det return 1027 vs "
                         "stochastic 79), so PPO never samples the "
                         "taught behavior; -2.5 (sigma 0.082) sits "
                         "inside the DART-noise recovery envelope the "
                         "demos cover")
    ap.add_argument("--vf-episodes", type=int, default=0,
                    help="if >0, pretrain the VALUE head on Monte-Carlo "
                         "returns from this many deterministic rollouts "
                         "of the freshly cloned policy, so PPO's first "
                         "updates do not run on a random critic's "
                         "advantages")
    ap.add_argument("--vf-epochs", type=int, default=300)
    ap.add_argument("--vf-lr", type=float, default=3e-3,
                    help="hotter than the BC lr: MC return targets are "
                         "O(100), far from the head's near-zero init")
    args = ap.parse_args()

    import torch
    from stable_baselines3 import PPO

    stack = dict(STACKS[args.stack])
    for spec in args.sets:
        key, _, val = spec.partition("=")
        sec, _, leaf = key.partition(".")
        # float when numeric, else the raw string (e.g.
        # bus.servo_vel_max_counts_s=write_speed) — same coercion as
        # probe_walk_income --set; float() alone crashed on the 08-19
        # profile-ceiling selector.
        try:
            stack[(sec, leaf)] = float(val)
        except ValueError:
            stack[(sec, leaf)] = val
    print(f"collecting {args.episodes} scripted-gait episodes "
          f"(teacher={args.teacher}, stack={args.stack}"
          + (f" +{len(args.sets)} overrides" if args.sets else "")
          + (f", period_scale={args.tripod_period_scale}"
             if args.tripod_period_scale != 1.0 else "")
          + ", dep-contract obs, DR off)...")
    X, Y, env = collect(args.episodes, args.seed, teacher=args.teacher,
                        stack=stack,
                        period_scale=args.tripod_period_scale)
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

    if args.vf_episodes > 0:
        gamma = 0.99
        # Horizon after which the missing bootstrap at truncation is
        # negligible; drop that tail from the regression targets.
        cut = int(round(1.0 / (1.0 - gamma)))
        vf_obs, vf_ret = [], []
        for ep in range(args.vf_episodes):
            env2 = make_env(10_000 + args.seed + ep, stack)
            obs2, _ = env2.reset()
            rows, rews = [], []
            while True:
                act2, _ = model.predict(obs2, deterministic=True)
                rows.append(np.asarray(obs2, dtype=np.float32).copy())
                obs2, r, term, trunc, _ = env2.step(act2)
                rews.append(float(r))
                if term or trunc:
                    break
            env2.close()
            rets = np.zeros(len(rews), dtype=np.float32)
            g = 0.0
            for i in range(len(rews) - 1, -1, -1):
                g = rews[i] + gamma * g
                rets[i] = g
            keep = len(rews) if term else max(0, len(rews) - cut)
            vf_obs.extend(rows[:keep])
            vf_ret.extend(rets[:keep])
        Xv = torch.as_tensor(np.stack(vf_obs))
        Rv = torch.as_tensor(np.asarray(vf_ret, dtype=np.float32))
        vparams = (list(pol.mlp_extractor.value_net.parameters())
                   + list(pol.value_net.parameters()))
        vopt = torch.optim.Adam(vparams, lr=args.vf_lr)
        pol.train()
        for epoch in range(args.vf_epochs):
            order = rng.permutation(Xv.shape[0])
            tot, nb = 0.0, 0
            for i0 in range(0, Xv.shape[0], args.batch):
                idx = torch.as_tensor(order[i0:i0 + args.batch])
                feats = pol.extract_features(
                    Xv[idx], pol.features_extractor)
                v = pol.value_net(
                    pol.mlp_extractor.forward_critic(feats)).squeeze(-1)
                loss = torch.nn.functional.mse_loss(v, Rv[idx])
                vopt.zero_grad()
                loss.backward()
                vopt.step()
                tot += float(loss)
                nb += 1
            if epoch % 50 == 0 or epoch == args.vf_epochs - 1:
                print(f"vf epoch {epoch:3d}  mse {tot / max(nb, 1):.3f}")
        pol.eval()
        print(f"value head pretrained on {Xv.shape[0]} states "
              f"({args.vf_episodes} det rollouts, gamma {gamma})")

    if args.log_std != -1.0:
        with torch.no_grad():
            pol.log_std.fill_(args.log_std)
        print(f"log_std set to {args.log_std} "
              f"(sigma {float(np.exp(args.log_std)):.3f})")

    out = ROOT / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    model.save(str(out))
    print(f"saved {out}")
    env.close()


if __name__ == "__main__":
    main()

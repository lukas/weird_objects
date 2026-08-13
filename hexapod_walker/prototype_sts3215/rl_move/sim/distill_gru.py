"""Distill the MLP champions into one recurrent GRU policy (BC).

Why: gru-r1..r4c showed from-scratch GRU+PPO at discovery budget (2M)
reliably finds the leg-sacrifice paddle before it finds walking. The
MLP reference (hist16-r7) needed 40M walk-only steps — nothing walks
from scratch at 2M. Instead of exploring, imitate: both champions are
single-frame 18-joint policies acting in the SAME action space as the
student, so BC labels are simply the teacher's actions.

Teachers (both run inside the student's walk env, the play.py bridge):
- walk episodes:  ppo_goal_cw_walk_longdist_r2 on the full walk obs;
- rise/lower/hold episodes: ppo_goal_cw_stance_dr10 on obs[:68]
  (walk env obs = stance obs + [vx_ref, vy_ref, vx_meas, vy_meas]).

The student is a RecurrentPPO GruActorCriticPolicy trained by BPTT
over WHOLE episodes (hidden state starts at zero only at true episode
starts — no mid-episode chunking, which would teach a bogus
"zero state mid-episode" convention). Saved as a normal SB3 zip for
``train_ppo_mjx --gru --init-from`` RL fine-tuning.

Run (from prototype_sts3215/):
    ../../.venv/bin/python -m rl_move.sim.distill_gru \
        --episodes 400 --epochs 30

TRANSITIONS_DIRECTIVE CODE item 2 (08-13): ``--transitions N`` adds N
teacher-chained SEQUENCE demo episodes on a ``goal.mode_seq=1`` env
(CODE item 1): the env chains grammar segments and re-anchors refs at
every switch, the ACTIVE segment's teacher drives and labels, and the
student sees one continuous stream with the mode one-hot flipping.
DAgger rounds then run on whole sequences. Default 0 = feature off,
behavior unchanged. Arm 1 recipe:
    ../../.venv/bin/python -m rl_move.sim.distill_gru --dual \
        --transitions 300 --episodes 200 \
        --mix walk=0.30,rise=0.40,lower=0.15,hold=0.15 --epochs 30 \
        --dagger-rounds 2 --dagger-episodes 100 \
        --out rl_move/sim/policies/ppo_goal_cw_gru_dual_bc_transdagger1.zip
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

from .servo_model import SimServoParams
from .walk_task import SimHexapodJointWalkEnv

POLICY_DIR = Path(__file__).resolve().parent / "policies"

# The gru-r3/r4c env config (anti-cheat + joystick stack) so demo obs /
# goal-command distributions match what RL fine-tuning will see.
R3_CFG = {
    "reward.k_step_event": 1.0,
    "reward.k_drag_loaded": 10.0,
    "reward.k_park_duty": 1.0,
    "reward.walk_kernel_prog_gate": 1.0,
    "reward.walk_anchor_gate": 1.0,
    "reward.anchor_tol_mm": 10.0,
    "goal.walk_speed_min_m_s": 0.05,
    "goal.walk_speed_max_m_s": 0.06,
    "goal.walk_park_start_frac": 0.25,
    "goal.walk_heading_max_rad": 0.7854,
    "goal.walk_cmd_resample_s": 1.5,
    "goal.walk_cmd_resample_jitter": 0.6,
    "goal.walk_cmd_blend_s_min": 0.1,
    "goal.walk_cmd_blend_s_max": 1.0,
    "goal.walk_stop_frac": 0.2,
}

# r3/r4c training diet; episode counts per mode follow these weights.
DIET = {"walk": 0.60, "rise": 0.15, "lower": 0.15, "hold": 0.10}


def _build_cfg(extra: dict | None = None) -> dict:
    from rl_move.config import load_config
    cfg = load_config()
    items = dict(R3_CFG)
    if extra:
        items.update(extra)
    for dotted, val in items.items():
        node = cfg
        *path, leaf = dotted.split(".")
        for k in path:
            node = node.setdefault(k, {})
        node[leaf] = val
    return cfg


def _make_env(args, cfg: dict, params) -> SimHexapodJointWalkEnv:
    return SimHexapodJointWalkEnv(
        params=params, randomize=True, dr_scale=args.dr_scale,
        episode_seconds=args.episode_seconds, seed=args.seed, cfg=cfg)


def collect(envs: dict, teachers: dict, episodes_by_mode: dict[str, int],
            stochastic_frac: float, rng: np.random.Generator):
    """Teacher rollouts -> list of per-episode (obs, act, val) arrays.

    ``envs`` maps mode -> env (stance modes may use the teacher's
    native episode length). Prints per-mode teacher episode returns —
    if the TEACHER scores badly in this env context, BC on more of its
    demos cannot help and the context is what needs fixing.
    """
    import torch

    episodes = []
    t0 = time.monotonic()
    for mode, n_ep in episodes_by_mode.items():
        teacher, n_t_obs = teachers["walk" if mode == "walk" else "stance"]
        env = envs[mode]
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        returns = []
        for _ in range(n_ep):
            deterministic = rng.random() >= stochastic_frac
            obs, info = env.reset()
            got = info.get("goal_mode", mode)
            obs_l, act_l, val_l = [], [], []
            done, ep_ret = False, 0.0
            while not done:
                t_obs = obs[:n_t_obs]
                act, _ = teacher.predict(t_obs, deterministic=deterministic)
                with torch.no_grad():
                    val = teacher.policy.predict_values(
                        torch.as_tensor(t_obs[None]).float()).item()
                obs_l.append(obs)
                act_l.append(np.clip(act, -1.0, 1.0))
                val_l.append(val)
                obs, r, term, trunc, _ = env.step(act)
                ep_ret += r
                done = term or trunc
            returns.append(ep_ret)
            episodes.append((got,
                             np.asarray(obs_l, dtype=np.float32),
                             np.asarray(act_l, dtype=np.float32),
                             np.asarray(val_l, dtype=np.float32)))
        print(f"[distill-gru] {mode}: {n_ep} eps, teacher return "
              f"med {np.median(returns):.0f} min {min(returns):.0f} "
              f"({time.monotonic() - t0:.0f}s elapsed)")
    return episodes


def _seq_teacher(mode: str, teachers: dict):
    """Per-tick teacher routing for sequence episodes: the ACTIVE
    segment's teacher labels (walk segments -> walk teacher, every
    stance-family segment -> stance teacher)."""
    return teachers["walk" if mode == "walk" else "stance"]


def collect_transitions(env, teachers: dict, n_ep: int,
                        stochastic_frac: float, rng: np.random.Generator,
                        verify_n: int = 12, verify_max_falls: int = 4):
    """Teacher-CHAINED sequence demos (TRANSITIONS_DIRECTIVE CODE item 2).

    ``env`` must run ``goal.mode_seq=1``: the env itself chains K
    grammar segments per episode and re-anchors the goal frame at every
    switch (CODE item 1 mechanics — same re-anchor semantics as
    eval_handoff), so the student sees ONE continuous obs/act stream
    with the mode one-hot flipping mid-episode; labels always come from
    the ACTIVE segment's teacher. Nothing is stitched post-hoc.

    In-context teacher verification (directive rule: "a teacher that
    scores badly in the sequence context cannot be distilled"): the
    first ``verify_n`` episodes run deterministic; if the chained
    teachers fall more than ``verify_max_falls`` times in that window
    the collection ABORTS loudly instead of distilling garbage.
    Returns (episodes, stats) — stats carries per-episode plan/fall/
    per-tick-mode records for diagnostics and tests.
    """
    import torch

    if float(env.cfg.get("goal", {}).get("mode_seq", 0.0)) != 1.0:
        raise SystemExit("collect_transitions needs a goal.mode_seq=1 env")
    episodes: list = []
    stats: dict = {"eps": [], "falls": 0, "fall_modes": {}, "verify": None}
    t0 = time.monotonic()
    for i in range(n_ep):
        deterministic = (True if i < verify_n
                         else bool(rng.random() >= stochastic_frac))
        obs, _info = env.reset()
        plan_modes = [str(p["mode"]) for p in env._seq_plan]
        obs_l, act_l, val_l, mode_l = [], [], [], []
        done, ep_ret, fall_mode = False, 0.0, None
        while not done:
            mode = str(env._goal_traj.mode)
            teacher, n_t = _seq_teacher(mode, teachers)
            t_obs = obs[:n_t]
            act, _ = teacher.predict(t_obs, deterministic=deterministic)
            with torch.no_grad():
                val = teacher.policy.predict_values(
                    torch.as_tensor(t_obs[None]).float()).item()
            obs_l.append(obs)
            act_l.append(np.clip(act, -1.0, 1.0))
            val_l.append(val)
            mode_l.append(mode)
            obs, r, term, trunc, _ = env.step(act)
            ep_ret += float(r)
            if term:                      # fall/over-current, not clock
                fall_mode = mode
            done = term or trunc
        episodes.append(("seq",
                         np.asarray(obs_l, dtype=np.float32),
                         np.asarray(act_l, dtype=np.float32),
                         np.asarray(val_l, dtype=np.float32)))
        if fall_mode is not None:
            stats["falls"] += 1
            stats["fall_modes"][fall_mode] = (
                stats["fall_modes"].get(fall_mode, 0) + 1)
        stats["eps"].append({"plan": plan_modes, "len": len(act_l),
                             "switches": int(env._seq_idx),
                             "fall": fall_mode, "ret": ep_ret,
                             "det": deterministic, "modes": mode_l})
        if verify_n > 0 and i == verify_n - 1:
            stats["verify"] = (stats["falls"], verify_n)
            print(f"[distill-gru] transitions verify: {stats['falls']} "
                  f"fall(s) in first {verify_n} det sequences "
                  f"(cap {verify_max_falls})")
            if stats["falls"] > verify_max_falls:
                raise SystemExit(
                    "[distill-gru] TEACHER NOT SEQUENCE-COMPETENT: "
                    f"{stats['falls']} falls in the first {verify_n} "
                    f"deterministic sequence episodes (> "
                    f"{verify_max_falls}); by the directive's rule this "
                    "teacher pair cannot be distilled on sequences — "
                    "fix the teacher/context, do not collect more demos "
                    f"(fall modes: {stats['fall_modes']})")
        if (i + 1) % 25 == 0 or i == n_ep - 1:
            rets = [e["ret"] for e in stats["eps"]]
            print(f"[distill-gru] transitions: {i + 1}/{n_ep} eps, "
                  f"falls {stats['falls']} {stats['fall_modes']}, "
                  f"teacher return med {np.median(rets):.0f} "
                  f"min {min(rets):.0f} "
                  f"({time.monotonic() - t0:.0f}s elapsed)")
    return episodes, stats


def collect_dagger_transitions(env, student, teachers: dict, n_ep: int):
    """Sequence DAgger round: the STUDENT drives WHOLE sequences
    (stateful, deterministic) through the mode_seq env; the ACTIVE
    segment's teacher labels every visited state — including lower
    segments (failure-ledger lesson 9: label EVERY segment so the
    dagger1 lower collapse cannot be inherited)."""
    import torch

    episodes: list = []
    falls = 0
    t0 = time.monotonic()
    for i in range(n_ep):
        obs, _info = env.reset()
        state, ep_start = None, np.ones((1,), dtype=bool)
        obs_l, act_l, val_l = [], [], []
        done = False
        while not done:
            mode = str(env._goal_traj.mode)
            teacher, n_t = _seq_teacher(mode, teachers)
            t_obs = obs[:n_t]
            label, _ = teacher.predict(t_obs, deterministic=True)
            with torch.no_grad():
                val = teacher.policy.predict_values(
                    torch.as_tensor(t_obs[None]).float()).item()
            obs_l.append(obs)
            act_l.append(np.clip(label, -1.0, 1.0))
            val_l.append(val)
            act, state = student.predict(
                obs, state=state, episode_start=ep_start,
                deterministic=True)
            ep_start = np.zeros((1,), dtype=bool)
            obs, _, term, trunc, _ = env.step(act)
            falls += int(bool(term))
            done = term or trunc
        episodes.append(("seq",
                         np.asarray(obs_l, dtype=np.float32),
                         np.asarray(act_l, dtype=np.float32),
                         np.asarray(val_l, dtype=np.float32)))
    print(f"[distill-gru] dagger transitions: {n_ep} eps, student "
          f"falls {falls} ({time.monotonic() - t0:.0f}s elapsed)")
    return episodes


def collect_dagger(envs: dict, student, teachers: dict,
                   episodes_by_mode: dict[str, int]):
    """DAgger round: STUDENT drives (stateful, deterministic), TEACHER
    labels every visited state. Fixes BC compounding error — the
    student learns recoveries on its own trajectory distribution."""
    import torch

    episodes = []
    t0 = time.monotonic()
    for mode, n_ep in episodes_by_mode.items():
        teacher, n_t_obs = teachers["walk" if mode == "walk" else "stance"]
        env = envs[mode]
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        for _ in range(n_ep):
            obs, info = env.reset()
            got = info.get("goal_mode", mode)
            state, ep_start = None, np.ones((1,), dtype=bool)
            obs_l, act_l, val_l = [], [], []
            done = False
            while not done:
                t_obs = obs[:n_t_obs]
                label, _ = teacher.predict(t_obs, deterministic=True)
                with torch.no_grad():
                    val = teacher.policy.predict_values(
                        torch.as_tensor(t_obs[None]).float()).item()
                obs_l.append(obs)
                act_l.append(np.clip(label, -1.0, 1.0))
                val_l.append(val)
                act, state = student.predict(
                    obs, state=state, episode_start=ep_start,
                    deterministic=True)
                ep_start = np.zeros((1,), dtype=bool)
                obs, _, term, trunc, _ = env.step(act)
                done = term or trunc
            episodes.append((got,
                             np.asarray(obs_l, dtype=np.float32),
                             np.asarray(act_l, dtype=np.float32),
                             np.asarray(val_l, dtype=np.float32)))
        print(f"[distill-gru] dagger {mode}: {n_ep} eps "
              f"({time.monotonic() - t0:.0f}s elapsed)")
    return episodes


def train_student(student, episodes, epochs: int, batch_eps: int = 8,
                  lr: float = 1e-3) -> float:
    """Sequence BC: BPTT over whole padded episodes, masked loss."""
    import torch
    import torch.nn.functional as F

    policy = student.policy
    policy.set_training_mode(True)
    opt = torch.optim.Adam(policy.parameters(), lr=lr)

    seqs = [(torch.as_tensor(o), torch.as_tensor(a), torch.as_tensor(v))
            for _, o, a, v in episodes]
    all_vals = np.concatenate([v for _, _, _, v in episodes])
    v_scale = 1.0 / max(float(np.var(all_vals)), 1.0)
    n = len(seqs)
    last_a = float("nan")
    for ep in range(epochs):
        perm = np.random.permutation(n)
        tot_a = tot_c = 0.0
        nb = 0
        for i in range(0, n, batch_eps):
            batch = [seqs[j] for j in perm[i:i + batch_eps]]
            t_max = max(o.shape[0] for o, _, _ in batch)
            b = len(batch)
            obs_p = torch.zeros(t_max, b, batch[0][0].shape[1])
            act_p = torch.zeros(t_max, b, batch[0][1].shape[1])
            val_p = torch.zeros(t_max, b, 1)
            mask = torch.zeros(t_max, b, 1)
            for k, (o, a, v) in enumerate(batch):
                t = o.shape[0]
                obs_p[:t, k] = o
                act_p[:t, k] = a
                val_p[:t, k, 0] = v
                mask[:t, k, 0] = 1.0

            feats = policy.extract_features(
                obs_p.reshape(t_max * b, -1)).reshape(t_max, b, -1)
            # Whole episodes from reset: zero initial hidden state is
            # the truth, so a plain fused GRU call is exact BPTT.
            if hasattr(policy, "bptt_forward"):
                # Dual-core policy: mode-gated routing lives inside.
                mu, v_pred = policy.bptt_forward(feats)
            else:
                lat_pi, _ = policy.lstm_actor(feats)
                lat_vf, _ = policy.lstm_critic(feats)
                mu = policy.action_net(
                    policy.mlp_extractor.forward_actor(lat_pi))
                v_pred = policy.value_net(
                    policy.mlp_extractor.forward_critic(lat_vf))
            m_sum = mask.sum()
            loss_a = (F.mse_loss(mu, act_p, reduction="none")
                      * mask).sum() / (m_sum * mu.shape[-1])
            loss_c = (F.mse_loss(v_pred, val_p, reduction="none")
                      * mask).sum() / m_sum
            loss = loss_a + 0.5 * v_scale * loss_c
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            opt.step()
            tot_a += float(loss_a.detach())
            tot_c += float(loss_c.detach())
            nb += 1
        last_a = tot_a / max(nb, 1)
        if ep % 5 == 0 or ep == epochs - 1:
            print(f"[distill-gru] epoch {ep:3d}  actor_mse {last_a:.5f}  "
                  f"critic_mse {tot_c / max(nb, 1):.1f}")
    policy.set_training_mode(False)
    return last_a


def probe_seq(student, env, n_ep: int = 2) -> None:
    """Stateful deterministic sequence sanity rollouts (mode_seq env)."""
    for _ in range(n_ep):
        obs, _ = env.reset()
        plan = [str(p["mode"]) for p in env._seq_plan]
        state, ep_start = None, np.ones((1,), dtype=bool)
        done, tot, fell = False, 0.0, False
        while not done:
            a, state = student.predict(
                obs, state=state, episode_start=ep_start,
                deterministic=True)
            ep_start = np.zeros((1,), dtype=bool)
            obs, r, term, trunc, _ = env.step(a)
            tot += float(r)
            fell = fell or bool(term)
            done = term or trunc
        print(f"[distill-gru] probe seq {'->'.join(plan)}: "
              f"return {tot:.0f} fell={fell}")


def quick_probe(student, env, modes=("walk", "rise", "hold"),
                n_ep: int = 2) -> None:
    """Stateful deterministic sanity rollouts (not the real harness)."""
    for mode in modes:
        env.set_goal_mix({m: (1.0 if m == mode else 0.0) for m in DIET})
        rews = []
        for _ in range(n_ep):
            obs, _ = env.reset()
            state, ep_start = None, np.ones((1,), dtype=bool)
            done, tot = False, 0.0
            while not done:
                a, state = student.predict(
                    obs, state=state, episode_start=ep_start,
                    deterministic=True)
                ep_start = np.zeros((1,), dtype=bool)
                obs, r, term, trunc, _ = env.step(a)
                tot += r
                done = term or trunc
            rews.append(tot)
        print(f"[distill-gru] probe {mode}: ep returns "
              f"{[f'{r:.0f}' for r in rews]}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--walk-teacher", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_walk_longdist_r2.zip")
    ap.add_argument("--stance-teacher", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_stance_dr10.zip")
    ap.add_argument("--out", type=Path,
                    default=POLICY_DIR / "ppo_goal_cw_gru_bc.zip")
    ap.add_argument("--episodes", type=int, default=400,
                    help="total demo episodes, split per --mix")
    ap.add_argument("--mix", type=str, default=None,
                    help="demo mix, e.g. walk=0.3,rise=0.35,lower=0.15,"
                         "hold=0.2 (default: the r3 training diet). "
                         "bc2/bc3 post-mortem (r3-cfg harness): walk "
                         "below ~0.5 degrades the gait (bc3 walk=0.40 -> "
                         "gait_valid 1/6, sacrificed legs); rise never "
                         "clones regardless of volume (60->270 eps, best "
                         "1/6) — compounding error on a knife-edge "
                         "maneuver, not data poverty. Fix rise with RL, "
                         "not more demos.")
    ap.add_argument("--dagger-rounds", type=int, default=0,
                    help="after BC, N rounds of student-drives/teacher-"
                         "labels data aggregation. bc2 used 2 rounds "
                         "(confounded with 10s stance demos): det "
                         "lower/hold collapsed vs bc1 and rise did not "
                         "improve — no evidence it helps here; default 0")
    ap.add_argument("--dagger-episodes", type=int, default=150,
                    help="episodes per DAgger round (same --mix split)")
    ap.add_argument("--stance-episode-seconds", type=float, default=15.0,
                    help="episode length for rise/lower/hold demos. Keep "
                         "equal to --episode-seconds: bc2 collected these "
                         "at the stance teacher's native 10s and the "
                         "student regressed on the 15s eval context")
    ap.add_argument("--dual", action="store_true",
                    help="distill into DualGruActorCriticPolicy (mode-"
                         "gated locomotion+stance cores; anchor1..3 "
                         "closure). Turns on obs.mode_onehot=1 — the "
                         "env appends the 6-wide skill-family one-hot "
                         "at the obs tail, teachers still read their "
                         "prefix slices")
    ap.add_argument("--transitions", type=int, default=0,
                    help="TRANSITIONS_DIRECTIVE CODE item 2: N teacher-"
                         "chained SEQUENCE demo episodes on a "
                         "goal.mode_seq=1 env (the env chains grammar "
                         "segments rise->{hold|walk}->{walk|lower}->"
                         "(rise..) and re-anchors refs at each switch; "
                         "the active segment's teacher drives AND "
                         "labels; the student sees one continuous "
                         "stream with the mode one-hot flipping). "
                         "Requires --dual (failure-ledger lesson 1). "
                         "With --dagger-rounds, DAgger rounds collect "
                         "SEQUENCE episodes (student drives whole "
                         "sequences, teacher labels every segment "
                         "incl. lower — lesson 9). 0 (default) = "
                         "feature off, behavior unchanged")
    ap.add_argument("--seq-episode-seconds", type=float, default=30.0,
                    help="sequence episode length (directive: 25-30 s; "
                         "keep demo/train/eval contexts matched — "
                         "lesson 8)")
    ap.add_argument("--seq-segment-s", type=str, default="6,8",
                    help="goal.mode_seq_segment_s_min,max for demo "
                         "sequences (directive default 6-8 s jittered)")
    ap.add_argument("--seq-first-mix", type=str,
                    default="rise=0.40,walk=0.30,lower=0.15,hold=0.15",
                    help="first-segment mode draw for sequences "
                         "(stance-heavy default = the 08-13 dagger1 "
                         "retention mix; rise-first sequences keep the "
                         "full flat/bridge/crouch start-kind diversity "
                         "via the legacy samplers — lesson 3)")
    ap.add_argument("--seq-verify", type=int, default=12,
                    help="first N sequence demos run deterministic as "
                         "the in-context teacher verification (0 = "
                         "skip)")
    ap.add_argument("--seq-verify-max-falls", type=int, default=4,
                    help="abort collection if the deterministic verify "
                         "window has more falls than this (a teacher "
                         "that falls in sequences cannot be distilled "
                         "on them)")
    ap.add_argument("--stochastic-frac", type=float, default=0.3)
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--gru-hidden-size", type=int, default=256)
    ap.add_argument("--dr-scale", type=float, default=0.5)
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    if args.transitions > 0 and not args.dual:
        raise SystemExit(
            "--transitions requires --dual: the TRANSITIONS_DIRECTIVE "
            "arms run the mode-gated dual-core GRU (failure-ledger "
            "lesson 1 — never a shared trunk), and sequence routing "
            "rides the obs mode one-hot that --dual turns on")

    from sb3_contrib import RecurrentPPO
    from stable_baselines3 import PPO

    from .gru_policy import DualGruActorCriticPolicy, GruActorCriticPolicy

    rng = np.random.default_rng(args.seed)
    params = SimServoParams.load()
    cfg = _build_cfg({"obs.mode_onehot": 1.0} if args.dual else None)

    walk_teacher = PPO.load(args.walk_teacher, device="cpu")
    stance_teacher = PPO.load(args.stance_teacher, device="cpu")
    n_walk = int(walk_teacher.observation_space.shape[0])
    n_stance = int(stance_teacher.observation_space.shape[0])
    print(f"[distill-gru] teachers: walk obs {n_walk}, stance obs {n_stance}")

    env = _make_env(args, cfg, params)
    n_env_obs = int(env.observation_space.shape[0])
    n_walk_want = n_walk + (6 if args.dual else 0)  # + mode one-hot tail
    if n_env_obs != n_walk_want:
        raise SystemExit(f"env obs {n_env_obs} != expected {n_walk_want} "
                         f"(walk teacher {n_walk}"
                         f"{' + 6 one-hot' if args.dual else ''})")
    if n_stance >= n_env_obs:
        raise SystemExit(f"stance teacher obs {n_stance} not a prefix of "
                         f"env obs {n_env_obs}")
    # Stance demos run at the stance teacher's native episode length.
    import copy as _copy
    stance_args = _copy.copy(args)
    stance_args.episode_seconds = args.stance_episode_seconds
    stance_env = _make_env(stance_args, cfg, params)
    envs = {"walk": env, "rise": stance_env, "lower": stance_env,
            "hold": stance_env}

    mix = DIET
    if args.mix:
        mix = {k: float(v) for k, v in
               (kv.split("=") for kv in args.mix.split(","))}
        unknown = set(mix) - set(DIET)
        if unknown:
            raise SystemExit(f"--mix unknown modes: {unknown}")
    episodes_by_mode = {m: max(1, round(args.episodes * w))
                        for m, w in mix.items() if w > 0}
    teachers = {"walk": (walk_teacher, n_walk),
                "stance": (stance_teacher, n_stance)}

    # ---- sequence demos FIRST (TRANSITIONS_DIRECTIVE item 2): the
    # in-context teacher verification aborts before any single-mode
    # collection time is spent on a teacher pair that cannot chain.
    seq_env = None
    episodes: list = []
    if args.transitions > 0:
        seg_lo, seg_hi = (float(x) for x in args.seq_segment_s.split(","))
        seq_cfg = _build_cfg({"obs.mode_onehot": 1.0,
                              "goal.mode_seq": 1.0,
                              "goal.mode_seq_segment_s_min": seg_lo,
                              "goal.mode_seq_segment_s_max": seg_hi})
        seq_args = _copy.copy(args)
        seq_args.episode_seconds = args.seq_episode_seconds
        seq_env = _make_env(seq_args, seq_cfg, params)
        if int(seq_env.observation_space.shape[0]) != n_env_obs:
            raise SystemExit("seq env obs != single-mode env obs "
                             f"({seq_env.observation_space.shape[0]} vs "
                             f"{n_env_obs})")
        first_mix = {k: float(v) for k, v in
                     (kv.split("=") for kv in args.seq_first_mix.split(","))}
        unknown = set(first_mix) - set(DIET)
        if unknown:
            raise SystemExit(f"--seq-first-mix unknown modes: {unknown}")
        seq_env.set_goal_mix({m: first_mix.get(m, 0.0) for m in DIET})
        seq_eps, _seq_stats = collect_transitions(
            seq_env, teachers, args.transitions, args.stochastic_frac,
            rng, args.seq_verify, args.seq_verify_max_falls)
        episodes.extend(seq_eps)

    episodes.extend(collect(envs, teachers, episodes_by_mode,
                            args.stochastic_frac, rng))
    n_steps_total = sum(len(a) for _, _, a, _ in episodes)
    print(f"[distill-gru] {len(episodes)} episodes, "
          f"{n_steps_total} transitions")

    student = RecurrentPPO(
        DualGruActorCriticPolicy if args.dual else GruActorCriticPolicy,
        env,
        n_steps=256, batch_size=8192, n_epochs=5, learning_rate=3e-4,
        gamma=0.99, gae_lambda=0.95, ent_coef=0.01, clip_range=0.2,
        target_kl=0.02,
        policy_kwargs=dict(lstm_hidden_size=args.gru_hidden_size),
        seed=args.seed, device="cpu", verbose=0)

    actor_mse = train_student(student, episodes, args.epochs)
    print(f"[distill-gru] BC actor RMS {np.sqrt(actor_mse):.4f} action "
          f"units (~{np.sqrt(actor_mse) * 85:.1f} deg on the knee axis)")

    dagger_by_mode = {m: max(1, round(args.dagger_episodes * w))
                      for m, w in mix.items() if w > 0}
    for rnd in range(args.dagger_rounds):
        if seq_env is not None:
            # Directive arm 1: DAgger rounds are SEQUENCE episodes —
            # the student drives whole chains, the active segment's
            # teacher labels every state (incl. lower, lesson 9).
            new_eps = collect_dagger_transitions(
                seq_env, student, teachers, args.dagger_episodes)
        else:
            new_eps = collect_dagger(envs, student, teachers,
                                     dagger_by_mode)
        episodes.extend(new_eps)
        actor_mse = train_student(student, episodes,
                                  max(args.epochs // 2, 5))
        print(f"[distill-gru] dagger round {rnd + 1}: dataset "
              f"{len(episodes)} eps, actor RMS {np.sqrt(actor_mse):.4f}")

    quick_probe(student, env)
    if seq_env is not None:
        probe_seq(student, seq_env)
        seq_env.close()
    env.close()
    stance_env.close()

    # BC never trains log_std (default 0.0 = full-range noise, which
    # wrecks both the sto harness pass and early RL fine-tune rollouts).
    student.policy.log_std.data.fill_(-1.5)
    student.save(args.out)
    print(f"[distill-gru] saved {args.out} (log_std -1.5)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""PPO trainer on batched MJX physics (MjxVecEnv) — separate entry point.

This is the EXPERIMENTAL large-batch trainer for the MJX/Warp backend.
It deliberately does NOT touch ``train_ppo_sim.py`` (the campaign's
production trainer keeps running unchanged); shared conventions
(task map, --cfg-set, W&B env file, policy dir) are imported from it.

Differences from the production trainer, on purpose:

- Physics: one ``MjxVecEnv`` (hundreds..thousands of envs, one
  accelerator) instead of Dummy/Subproc C-MuJoCo envs.
- Hyperparameters default to the large-batch regime: short rollouts
  (``--n-steps 16``) so an update happens every ~65k transitions at
  4096 envs, instead of 256-step rollouts tuned for 8-48 envs. These
  are STARTING points — the recipe rework is phase-2 item 6 in
  MJX_PORT.md and nothing here is validated for wall-clock learning yet.
- Monitoring eval/video runs in a background C-MuJoCo process, preserving
  the cross-simulator behavioral A/B without occupying the training GPU.
  Recovery curriculum decisions use a separate small deterministic MJX
  pool on the training backend; C evaluation is never an admission signal.
- v1 backend limits apply (MJX_PORT.md): model-field DR is OFF (shared
  nominal model), actuation/sensing DR is per-env as usual.

Typical GPU-pod use (after HEXAPOD_MJX=1 setup, mujoco-warp installed):

    python -m rl_move.sim.train_ppo_mjx --task joint_walk \
        --n-envs 4096 --impl warp --steps 20000000 --run-name mjx-trial0

Laptop smoke (CPU XLA, tiny batch):

    python -m rl_move.sim.train_ppo_mjx --smoke --no-wandb
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

_RL = Path(__file__).resolve().parents[1]
_PROTO = _RL.parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from .mjx_backend import mjx_is_available  # noqa: E402
from .servo_model import SimServoParams  # noqa: E402
from .train_ppo_sim import (  # noqa: E402
    ENV_CLASSES, POLICY_DIR, WANDB_ENTITY_DEFAULT, WANDB_PROJECT_DEFAULT,
    _learning_line, _load_wandb_env, _parse_cfg_set, _parse_goal_mix,
    _resolved_reward_cfg, _reward_notes, _warn_if_defaults,
)


def _recover_episode_outcome(info: dict) -> tuple[int, bool] | None:
    """Extract one exact terminal training outcome from an env info."""
    if not any(str(k).startswith("recover_episode_bucket_")
               for k in info):
        return None
    bucket = int(float(info.get("recover_start_bucket", -1)))
    if bucket < 0:
        return None
    success = bool(
        info.get("recover_success", 0.0) > 0.0
        or info.get("termination_reason") == "recover_success")
    return bucket, success


def _recover_cert_bucket_plan(frontier: int, retention_count: int,
                              cursor: int,
                              weak_bucket: int | None) -> tuple[list[int], int]:
    """Frontier plus a weak bucket and rotating old-bucket assays."""
    frontier = max(0, int(frontier))
    old = list(range(frontier))
    buckets = [frontier]
    weak = -1 if weak_bucket is None else int(weak_bucket)
    if weak in old:
        buckets.append(weak)
    if not old or retention_count <= 0:
        return buckets, 0
    cursor = int(cursor) % len(old)
    scanned = 0
    added = 0
    while scanned < len(old) and added < int(retention_count):
        bucket = old[(cursor + scanned) % len(old)]
        scanned += 1
        if bucket in buckets:
            continue
        buckets.append(bucket)
        added += 1
    return buckets, (cursor + scanned) % len(old)


def _recover_score_payload(state: dict, best_score: float = 0.0,
                           cert_ages: dict[int, int] | None = None
                           ) -> tuple[dict, float]:
    """Build the dedicated W&B recovery scoreboard.

    Bucket B contributes B+1 points times its latest deterministic success
    fraction. The denominator includes every curriculum bucket, including
    locked/untested ones, so the normalized score rises as harder abilities
    are unlocked rather than renormalizing the task underneath the policy.
    """
    total = int(state["total_buckets"])
    maximum = total * (total + 1) / 2.0
    rows = state.get("buckets", {})
    points = 0.0
    certified_weight = 0.0
    payload = {
        "RECOVER_SCORE/max_unlocked_bucket": float(
            state["max_unlocked_bucket"]),
        "RECOVER_SCORE/focus_bucket": float(state["focus_bucket"]),
        "RECOVER_SCORE/weakest_bucket": float(state["weakest_bucket"]),
        "RECOVER_SCORE/maximum_points": maximum,
    }
    gate_fractions = []
    for bucket in range(total):
        key = str(bucket)
        row = rows.get(key)
        weight = float(bucket + 1)
        if row is not None:
            fraction = float(row["success_fraction"])
            gate_fraction = float(row["gate_fraction"])
            bucket_points = weight * fraction
            points += bucket_points
            certified_weight += weight
            gate_fractions.append(gate_fraction)
            stem = f"RECOVER_SCORE/bucket_{bucket:02d}"
            payload[f"{stem}_success_fraction"] = fraction
            payload[f"{stem}_gate_fraction"] = gate_fraction
            payload[f"{stem}_successes"] = float(row["successes"])
            payload[f"{stem}_episodes"] = float(row["episodes"])
            payload[f"{stem}_points"] = bucket_points
            if cert_ages is not None and bucket in cert_ages:
                payload[f"{stem}_cert_age_rounds"] = float(
                    cert_ages[bucket])
        probability = state.get("sample_probabilities", {}).get(key)
        if probability is not None:
            payload[
                f"RECOVER_SCORE/bucket_{bucket:02d}_sample_probability"
            ] = float(probability)
    score = points / maximum if maximum > 0.0 else 0.0
    best = max(float(best_score), score)
    payload.update({
        "RECOVER_SCORE/overall_points": points,
        "RECOVER_SCORE/overall_weighted_success": score,
        "RECOVER_SCORE/best_overall_weighted_success": best,
        "RECOVER_SCORE/certified_weight_fraction": (
            certified_weight / maximum if maximum > 0.0 else 0.0),
        "RECOVER_SCORE/min_certified_gate_fraction": (
            min(gate_fractions) if gate_fractions else 0.0),
    })
    return payload, best


def _run_recover_cert_kind(vec_env, model, kind: str) -> dict:
    """One deterministic first-episode recovery assay on an MJX VecEnv."""
    n_envs = int(vec_env.num_envs)
    vec_env.set_attr("force_recover_start", str(kind))
    obs = vec_env.reset()
    state = None
    episode_start = np.ones(n_envs, dtype=bool)
    finished = np.zeros(n_envs, dtype=bool)
    outcomes = np.zeros(n_envs, dtype=bool)
    finish_ticks = np.zeros(n_envs, dtype=np.int64)
    max_ticks = int(getattr(vec_env, "_episode_steps", 0)) + 2
    if max_ticks <= 2:
        raise RuntimeError("MJX recovery cert env has no episode horizon")
    ticks = 0
    while not bool(np.all(finished)):
        actions, state = model.predict(
            obs, state=state, episode_start=episode_start,
            deterministic=True)
        obs, _rewards, dones, infos = vec_env.step(actions)
        ticks += 1
        episode_start = np.asarray(dones, dtype=bool)
        for i in np.flatnonzero(np.asarray(dones) & ~finished):
            info = infos[int(i)]
            outcomes[i] = bool(
                info.get("recover_success", 0.0) > 0.0
                or info.get("termination_reason") == "recover_success")
            finished[i] = True
            finish_ticks[i] = ticks
        if ticks > max_ticks:
            missing = np.flatnonzero(~finished).tolist()
            raise RuntimeError(
                f"MJX recovery certification exceeded the episode "
                f"horizon for envs {missing}")
    dt = float(getattr(vec_env, "_dt", 0.0))
    return {
        "kind": str(kind),
        "outcomes": outcomes.tolist(),
        "successes": int(outcomes.sum()),
        "episodes": n_envs,
        "success": float(outcomes.mean()),
        "time_mean_s": float(finish_ticks.mean() * dt),
    }


def _env_kwargs(args, params: SimServoParams | None = None) -> dict:
    """Per-shim-env kwargs — mirrors train_ppo_sim._build_env, minus the
    model-DR pieces the shared-model backend can't honor yet.

    ``params=None`` resolves the actuator set from the run's cfg
    (bus.servo_params: "" = air fit, "loaded" = loaded bench fit)."""
    kw = dict(randomize=not args.no_dr,
              dr_scale=args.dr_scale,
              episode_seconds=args.episode_seconds)
    overrides = _parse_cfg_set(args.cfg_set)
    external_recover_cert = (
        args.recover_cert_every > 0
        and args.recover_cert_envs > 0
        and float(_parse_goal_mix(args.goal_mix).get("recover", 0.0)) > 0.0)
    if overrides or external_recover_cert:
        from rl_move.config import load_config
        cfg = load_config()
        for dotted, val in overrides.items():
            node = cfg
            *path, leaf = dotted.split(".")
            for k in path:
                node = node.setdefault(k, {})
            node[leaf] = val
        if external_recover_cert:
            cfg.setdefault("goal", {})[
                "recover_external_certification"] = 1.0
        kw["cfg"] = cfg
    kw["params"] = (params if params is not None
                    else SimServoParams.from_cfg(kw.get("cfg")))
    return kw


def _resolve_impl(requested: str) -> str | None:
    """'auto' = warp when importable (GPU pods), else the mjx default."""
    if requested == "auto":
        try:
            import mujoco_warp  # noqa: F401
            return "warp"
        except Exception:
            return None
    return None if requested == "default" else requested


def _init_wandb(args, params: SimServoParams):
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except Exception:
        print("[wandb] not installed — logging skipped")
        return None
    import os
    if not os.environ.get("WANDB_API_KEY"):
        try:
            if not wandb.api.api_key:
                raise RuntimeError
        except Exception:
            print("[wandb] no API key — logging skipped")
            return None
    # Plain-English objective FIRST (operator 08-10: the overview must
    # open with what the run is learning, not lineage babble).
    notes = (_learning_line(args) + "\n\n" + (args.notes or "")).strip()
    notes += "\n\n" + _reward_notes(args.cfg_set)
    run = wandb.init(
        entity=WANDB_ENTITY_DEFAULT,
        project=os.environ.get("WANDB_PROJECT", WANDB_PROJECT_DEFAULT),
        # Research tracks (operator 08-11) arrive as WANDB_TAGS
        # (track:<id>), which wandb.init honors natively.
        group="mjx-trainer", name=args.run_name, notes=notes,
        sync_tensorboard=True,   # SB3 train/* metrics, like the campaign
        config={"trainer": "train_ppo_mjx", "task": args.task,
                "n_envs": args.n_envs, "impl": args.impl,
                "n_steps": args.n_steps, "batch_size": args.batch_size,
                "learning_rate": args.lr, "seed": args.seed,
                "dr_scale": args.dr_scale, "no_dr": args.no_dr,
                "cfg_set": args.cfg_set,
                "reward_cfg": _resolved_reward_cfg(args.cfg_set),
                "episode_seconds": args.episode_seconds,
                "mjx_iterations": args.mjx_iterations,
                "mjx_ls_iterations": args.mjx_ls_iterations,
                "recover_cert_every": args.recover_cert_every,
                "recover_cert_envs": args.recover_cert_envs,
                "recover_retention_buckets": args.recover_retention_buckets,
                "recover_replay_mix": {
                    "focus": 0.50, "recent_three": 0.25,
                    "weakest": 0.15, "uniform_older": 0.10},
                "recover_buckets": {
                    str(i): list(family) for i, family in enumerate(
                        getattr(ENV_CLASSES[args.task],
                                "RECOVER_FAMILIES", ()))},
                "model_dr": False,  # v1 backend limit, see MJX_PORT.md
                "sim_model_source": params.source})
    # Same headline-score pinning as the C trainer (the periodic eval
    # is shared code, so MJX runs emit identical SCORE/* names).
    run.define_metric("SCORE/*", step_metric="global_step",
                      summary="last")
    run.define_metric("eval/*", step_metric="global_step")
    run.define_metric("CERT/*", step_metric="global_step", summary="last")
    run.define_metric("TRAIN/*", step_metric="global_step", summary="last")
    run.define_metric("RECOVER_SCORE/*", step_metric="global_step",
                      summary="last")
    print(f"[wandb] logging to {run.url or 'offline run dir'}")
    return run


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", choices=sorted(ENV_CLASSES),
                    default="joint_walk")
    ap.add_argument("--steps", type=int, default=5_000_000)
    ap.add_argument("--n-envs", type=int, default=1024)
    ap.add_argument("--impl", default="auto",
                    choices=["auto", "warp", "jax", "default"],
                    help="mjx backend; auto = warp if installed (GPU "
                         "pods), else the XLA default (laptop smoke)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--no-dr", action="store_true")
    ap.add_argument("--dr-scale", type=float, default=1.0)
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="SEC.KEY=VAL")
    ap.add_argument("--goal-mix", type=str, default=None,
                    help='e.g. "walk=0.7,rise=0.15" (goal tasks only)')
    ap.add_argument("--pool-per-env", type=int, default=2)
    ap.add_argument("--host-workers", type=int, default=0,
                    help="shard the per-env host halves across N worker "
                         "processes (MjxShardedVecEnv) — THE throughput "
                         "knob; size to the node's spare cores. 0 = "
                         "in-process reference implementation")
    ap.add_argument("--mjx-iterations", type=int, default=None,
                    help="solver iterations (default: 1 under warp — "
                         "parity-checked on-pod; 8 otherwise)")
    ap.add_argument("--mjx-ls-iterations", type=int, default=None)
    # Large-batch PPO knobs (defaults are STARTING points, unvalidated).
    ap.add_argument("--n-steps", type=int, default=16,
                    help="rollout length per env per update")
    ap.add_argument("--batch-size", type=int, default=8192)
    ap.add_argument("--gamma", type=float, default=None,
                    help="PPO discount. Default None = legacy exact: "
                         "0.99 on fresh/transplant constructors, the "
                         "checkpoint's own value on a plain --init-from "
                         "warm start. Long-horizon tasks (e.g. the "
                         "recover mode's 3-5 s recoveries) want 0.995.")
    ap.add_argument("--gae-lambda", type=float, default=None,
                    help="GAE lambda. Default None = legacy exact "
                         "(0.95 / checkpoint's own), same contract as "
                         "--gamma.")
    ap.add_argument("--n-epochs", type=int, default=5)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--ent-coef", type=float, default=1e-3)
    ap.add_argument("--target-kl", type=float, default=0.02)
    ap.add_argument("--log-std-init", type=float, default=-1.0)
    ap.add_argument("--net-arch", type=str, default="128,128",
                    help="MLP hidden sizes, comma-separated (from-"
                         "scratch and transplant builds only — a plain "
                         "--init-from warm start keeps the parent's "
                         "stored architecture). Flagship unified-policy "
                         "arms use 256,256 (RL_PLAN Architecture).")
    ap.add_argument("--gru", action="store_true",
                    help="recurrent GRU actor-critic (sb3-contrib "
                         "RecurrentPPO + gru_policy.py) instead of the "
                         "frame-stack MLP; run single-frame obs "
                         "(obs.history_frames=1). BPTT window = "
                         "--n-steps, so recurrent runs want longer "
                         "rollouts (e.g. --n-steps 64) than the MLP "
                         "large-batch default of 16. From-scratch or "
                         "GRU-parent warm starts only")
    ap.add_argument("--gru-hidden-size", type=int, default=128,
                    help="GRU hidden units per layer (actor and critic "
                         "each get their own single-layer GRU)")
    ap.add_argument("--gru-dual", action="store_true",
                    help="mode-gated dual-core GRU (gru_policy."
                         "DualGruActorCriticPolicy): separate locomotion"
                         " and stance cores+heads routed per tick by the"
                         " obs.mode_onehot tail — REQUIRES --cfg-set "
                         "obs.mode_onehot=1. Born from the cw-arch-gru-"
                         "anchor1..3 closure: one shared trunk cannot "
                         "hold anchored stance and a displacing walk at "
                         "once. Implies --gru")
    ap.add_argument("--gru-experts", action="store_true",
                    help="mode-gated FOUR-expert GRU (gru_policy."
                         "ModeExpertsGruActorCriticPolicy): fully "
                         "isolated rise/hold/lower/locomotion experts, "
                         "each with its own actor GRU, critic GRU, "
                         "heads and learnable log_std; only the active "
                         "expert's output/gradient is selected. "
                         "Operator directive fb_20260815T013349_488ffd. "
                         "Requires --cfg-set obs.mode_onehot=1. "
                         "Implies --gru; exclusive with --gru-dual")
    ap.add_argument("--gru-experts-adapter", type=int, default=0,
                    help="hidden width of the optional transition "
                         "adapter (zero-init residual MLP on the "
                         "selected action mean; 0 = no adapter module, "
                         "bit-exact expert-only path). Only with "
                         "--gru-experts")
    ap.add_argument("--gru-experts-adapter-scale", type=float,
                    default=0.05,
                    help="residual multiplier for the transition "
                         "adapter (default 0.05)")
    ap.add_argument("--gru-experts-freeze", action="store_true",
                    help="freeze all four expert ACTOR bodies (cores, "
                         "actor latents, action heads, per-expert "
                         "log_std); critics + transition adapter keep "
                         "training. Arm A stage 1 (frozen-expert "
                         "transition-adapter composition). Requires an "
                         "adapter on the policy")
    ap.add_argument("--transformer", action="store_true",
                    help="causal-transformer actor-critic (transformer_"
                         "policy.py) over the env-side frame stack: "
                         "attends over the obs.history_frames window "
                         "instead of flattening it into one MLP input. "
                         "REQUIRES --cfg-set obs.history_frames=K "
                         "(K>=2; the hist16 lineage uses 16). Stock "
                         "non-recurrent PPO — n_steps/batching identical "
                         "to the frame-stack MLP. From-scratch or "
                         "transformer-parent warm starts only")
    ap.add_argument("--tf-layers", type=int, default=2,
                    help="transformer encoder layers (per actor/critic)")
    ap.add_argument("--tf-width", type=int, default=128,
                    help="transformer d_model (token width)")
    ap.add_argument("--tf-heads", type=int, default=4,
                    help="attention heads")
    ap.add_argument("--tf-ff", type=int, default=256,
                    help="feed-forward hidden width inside each layer")
    ap.add_argument("--device", default="auto",
                    help="torch device for PPO (auto: cuda if available "
                         "— the big-batch MLP pays off on GPU)")
    ap.add_argument("--init-from", type=Path, default=None,
                    help="warm-start from a train_ppo_sim checkpoint "
                         "(same task/obs config)")
    ap.add_argument("--obs-pad-transplant", type=int, default=0,
                    help="warm-start across an obs WIDENING of N dims "
                         "appended at the obs tail (e.g. walk phase "
                         "clock, +2); zero-pads first-layer columns so "
                         "the parent policy is bit-identical until "
                         "training moves them (port of the "
                         "train_ppo_sim mechanism)")
    ap.add_argument("--eval-every", type=int, default=1_000_000,
                    help="background per-mode eval on a C-MuJoCo env every "
                         "N steps (0 = off). Doubles as a continuous "
                         "MJX-vs-C behavioral A/B — the eval env is real "
                         "C physics. Default is 5x the campaign's 200k "
                         "because MJX runs ~10x the steps/s.")
    ap.add_argument("--video-every", type=int, default=2_000_000,
                    help="background telemetry-overlay video reel every "
                         "N steps (0 = off); rendered on a C-MuJoCo env")
    ap.add_argument("--video-episodes", type=int, default=4)
    ap.add_argument("--recover-cert-every", type=int, default=1_000_000,
                    help="deterministic same-backend MJX recovery "
                         "certification every N training env-steps; this "
                         "alone controls monotonic recovery curriculum "
                         "admission (0 = off)")
    ap.add_argument("--recover-cert-envs", type=int, default=8,
                    help="parallel deterministic MJX episodes per active "
                         "recovery start kind")
    ap.add_argument("--recover-retention-buckets", type=int, default=3,
                    help="rotating previously unlocked buckets assayed at "
                         "each recovery certification; the current weakest "
                         "old bucket is assayed in addition (default: 3)")
    ap.add_argument("--no-canary", action="store_true",
                    help="disable the fixed-seed canary probes + "
                         "regression auto-stop (on by default for warm "
                         "starts, same as the campaign trainer)")
    ap.add_argument("--canary-stop-after", type=int, default=3,
                    help="consecutive full-group canary failures before "
                         "auto-stop (0 = monitor only)")
    ap.add_argument("--run-name", type=str, default=None)
    ap.add_argument("--notes", type=str, default=None)
    ap.add_argument("--out-name", type=str, default=None)
    ap.add_argument("--save-every", type=int, default=1_000_000,
                    help="checkpoint every N env-steps (0 = end only)")
    ap.add_argument("--no-wandb", action="store_true")
    ap.add_argument("--smoke", action="store_true",
                    help="tiny CPU run to validate the pipeline")
    args = ap.parse_args(argv)

    if not mjx_is_available():
        raise SystemExit("mujoco-mjx / jax not installed — "
                         "pip install -r rl_move/sim/requirements-mjx.txt")
    if args.smoke:
        args.n_envs = 4
        args.steps = 256
        args.n_steps = 8
        args.batch_size = 32
        args.episode_seconds = min(args.episode_seconds, 5.0)
        args.save_every = 0
        args.eval_every = 0
        args.video_every = 0
        args.recover_cert_every = 0
        args.device = "cpu"
    # Canaries ride the periodic C-env eval (campaign parity): on by
    # default for warm starts, since the failure class is a warm-started
    # run silently destroying a parent skill. Read by the bg-eval child.
    args.canary = (bool(args.init_from) and not args.no_canary
                   and args.eval_every > 0)

    impl = _resolve_impl(args.impl)
    iters = args.mjx_iterations if args.mjx_iterations is not None \
        else (1 if impl == "warp" else 8)
    ls_iters = args.mjx_ls_iterations if args.mjx_ls_iterations is not None \
        else (4 if impl == "warp" else 8)

    env_kw = _env_kwargs(args)      # resolves params via bus.servo_params
    params = env_kw["params"]
    _warn_if_defaults(params)
    env_cls = ENV_CLASSES[args.task]

    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import VecMonitor

    # Mirror-symmetry regularizer (cfg-gated, default off — see
    # rl_move/sim/mirror.py). Read via --cfg-set so respec --cfg can
    # tune it; the key also rides into the env cfg dict, harmlessly.
    mirror_coef = float(_parse_cfg_set(args.cfg_set).get(
        "train.mirror_loss_coef", 0.0) or 0.0)
    if mirror_coef > 0.0:
        from .mirror import attach_mirror, make_mirror_ppo_class
        algo_cls = make_mirror_ppo_class()
        print(f"[mjx-train] mirror symmetry loss ON (coef={mirror_coef})")
    else:
        algo_cls = PPO
    # Reference BC anchor (cfg-gated, default off — see
    # rl_move/sim/bc_anchor.py; rise lever (a), RL_PLAN queue 2a).
    # Composes with MirrorPPO if both are requested.
    bc_coef = float(_parse_cfg_set(args.cfg_set).get(
        "train.bc_anchor_coef", 0.0) or 0.0)
    if bc_coef > 0.0:
        from .bc_anchor import make_bc_anchor_ppo_class
        algo_cls = make_bc_anchor_ppo_class(algo_cls)
        print(f"[mjx-train] BC anchor loss ON (coef={bc_coef})")

    policy_cls: str | type = "MlpPolicy"
    extra_pk: dict = {}
    if args.gru_dual and args.gru_experts:
        raise SystemExit("--gru-dual and --gru-experts are exclusive")
    if args.gru_dual or args.gru_experts:
        args.gru = True
        if float(_parse_cfg_set(args.cfg_set).get(
                "obs.mode_onehot", 0.0)) <= 0.0:
            raise SystemExit(
                f"--gru-{'dual' if args.gru_dual else 'experts'} requires "
                "--cfg-set obs.mode_onehot=1 (the policy routes by the "
                "obs-tail skill one-hot)")
    if args.gru:
        if mirror_coef > 0.0:
            raise SystemExit("--gru + mirror loss is not implemented "
                             "(it wraps stock PPO)")
        if args.obs_pad_transplant:
            raise SystemExit("--gru + --obs-pad-transplant is not "
                             "implemented (recurrent weights don't "
                             "transplant from MLP checkpoints)")
        from sb3_contrib import RecurrentPPO
        from .gru_policy import (DualGruActorCriticPolicy,
                                 GruActorCriticPolicy,
                                 ModeExpertsGruActorCriticPolicy)
        algo_cls = RecurrentPPO
        if bc_coef > 0.0:
            # Recurrent BC anchor: pairs carry the rollout hidden state
            # and the aux step runs one GRU cell step from it (see
            # bc_anchor.py::_bc_policy_mean).
            from .bc_anchor import make_bc_anchor_ppo_class
            algo_cls = make_bc_anchor_ppo_class(RecurrentPPO)
        policy_cls = (ModeExpertsGruActorCriticPolicy if args.gru_experts
                      else DualGruActorCriticPolicy if args.gru_dual
                      else GruActorCriticPolicy)
        extra_pk = dict(lstm_hidden_size=args.gru_hidden_size)
        if args.gru_experts:
            extra_pk.update(
                experts_adapter_hidden=args.gru_experts_adapter,
                experts_adapter_scale=args.gru_experts_adapter_scale)
        cores_note = (" x4 isolated mode experts" if args.gru_experts
                      else " x2 mode-gated cores" if args.gru_dual
                      else "")
        print(f"[mjx-train] GRU policy: hidden {args.gru_hidden_size}"
              f"{cores_note}, "
              f"BPTT window = n_steps = {args.n_steps} "
              f"({args.n_steps / 25.0:.2f}s at 25 Hz)")
    if args.transformer:
        if args.gru:
            raise SystemExit("--transformer and --gru are mutually "
                             "exclusive (pick one memory mechanism)")
        if args.obs_pad_transplant:
            raise SystemExit("--transformer + --obs-pad-transplant is not "
                             "implemented (transformer weights don't "
                             "transplant from MLP checkpoints)")
        hist = int(float(_parse_cfg_set(args.cfg_set).get(
            "obs.history_frames", 1)))
        if hist < 2:
            raise SystemExit(
                "--transformer needs the env-side frame stack: add "
                "--cfg-set obs.history_frames=K (K>=2; hist16 lineage "
                "uses 16)")
        from .transformer_policy import TransformerActorCriticPolicy
        policy_cls = TransformerActorCriticPolicy
        extra_pk = dict(n_frames=hist, d_model=args.tf_width,
                        n_layers=args.tf_layers, n_heads=args.tf_heads,
                        ff_dim=args.tf_ff)
        print(f"[mjx-train] transformer policy: {args.tf_layers} layers, "
              f"d_model {args.tf_width}, {args.tf_heads} heads, "
              f"ff {args.tf_ff}, context {hist} frames "
              f"({hist / 25.0:.2f}s at 25 Hz), separate actor/critic")

    print(f"[mjx-train] task={args.task} n_envs={args.n_envs} "
          f"impl={impl or 'jax(default)'} iterations={iters}/{ls_iters} "
          f"host_workers={args.host_workers or 'in-process'} "
          f"rollout={args.n_envs * args.n_steps}/update "
          f"servo_params={Path(params.source).name}")
    t0 = time.monotonic()
    vec_kw = dict(env_kwargs=env_kw, seed=args.seed,
                  impl=impl, pool_per_env=args.pool_per_env,
                  mjx_iterations=iters, mjx_ls_iterations=ls_iters)
    if args.host_workers > 0:
        from .mjx_sharded_vec_env import MjxShardedVecEnv
        venv = MjxShardedVecEnv(env_cls, args.n_envs,
                                host_workers=args.host_workers, **vec_kw)
    else:
        from .mjx_vec_env import MjxVecEnv
        venv = MjxVecEnv(env_cls, args.n_envs, **vec_kw)
    gm = _parse_goal_mix(args.goal_mix)
    if gm:
        if not hasattr(env_cls, "set_goal_mix"):
            raise SystemExit(f"--goal-mix needs a goal task, not {args.task}")
        venv.env_method("set_goal_mix", gm)
    venv = VecMonitor(venv)
    print(f"[mjx-train] vec env up in {time.monotonic() - t0:.1f}s "
          f"(compile + {args.pool_per_env + 1} reset choreographies)")

    run = _init_wandb(args, params)

    # Checkpoint lineage via W&B artifacts (operator, 08-09): declare
    # the parent checkpoint as an input so the W&B artifact DAG shows
    # run/checkpoint ancestry. Best-effort — parents trained before
    # this feature have no artifact, and that must never block a run.
    if run is not None and args.init_from is not None:
        try:
            import wandb
            run.use_artifact(f"ckpt-{args.init_from.stem}:latest")
            print(f"[wandb] lineage: consumes ckpt-{args.init_from.stem}")
        except Exception:
            print(f"[wandb] no artifact for parent {args.init_from.stem} "
                  "(pre-artifact lineage) — continuing")

    tb_dir = None if run is None else str(POLICY_DIR / "tb")
    net_arch = [int(x) for x in str(args.net_arch).split(",") if x.strip()]
    if args.init_from is not None:
        if args.obs_pad_transplant:
            # Obs-widening warm start (port of train_ppo_sim's
            # --obs-pad-transplant): parent weights copy exactly, the
            # policy/value first layers gain N zero columns for the new
            # tail dims. Optimizer state is fresh (architecture changed).
            from .train_ppo_sim import pad_obs_transplant
            old = PPO.load(args.init_from, device="cpu")
            model = algo_cls(
                "MlpPolicy", venv,
                n_steps=args.n_steps, batch_size=args.batch_size,
                n_epochs=args.n_epochs, learning_rate=args.lr,
                gamma=(0.99 if args.gamma is None else args.gamma),
                gae_lambda=(0.95 if args.gae_lambda is None
                            else args.gae_lambda),
                ent_coef=args.ent_coef,
                clip_range=0.2,
                target_kl=(args.target_kl if args.target_kl > 0
                           else None),
                policy_kwargs=dict(net_arch=net_arch,
                                   log_std_init=args.log_std_init),
                seed=args.seed, verbose=1, device=args.device,
                tensorboard_log=tb_dir)
            pad_obs_transplant(old, model, args.obs_pad_transplant)
            model.num_timesteps = old.num_timesteps
            del old
            print(f"[mjx-train] warm start from {args.init_from} "
                  f"(+{args.obs_pad_transplant} obs-pad transplant)")
        else:
            from .gru_policy import is_recurrent_checkpoint
            if args.gru and not is_recurrent_checkpoint(args.init_from):
                raise SystemExit(
                    "--gru cannot warm-start from an MLP checkpoint "
                    f"({args.init_from}); GRU runs start from scratch "
                    "or from a previous GRU checkpoint")
            _ld_kw = {}
            if args.gamma is not None:
                _ld_kw["gamma"] = args.gamma
            if args.gae_lambda is not None:
                _ld_kw["gae_lambda"] = args.gae_lambda
            model = algo_cls.load(args.init_from, env=venv,
                                  device=args.device,
                             n_steps=args.n_steps,
                             batch_size=args.batch_size,
                             n_epochs=args.n_epochs, learning_rate=args.lr,
                             ent_coef=args.ent_coef, **_ld_kw,
                             target_kl=(args.target_kl or None),
                             tensorboard_log=tb_dir)
            # A plain --init-from warm start keeps the checkpoint's own
            # architecture. --net-arch used to be a hard error here, but
            # respec-cloned continuations legitimately carry the SAME
            # value the checkpoint was built with (08-11: every retry of
            # cw-uni-flag-a1-h1 crashed at 0 steps on this). Accept a
            # matching --net-arch; refuse only a genuine mismatch.
            if net_arch != [128, 128]:
                ck = getattr(model.policy, "net_arch", None)
                if isinstance(ck, dict):  # SB3 dict(pi=..., vf=...) form
                    ck = ck.get("pi", ck.get("vf"))
                if ck is not None and list(ck) != net_arch:
                    raise SystemExit(
                        f"--net-arch {net_arch} conflicts with the "
                        f"checkpoint's architecture {list(ck)} on a plain "
                        "--init-from warm start (the checkpoint carries "
                        "its own architecture); drop the flag or start "
                        "from scratch")
                print(f"[mjx-train] --net-arch {net_arch} matches the "
                      "warm-start checkpoint; proceeding")
            print(f"[mjx-train] warm start from {args.init_from}")
    else:
        model = algo_cls(
            policy_cls, venv,
            n_steps=args.n_steps, batch_size=args.batch_size,
            n_epochs=args.n_epochs, learning_rate=args.lr,
            gamma=(0.99 if args.gamma is None else args.gamma),
            gae_lambda=(0.95 if args.gae_lambda is None
                        else args.gae_lambda),
            ent_coef=args.ent_coef,
            clip_range=0.2,
            target_kl=(args.target_kl if args.target_kl > 0 else None),
            policy_kwargs=dict(net_arch=net_arch,
                               log_std_init=args.log_std_init,
                               **extra_pk),
            seed=args.seed, verbose=1, device=args.device,
            tensorboard_log=tb_dir)

    if args.gru_experts_freeze:
        from .gru_policy import ModeExpertsGruActorCriticPolicy as _MEP
        if not isinstance(model.policy, _MEP):
            raise SystemExit(
                "--gru-experts-freeze needs a ModeExperts policy "
                "(pass --gru-experts, or warm-start from an experts "
                "checkpoint)")
        if model.policy.experts_adapter is None:
            raise SystemExit(
                "--gru-experts-freeze with no transition adapter on "
                "the policy would train nothing actor-side; distill "
                "the init with an adapter or drop the freeze")
        model.policy.set_experts_frozen(True)
        print("[mjx-train] mode-experts ACTOR bodies FROZEN (cores, "
              "actor latents, heads, per-expert log_std); training "
              "only the transition adapter + critics")

    if mirror_coef > 0.0:
        attach_mirror(model, coef=mirror_coef, task=args.task,
                      cfg=env_kw.get("cfg"),
                      obs_dim=int(venv.observation_space.shape[0]))
    if bc_coef > 0.0:
        from .bc_anchor import attach_bc_anchor
        attach_bc_anchor(model, coef=bc_coef, cfg=env_kw.get("cfg"),
                         task=args.task)

    out_name = args.out_name or (
        f"ppo_mjx_{args.task}" + (f"_{args.run_name}" if args.run_name
                                  else ""))
    out_path = POLICY_DIR / f"{out_name}.zip"
    POLICY_DIR.mkdir(parents=True, exist_ok=True)

    class _Track(BaseCallback):
        """fps, ep stats, env/reward-part means (campaign parity, sampled
        across envs to stay cheap at B=4096), termination-reason counts,
        periodic checkpoints. Mirrors train_ppo_sim's
        _make_reward_parts_callback so MJX runs chart like C runs."""

        PART_KEYS = (
            "reward_task", "reward_roll", "reward_pitch", "reward_height",
            "reward_gyro", "reward_action", "reward_action_delta",
            "reward_current", "reward_unload", "reward_rise_progress",
            "reward_rise_milestone", "reward_rise_finish",
            "reward_curl_progress", "reward_curl_milestone", "reward_walk",
            "reward_walk_prog", "reward_swing", "reward_current_hot",
            "reward_stance", "reward_clearance", "reward_flag_leg",
            "reward_current_max", "reward_termination",
            "reward_phase_contact", "reward_support_margin",
            "reward_load_even", "reward_step_event", "reward_drag",
            "reward_park_duty", "reward_end_posture", "reward_effort",
            "reward_walk_yaw", "reward_quad_clear", "reward_quad_plant",
            # Gate factors (08-10 postgate1 dig-in: rise_posture_factor
            # was computed in-env but never logged, blinding triage to
            # the gate's live value).
            "rise_posture_factor", "rise_income_factor")
        AUX_ABS = ("roll_deg", "pitch_deg")       # logged as abs_<k>
        AUX = ("track_err_deg", "height_err_mm", "mean_current_a",
               "walk_vel_err", "walk_speed",
               "phase_agreement",
               "walk_anchor_frac",
               "walk_step_denied", "walk_step_bank_m",
               "walk_loadslip_ratio", "walk_loadslip_factor",
               "walk_yaw_err",
               "quad_clear_mm", "quad_fronts_off",
               "quad_planted_frac")  # own names
        # Any OTHER numeric scalar the env drops into info is logged
        # as a plain mean under env/<k> (operator 08-10: the whitelist
        # above kept drifting behind the envs — reward_rise_ref,
        # walk_wz, rise_plant_factor etc. were computed but invisible
        # in W&B, and W&B is the primary triage surface). The lists
        # stay for their special semantics (AUX* means abs()).
        SKIP = ("TimeLimit.truncated", "terminal_observation",
                "termination_reason", "goal_mode", "walk_bucket",
                "episode", "bc_target")
        SAMPLE = 256      # envs sampled per step for the means

        def __init__(self):
            super().__init__()
            self._t0 = time.monotonic()
            self._last_save = 0
            self._sum: dict[str, float] = {}
            self._cnt: dict[str, int] = {}
            self._terms: dict[str, int] = {}
            # Mode-experts active-tick accounting (directive
            # fb_20260815T013349_488ffd: report ACTIVE ticks per
            # expert, not just total env steps). Cumulative over the
            # whole run; indices follow gru_policy.EXPERTS_ORDER.
            self._exp_ticks = np.zeros(4, dtype=np.float64)
            # Joystick command telemetry (08-15, operator directive
            # fb_20260815T114414, SIMPLIFIED by fb_20260815T115650):
            # cumulative ACTIVE-TICK accounting of the env's
            # goal.walk_cmd_metrics info keys — raw signed v_along and
            # ratio-of-sums (NOT mean of per-tick ratios). No
            # per-heading bins in training: uniform [-pi,pi] heading
            # sampling + the signed average already zeroes out
            # command-ignorant motion; fixed-direction panels are
            # held-out EVAL tools. Zero-cost when the env never emits
            # the keys.
            self._cmd_cum = {"along": 0.0, "cmd": 0.0, "cross": 0.0,
                             "wrong": 0.0, "n": 0.0}
            self._cmd_stride = 1
            # Overall optimization-progress metric (operator feedback
            # fb_20260815T131225_c8442f, 08-15): "is PPO still getting
            # more total reward per real transition" — computed
            # directly from the raw per-step scalar rewards SB3 hands
            # the callback (self.locals["rewards"], the actual PPO
            # training signal, captured before the truncation-bootstrap
            # adjustment further down collect_rollouts), NOT from
            # ep_rew_mean/ep_len_mean (those distort under changing
            # episode length / partial episodes). Per-rollout sum+count
            # reset every _on_rollout_end; cumulative + EMA never reset.
            self._reward_sum = 0.0
            self._reward_n = 0
            self._reward_sum_cum = 0.0
            self._reward_n_cum = 0
            self._reward_ema: float | None = None
            # Exact completed-episode recovery scores. These inspect every
            # env terminal, not the 256-env telemetry sample above.
            self._recover_window: dict[int, list[int]] = {}
            self._recover_cumulative: dict[int, list[int]] = {}

        def _acc(self, k: str, v: float) -> None:
            self._sum[k] = self._sum.get(k, 0.0) + v
            self._cnt[k] = self._cnt.get(k, 0) + 1

        def _on_step(self) -> bool:
            rewards = self.locals.get("rewards")
            if rewards is not None:
                arr = np.asarray(rewards)
                self._reward_sum += float(arr.sum())
                self._reward_n += int(arr.size)
            if args.gru_experts:
                no = self.locals.get("new_obs")
                if no is not None and getattr(no, "ndim", 0) == 2 \
                        and no.shape[1] >= 6:
                    tail = no[:, -6:]
                    # EXPERTS_ORDER = (rise, hold, lower, loco);
                    # obs tail = (hold, rise, lower, walk, turn, quad)
                    self._exp_ticks[0] += float(tail[:, 1].sum())
                    self._exp_ticks[1] += float(tail[:, 0].sum())
                    self._exp_ticks[2] += float(tail[:, 2].sum())
                    self._exp_ticks[3] += float(tail[:, 3:].sum())
            infos = self.locals.get("infos", ())
            stride = max(1, len(infos) // self.SAMPLE)
            for info in infos[::stride]:
                for k in self.PART_KEYS:
                    if k in info:
                        self._acc(k, float(info[k]))
                for k in self.AUX_ABS:
                    if k in info:
                        self._acc(f"abs_{k}", abs(float(info[k])))
                for k in self.AUX:
                    if k in info:
                        self._acc(k, abs(float(info[k])))
                for k, v in info.items():
                    if (k in self.PART_KEYS or k in self.AUX
                            or k in self.AUX_ABS or k in self.SKIP):
                        continue
                    if isinstance(v, bool) or not isinstance(
                            v, (int, float, np.integer, np.floating)):
                        continue
                    self._acc(k, float(v))
                if "track_err_deg" in info:
                    self._acc("pct_within_1deg",
                              1.0 if float(info["track_err_deg"]) <= 1.0
                              else 0.0)
                if "v_along_cmd_m_s" in info:
                    # Cumulative active-tick command telemetry (see
                    # __init__); sums, so ratios come out as
                    # sum(v_along)/sum(cmd_speed), never mean-of-ratios.
                    self._cmd_stride = stride
                    al = float(info["v_along_cmd_m_s"])
                    self._cmd_cum["along"] += al
                    self._cmd_cum["cmd"] += float(
                        info.get("cmd_speed_m_s", 0.0))
                    self._cmd_cum["cross"] += float(
                        info.get("v_cross_abs_m_s", 0.0))
                    self._cmd_cum["wrong"] += float(
                        info.get("wrong_way", 0.0))
                    self._cmd_cum["n"] += 1.0
            dones = self.locals.get("dones")
            if dones is not None and np.any(dones):
                for i in np.flatnonzero(dones):
                    outcome = _recover_episode_outcome(infos[i])
                    if outcome is not None:
                        bucket, success = outcome
                        for bank in (self._recover_window,
                                     self._recover_cumulative):
                            counts = bank.setdefault(bucket, [0, 0])
                            counts[0] += int(success)
                            counts[1] += 1
                    r = infos[i].get("termination_reason") or (
                        "truncated"
                        if infos[i].get("TimeLimit.truncated")
                        else "done")
                    self._terms[r] = self._terms.get(r, 0) + 1
            return True

        def _on_rollout_end(self) -> None:
            fps = self.num_timesteps / max(time.monotonic() - self._t0,
                                           1e-9)
            payload = {"time/env_steps_per_s": fps,
                       "time/total_env_steps": self.num_timesteps,
                       "global_step": self.num_timesteps}
            payload.update({f"env/{k}": self._sum[k] / self._cnt[k]
                            for k in self._sum})
            payload.update({f"terminations/{k}": v
                            for k, v in self._terms.items()})
            for bucket, (successes, episodes) in sorted(
                    self._recover_window.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_success_fraction"] = successes / episodes
                payload[f"{stem}_successes"] = successes
                payload[f"{stem}_episodes"] = episodes
            for bucket, (successes, episodes) in sorted(
                    self._recover_cumulative.items()):
                stem = f"TRAIN/recover_bucket_{bucket}"
                payload[f"{stem}_success_fraction_cumulative"] = (
                    successes / episodes)
                payload[f"{stem}_episodes_cumulative"] = episodes
            if self._reward_n > 0:
                # optimization/* (fb_20260815T131225_c8442f): "is PPO
                # continuing to get more total reward per real
                # transition" — an OPTIMIZATION/objective score, not a
                # behavioral-success claim; read it beside the task
                # (joystick/v_along_m_s) and safety (terminations/*)
                # metrics, never alone. reward/tick rising + task
                # rising = useful learning; reward/tick rising + task
                # falling = exploiting/prioritizing a different reward
                # term; reward/tick flat = optimization stalled.
                rpt = self._reward_sum / self._reward_n
                payload["optimization/reward_per_tick"] = rpt
                self._reward_sum_cum += self._reward_sum
                self._reward_n_cum += self._reward_n
                payload["optimization/reward_per_tick_cumulative"] = (
                    self._reward_sum_cum / self._reward_n_cum)
                self._reward_ema = (rpt if self._reward_ema is None else
                                    0.9 * self._reward_ema + 0.1 * rpt)
                payload["optimization/reward_per_tick_ema"] = (
                    self._reward_ema)
            self._reward_sum, self._reward_n = 0.0, 0
            if self._cmd_cum["n"] > 0:
                # Operator-named joystick command-following metrics
                # (fb_20260815T114414, simplified fb_20260815T115650).
                # HEADLINE (joystick/*): raw signed m/s along the
                # requested direction over active ticks — per-rollout
                # mean, active-tick-weighted cumulative mean, and the
                # active-tick audit count. Everything else (cross-track,
                # wrong-way, ratio-of-sums) is secondary under train/.
                # NO per-heading series here — fixed-direction checks
                # live in held-out EVAL only.
                c = self._cmd_cum
                n_r = self._cnt.get("v_along_cmd_m_s", 0)
                if n_r:
                    s_al = self._sum["v_along_cmd_m_s"]
                    s_cmd = self._sum.get("cmd_speed_m_s", 0.0)
                    payload["joystick/v_along_m_s"] = s_al / n_r
                    payload["train/cmd_speed_active_m_s"] = (
                        s_cmd / max(self._cnt.get("cmd_speed_m_s", 1), 1))
                    if s_cmd > 0.0:
                        payload["train/v_along_ratio_active"] = (
                            s_al / s_cmd)
                payload["joystick/v_along_m_s_cumulative"] = (
                    c["along"] / c["n"])
                if c["cmd"] > 0.0:
                    payload["train/v_along_ratio_active_cumulative"] = (
                        c["along"] / c["cmd"])
                payload["train/v_cross_abs_m_s"] = c["cross"] / c["n"]
                payload["train/wrong_way_frac"] = c["wrong"] / c["n"]
                # Estimate: sampled count x sample stride (the info
                # sweep reads every stride-th env).
                payload["joystick/active_ticks"] = (
                    c["n"] * self._cmd_stride)
            if args.gru_experts:
                from .gru_policy import EXPERTS_ORDER
                tot = max(float(self._exp_ticks.sum()), 1.0)
                for i, name in enumerate(EXPERTS_ORDER):
                    payload[f"experts/active_ticks_{name}"] = float(
                        self._exp_ticks[i])
                    payload[f"experts/tick_frac_{name}"] = float(
                        self._exp_ticks[i]) / tot
                pol = self.model.policy
                if hasattr(pol, "_log_stds"):
                    for name, ls in zip(EXPERTS_ORDER,
                                        pol._log_stds()):
                        payload[f"experts/std_{name}"] = float(
                            ls.detach().exp().mean())
            buf = self.model.ep_info_buffer
            if buf:
                payload["rollout/ep_rew_mean"] = float(
                    np.mean([e["r"] for e in buf]))
                payload["rollout/ep_len_mean"] = float(
                    np.mean([e["l"] for e in buf]))
            if run is not None:
                import wandb
                wandb.log(payload)
            self._sum, self._cnt, self._terms = {}, {}, {}
            self._recover_window = {}
            if (args.save_every and self.num_timesteps - self._last_save
                    >= args.save_every):
                self._last_save = self.num_timesteps
                self.model.save(out_path)
                print(f"[mjx-train] checkpoint @ {self.num_timesteps:,} "
                      f"-> {out_path} ({fps:,.0f} env-steps/s)")


    callbacks: list = [_Track()]
    if bc_coef > 0.0:
        from .bc_anchor import make_bc_collect_callback
        callbacks.append(make_bc_collect_callback())

    cert_cb = None
    if (args.recover_cert_every > 0 and args.recover_cert_envs > 0
            and float(gm.get("recover", 0.0)) > 0.0):
        class _MjxRecoverCert(BaseCallback):
            """Deterministic curriculum authority on training physics."""

            def __init__(self):
                super().__init__()
                self._next = int(args.recover_cert_every)
                self._env = None
                self._retention_cursor = 0
                self._cert_round = 0
                self._last_cert_round: dict[int, int] = {}
                self._best_score = 0.0

            def _build(self):
                cert_kw = dict(vec_kw)
                cert_kw.update(
                    seed=args.seed + 515151,
                    pool_per_env=max(2, args.pool_per_env),
                    desync_episodes=False)
                if args.host_workers > 0:
                    from .mjx_sharded_vec_env import MjxShardedVecEnv
                    workers = min(args.recover_cert_envs,
                                  max(1, args.host_workers))
                    env = MjxShardedVecEnv(
                        env_cls, args.recover_cert_envs,
                        host_workers=workers, **cert_kw)
                else:
                    from .mjx_vec_env import MjxVecEnv
                    env = MjxVecEnv(
                        env_cls, args.recover_cert_envs, **cert_kw)
                env.env_method("set_goal_mix", gm)
                print("[recover-cert] deterministic MJX pool ready: "
                      f"{args.recover_cert_envs} envs, "
                      f"{impl or 'jax(default)'} backend")
                return env

            def _on_step(self) -> bool:
                return True

            def _on_rollout_end(self) -> None:
                if self.num_timesteps < self._next:
                    return
                self._next = ((self.num_timesteps
                               // args.recover_cert_every) + 1
                              ) * args.recover_cert_every
                if self._env is None:
                    self._env = self._build()
                active_before = int(venv.get_attr(
                    "_rec_active_n", indices=0)[0])
                frontier_before = int(venv.get_attr(
                    "_rec_focus_bucket", indices=0)[0])
                weak = venv.get_attr(
                    "_rec_weak_bucket", indices=0)[0]
                buckets, self._retention_cursor = (
                    _recover_cert_bucket_plan(
                        frontier_before, args.recover_retention_buckets,
                        self._retention_cursor, weak))
                rows_by_bucket = {}
                self._cert_round += 1
                for bucket in buckets:
                    kinds = venv.env_method(
                        "_recover_family_kinds", bucket, indices=0)[0]
                    rows = []
                    for kind in kinds:
                        row = _run_recover_cert_kind(
                            self._env, self.model, kind)
                        rows.append(row)
                        venv.env_method(
                            "apply_recover_certification", kind,
                            row["outcomes"], False)
                    rows_by_bucket[bucket] = rows
                    self._last_cert_round[bucket] = self._cert_round
                # Apply all kind and retention results before changing the
                # frontier. This keeps multi-kind promotion atomic and makes
                # the score/sampler consume one coherent assay round.
                venv.env_method("_recover_update_admission")
                active_after = int(venv.get_attr(
                    "_rec_active_n", indices=0)[0])
                payload = {
                    "global_step": self.num_timesteps,
                    "CERT/recover_frontier_before": frontier_before,
                    "CERT/recover_frontier_after": active_after - 1,
                    "CERT/recover_max_unlocked_before": active_before - 1,
                    "CERT/recover_max_unlocked_after": active_after - 1,
                    "CERT/recover_assayed_bucket_count": len(buckets),
                }
                for bucket, rows in rows_by_bucket.items():
                    successes = sum(r["successes"] for r in rows)
                    episodes = sum(r["episodes"] for r in rows)
                    stem = f"CERT/recover_bucket_{bucket}"
                    payload[f"{stem}_success_fraction"] = (
                        successes / max(episodes, 1))
                    payload[f"{stem}_gate_fraction"] = min(
                        r["success"] for r in rows)
                    payload[f"{stem}_successes"] = successes
                    payload[f"{stem}_episodes"] = episodes
                    for row in rows:
                        kind = row["kind"]
                        payload[f"CERT/recover_{kind}_success_fraction"] = (
                            row["success"])
                        payload[f"CERT/recover_{kind}_successes"] = (
                            row["successes"])
                        payload[f"CERT/recover_{kind}_episodes"] = (
                            row["episodes"])
                        payload[f"CERT/recover_{kind}_time_s"] = (
                            row["time_mean_s"])
                score_state = venv.env_method(
                    "recover_score_state", indices=0)[0]
                ages = {
                    bucket: self._cert_round - last_round
                    for bucket, last_round in self._last_cert_round.items()
                }
                score_payload, self._best_score = _recover_score_payload(
                    score_state, self._best_score, ages)
                payload.update(score_payload)
                if run is not None:
                    import wandb
                    wandb.log(payload)
                bits = " | ".join(
                    f"B{bucket} " + " ".join(
                        f"{r['kind']}={r['successes']}/{r['episodes']}"
                        for r in rows)
                    for bucket, rows in rows_by_bucket.items())
                score_points = score_payload[
                    "RECOVER_SCORE/overall_points"]
                score_max = score_payload["RECOVER_SCORE/maximum_points"]
                print(f"[recover-cert] step {self.num_timesteps:,} "
                      f"{bits}; frontier "
                      f"B{frontier_before}->B{active_after - 1}; "
                      f"score={score_points:.1f}/{score_max:.0f}")

            def close(self):
                if self._env is not None:
                    self._env.close()
                    self._env = None

            def _on_training_end(self) -> None:
                self.close()

        cert_cb = _MjxRecoverCert()
        callbacks.append(cert_cb)
        print("[recover-cert] armed: deterministic MJX certification "
              f"every {args.recover_cert_every:,} steps, "
              f"{args.recover_cert_envs} episodes/kind, frontier + weakest "
              f"+ {args.recover_retention_buckets} rotating retention "
              "buckets")
    bg = None
    if run is not None and (args.eval_every > 0 or args.video_every > 0):
        # The campaign's background eval/video worker, reused verbatim:
        # a spawn process holding C-MuJoCo envs. Every periodic eval is
        # therefore an MJX-vs-C behavioral A/B on the live checkpoint.
        from .train_ppo_sim import (
            _ActFn, _BgEval, _build_env, _make_canary_stop_callback,
            _make_periodic_eval_callback, _make_video_callback,
            _protected_groups, _run_canaries)
        canary_protected: list[str] = []
        if args.canary:
            # Baseline on the PARENT policy (C env, fixed seeds): groups
            # the parent passes 2/2 become the protected set.
            cenv = _build_env(env_cls, params, args, seed=args.seed + 77777)
            act0 = _ActFn(model.policy)
            baseline = _run_canaries(cenv, act0)
            cenv.close()
            canary_protected = _protected_groups(baseline)
            print("[canary] parent baseline: "
                  + " ".join(f"{c}={int(v)}" for c, v in baseline.items()))
            print(f"[canary] protected groups (parent passed 2/2): "
                  f"{canary_protected or 'none'}")
            run.config.update({
                "canary_baseline": {k: int(v) for k, v in baseline.items()},
                "canary_protected": canary_protected,
                "canary_stop_after": args.canary_stop_after,
            }, allow_val_change=True)
        bg = _BgEval(args.task, args)
        if args.eval_every > 0:
            callbacks.append(_make_periodic_eval_callback(
                bg, every=args.eval_every))
            if args.canary and canary_protected:
                callbacks.append(_make_canary_stop_callback(
                    bg, canary_protected,
                    stop_after=args.canary_stop_after))
                print("[canary] regression auto-stop armed "
                      f"(stop after {args.canary_stop_after} consecutive "
                      "full-group failures)")
        if args.video_every > 0:
            callbacks.append(_make_video_callback(bg, args.video_every,
                                                  args))
    try:
        model.learn(total_timesteps=args.steps, callback=callbacks,
                    progress_bar=False)
    finally:
        if cert_cb is not None:
            cert_cb.close()
        if bg is not None:
            bg.shutdown()
    model.save(out_path)
    dt = time.monotonic() - t0
    print(f"[mjx-train] done: {args.steps:,} steps in {dt:.0f}s "
          f"({args.steps / dt:,.0f} env-steps/s incl. setup) -> {out_path}")
    print("[mjx-train] evaluate with the C-env harness before trusting "
          "anything (MJX_PORT.md phase-2 item 4).")
    if run is not None:
        # Publish the final checkpoint as a W&B artifact so every future
        # warm start (use_artifact above) links into the lineage DAG.
        try:
            import hashlib
            import wandb
            md5 = hashlib.md5(out_path.read_bytes()).hexdigest()[:8]
            art = wandb.Artifact(
                f"ckpt-{out_name}", type="policy-checkpoint",
                metadata={"run": args.run_name, "md5": md5,
                          "steps": args.steps, "task": args.task,
                          "parent_ckpt": (args.init_from.stem
                                          if args.init_from else None)})
            art.add_file(str(out_path))
            run.log_artifact(art, aliases=["latest", args.run_name])
            print(f"[wandb] checkpoint artifact ckpt-{out_name} (md5 {md5})")
        except Exception as ex:
            print(f"[wandb] artifact publish failed (non-fatal): {ex}")
        run.finish()
    venv.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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
- No eval/video worker, no canary, no lineage stitching — this trainer
  is for physics-throughput and recipe experiments. Evaluate its
  checkpoints with the normal C-env harness (that IS the behavioral
  A/B, phase-2 item 4).
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


def _env_kwargs(args, params: SimServoParams | None = None) -> dict:
    """Per-shim-env kwargs — mirrors train_ppo_sim._build_env, minus the
    model-DR pieces the shared-model backend can't honor yet.

    ``params=None`` resolves the actuator set from the run's cfg
    (bus.servo_params: "" = air fit, "loaded" = loaded bench fit)."""
    kw = dict(randomize=not args.no_dr,
              dr_scale=args.dr_scale,
              episode_seconds=args.episode_seconds)
    overrides = _parse_cfg_set(args.cfg_set)
    if overrides:
        from rl_move.config import load_config
        cfg = load_config()
        for dotted, val in overrides.items():
            node = cfg
            *path, leaf = dotted.split(".")
            for k in path:
                node = node.setdefault(k, {})
            node[leaf] = val
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
        entity=WANDB_ENTITY_DEFAULT, project=WANDB_PROJECT_DEFAULT,
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
                "model_dr": False,  # v1 backend limit, see MJX_PORT.md
                "sim_model_source": params.source})
    # Same headline-score pinning as the C trainer (the periodic eval
    # is shared code, so MJX runs emit identical SCORE/* names).
    run.define_metric("SCORE/*", step_metric="global_step",
                      summary="last")
    run.define_metric("eval/*", step_metric="global_step")
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
    ap.add_argument("--n-epochs", type=int, default=5)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--ent-coef", type=float, default=1e-3)
    ap.add_argument("--target-kl", type=float, default=0.02)
    ap.add_argument("--log-std-init", type=float, default=-1.0)
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
                gamma=0.99, gae_lambda=0.95, ent_coef=args.ent_coef,
                clip_range=0.2,
                target_kl=(args.target_kl if args.target_kl > 0
                           else None),
                policy_kwargs=dict(net_arch=[128, 128],
                                   log_std_init=args.log_std_init),
                seed=args.seed, verbose=1, device=args.device,
                tensorboard_log=tb_dir)
            pad_obs_transplant(old, model, args.obs_pad_transplant)
            model.num_timesteps = old.num_timesteps
            del old
            print(f"[mjx-train] warm start from {args.init_from} "
                  f"(+{args.obs_pad_transplant} obs-pad transplant)")
        else:
            model = algo_cls.load(args.init_from, env=venv,
                                  device=args.device,
                             n_steps=args.n_steps,
                             batch_size=args.batch_size,
                             n_epochs=args.n_epochs, learning_rate=args.lr,
                             ent_coef=args.ent_coef,
                             target_kl=(args.target_kl or None),
                             tensorboard_log=tb_dir)
            print(f"[mjx-train] warm start from {args.init_from}")
    else:
        model = algo_cls(
            "MlpPolicy", venv,
            n_steps=args.n_steps, batch_size=args.batch_size,
            n_epochs=args.n_epochs, learning_rate=args.lr,
            gamma=0.99, gae_lambda=0.95, ent_coef=args.ent_coef,
            clip_range=0.2,
            target_kl=(args.target_kl if args.target_kl > 0 else None),
            policy_kwargs=dict(net_arch=[128, 128],
                               log_std_init=args.log_std_init),
            seed=args.seed, verbose=1, device=args.device,
            tensorboard_log=tb_dir)

    if mirror_coef > 0.0:
        attach_mirror(model, coef=mirror_coef, task=args.task,
                      cfg=env_kw.get("cfg"),
                      obs_dim=int(venv.observation_space.shape[0]))

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
                "episode")
        SAMPLE = 256      # envs sampled per step for the means

        def __init__(self):
            super().__init__()
            self._t0 = time.monotonic()
            self._last_save = 0
            self._sum: dict[str, float] = {}
            self._cnt: dict[str, int] = {}
            self._terms: dict[str, int] = {}

        def _acc(self, k: str, v: float) -> None:
            self._sum[k] = self._sum.get(k, 0.0) + v
            self._cnt[k] = self._cnt.get(k, 0) + 1

        def _on_step(self) -> bool:
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
            dones = self.locals.get("dones")
            if dones is not None and np.any(dones):
                for i in np.flatnonzero(dones):
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
            if (args.save_every and self.num_timesteps - self._last_save
                    >= args.save_every):
                self._last_save = self.num_timesteps
                self.model.save(out_path)
                print(f"[mjx-train] checkpoint @ {self.num_timesteps:,} "
                      f"-> {out_path} ({fps:,.0f} env-steps/s)")


    callbacks: list = [_Track()]
    bg = None
    if run is not None and (args.eval_every > 0 or args.video_every > 0):
        # The campaign's background eval/video worker, reused verbatim:
        # a spawn process holding C-MuJoCo envs. Every periodic eval is
        # therefore an MJX-vs-C behavioral A/B on the live checkpoint.
        from .train_ppo_sim import (
            _BgEval, _build_env, _make_canary_stop_callback,
            _make_periodic_eval_callback, _make_video_callback,
            _protected_groups, _run_canaries)
        canary_protected: list[str] = []
        if args.canary:
            # Baseline on the PARENT policy (C env, fixed seeds): groups
            # the parent passes 2/2 become the protected set.
            cenv = _build_env(env_cls, params, args, seed=args.seed + 77777)
            act0 = lambda o: model.policy.predict(  # noqa: E731
                o, deterministic=True)[0]
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
    model.learn(total_timesteps=args.steps, callback=callbacks,
                progress_bar=False)
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

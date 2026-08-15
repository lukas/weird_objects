"""train_ppo_transfer.py — the A/B/C representation-transfer experiment.

Three PPO conditions, identical env / reward / budget / seeds; the ONLY
variable is where the policy's representation comes from
(rl_docs/DYNREP.md "First experimental comparison"):

    A  scratch    MlpPolicy on the raw stacked obs (H=16 frames)
    B  frozen     pretrained dyn encoder (dyn_v2_obs) -> frozen z ->
                  policy/value heads learn
    C  anchored   as B, but the encoder fine-tunes at a scaled-down LR
                  AND keeps training on the predictive objective: after
                  every rollout a few dynamics-loss steps on the
                  OFFLINE pretraining dataset anchor the encoder
                  (v1 approximation of L_total = L_PPO + lambda*L_dyn;
                  an online-window anchor is a documented follow-up).

Local pilot tasks (Mac-scale budgets; the brief's walk/yaw tasks need
pod-scale steps): "hold" (quiet plant stance) then "lower" (controlled
descent to belly = return toward the zero pose). Pod-scale transfer
task (08-13 operator directive — lower is too close to hold to
discriminate acquisition speed): "walk" (commanded velocity tracking,
the campaign walk goal mode; runner: pod_holdwalk.sh). Transfer protocol:
train task 1, checkpoint, warm-start task 2 with --init-from; the eval
callback measures BOTH tasks at every eval so retention curves come
for free.

    ../../.venv/bin/python -m rl_move.dynamics.train_ppo_transfer \
        --condition B --task hold --steps 150000 --seed 0 \
        --name pilot_hold_B

Outputs: models/ppo_<name>.zip + logs/ppo_<name>_eval.csv
(step, per-task mean return / episode length / early-termination rate).
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config                  # noqa: E402
from rl_move.sim.servo_model import SimServoParams      # noqa: E402

MODEL_DIR = ROOT / "rl_move" / "dynamics" / "models"
LOG_DIR = ROOT / "rl_move" / "dynamics" / "logs"
DEFAULT_ENCODER = "rl_move/dynamics/models/dyn_v2_obs.pt"
WANDB_ENV_FILE = ROOT / "rl_move" / "sim" / "wandb.env"
WANDB_ENTITY_DEFAULT = "l2k2"
WANDB_PROJECT_DEFAULT = "hexapod-balance"

HISTORY = 16
FRAME_WIDTH = 72          # walk-env frame: 59 proprio + 13 goal/cmd
TASKS = ("hold", "lower", "walk", "rise")
TASK_GOALS = {"hold": {"hold": 1.0}, "lower": {"lower": 1.0},
              # walk (08-13, operator directive: hold->walk is the pod
              # transfer pair — lower is too close to hold to
              # discriminate). p_walk=1.0 pins every episode to the
              # walk goal mode; the goal gen samples velocity commands
              # exactly as the campaign's eval harnesses do
              # (eval_cmd_suite/drive_policy set gen.p_walk = 1.0).
              "walk": {"walk": 1.0},
              # rise (08-14, operator next-steps: hard rise-retention
              # canary — the measured failure mode is DAgger rise
              # competence erased by PPO walk training). Belly/bridge
              # starts per the campaign rise goal mode.
              "rise": {"rise": 1.0}}
ALL_MODES = ("hold", "lean", "track", "unload", "raise", "rise",
             "lower", "quad", "walk")

# Held-out dynamics suites (operator next-steps: robustness to
# actuator/model mismatch is the sim-to-real reason the representation
# exists). Each entry is (name, dr_scale, dr-overrides) — overrides use
# the campaign's cfg dr.<field> mechanism, which pins the randomizer
# RANGES post-scaling, so dr_scale=0 + one override = one isolated
# held-out axis exactly like the cw-walk-latjit25/deadband30/velsag30
# ledger runs. Values sit OUTSIDE the training DR envelope at the
# pilot's --dr-scale 0.3 (e.g. latency trains at ~1±0.3*(0.3,0.8)).
HELDOUT_SUITES = (
    ("dr10", 1.0, None),                                # broad unseen DR
    ("lat2x", 0.0, {"latency_scale": "2.0,2.0"}),       # slow bus
    ("vel07", 0.0, {"vel_scale": "0.7,0.7"}),           # servo speed sag
    ("db25", 0.0, {"deadband_scale": "2.5,2.5"}),       # worn deadband
    ("tq07", 0.0, {"torque_scale": "0.7,0.7"}),         # battery sag
)


def _load_wandb_env() -> None:
    if not WANDB_ENV_FILE.is_file():
        return
    for raw in WANDB_ENV_FILE.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        if key.strip() and value.strip():
            os.environ.setdefault(key.strip(), value.strip())


def _init_wandb(args):
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except ImportError as exc:
        raise RuntimeError(
            "W&B tracking is required; install wandb or pass --no-wandb"
        ) from exc
    has_key = bool(os.environ.get("WANDB_API_KEY"))
    if not has_key:
        try:
            has_key = bool(wandb.api.api_key)
        except Exception:
            has_key = False
    if not has_key:
        raise RuntimeError(
            f"W&B tracking is required but no key was found in {WANDB_ENV_FILE}"
        )

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    # W&B display name gets a per-attempt UTC stamp (operator 08-15:
    # crash-relaunches were creating W&B runs with the EXACT same name,
    # e.g. two rw_rise_C_s6). Files/checkpoints keep the bare args.name
    # so --init-from chains and eval CSV paths are unaffected.
    attempt = time.strftime("%m%d-%H%MZ", time.gmtime())
    condition_blurb = {
        "A": "A=scratch (MlpPolicy on raw stacked obs, no encoder)",
        "B": "B=frozen pretrained dyn encoder, policy/value heads learn",
        "C": ("C=anchored: encoder fine-tunes at scaled-down LR + "
              "offline dynamics-loss anchor steps after every rollout"),
    }.get(args.condition.upper(), f"condition {args.condition}")
    run = wandb.init(
        entity=os.environ.get("WANDB_ENTITY", WANDB_ENTITY_DEFAULT),
        project=os.environ.get("WANDB_PROJECT", WANDB_PROJECT_DEFAULT),
        dir=str(LOG_DIR),
        group="dynrep-transfer",
        job_type=f"condition-{args.condition.lower()}-{args.task}",
        name=f"{args.name}.{attempt}",
        tags=[f"cond-{args.condition.upper()}", f"task-{args.task}",
              f"seed-{args.seed}", f"base-{args.name}"],
        notes=(
            f"dynrep A/B/C representation-transfer PPO — {condition_blurb}. "
            f"task={args.task} seed={args.seed} steps={args.steps} "
            f"encoder={os.path.basename(args.encoder) if args.encoder else 'none'} "
            + (f"warm-started from {os.path.basename(args.init_from)} "
               if args.init_from else "from scratch ")
            + f"host={os.uname().nodename}. Attempt {attempt} of base run "
            f"{args.name} (re-runs of the same condition/seed get a new "
            f"attempt stamp instead of a duplicate name)."
        ),
        config={
            "trainer": "train_ppo_transfer",
            "condition": args.condition,
            "task": args.task,
            "steps": args.steps,
            "seed": args.seed,
            "n_envs": args.n_envs,
            "dr_scale": args.dr_scale,
            "episode_seconds": args.episode_seconds,
            "encoder": args.encoder,
            "encoder_lr_scale": args.encoder_lr_scale,
            "anchor_data": args.anchor_data,
            "anchor_batches": args.anchor_batches,
            "anchor_batch_size": args.anchor_batch_size,
            "init_from": args.init_from,
            "eval_every": args.eval_every,
            "eval_tasks": args.eval_tasks,
            "eval_heldout": args.eval_heldout,
        },
        sync_tensorboard=True,
    )
    run.define_metric("eval/*", step_metric="global_step", summary="last")
    run.define_metric("anchor/*", step_metric="global_step", summary="last")
    metadata = {"id": run.id, "url": run.url, "name": args.name,
                "wandb_name": f"{args.name}.{attempt}"}
    (LOG_DIR / f"ppo_{args.name}_wandb.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(f"[wandb] logging to {run.url}", flush=True)
    return run


def make_task_env(task: str, seed: int, dr_scale: float,
                  episode_seconds: float,
                  dr_overrides: dict | None = None):
    """One walk-family env pinned to a single goal mode. Uses the walk
    env class for every task so obs width (72) and checkpoints are
    interchangeable across tasks."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    cfg.setdefault("obs", {})["history_frames"] = HISTORY
    if dr_overrides:
        cfg["dr"] = {**(cfg.get("dr") or {}), **dr_overrides}
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None),
        randomize=(dr_scale > 0.0 or bool(dr_overrides)),
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        cfg=cfg)
    gen = env._goal_gen
    for mode in ALL_MODES:
        if hasattr(gen, f"p_{mode}"):
            setattr(gen, f"p_{mode}",
                    float(TASK_GOALS[task].get(mode, 0.0)))
    return env


def _term_penalty_wrapper(env, penalty: float):
    """TRAINING-ONLY shaping: subtract a one-time penalty on early
    termination (tip/trip). The pilot's hold task has no alive bonus,
    so at small budgets every condition discovered that tipping over
    ~35 ticks in beats holding badly (measured 08-12: return -228 ->
    ~+1 by suicide). The terminal charge removes that escape without
    adding per-tick income. Evals run the RAW env so metrics stay
    comparable across phases and with the 08-12 pilot."""
    import gymnasium as gym

    class TermPenaltyWrapper(gym.Wrapper):
        def step(self, action):
            obs, r, term, trunc, info = self.env.step(action)
            if term and not trunc:
                r = float(r) - penalty
            return obs, r, term, trunc, info

    return TermPenaltyWrapper(env)


def _env_factory(task: str, seed: int, dr_scale: float,
                 episode_seconds: float, term_penalty: float):
    def _make():
        env = make_task_env(task, seed, dr_scale, episode_seconds)
        if term_penalty > 0.0:
            env = _term_penalty_wrapper(env, term_penalty)
        return env
    return _make


# Gait/transition-quality metric keys appended per task at every eval
# point (operator next-steps: "measure transition quality and gait
# quality, not only return"; a representation that merely learns the
# sliding exploit faster is not a success).
QUALITY_KEYS = ("peak_roll_deg", "peak_pitch_deg", "peak_gyro_dps",
                "slip_m", "fwd_m", "contact_sw_per_s", "slew_sat",
                "slew_sat_all", "mean_h_m", "dh_m", "vx_rmse")


def _foot_site_ids(env) -> list[int]:
    import mujoco
    return [mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE,
                              f"L{i}_foot_site") for i in range(6)]


def eval_task(model, task: str, episodes: int, dr_scale: float,
              episode_seconds: float, seed0: int = 10_000,
              dr_overrides: dict | None = None) -> dict:
    """Deterministic episodes on fixed seeds -> mean return / length /
    early-termination (fall/trip) rate + physical quality metrics:

        peak_roll/pitch_deg   worst |tilt| vs the episode tilt ref
        peak_gyro_dps         worst |roll/pitch rate|
        slip_m                summed horizontal motion of LOADED feet
        fwd_m                 chassis xy displacement over the episode
        contact_sw_per_s      foot touchdown/liftoff transitions per s
        slew_sat              fraction of joint-ticks at the safety
                              layer's per-tick rate limit
        slew_sat_all          fraction of ticks with >=6 joints
                              saturated SIMULTANEOUSLY (the takeoff
                              posture-snap signature)
        mean_h_m / dh_m       chassis height mean / final-minus-start
                              (dh_m is the rise-success canary signal)
        vx_rmse               commanded-vs-measured body vx (walk mode
                              refs only; nan elsewhere)
    """
    rets, lens, terms = [], [], 0
    env = make_task_env(task, seed0, dr_scale, episode_seconds,
                        dr_overrides)
    sids = _foot_site_ids(env)
    rad2deg = 180.0 / np.pi
    agg = {k: [] for k in QUALITY_KEYS}
    for i in range(episodes):
        obs, _ = env.reset(seed=seed0 + i)
        tr = getattr(env, "_tilt_ref0", (0.0, 0.0))
        sat_limit = 0.98 * env.safety.max_dq
        prev_cmd = env.safety._last_safe.copy()
        prev_on: list[bool] | None = None
        prev_xy: list = [None] * 6
        peak_roll = peak_pitch = peak_gyro = 0.0
        slip = 0.0
        sw = sat_jt = sat_all = 0
        h0 = float(env.data.xpos[env._chassis_bid, 2])
        xy0 = env.data.xpos[env._chassis_bid, :2].copy()
        h_sum = 0.0
        vx_se, vx_n = 0.0, 0
        ret, n = 0.0, 0
        while True:
            act, _ = model.predict(obs, deterministic=True)
            obs, r, term, trunc, _ = env.step(act)
            ret += float(r)
            n += 1
            st = env._state
            peak_roll = max(peak_roll, abs(st.imu_roll - tr[0]))
            peak_pitch = max(peak_pitch, abs(st.imu_pitch - tr[1]))
            peak_gyro = max(peak_gyro,
                            float(np.max(np.abs(st.imu_gyro[:2]))))
            cmd = env.safety._last_safe
            n_sat = int(np.sum(np.abs(cmd - prev_cmd) >= sat_limit))
            sat_jt += n_sat
            sat_all += int(n_sat >= 6)
            prev_cmd = cmd.copy()
            on: list[bool] = []
            for f in range(6):
                adr = env._touch_adr[f]
                is_on = bool(adr >= 0
                             and float(env.data.sensordata[adr]) > 0.5)
                xy = (env.data.site_xpos[sids[f], :2].copy()
                      if sids[f] >= 0 else None)
                if prev_on is not None:
                    if is_on != prev_on[f]:
                        sw += 1
                    if (is_on and prev_on[f] and xy is not None
                            and prev_xy[f] is not None):
                        slip += float(np.hypot(*(xy - prev_xy[f])))
                prev_xy[f] = xy
                on.append(is_on)
            prev_on = on
            h_sum += float(env.data.xpos[env._chassis_bid, 2])
            vxr = getattr(env._goal_traj, "vx", None)
            if vxr is not None and hasattr(env, "_body_vel_xy"):
                j = min(max(n - 1, 0), len(vxr) - 1)
                vx_meas, _ = env._body_vel_xy()
                vx_se += (float(vxr[j]) - float(vx_meas)) ** 2
                vx_n += 1
            if term or trunc:
                terms += int(term)
                break
        rets.append(ret)
        lens.append(n)
        xyN = env.data.xpos[env._chassis_bid, :2]
        agg["peak_roll_deg"].append(peak_roll * rad2deg)
        agg["peak_pitch_deg"].append(peak_pitch * rad2deg)
        agg["peak_gyro_dps"].append(peak_gyro * rad2deg)
        agg["slip_m"].append(slip)
        agg["fwd_m"].append(float(np.hypot(*(xyN - xy0))))
        agg["contact_sw_per_s"].append(sw / max(n * env.dt, 1e-9))
        agg["slew_sat"].append(sat_jt / max(n * 18, 1))
        agg["slew_sat_all"].append(sat_all / max(n, 1))
        agg["mean_h_m"].append(h_sum / max(n, 1))
        agg["dh_m"].append(
            float(env.data.xpos[env._chassis_bid, 2]) - h0)
        agg["vx_rmse"].append(float(np.sqrt(vx_se / vx_n))
                              if vx_n else float("nan"))
    env.close()
    out = {"return": float(np.mean(rets)),
           "ep_len": float(np.mean(lens)),
           "early_term_rate": terms / episodes}
    out.update({k: float(np.mean(v)) for k, v in agg.items()})
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--condition", required=True, choices=("A", "B", "C"))
    ap.add_argument("--task", required=True, choices=TASKS)
    ap.add_argument("--name", required=True)
    ap.add_argument("--steps", type=int, default=150_000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--term-penalty", type=float, default=30.0,
                    help="training-only one-time reward penalty on "
                         "early termination (0 disables); evals are "
                         "always raw")
    ap.add_argument("--dr-scale", type=float, default=0.3)
    ap.add_argument("--episode-seconds", type=float, default=10.0)
    ap.add_argument("--encoder", default=DEFAULT_ENCODER)
    ap.add_argument("--init-from", default=None,
                    help="warm-start checkpoint (transfer phase)")
    # 25k -> 10k default (operator directive 08-13: the laptop sweep's
    # 25k grid could not resolve steps-to-threshold differences; every
    # NEW cohort runs eval-every <= 10k and seeds >= 5). Eval uses
    # separate fixed-seed envs + deterministic predict, so cadence does
    # not perturb the training trajectory — only wall clock.
    ap.add_argument("--eval-every", type=int, default=10_000)
    ap.add_argument("--eval-tasks", default="hold,lower",
                    help="comma-separated task list measured at every "
                         "eval point (retention curves). Default keeps "
                         "the pilot CSV schema; hold->walk cohorts pass "
                         "hold,walk")
    ap.add_argument("--eval-episodes", type=int, default=4)
    ap.add_argument("--eval-heldout", action="store_true",
                    help="also evaluate the TRAINED task under the "
                         "fixed held-out dynamics suites (broad DR + "
                         "isolated latency/speed/deadband/torque axes "
                         "outside the training envelope) every "
                         "--heldout-every steps")
    ap.add_argument("--heldout-every", type=int, default=50_000)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--encoder-lr-scale", type=float, default=0.1)
    ap.add_argument("--anchor-data",
                    default="rl_move/dynamics/datasets/v2")
    ap.add_argument("--anchor-batches", type=int, default=4)
    ap.add_argument("--anchor-batch-size", type=int, default=256)
    ap.add_argument("--no-wandb", action="store_true",
                    help="explicitly disable required W&B tracking")
    args = ap.parse_args()

    wandb_run = _init_wandb(args)

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv

    from rl_move.dynamics import data as dd
    from rl_move.dynamics.model import dynamics_loss
    from rl_move.dynamics.sb3_encoder import (
        DynFeaturesExtractor, ScaledLRPPO, set_group_lrs,
    )

    torch.set_num_threads(2)
    venv = SubprocVecEnv(
        [_env_factory(args.task, args.seed * 1000 + i, args.dr_scale,
                      args.episode_seconds, args.term_penalty)
         for i in range(args.n_envs)])

    common = dict(
        n_steps=256, batch_size=min(2048, 256 * args.n_envs),
        learning_rate=args.lr, gamma=0.99, gae_lambda=0.95,
        ent_coef=1e-3, clip_range=0.2, seed=args.seed, verbose=0,
        device="cpu", tensorboard_log=str(LOG_DIR / "tensorboard"))
    enc_kwargs = dict(ckpt_path=str(ROOT / args.encoder),
                      frame_width=FRAME_WIDTH, history=HISTORY)

    if args.condition == "A":
        model = PPO("MlpPolicy", venv,
                    policy_kwargs=dict(net_arch=[128, 128],
                                       log_std_init=-1.0), **common)
    else:
        freeze = args.condition == "B"
        cls = PPO if freeze else ScaledLRPPO
        model = cls("MlpPolicy", venv, policy_kwargs=dict(
            net_arch=[128, 128], log_std_init=-1.0,
            features_extractor_class=DynFeaturesExtractor,
            features_extractor_kwargs={**enc_kwargs, "freeze": freeze},
        ), **common)
        # sb3's ortho_init wipes the extractor's Linears — restore.
        model.policy.features_extractor.reload_pretrained()
    if args.init_from:
        # Warm-start = policy WEIGHTS only, into the freshly built
        # model: every condition gets a fresh optimizer at the task
        # switch (also avoids the group-count mismatch that a full
        # PPO.load hits on condition C's two-group optimizer).
        from stable_baselines3.common.save_util import load_from_zip_file
        _, params, _ = load_from_zip_file(str(ROOT / args.init_from),
                                          device="cpu")
        model.policy.load_state_dict(params["policy"])
    if args.condition == "C":
        set_group_lrs(model.policy, args.lr, args.encoder_lr_scale)

    n_train = sum(p.numel() for p in model.policy.parameters()
                  if p.requires_grad)
    print(f"[{args.name}] condition {args.condition} task {args.task} "
          f"seed {args.seed}: {n_train / 1e6:.2f}M trainable params, "
          f"obs {venv.observation_space.shape}, "
          f"init_from={args.init_from}, term_penalty={args.term_penalty}")

    eval_tasks = tuple(t.strip() for t in args.eval_tasks.split(",")
                       if t.strip())
    assert all(t in TASKS for t in eval_tasks), \
        f"--eval-tasks entries must be in {TASKS}"
    assert args.task in eval_tasks, \
        "--eval-tasks must include the trained task"

    LOG_DIR.mkdir(parents=True, exist_ok=True)
    MODEL_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = LOG_DIR / f"ppo_{args.name}_eval.csv"
    csv_f = open(csv_path, "w", newline="")
    base_metrics = ("return", "ep_len", "early_term_rate")
    heldout_metrics = ("return", "early_term_rate", "slip_m",
                       "peak_roll_deg")
    csv_w = csv.DictWriter(csv_f, fieldnames=[
        "step", "wall_s",
        *[f"{t}/{m}" for t in eval_tasks
          for m in (*base_metrics, *QUALITY_KEYS)],
        *([f"{args.task}@{h}/{m}" for h, _, _ in HELDOUT_SUITES
           for m in heldout_metrics] if args.eval_heldout else []),
        "anchor_loss"], restval="")
    csv_w.writeheader()
    t0 = time.time()
    anchor_state = {"loss": float("nan")}

    def run_evals(step: int, heldout: bool = False):
        row = {"step": step, "wall_s": round(time.time() - t0, 1),
               "anchor_loss": anchor_state["loss"]}
        for t in eval_tasks:
            m = eval_task(model, t, args.eval_episodes, args.dr_scale,
                          args.episode_seconds)
            row.update({f"{t}/{k}": round(v, 4) for k, v in m.items()})
        if heldout:
            for name, ho_dr, ho_over in HELDOUT_SUITES:
                m = eval_task(model, args.task, args.eval_episodes,
                              ho_dr, args.episode_seconds,
                              dr_overrides=ho_over)
                row.update({f"{args.task}@{name}/{k}": round(m[k], 4)
                            for k in heldout_metrics})
        csv_w.writerow(row)
        csv_f.flush()
        if wandb_run is not None:
            wandb_run.log({
                "global_step": step,
                **{f"eval/{key}": value for key, value in row.items()
                   if key not in ("step", "anchor_loss")},
                "anchor/loss": row["anchor_loss"],
            })
        print(f"  eval @ {step}: " + "  ".join(
            f"{t} ret={row[f'{t}/return']:.1f} "
            f"term={row[f'{t}/early_term_rate']:.2f}" for t in eval_tasks)
            + ("  [+heldout]" if heldout else "")
            + (f"  anchor={anchor_state['loss']:.3f}"
               if args.condition == "C" else ""))

    class EvalCb(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = args.eval_every
            self._next_ho = args.heldout_every

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                ho = (args.eval_heldout
                      and self.num_timesteps >= self._next_ho)
                run_evals(self.num_timesteps, heldout=ho)
                self._next += args.eval_every
                if ho:
                    self._next_ho += args.heldout_every
            return True

    callbacks = [EvalCb()]

    if args.condition == "C":
        eps = dd.load_dataset(ROOT / args.anchor_data)
        enc_ckpt = torch.load(ROOT / args.encoder, map_location="cpu",
                              weights_only=False)
        stats = dd.Stats.from_dict(enc_ckpt["stats"])
        dyn = model.policy.features_extractor.dyn
        sampler = dd.WindowSampler(eps, stats, HISTORY, dyn.horizons,
                                   val=False, seed=args.seed)
        lambdas = {"joint_pos": 1.0, "joint_vel": 1.0, "imu": 1.0,
                   "contact": 0.5, "latent": 1.0,
                   "priv_current": 0.25, "priv_future": 0.25}
        anchor_opt = torch.optim.Adam(
            dyn.parameters(), lr=args.lr * args.encoder_lr_scale)

        def anchor_batch_to_torch(b: dict) -> dict:
            return {
                "hist": torch.as_tensor(b["hist"]),
                "fut_actions": torch.as_tensor(b["fut_actions"]),
                "state": {k: torch.as_tensor(v)
                          for k, v in b["state"].items()},
                "contact": {k: torch.as_tensor(v)
                            for k, v in b["contact"].items()},
                "priv_now": torch.as_tensor(b["priv_now"]),
                "priv": {k: torch.as_tensor(v)
                         for k, v in b["priv"].items()},
                "fut_hist": {k: torch.as_tensor(v)
                             for k, v in b["fut_hist"].items()},
            }

        class AnchorCb(BaseCallback):
            """Continue the predictive objective on the offline
            dataset after every PPO rollout (task-independent anchor
            for the shared encoder)."""

            def _on_training_start(self) -> None:
                # Loss of the untouched pretrained model — if this is
                # far above the pretraining val total, the checkpoint
                # or normalization wiring is wrong.
                with torch.no_grad():
                    b = sampler.batch(args.anchor_batch_size)
                    bt = anchor_batch_to_torch(b)
                    out = dyn(bt["hist"], bt["fut_actions"])
                    loss, _ = dynamics_loss(out, bt, lambdas, dyn)
                print(f"  anchor loss at start (pretrained, untouched): "
                      f"{float(loss):.3f}")
                if wandb_run is not None:
                    wandb_run.log({"global_step": 0,
                                   "anchor/pretrained_loss": float(loss)})

            def _on_rollout_end(self) -> None:
                dyn.train()
                losses = []
                for _ in range(args.anchor_batches):
                    b = sampler.batch(args.anchor_batch_size)
                    bt = anchor_batch_to_torch(b)
                    out = dyn(bt["hist"], bt["fut_actions"])
                    loss, _ = dynamics_loss(out, bt, lambdas, dyn)
                    anchor_opt.zero_grad()
                    loss.backward()
                    torch.nn.utils.clip_grad_norm_(dyn.parameters(), 1.0)
                    anchor_opt.step()
                    losses.append(float(loss.detach()))
                anchor_state["loss"] = float(np.mean(losses))
                if wandb_run is not None:
                    wandb_run.log({"global_step": self.num_timesteps,
                                   "anchor/loss": anchor_state["loss"]})

            def _on_step(self) -> bool:
                return True

        callbacks.append(AnchorCb())

    run_evals(0, heldout=args.eval_heldout)
    model.learn(total_timesteps=args.steps, callback=callbacks,
                reset_num_timesteps=True, progress_bar=False,
                tb_log_name=args.name)
    run_evals(model.num_timesteps, heldout=args.eval_heldout)
    out = MODEL_DIR / f"ppo_{args.name}.zip"
    model.save(str(out))
    csv_f.close()
    venv.close()
    print(f"[{args.name}] done in {(time.time() - t0) / 60:.1f} min "
          f"-> {out}\n  eval log: {csv_path}")
    if wandb_run is not None:
        wandb_run.finish()


if __name__ == "__main__":
    main()

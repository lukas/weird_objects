"""train_ppo_transfer.py — the A/B/C representation-transfer experiment.

Three PPO conditions, identical env / reward / budget / seeds; the ONLY
variable is where the policy's representation comes from
(rl_docs/DYNREP.md "First experimental comparison"):

    A  scratch    MlpPolicy on the raw stacked obs (H=16 frames)
    B  frozen     pretrained dyn encoder (dyn_v2_obs) -> frozen z ->
                  policy/value heads learn (the heads' entire run IS
                  the head warmup: the encoder never unfreezes)
    C  joint-aux  pretrained encoder, brief encoder-frozen head warmup,
                  then PPO actor/value/transformer train JOINTLY: the
                  future-state auxiliary loss joins every PPO minibatch
                  (same backward/optimizer step; transformer in a
                  scaled-LR param group), on online rollout windows
                  with 20-30% rehearsal from the pretraining corpus,
                  with a total action-KL guard (rollback/stop on
                  breach). This is L_total = L_PPO + coef*L_dyn done
                  properly — the retired v1 "AnchorCb" mechanism
                  (out-of-band Adam steps on the shared transformer
                  between rollout collection and the PPO update) was
                  measured harmful (metrics1: C led at 1M, regressed to
                  last at 2M, approx_kl 4x A/B) and is now impossible:
                  JointAuxPPO raises if the shared transformer changes
                  out-of-band (fb_20260816T203212_af7c64).
    D  frozen-critic  scratch-A actor + raw critic with a stop-gradient
                  predictive-latent residual (zero-init learned gate)
                  from the FROZEN pretrained transformer snapshot; the
                  actor never sees a latent (fb_20260817T052333_e5ae09).
    E  online-critic  like D, plus a SECOND transformer instance (the
                  online predictor) training continuously with its own
                  optimizer on fresh rollout windows + rehearsal; a
                  guarded EMA snapshot — updated atomically only
                  BETWEEN rollout+PPO iterations — feeds the critic
                  residual. Actor stays completely independent
                  (asserted: zero action-KL from predictor updates).
    F  live-critic  the command-rich LIVE extension of E (operator order
                  fb_20260817T210422_9df9c7): starts as exact frozen D;
                  the online predictor trains on CUDA-resident windows
                  from the policy's own rollouts, stratified 75% WALK /
                  25% RISE (75% fresh + 25% v5 rehearsal per batch),
                  with per-bin composition + prediction-error logging;
                  the critic-facing snapshot is VERSIONED and may change
                  no faster than --snap-boundary-steps PPO-step
                  boundaries, only if the full gate battery passes
                  (generic heldout retention, live command-rich walk
                  improvement, live rise retention, latent drift,
                  critic value-jump). Actor stays raw-obs, fully
                  independent (zero action-KL asserted).

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
                          # (default; the actual width is derived from
                          # the live env obs — goal.walk_yaw_cmd adds 1)
TASKS = ("hold", "lower", "walk", "rise", "walkrise")
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
              "rise": {"rise": 1.0},
              # walkrise (fb_20260817T210422_9df9c7 arm A): 75% walk /
              # 25% rise episode mix — the live-replay stratification
              # target; rise episodes cover flat/bridge/crouch (default
              # 35/40/25 start mix) plus post-lower bank starts via
              # --goal-set rise_start_bank=....
              "walkrise": {"walk": 0.75, "rise": 0.25}}
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
        "C": ("C=joint-aux: pretrained encoder + head warmup, then the "
              "future-state auxiliary loss trains INSIDE every PPO "
              "minibatch (online rollout windows + rehearsal mix, "
              "scaled encoder LR, total action-KL guard with rollback)"),
        "D": ("D=frozen-critic: scratch-A actor untouched; the CRITIC "
              "adds a zero-gated stop-gradient latent residual from "
              "the frozen pretrained transformer snapshot"),
        "E": ("E=online-critic: scratch-A actor untouched; a separate "
              "online transformer keeps learning dynamics from fresh "
              "rollouts (+rehearsal); a guarded EMA snapshot, updated "
              "only between rollout+PPO iterations, feeds the critic's "
              "zero-gated stop-gradient residual"),
        "F": ("F=live-critic: scratch-A actor untouched; starts as "
              "exact frozen D; the online transformer trains on "
              "CUDA-resident live rollout windows stratified 75% walk "
              "/ 25% rise (75% fresh + 25% v5 rehearsal), command-rich "
              "walk commands incl. stops/direction changes/yaw; the "
              "critic-facing snapshot is versioned and may only change "
              "at >=--snap-boundary-steps PPO-step boundaries behind "
              "retention/improvement/drift/value-jump gates"),
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
            "head_warmup_steps": args.head_warmup_steps,
            "aux_coef": args.aux_coef,
            "aux_batch_size": args.aux_batch_size,
            "rehearsal_frac": args.rehearsal_frac,
            "aux_kl_target": args.aux_kl_target,
            "aux_kl_guard": args.aux_kl_guard,
            "aux_stop_after": args.aux_stop_after,
            "online_buffer_frames": args.online_buffer_frames,
            "checkpoint_every": args.checkpoint_every,
            "device": args.device,
            "init_from": args.init_from,
            "eval_every": args.eval_every,
            "eval_tasks": args.eval_tasks,
            "eval_heldout": args.eval_heldout,
            "pred_lr": args.pred_lr,
            "pred_steps_per_iter": args.pred_steps_per_iter,
            "snap_ema_tau": args.snap_ema_tau,
            "snap_drift_guard": args.snap_drift_guard,
            "goal_set": args.goal_set,
            "select_quality": args.select_quality,
            "encoder_md5": args.encoder_md5,
            "snap_boundary_steps": args.snap_boundary_steps,
            "gate_heldout_band": args.gate_heldout_band,
            "gate_live_improve": args.gate_live_improve,
            "gate_rise_band": args.gate_rise_band,
            "gate_value_jump": args.gate_value_jump,
            "live_walk_frac": args.live_walk_frac,
            "metrics_contract": (
                "transfer-v5-livecritic"
                if args.condition.upper() == "F" else
                "transfer-v4-predcritic"
                if args.condition.upper() in ("D", "E")
                else "transfer-v3-jointaux"),
        },
        sync_tensorboard=True,
    )
    run.define_metric("eval/*", step_metric="global_step", summary="last")
    run.define_metric("SCORE/*", step_metric="global_step", summary="last")
    run.define_metric("rollout/*", step_metric="global_step", summary="last")
    run.define_metric("anchor/*", step_metric="global_step", summary="last")
    run.define_metric("pred/*", step_metric="global_step", summary="last")
    run.define_metric("critic/*", step_metric="global_step",
                      summary="last")
    run.define_metric("data/*", step_metric="global_step",
                      summary="last")
    metadata = {"id": run.id, "url": run.url, "name": args.name,
                "wandb_name": f"{args.name}.{attempt}"}
    (LOG_DIR / f"ppo_{args.name}_wandb.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print(f"[wandb] logging to {run.url}", flush=True)
    return run


def make_task_env(task: str, seed: int, dr_scale: float,
                  episode_seconds: float,
                  dr_overrides: dict | None = None,
                  goal_set: dict | None = None):
    """One walk-family env pinned to a single goal mode (or the
    walkrise mix). Uses the walk env class for every task so obs width
    and checkpoints are interchangeable across tasks. ``goal_set``
    (--goal-set k=v) overrides cfg goal.* keys — the command-diversity
    lever (walk_cmd_resample_s / walk_stop_frac / walk_yaw_cmd /
    rise_start_bank...); it applies to TRAINING AND EVAL envs alike so
    gates measure the distribution actually trained."""
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    cfg.setdefault("obs", {})["history_frames"] = HISTORY
    if goal_set:
        cfg["goal"] = {**(cfg.get("goal") or {}), **goal_set}
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
                 episode_seconds: float, term_penalty: float,
                 capture_windows: bool = False,
                 goal_set: dict | None = None):
    def _make():
        env = make_task_env(task, seed, dr_scale, episode_seconds,
                            goal_set=goal_set)
        if capture_windows:
            # Conditions C/E/F: record collector-contract episodes so
            # the predictor trains on FRESH on-policy windows.
            from rl_move.dynamics.online_windows import OnlineEpisodeCapture
            env = OnlineEpisodeCapture(env, dr_scale)
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
                "slew_sat_all", "mean_h_m", "dh_m", "vx_rmse",
                # command-following quality (fb_20260817T210422_9df9c7):
                # body-frame vy/yaw tracking, achieved progress along
                # the commanded direction (m and fraction of commanded
                # distance), and slip normalized per progress meter.
                "vy_rmse", "wz_rmse_dps", "cmd_prog_m", "cmd_prog_frac",
                "slip_per_m")


def _nn(x, default=0.0) -> float:
    """nan-safe float."""
    x = float(x) if x is not None else float("nan")
    return default if x != x else x


def locomotion_quality(m: dict) -> float:
    """Pre-registered composite walking-quality score in [0, 100]
    (operator order fb_20260817T210422_9df9c7 arm B: checkpoint
    selection by actual command progress, body-frame vx/vy+yaw
    tracking, slip per progress meter, roll, falls, slew and contact
    gait — never scalar reward alone):

        100 * (1 - early_term_rate) * min(cmd_prog_frac, 1)
            * exp(-hypot(vx_rmse, vy_rmse) / 0.05)
            * exp(-wz_rmse_dps / 20)
            * exp(-slip_per_m / 1.5)
            * exp(-peak_roll_deg / 10)
            * exp(-max(slew_sat - 0.5, 0) / 0.25)
            * (1.0 if contact_sw_per_s >= 3 else 0.25)

    The contact-switch floor marks gaits whose feet do not actually
    cycle (a parked/sliding "walk" is not walking)."""
    import math as _m
    s_fall = 1.0 - min(max(_nn(m.get("early_term_rate"), 1.0), 0.0), 1.0)
    prog = min(max(_nn(m.get("cmd_prog_frac")), 0.0), 1.0)
    vxy = _m.hypot(_nn(m.get("vx_rmse"), 1.0), _nn(m.get("vy_rmse"), 0.0))
    score = (100.0 * s_fall * prog
             * _m.exp(-vxy / 0.05)
             * _m.exp(-_nn(m.get("wz_rmse_dps"), 60.0) / 20.0)
             * _m.exp(-min(_nn(m.get("slip_per_m"), 10.0), 50.0) / 1.5)
             * _m.exp(-_nn(m.get("peak_roll_deg"), 30.0) / 10.0)
             * _m.exp(-max(_nn(m.get("slew_sat"), 1.0) - 0.5, 0.0) / 0.25))
    if _nn(m.get("contact_sw_per_s")) < 3.0:
        score *= 0.25
    return float(score)


def _foot_site_ids(env) -> list[int]:
    import mujoco
    return [mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE,
                              f"L{i}_foot_site") for i in range(6)]


def eval_task(model, task: str, episodes: int, dr_scale: float,
              episode_seconds: float, seed0: int = 10_000,
              dr_overrides: dict | None = None,
              goal_set: dict | None = None) -> dict:
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
                        dr_overrides, goal_set=goal_set)
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
        vy_se = wz_se = 0.0
        cmd_dist = prog_m = 0.0
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
                vyr = getattr(env._goal_traj, "vy", None)
                wzr = getattr(env._goal_traj, "wz", None)
                vx_meas, vy_meas = env._body_vel_xy()
                vx_c = float(vxr[j])
                vy_c = float(vyr[j]) if vyr is not None else 0.0
                vx_se += (vx_c - float(vx_meas)) ** 2
                vy_se += (vy_c - float(vy_meas)) ** 2
                if hasattr(env, "_body_wz"):
                    wz_c = float(wzr[j]) if wzr is not None else 0.0
                    wz_se += (wz_c - float(env._body_wz())) ** 2
                vx_n += 1
                # progress along the commanded direction vs the
                # commanded distance (both in meters)
                s_ref = float(np.hypot(vx_c, vy_c))
                cmd_dist += s_ref * env.dt
                if s_ref > 1e-6:
                    prog_m += ((float(vx_meas) * vx_c
                                + float(vy_meas) * vy_c) / s_ref
                               * env.dt)
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
        agg["vy_rmse"].append(float(np.sqrt(vy_se / vx_n))
                              if vx_n else float("nan"))
        agg["wz_rmse_dps"].append(
            float(np.sqrt(wz_se / vx_n)) * rad2deg
            if vx_n else float("nan"))
        agg["cmd_prog_m"].append(prog_m)
        agg["cmd_prog_frac"].append(prog_m / cmd_dist
                                    if cmd_dist > 0.01 else float("nan"))
        agg["slip_per_m"].append(slip / max(prog_m, 0.05))
    env.close()
    out = {"return": float(np.mean(rets)),
           "ep_len": float(np.mean(lens)),
           "early_term_rate": terms / episodes}
    out.update({k: float(np.mean(v)) for k, v in agg.items()})
    return out


def anchor_batch_to_torch(b: dict, device=None) -> dict:
    """Convert a WindowSampler/GpuWindowSampler batch (numpy/torch) into
    the exact key set ``dynamics_loss`` (model.py) reads.

    08-15: dynamics_loss grew "current"-state heads (contact_now/
    current_now/current[k], commit 6a8560c0, the GPU-transformer
    scaling work) but this PPO-side condition-C anchor converter was
    never updated to forward them -- any condition C run launched on
    code synced after that commit crashed at _on_training_start with
    KeyError('contact_now') the instant AnchorCb touched the raw batch
    (silent since any already-running process keeps its old in-memory
    code; caught 08-15 ~20:2x UTC when 3 freshly-launched risewalk
    -single seeds died identically ~1min into their condition-C phase,
    right after the NPZ-caching OOM fix landed). Forward every key
    dynamics_loss actually reads; priv_mask_now is optional there
    (falls back to all-ones) but forwarding it keeps the anchor loss
    numerically identical to pretraining's own loss.
    """
    import torch  # deferred import (matches main()'s lazy torch import)
    return {
        "hist": torch.as_tensor(b["hist"], device=device),
        "fut_actions": torch.as_tensor(b["fut_actions"], device=device),
        "state": {k: torch.as_tensor(v, device=device)
                  for k, v in b["state"].items()},
        "contact": {k: torch.as_tensor(v, device=device)
                    for k, v in b["contact"].items()},
        "contact_now": torch.as_tensor(b["contact_now"], device=device),
        "current": {k: torch.as_tensor(v, device=device)
                    for k, v in b["current"].items()},
        "current_now": torch.as_tensor(b["current_now"], device=device),
        "priv_now": torch.as_tensor(b["priv_now"], device=device),
        "priv_mask_now": torch.as_tensor(b["priv_mask_now"], device=device),
        "priv": {k: torch.as_tensor(v, device=device)
                 for k, v in b["priv"].items()},
        "fut_hist": {k: torch.as_tensor(v, device=device)
                     for k, v in b["fut_hist"].items()},
    }


def require_torch_device(torch, requested: str):
    """Resolve the training device and make CUDA requests fail fast."""
    device = torch.device(requested)
    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA training was requested but torch.cuda.is_available() "
                "is false; refusing to fall back to CPU"
            )
        # Allocation catches missing drivers/runtime before W&B creates a run.
        torch.empty(1, device=device)
    return device


def rollout_metrics_payload(
        ep_info_buffer, *, reward_sum: float, reward_count: int,
        reward_sum_cumulative: float, reward_count_cumulative: int,
        reward_ema: float | None, episodes_completed: int,
        early_terminations: int, time_limit_truncations: int) -> dict:
    """Build the campaign-standard rollout metrics for one PPO rollout."""
    payload = {}
    if reward_count:
        payload["rollout/reward_per_transition"] = (
            reward_sum / reward_count)
    if reward_count_cumulative:
        payload["rollout/reward_per_transition_cumulative"] = (
            reward_sum_cumulative / reward_count_cumulative)
    if reward_ema is not None:
        payload["rollout/reward_per_transition_ema"] = reward_ema
    if ep_info_buffer:
        payload["rollout/ep_rew_mean"] = float(np.mean(
            [episode["r"] for episode in ep_info_buffer]))
        payload["rollout/ep_len_mean"] = float(np.mean(
            [episode["l"] for episode in ep_info_buffer]))
    payload["rollout/episodes_completed"] = episodes_completed
    payload["rollout/early_terminations"] = early_terminations
    payload["rollout/time_limit_truncations"] = time_limit_truncations
    if episodes_completed:
        payload["rollout/early_term_rate"] = (
            early_terminations / episodes_completed)
        payload["rollout/time_limit_rate"] = (
            time_limit_truncations / episodes_completed)
    return payload


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--condition", required=True,
                    choices=("A", "B", "C", "D", "E", "F"))
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
                    default="rl_move/dynamics/datasets/v2",
                    help="pretraining corpus: rehearsal mix source + "
                         "heldout prediction reference (condition C)")
    ap.add_argument("--head-warmup-steps", type=int, default=50_000,
                    help="condition C: encoder-frozen steps so the "
                         "actor/value heads learn to consume the "
                         "pretrained latent before joint training")
    ap.add_argument("--aux-coef", type=float, default=1.0,
                    help="weight of the future-state auxiliary loss "
                         "inside each PPO minibatch loss")
    ap.add_argument("--aux-batch-size", type=int, default=256)
    ap.add_argument("--rehearsal-frac", type=float, default=0.25,
                    help="fraction of each auxiliary batch drawn from "
                         "the pretraining corpus (directive: 20-30%%); "
                         "the rest is fresh online rollout windows")
    ap.add_argument("--aux-kl-target", type=float, default=0.02,
                    help="logged reference for total post-update "
                         "action KL of the combined PPO+aux update")
    ap.add_argument("--aux-kl-guard", type=float, default=0.04,
                    help="reject/rollback threshold on total "
                         "post-update action KL while aux is active")
    ap.add_argument("--aux-stop-after", type=int, default=3,
                    help="consecutive rejected updates before the "
                         "auxiliary objective stops permanently")
    ap.add_argument("--online-buffer-frames", type=int, default=120_000,
                    help="FIFO capacity (frames) of the online rollout "
                         "window buffer")
    ap.add_argument("--pred-lr", type=float, default=1e-4,
                    help="condition E: Adam LR of the ONLINE predictor "
                         "(its own optimizer, never the policy's)")
    ap.add_argument("--pred-steps-per-iter", type=int, default=8,
                    help="condition E: online-predictor gradient steps "
                         "per rollout+PPO iteration")
    ap.add_argument("--snap-ema-tau", type=float, default=0.05,
                    help="condition E: EMA rate of the guarded "
                         "between-iteration critic-snapshot update")
    ap.add_argument("--snap-drift-guard", type=float, default=0.05,
                    help="condition E: max probe-latent MSE a candidate "
                         "snapshot may move per update; larger drifts "
                         "are logged and SKIPPED")
    ap.add_argument("--checkpoint-every", type=int, default=250_000,
                    help="periodic checkpoint cadence in steps (0 "
                         "disables); best-by-heldout-walk checkpoint "
                         "is always kept when --eval-heldout is on")
    ap.add_argument("--device", choices=("cuda", "cpu"), default="cuda",
                    help="torch device; defaults to CUDA and never silently "
                         "falls back to CPU")
    ap.add_argument("--no-wandb", action="store_true",
                    help="explicitly disable required W&B tracking")
    # -- fb_20260817T210422_9df9c7 additions --------------------------
    ap.add_argument("--goal-set", action="append", default=[],
                    metavar="K=V",
                    help="override cfg goal.* keys on TRAIN AND EVAL "
                         "envs (command diversity: walk_cmd_resample_s, "
                         "walk_stop_frac, walk_yaw_cmd, rise_start_bank"
                         "...); repeatable")
    ap.add_argument("--select-quality", action="store_true",
                    help="select the best checkpoint by the "
                         "pre-registered locomotion_quality composite "
                         "(command progress/tracking/slip-per-m/roll/"
                         "falls/slew/gait) on the own-DR + dr10 walk "
                         "evals instead of heldout return")
    ap.add_argument("--encoder-md5", default=None,
                    help="assert the md5 of --encoder before training "
                         "(provenance pin, e.g. the vt2ovznc artifact)")
    ap.add_argument("--snap-boundary-steps", type=int, default=1_000_000,
                    help="condition F: minimum PPO-step spacing of "
                         "critic-snapshot update attempts")
    ap.add_argument("--gate-heldout-band", type=float, default=0.15,
                    help="condition F: generic corpus-val retention band "
                         "vs the pretrained reference")
    ap.add_argument("--gate-live-improve", type=float, default=0.0,
                    help="condition F: required fractional improvement "
                         "over the current snapshot on the live "
                         "command-rich walk heldout")
    ap.add_argument("--gate-rise-band", type=float, default=0.05,
                    help="condition F: live rise heldout retention band")
    ap.add_argument("--gate-value-jump", type=float, default=0.10,
                    help="condition F: max critic value jump from a "
                         "snapshot swap, as a fraction of mean|V|+1")
    ap.add_argument("--live-walk-frac", type=float, default=0.75,
                    help="condition F: walk fraction of the FRESH part "
                         "of each predictor batch (rise = remainder)")
    ap.add_argument("--live-windows-per-episode", type=int, default=64)
    ap.add_argument("--live-max-walk-windows", type=int, default=30_000)
    ap.add_argument("--live-max-rise-windows", type=int, default=12_000)
    ap.add_argument("--live-max-val-windows", type=int, default=5_000)
    ap.add_argument("--live-val-every", type=int, default=8,
                    help="condition F: every Nth accepted episode per "
                         "mode goes to the live VAL store (the "
                         "command-rich heldout the gates read)")
    ap.add_argument("--live-min-windows", type=int, default=512,
                    help="condition F: live windows required before "
                         "fresh data enters predictor batches")
    args = ap.parse_args()
    goal_set = {}
    for kv in args.goal_set:
        key, sep, val = kv.partition("=")
        if not sep:
            ap.error(f"--goal-set needs K=V, got {kv!r}")
        try:
            goal_set[key] = json.loads(val)
        except (json.JSONDecodeError, ValueError):
            goal_set[key] = val

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor

    from rl_move.dynamics import data as dd
    from rl_move.dynamics.joint_aux import (
        AuxConfig, JointAuxPPO, priv_group_metrics,
    )
    from rl_move.dynamics.model import dynamics_loss
    from rl_move.dynamics.online_windows import OnlineWindowBuffer
    from rl_move.dynamics.predictive_critic import (
        PredictiveCriticPolicy, PredictiveCriticPPO, PredictorConfig,
    )
    from rl_move.dynamics.sb3_encoder import (
        DynFeaturesExtractor, set_group_lrs,
    )

    device = require_torch_device(torch, args.device)
    if device.type == "cuda":
        gpu_name = torch.cuda.get_device_name(device)
        print(f"[device] CUDA required and active: {gpu_name}", flush=True)
    else:
        print("[device] explicit CPU training", flush=True)
    if args.encoder_md5:
        import hashlib
        got = hashlib.md5((ROOT / args.encoder).read_bytes()).hexdigest()
        if got != args.encoder_md5.strip().lower():
            raise RuntimeError(
                f"encoder md5 mismatch: {args.encoder} is {got}, "
                f"expected {args.encoder_md5} — wrong/corrupt "
                f"pretrained transformer, refusing to train")
        print(f"[encoder] md5 verified: {got}", flush=True)
    wandb_run = _init_wandb(args)

    torch.set_num_threads(2)
    # C, E and F train a predictor on FRESH rollout windows.
    capture = args.condition in ("C", "E", "F")
    venv = VecMonitor(SubprocVecEnv(
        [_env_factory(args.task, args.seed * 1000 + i, args.dr_scale,
                      args.episode_seconds, args.term_penalty,
                      capture_windows=capture, goal_set=goal_set)
         for i in range(args.n_envs)]))
    # Frame width is derived from the live env: goal.walk_yaw_cmd=1
    # (--goal-set) appends the commanded yaw rate to the goal obs.
    n_obs = int(np.prod(venv.observation_space.shape))
    assert n_obs % HISTORY == 0, \
        f"obs dim {n_obs} not divisible by history {HISTORY}"
    frame_width = n_obs // HISTORY

    common = dict(
        n_steps=256, batch_size=min(2048, 256 * args.n_envs),
        learning_rate=args.lr, gamma=0.99, gae_lambda=0.95,
        ent_coef=1e-3, clip_range=0.2, seed=args.seed, verbose=0,
        device=device, tensorboard_log=str(LOG_DIR / "tensorboard"))
    enc_kwargs = dict(ckpt_path=str(ROOT / args.encoder),
                      frame_width=frame_width, history=HISTORY)

    if args.condition == "A":
        model = PPO("MlpPolicy", venv,
                    policy_kwargs=dict(net_arch=[128, 128],
                                       log_std_init=-1.0), **common)
    elif args.condition in ("D", "E", "F"):
        # Decoupled predictive critic: the actor is the scratch-A
        # MlpPolicy on the raw stacked obs (bit-identical init at the
        # same seed — the predictive modules are built after SB3's
        # standard RNG draws); only the CRITIC reads the frozen
        # snapshot transformer, through a zero-init learned gate.
        model = PredictiveCriticPPO(
            PredictiveCriticPolicy, venv, policy_kwargs=dict(
                net_arch=[128, 128], log_std_init=-1.0,
                predictor_ckpt=str(ROOT / args.encoder),
                frame_width=frame_width, history=HISTORY), **common)
    else:
        freeze = args.condition == "B"
        cls = PPO if freeze else JointAuxPPO
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
                                          device=device)
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
                       "peak_roll_deg", "cmd_prog_frac", "vx_rmse",
                       "vy_rmse", "wz_rmse_dps", "slip_per_m",
                       "slew_sat", "contact_sw_per_s")
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
    # Best-checkpoint selection (directive: retain/select the winning
    # checkpoint by HELDOUT walking evaluation, not the final step).
    best_state = {"score": float("-inf"), "step": -1}
    best_path = MODEL_DIR / f"ppo_{args.name}_best.zip"
    # Condition C fills this with its heldout-prediction closure so the
    # phase-1 predictor-quality reference is measured alongside RL evals.
    aux_ctx: dict = {}

    def run_evals(step: int, heldout: bool = False):
        row = {"step": step, "wall_s": round(time.time() - t0, 1),
               "anchor_loss": anchor_state["loss"]}
        for t in eval_tasks:
            m = eval_task(model, t, args.eval_episodes, args.dr_scale,
                          args.episode_seconds, goal_set=goal_set)
            row.update({f"{t}/{k}": round(v, 4) for k, v in m.items()})
        heldout_score = None
        loco_score = None
        if heldout:
            for name, ho_dr, ho_over in HELDOUT_SUITES:
                m = eval_task(model, args.task, args.eval_episodes,
                              ho_dr, args.episode_seconds,
                              dr_overrides=ho_over, goal_set=goal_set)
                row.update({f"{args.task}@{name}/{k}": round(m[k], 4)
                            for k in heldout_metrics})
            heldout_score = float(np.mean(
                [row[f"{args.task}@{name}/return"]
                 for name, _, _ in HELDOUT_SUITES]))
            selection = heldout_score
            if args.select_quality:
                # Pre-registered composite quality on the own-DR eval
                # of the trained task + its broad-DR heldout (dr10);
                # this — not scalar reward — picks the best checkpoint.
                own = {k: row.get(f"{args.task}/{k}")
                       for k in ("early_term_rate", *QUALITY_KEYS)}
                ho = {k: row.get(f"{args.task}@dr10/{k}")
                      for k in heldout_metrics}
                loco_score = 0.5 * (locomotion_quality(own)
                                    + locomotion_quality(ho))
                selection = loco_score
            if selection > best_state["score"]:
                best_state.update(score=selection, step=step)
                model.save(str(best_path))
                kind = ("loco-quality" if args.select_quality
                        else f"heldout-{args.task}")
                print(f"  new best {kind} checkpoint @ "
                      f"{step}: {selection:.1f} -> {best_path.name}")
        heldout_pred = (aux_ctx["heldout_pred"]()
                        if heldout and aux_ctx.get("heldout_pred")
                        else None)
        csv_w.writerow(row)
        csv_f.flush()
        if wandb_run is not None:
            payload = {
                "global_step": step,
                **{f"eval/{key}": value for key, value in row.items()
                   if key not in ("step", "anchor_loss")},
                "anchor/loss": row["anchor_loss"],
            }
            if heldout_score is not None:
                payload["eval/heldout_walk_score"] = heldout_score
                payload["eval/best_heldout_walk_score"] = (
                    best_state["score"])
                payload["eval/best_heldout_walk_step"] = (
                    best_state["step"])
            if loco_score is not None:
                payload["SCORE/loco_quality"] = loco_score
                payload["eval/best_loco_quality"] = best_state["score"]
                payload["eval/best_loco_step"] = best_state["step"]
            if heldout_pred is not None:
                payload.update({f"aux/heldout/{k}": v
                                for k, v in heldout_pred.items()})
            if heldout and aux_ctx.get("bin_report"):
                payload.update(aux_ctx["bin_report"]())
            for task in eval_tasks:
                payload[f"SCORE/{task}_total_reward"] = row[
                    f"{task}/return"]
                payload[f"SCORE/{task}_early_term_rate"] = row[
                    f"{task}/early_term_rate"]
            wandb_run.log(payload)
        print(f"  eval @ {step}: " + "  ".join(
            f"{t} ret={row[f'{t}/return']:.1f} "
            f"term={row[f'{t}/early_term_rate']:.2f}" for t in eval_tasks)
            + ("  [+heldout]" if heldout else "")
            + (f"  aux_train_total={anchor_state['loss']:.3f}"
               if args.condition == "C" else "")
            + (f"  heldout_pred_total={heldout_pred['total']:.3f}"
               if heldout_pred else ""))

    class EvalCb(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = args.eval_every
            self._next_ho = args.heldout_every
            self._next_ck = args.checkpoint_every

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                ho = (args.eval_heldout
                      and self.num_timesteps >= self._next_ho)
                run_evals(self.num_timesteps, heldout=ho)
                self._next += args.eval_every
                if ho:
                    self._next_ho += args.heldout_every
            if args.checkpoint_every > 0 and (
                    self.num_timesteps >= self._next_ck):
                ck = MODEL_DIR / (f"ppo_{args.name}_"
                                  f"ck{self.num_timesteps}.zip")
                self.model.save(str(ck))
                self._next_ck += args.checkpoint_every
                # Keep the newest 3 periodic checkpoints (best/final
                # zips live under different names and are never pruned).
                cks = sorted(MODEL_DIR.glob(f"ppo_{args.name}_ck*.zip"),
                             key=lambda p: int(p.stem.rsplit("ck", 1)[1]))
                for old in cks[:-3]:
                    old.unlink()
            return True

    class RolloutMetricsCb(BaseCallback):
        """Log the actual PPO signal and completed-episode outcomes."""

        def __init__(self):
            super().__init__()
            self._reward_sum = 0.0
            self._reward_count = 0
            self._reward_sum_cumulative = 0.0
            self._reward_count_cumulative = 0
            self._reward_ema = None
            self._episodes_completed = 0
            self._early_terminations = 0
            self._time_limit_truncations = 0

        def _on_step(self) -> bool:
            rewards = self.locals.get("rewards")
            if rewards is not None:
                values = np.asarray(rewards, dtype=np.float64)
                self._reward_sum += float(values.sum())
                self._reward_count += int(values.size)
            dones = self.locals.get("dones")
            infos = self.locals.get("infos")
            if dones is not None and infos is not None:
                for done, info in zip(dones, infos):
                    if not done:
                        continue
                    self._episodes_completed += 1
                    if info.get("TimeLimit.truncated", False):
                        self._time_limit_truncations += 1
                    else:
                        self._early_terminations += 1
            return True

        def _on_rollout_end(self) -> None:
            self._reward_sum_cumulative += self._reward_sum
            self._reward_count_cumulative += self._reward_count
            if self._reward_count:
                current = self._reward_sum / self._reward_count
                self._reward_ema = (current if self._reward_ema is None
                                    else 0.9 * self._reward_ema
                                    + 0.1 * current)
            payload = rollout_metrics_payload(
                self.model.ep_info_buffer,
                reward_sum=self._reward_sum,
                reward_count=self._reward_count,
                reward_sum_cumulative=self._reward_sum_cumulative,
                reward_count_cumulative=self._reward_count_cumulative,
                reward_ema=self._reward_ema,
                episodes_completed=self._episodes_completed,
                early_terminations=self._early_terminations,
                time_limit_truncations=self._time_limit_truncations,
            )
            if wandb_run is not None:
                wandb_run.log({"global_step": self.num_timesteps,
                               **payload})
            self._reward_sum = 0.0
            self._reward_count = 0
            self._episodes_completed = 0
            self._early_terminations = 0
            self._time_limit_truncations = 0

    callbacks = [RolloutMetricsCb(), EvalCb()]

    if args.condition in ("D", "E", "F"):
        eps = dd.load_dataset(ROOT / args.anchor_data)
        enc_ckpt = torch.load(ROOT / args.encoder, map_location="cpu",
                              weights_only=False)
        stats = dd.Stats.from_dict(enc_ckpt["stats"])
        snap = model.policy.critic_predictor
        pred_cfg = PredictorConfig(
            mode={"D": "frozen", "E": "online",
                  "F": "live"}[args.condition],
            batch_size=args.aux_batch_size,
            rehearsal_frac=args.rehearsal_frac,
            steps_per_iter=args.pred_steps_per_iter,
            lr=args.pred_lr, ema_tau=args.snap_ema_tau,
            drift_guard=args.snap_drift_guard,
            snapshot_boundary_steps=args.snap_boundary_steps,
            gate_heldout_band=args.gate_heldout_band,
            gate_live_improve=args.gate_live_improve,
            gate_rise_band=args.gate_rise_band,
            gate_value_jump_frac=args.gate_value_jump)
        rehearsal = dd.WindowSampler(eps, stats, HISTORY, snap.horizons,
                                     val=False, seed=args.seed)
        heldout_sampler = dd.WindowSampler(eps, stats, HISTORY,
                                           snap.horizons, val=True,
                                           seed=args.seed)
        online_buf = None
        live_store = None
        live_batch_fn = None
        gate_fns = None
        pretrained_ref = {"total": float("nan")}
        if args.condition == "E":
            online_buf = OnlineWindowBuffer(
                stats, HISTORY, snap.horizons,
                max_frames=args.online_buffer_frames, seed=args.seed)
        if args.condition == "F":
            from rl_move.dynamics.live_replay import (
                LiveWindowStore, stratified_live_batch,
            )
            live_store = LiveWindowStore(
                stats, HISTORY, snap.horizons, device,
                max_walk_windows=args.live_max_walk_windows,
                max_rise_windows=args.live_max_rise_windows,
                max_val_windows=args.live_max_val_windows,
                windows_per_episode=args.live_windows_per_episode,
                val_every=args.live_val_every, seed=args.seed)

            def live_batch_fn(n: int):
                return stratified_live_batch(
                    live_store, rehearsal, anchor_batch_to_torch,
                    device, n, rehearsal_frac=args.rehearsal_frac,
                    walk_frac=args.live_walk_frac,
                    min_live_windows=args.live_min_windows)

            def _model_val_total(m, batches) -> float | None:
                was_training = m.training
                m.eval()
                tot, n_b = 0.0, 0
                with torch.no_grad():
                    for bt in batches:
                        out = m(bt["hist"], bt["fut_actions"])
                        loss, _ = dynamics_loss(out, bt,
                                                pred_cfg.lambdas, m)
                        tot += float(loss)
                        n_b += 1
                if was_training:
                    m.train()
                return tot / n_b if n_b else None

            def corpus_val(m) -> float:
                return _model_val_total(
                    m, (anchor_batch_to_torch(b, device=device)
                        for b in heldout_sampler.val_batches(
                            1024, args.aux_batch_size)))

            def live_val(m, mode: str) -> float | None:
                if live_store.num_windows(mode, "val") < 128:
                    return None
                return _model_val_total(
                    m, live_store.val_batches(
                        mode, args.aux_batch_size, max_windows=2048))

            gate_fns = {
                "corpus_val": corpus_val,
                "live_val": live_val,
                "pretrained_ref": lambda: pretrained_ref["total"],
            }

        def pred_sink(payload: dict, step: int) -> None:
            anchor_state["loss"] = payload.get(
                "pred/train/total", anchor_state["loss"])
            if wandb_run is not None:
                wandb_run.log({"global_step": step, **payload})

        model.configure_predictor(pred_cfg, rehearsal,
                                  anchor_batch_to_torch,
                                  online_buffer=online_buf,
                                  metrics_sink=pred_sink,
                                  live_batch_fn=live_batch_fn,
                                  gate_fns=gate_fns)

        def heldout_pred() -> dict:
            """Predictor quality on the corpus VAL split. E/F: the
            ONLINE predictor (the preservation gate); D: the frozen
            snapshot (constant reference)."""
            live = (model._online_dyn if args.condition in ("E", "F")
                    else snap)
            was_training = live.training
            live.eval()
            sums: dict[str, float] = {}
            n = 0
            with torch.no_grad():
                for b in heldout_sampler.val_batches(
                        1024, args.aux_batch_size):
                    bt = anchor_batch_to_torch(b, device=device)
                    out = live(bt["hist"], bt["fut_actions"])
                    _, logs = dynamics_loss(out, bt, pred_cfg.lambdas,
                                            live)
                    logs.update(priv_group_metrics(out, bt, prefix=""))
                    for k, v in logs.items():
                        sums[k] = sums.get(k, 0.0) + v
                    n += 1
            if was_training:
                live.train()
            return {k: v / max(n, 1) for k, v in sums.items()}

        aux_ctx["heldout_pred"] = heldout_pred
        if args.condition == "F":
            # Per-bin composition + prediction-error report for the
            # heldout eval points (run_evals reads this closure).
            aux_ctx["bin_report"] = lambda: {
                **live_store.composition_report(),
                **live_store.bin_report(model._online_dyn,
                                        pred_cfg.lambdas),
            }

        class PredOnlineWindowCb(BaseCallback):
            def _on_training_start(self) -> None:
                ref = heldout_pred()
                pretrained_ref["total"] = ref["total"]
                print(f"  heldout pred loss at start (pretrained, "
                      f"untouched): {ref['total']:.3f}")
                if wandb_run is not None:
                    wandb_run.log({
                        "global_step": 0,
                        "anchor/pretrained_loss": ref["total"],
                        **{f"aux/heldout/{k}": v for k, v in ref.items()},
                    })

            def _on_step(self) -> bool:
                if online_buf is None and live_store is None:
                    return True
                for info in self.locals.get("infos", ()):
                    ep = info.get("dynrep_episode")
                    if ep is not None:
                        if live_store is not None:
                            live_store.add_episode(ep)
                        else:
                            online_buf.add_episode(ep)
                return True

            def _on_rollout_end(self) -> None:
                if live_store is not None and wandb_run is not None:
                    wandb_run.log({
                        "global_step": self.num_timesteps,
                        **live_store.composition_report(),
                    })

        callbacks.append(PredOnlineWindowCb())

    if args.condition == "C":
        eps = dd.load_dataset(ROOT / args.anchor_data)
        enc_ckpt = torch.load(ROOT / args.encoder, map_location="cpu",
                              weights_only=False)
        stats = dd.Stats.from_dict(enc_ckpt["stats"])
        dyn = model.policy.features_extractor.dyn
        aux_cfg = AuxConfig(
            coef=args.aux_coef, batch_size=args.aux_batch_size,
            rehearsal_frac=args.rehearsal_frac,
            warmup_steps=args.head_warmup_steps,
            kl_target=args.aux_kl_target, kl_guard=args.aux_kl_guard,
            stop_after=args.aux_stop_after)
        rehearsal = dd.WindowSampler(eps, stats, HISTORY, dyn.horizons,
                                     val=False, seed=args.seed)
        heldout_sampler = dd.WindowSampler(eps, stats, HISTORY,
                                           dyn.horizons, val=True,
                                           seed=args.seed)
        online_buf = OnlineWindowBuffer(
            stats, HISTORY, dyn.horizons,
            max_frames=args.online_buffer_frames, seed=args.seed)

        def aux_sink(payload: dict, step: int) -> None:
            anchor_state["loss"] = payload.get(
                "aux/train/total", anchor_state["loss"])
            if wandb_run is not None:
                wandb_run.log({"global_step": step, **payload,
                               "anchor/loss": anchor_state["loss"]})

        model.configure_aux(aux_cfg, online_buf, rehearsal,
                            anchor_batch_to_torch, metrics_sink=aux_sink)

        def heldout_pred() -> dict:
            """Phase-1 predictor quality on the corpus VAL split (the
            gate: joint training must not degrade heldout prediction)."""
            dyn.eval()
            sums: dict[str, float] = {}
            n = 0
            with torch.no_grad():
                for b in heldout_sampler.val_batches(
                        1024, args.aux_batch_size):
                    bt = anchor_batch_to_torch(b, device=device)
                    out = dyn(bt["hist"], bt["fut_actions"])
                    _, logs = dynamics_loss(out, bt, aux_cfg.lambdas, dyn)
                    logs.update(priv_group_metrics(out, bt, prefix=""))
                    for k, v in logs.items():
                        sums[k] = sums.get(k, 0.0) + v
                    n += 1
            dyn.train()
            return {k: v / max(n, 1) for k, v in sums.items()}

        aux_ctx["heldout_pred"] = heldout_pred

        class OnlineWindowCb(BaseCallback):
            """Harvest captured episodes from env infos into the
            online window buffer (main-process side of the capture)."""

            def _on_training_start(self) -> None:
                # Loss of the untouched pretrained model on the corpus
                # val split — if far above the pretraining val total,
                # the checkpoint or normalization wiring is wrong.
                ref = heldout_pred()
                print(f"  heldout pred loss at start (pretrained, "
                      f"untouched): {ref['total']:.3f}")
                if wandb_run is not None:
                    wandb_run.log({
                        "global_step": 0,
                        "anchor/pretrained_loss": ref["total"],
                        **{f"aux/heldout/{k}": v for k, v in ref.items()},
                    })

            def _on_step(self) -> bool:
                for info in self.locals.get("infos", ()):
                    ep = info.get("dynrep_episode")
                    if ep is not None:
                        online_buf.add_episode(ep)
                return True

        callbacks.append(OnlineWindowCb())

    run_evals(0, heldout=args.eval_heldout)
    model.learn(total_timesteps=args.steps, callback=callbacks,
                reset_num_timesteps=True, progress_bar=False,
                tb_log_name=args.name)
    run_evals(model.num_timesteps, heldout=args.eval_heldout)
    out = MODEL_DIR / f"ppo_{args.name}.zip"
    model.save(str(out))
    if args.condition in ("E", "F"):
        # The SB3 zip carries only the critic SNAPSHOT (inside the
        # policy); persist the online predictor separately in the
        # standard encoder-checkpoint format (runtime is excluded from
        # SB3 saves by design — the tfwalk-joint1 12.5GB lesson).
        online_out = MODEL_DIR / f"dyn_online_{args.name}.pt"
        torch.save({"layout_version": enc_ckpt["layout_version"],
                    "config": model._online_dyn.config(),
                    "model": model._online_dyn.state_dict(),
                    "stats": enc_ckpt["stats"],
                    "history": HISTORY}, online_out)
        print(f"  online predictor -> {online_out}")
    csv_f.close()
    venv.close()
    print(f"[{args.name}] done in {(time.time() - t0) / 60:.1f} min "
          f"-> {out}\n  eval log: {csv_path}")
    if wandb_run is not None:
        wandb_run.finish()


if __name__ == "__main__":
    main()

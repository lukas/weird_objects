"""PPO training on the MuJoCo twin (stable-baselines3).

Trains entirely in sim with domain randomization on. Two tasks:

- ``--task goal`` (default): goal-conditioned lean / track / weight-shift
  / rise (56-dim obs = 47 + 9-dim goal, 6-dim body-offset+curl action).
- ``--task balance``: the plain hold-level task (47-dim obs).

Curriculum: ``--dr-scale 0.2`` shrinks all randomization ranges toward
the calibrated nominal sim (sensor noise stays realistic) — learn the
skill first, make it robust after.

Policy I/O matches the hardware ``HexapodBalanceEnv`` conventions, so a
trained policy can later be evaluated on the robot — gated, as always,
on the operator explicitly asking for hardware motion.

Run (from prototype_sts3215/):
    # ~1 min smoke
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim --smoke

    # real run
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim \
        --steps 2000000 --n-envs 8

    # evaluate a checkpoint (sim only)
    ../../.venv/bin/python -m rl_move.sim.train_ppo_sim \
        --eval rl_move/sim/policies/ppo_balance.zip

Weights & Biases: runs log to wandb entity ``l2k2``, project
``hexapod-balance`` automatically when a key is available — put it in
``rl_move/sim/wandb.env`` (gitignored; see the placeholder there), or
``wandb login`` once, or export WANDB_API_KEY. ``--no-wandb`` disables.
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

from .servo_model import SimServoParams  # noqa: E402
from .sim_env import SimHexapodBalanceEnv  # noqa: E402
from .goal_task import SimHexapodGoalEnv  # noqa: E402
from .joint_task import SimHexapodJointGoalEnv  # noqa: E402
from .walk_task import SimHexapodJointWalkEnv  # noqa: E402

ENV_CLASSES = {"balance": SimHexapodBalanceEnv, "goal": SimHexapodGoalEnv,
               "joint_goal": SimHexapodJointGoalEnv,
               "joint_walk": SimHexapodJointWalkEnv}


def _parse_goal_mix(spec: str | None) -> dict[str, float]:
    """Parse ``--goal-mix "rise=0.4,lower=0.3"`` into {mode: prob}."""
    if not spec:
        return {}
    out = {}
    for part in spec.split(","):
        k, _, v = part.partition("=")
        out[k.strip()] = float(v)
    return out


def _parse_cfg_set(specs: list[str] | None) -> dict[str, float]:
    """Parse --cfg-set 'reward.k_current_max=0.05' overrides.

    Values parse as float; non-numeric values stay strings (cycle 27:
    goal.walk_park_bank is an npz PATH). Numeric behavior unchanged."""
    out = {}
    for part in (specs or []):
        k, _, v = part.partition("=")
        try:
            out[k.strip()] = float(v)
        except ValueError:
            out[k.strip()] = v.strip()
    return out


def _privileged_idx(args, n_obs: int) -> tuple[int, ...]:
    """Obs indices of the privileged measured-velocity dims (asym critic).

    Frame layout (walk task): [.., 2 measured-vel, (2 phase if
    goal.walk_phase_obs)]. With obs.history_frames=K the frame repeats K
    times (newest first), so the vel dims recur once per frame. The old
    hardcoded (-2, -1) is the K=1, phase-off special case.
    """
    ov = _parse_cfg_set(getattr(args, "cfg_set", None))
    k = max(1, int(ov.get("obs.history_frames", 1)))
    off = 4 if ov.get("goal.walk_phase_obs", 0.0) == 1.0 else 2
    if n_obs % k:
        raise SystemExit(f"obs width {n_obs} not divisible by "
                         f"history_frames {k}")
    w = n_obs // k
    return tuple(i * w + w - off + j for i in range(k) for j in (0, 1))


def _build_env(env_cls, params, args, **extra):
    """Construct an env honoring --friction-range, --goal-mix, --cfg-set."""
    kw = dict(params=params, randomize=not args.no_dr,
              dr_scale=args.dr_scale,
              episode_seconds=args.episode_seconds)
    overrides = _parse_cfg_set(getattr(args, "cfg_set", None))
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
    fr = getattr(args, "friction_range", None)
    if fr:
        # Friction randomized over the FULL given range regardless of
        # dr_scale (everything else still shrinks with the curriculum):
        # the belly/foot drag during rise is the top sim-to-real worry.
        from .domain_rand import DomainRandomizer
        rnd = DomainRandomizer.from_params(params, scale=args.dr_scale)
        rnd.ranges.friction_scale = (float(fr[0]), float(fr[1]))
        kw["randomizer"] = rnd
    kw.update(extra)
    env = env_cls(**kw)
    gen = getattr(env, "_goal_gen", None)
    if gen is not None:
        for mode, p in _parse_goal_mix(
                getattr(args, "goal_mix", None)).items():
            setattr(gen, f"p_{mode}", p)
    return env

POLICY_DIR = Path(__file__).resolve().parent / "policies"
WANDB_ENV_FILE = Path(__file__).resolve().parent / "wandb.env"
WANDB_ENTITY_DEFAULT = "l2k2"
WANDB_PROJECT_DEFAULT = "hexapod-balance"
# Every run in the ppo_goal continuation chain shares this W&B group,
# forks from its parent run (run stitching), and appends a record here
# so the NEXT warm start knows its parent. See _load_lineage().
WANDB_GROUP = "ppo-goal-lineage"
LINEAGE_FILE = POLICY_DIR / "lineage.json"


def _load_lineage() -> list[dict]:
    import json
    if LINEAGE_FILE.is_file():
        try:
            return json.loads(LINEAGE_FILE.read_text())
        except Exception:
            return []
    return []


def _append_lineage(rec: dict) -> None:
    import json
    records = _load_lineage()
    records.append(rec)
    LINEAGE_FILE.write_text(json.dumps(records, indent=2) + "\n")


def _parent_record(init_from: Path | None) -> dict | None:
    """Latest lineage record that produced ``init_from``."""
    if init_from is None:
        return None
    target = str(Path(init_from).resolve())
    for rec in reversed(_load_lineage()):
        if rec.get("policy") == target and rec.get("run_id"):
            return rec
    return None


def pad_obs_transplant(old_model, new_model, n_pad: int) -> None:
    """Transplant policy weights across an obs WIDENING of ``n_pad`` dims.

    The new dims must be appended at the END of the obs vector (walk
    phase clock, +2). Every tensor whose shape matches copies exactly;
    the first-layer weights of the policy/value MLPs gain ``n_pad``
    zero columns, so the transplanted policy's outputs are bit-identical
    to the parent for ANY value of the new dims until training moves
    the zero columns. Optimizer state is fresh (architecture changed).
    """
    import torch
    n_new = int(new_model.observation_space.shape[0])
    n_old = int(old_model.observation_space.shape[0])
    if n_new - n_old != n_pad:
        raise SystemExit(
            f"--obs-pad-transplant {n_pad} but obs widened by "
            f"{n_new - n_old} ({n_old} -> {n_new}); check cfg-sets")
    sd_old = old_model.policy.state_dict()
    sd_new = new_model.policy.state_dict()
    if set(sd_old) != set(sd_new):
        raise SystemExit("state_dict key mismatch; transplant needs the "
                         "same net_arch as the parent")
    widened = []
    with torch.no_grad():
        for k, v_new in sd_new.items():
            v_old = sd_old[k]
            if v_new.shape == v_old.shape:
                v_new.copy_(v_old)
            elif (v_new.dim() == 2 and v_new.shape[0] == v_old.shape[0]
                  and v_new.shape[1] == n_new
                  and v_old.shape[1] == n_old):
                v_new.zero_()
                v_new[:, :n_old].copy_(v_old)
                widened.append(k)
            else:
                raise SystemExit(f"unexpected shape change for {k}: "
                                 f"{tuple(v_old.shape)} -> "
                                 f"{tuple(v_new.shape)}")
    new_model.policy.load_state_dict(sd_new, strict=True)
    print(f"[train] obs-pad transplant: {n_old} -> {n_new} dims; "
          f"zero-padded first-layer columns in {widened}")


def _warn_if_defaults(params: SimServoParams) -> None:
    if params.source == "defaults":
        print("[train] WARNING: sim_model.json is defaults (no hardware "
              "fit yet). Fine for pipeline checks; re-train after "
              "fit_motor_model runs on real battery data.")


def _load_wandb_env() -> None:
    """Pull KEY=VALUE lines from wandb.env into os.environ (no override)."""
    if not WANDB_ENV_FILE.is_file():
        return
    for raw in WANDB_ENV_FILE.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, val = line.partition("=")
        key, val = key.strip(), val.strip()
        if key and val:
            os.environ.setdefault(key, val)


def _spec_notes() -> str:
    """Standing spec appended to every run's W&B notes so a run page is
    self-contained: action/observation space, goal modes, and the exact
    definition of every eval metric. Values are read from config.yaml so
    the text can't drift from what actually trained."""
    from rl_move.config import load_config
    cfg = load_config()
    act = cfg.get("actions", {})
    goal = cfg.get("goal", {})

    def g(sec, key, default=None):
        return sec.get(key, default)

    return f"""
=== ENVIRONMENT SPEC (auto-generated at launch) ===

ACTION SPACE — 6 continuous channels in [-1, 1] at 25 Hz. Actions are
BODY-POSE offsets, not joint angles: fixed-foot body IK converts each
action into 18 joint targets (feet stay pinned where they touched down).
  a[0] roll   -> body roll,  ±{g(act, 'max_roll_deg', 5.0)} deg
  a[1] pitch  -> body pitch, ±{g(act, 'max_pitch_deg', 5.0)} deg
  a[2] height -> body height offset, ±{g(act, 'max_height_mm', 80.0)} mm
  a[3] x      -> body fore/aft shift, ±{g(act, 'max_x_mm', 40.0)} mm
  a[4] y      -> body lateral shift,  ±{g(act, 'max_y_mm', 40.0)} mm
  a[5] curl   -> curl-in RATE (negative = no-op). Positive values ratchet
                 the foot anchors from where they started toward the plant
                 footprint (full travel in ~2.5 s at rate 1.0); anchors
                 NEVER slide back out within an episode. Needed because
                 rising from the belly with legs straight out is
                 kinematically impossible without first dragging the feet
                 under the body.
A one-pole low-pass filter smooths commanded actions before IK.

OBSERVATION — 56 dims = 47 proprioception + 9 goal. Proprioception:
complementary-filtered roll/pitch, gyro rates, 18 joint positions +
velocities, per-servo current estimate, previous action. Goal block:
roll/pitch/height references, unload-leg one-hot, mode flags. IMU
placement, geometry, latency, friction etc. are domain-randomized
(scaled by dr_scale; 0 = nominal sim, 1 = full ranges).

GOAL MODES (episode-level, sampled with config probabilities):
  hold   p={g(goal, 'p_hold')}: keep the plant stance still.
  lean   p={g(goal, 'p_lean')}: hit a fixed roll/pitch reference (<= {g(goal, 'max_ref_deg')} deg).
  track  p={g(goal, 'p_track')}: follow a moving roll/pitch reference
         (period {g(goal, 'track_period_s')} s).
  unload p={g(goal, 'p_unload')}: shift weight off one commanded leg
         (measured by foot contact force, not servo current).
  rise   p={g(goal, 'p_rise')}: start LOW and stand up
         {g(goal, 'rise_height_mm')} mm. Start kinds: 'flat' = belly on
         ground, legs straight out (must curl first); 'bridge' =
         partially curled start (reverse curriculum); 'crouch' = feet
         already under body (pure height control). Height ref holds 0
         for {g(goal, 'rise_hold_s')} s (curl window) then ramps up over
         {g(goal, 'rise_ramp_s')} s.
  raise  p={g(goal, 'p_raise')}: canary task — from the plant stance,
         follow a {g(goal, 'raise_height_mm')} mm height ramp up. If
         this isn't ~100% the height pathway is broken, not
         under-trained.

EVAL DEFINITIONS (periodic, every ~20k steps, deterministic policy):
  eval/<mode>/return          mean episode return, 2 eps/mode.
  eval/<mode>/survived_frac   fraction not safety-terminated.
  eval/<mode>/track_err_deg   mean |tilt - reference| over the episode.
  eval/<mode>/height_err_end_mm  |height - ref| at episode end.
  eval/raise_success_frac     survived AND final height err <= 5 mm
                              (tight bar on purpose — canary).
  eval/rise_flat_frac         rise completion split BY START KIND,
  eval/rise_bridge_frac       2 eps each. Completed = survived AND
  eval/rise_crouch_frac       final height err <= 15 mm. The flat/
                              bridge lines are THE metric this training
                              effort is trying to move; crouch has been
                              solved since run 02 and should stay 1.0.
Mature skills (hold/lean/track/unload/raise) are near ceiling on warm
starts — flat lines there are expected; regression is the failure signal.
Final gate after training additionally compares the policy against a
zero-action baseline on pooled episodes.
""".rstrip()


def _init_wandb(args, params: SimServoParams, parent: dict | None = None):
    """Start a W&B run, or return None (with the reason printed).

    Continuation runs (--init-from with a known parent) are stitched to
    the parent: same group, ``fork_from=<parent>?_step=<end>`` so the
    W&B UI draws them as one lineage, and the parent's id in config.
    """
    if args.no_wandb:
        return None
    _load_wandb_env()
    try:
        import wandb
    except ImportError:
        print("[wandb] not installed — pip install wandb (logging skipped)")
        return None
    has_key = bool(os.environ.get("WANDB_API_KEY"))
    if not has_key:  # `wandb login` stores the key in ~/.netrc
        try:
            has_key = bool(wandb.api.api_key)
        except Exception:
            has_key = False
    if not has_key:
        print(f"[wandb] no API key — fill in {WANDB_ENV_FILE} "
              "(or `wandb login`); logging skipped")
        return None

    config = {
        "task": args.task,
        "steps": args.steps,
        "n_envs": args.n_envs,
        "seed": args.seed,
        "episode_seconds": args.episode_seconds,
        "domain_randomization": not args.no_dr,
        "dr_scale": args.dr_scale,
        "init_from": str(args.init_from) if args.init_from else None,
        "shift_curl_bias": getattr(args, "shift_curl_bias", 0.0),
        "sim_model_source": params.source,
        "sim_model_timestamp": params.timestamp,
        "net_arch": [128, 128],
        "log_std_init": args.log_std_init,
        "n_steps": 256,
        "learning_rate": 3e-4,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "ent_coef": (args.ent_coef if args.ent_coef is not None
                     else 1e-3),
        "clip_range": 0.2,
        "target_kl": (args.target_kl if args.target_kl > 0 else None),
    }
    for axis, ax in params.axes.items():
        config[f"servo_{axis}"] = {
            "kp": ax.kp, "kv": ax.kv, "frictionloss": ax.frictionloss,
            "latency_ms": ax.latency_ms, "vel_max_deg_s": ax.vel_max_deg_s,
            "deadband_deg": ax.deadband_deg,
        }
    config["parent_run"] = parent["run_id"] if parent else None

    notes = args.notes or ""
    if parent:
        notes += (f"\n\nContinues run {parent['run_id']} "
                  f"({parent.get('run_name', '?')}) from step "
                  f"{parent.get('steps_end', '?')} via warm start of "
                  f"{Path(str(args.init_from)).name}.")
    notes += "\n\n" + _spec_notes()
    kwargs = dict(
        entity=os.environ.get("WANDB_ENTITY", WANDB_ENTITY_DEFAULT),
        project=os.environ.get("WANDB_PROJECT", WANDB_PROJECT_DEFAULT),
        dir=str(POLICY_DIR),
        group=WANDB_GROUP,
        name=args.run_name,
        notes=notes.strip() or None,
        config=config,
        sync_tensorboard=True,  # SB3 scalars (losses, ep_rew_mean, fps)
    )
    run = None
    if parent and parent.get("steps_end"):
        try:
            run = wandb.init(
                fork_from=f"{parent['run_id']}?_step="
                          f"{int(parent['steps_end'])}", **kwargs)
            print(f"[wandb] forked from {parent['run_id']} "
                  f"@ step {parent['steps_end']}")
        except Exception as e:
            print(f"[wandb] fork_from unavailable ({e}); "
                  "plain run with lineage metadata instead")
    if run is None:
        run = wandb.init(**kwargs)
    print(f"[wandb] logging to {run.url or 'offline run dir'}")
    return run


def _make_lp_curriculum_callback(every: int = 100_000,
                                 min_count: int = 500,
                                 eps: float = 0.05,
                                 lp_cap: float = 0.5):
    """Learning-progress curriculum over walk-speed buckets.

    Accumulates per-bucket mean walk_vel_err over each `every`-step
    window, converts the RELATIVE improvement vs the previous window
    into a learning-progress score |ΔE|/E_prev (capped at lp_cap; both
    improving and regressing buckets get samples — regression needs
    rehearsal), and re-weights env-side bucket sampling to
    eps + LP per bucket (normalized). Buckets that are solved or
    currently impossible have ~zero LP and decay to the eps floor.
    Weights broadcast via env_method("set_walk_bucket_weights").
    Command-speed→performance curve is logged as lp/vel_err_b*,
    lp/weight_b* (first-class W&B metrics per the plan).
    """
    from stable_baselines3.common.callbacks import BaseCallback
    from .walk_task import LP_BUCKETS

    class LPCurriculumCallback(BaseCallback):
        def __init__(self):
            super().__init__()
            self.n_b = len(LP_BUCKETS)
            self.sums = np.zeros(self.n_b)
            self.counts = np.zeros(self.n_b)
            self.prev = [None] * self.n_b
            self.next_update = None

        def _on_step(self) -> bool:
            if self.next_update is None:
                self.next_update = self.num_timesteps + every
            for info in self.locals.get("infos", ()):
                b = info.get("walk_bucket")
                if b is not None and "walk_vel_err" in info:
                    self.sums[b] += float(info["walk_vel_err"])
                    self.counts[b] += 1
            if self.num_timesteps >= self.next_update:
                self.next_update += every
                w = np.full(self.n_b, eps)
                payload = {"lp/steps": self.num_timesteps}
                for b in range(self.n_b):
                    if self.counts[b] < min_count:
                        continue
                    cur = self.sums[b] / self.counts[b]
                    payload[f"lp/vel_err_b{b}"] = cur
                    if self.prev[b] is not None:
                        rel = abs(self.prev[b] - cur) / max(
                            abs(self.prev[b]), 1e-6)
                        w[b] += min(rel, lp_cap)
                    self.prev[b] = cur
                w = w / w.sum()
                for b in range(self.n_b):
                    payload[f"lp/weight_b{b}"] = float(w[b])
                self.sums[:] = 0.0
                self.counts[:] = 0.0
                self.training_env.env_method(
                    "set_walk_bucket_weights", w)
                try:
                    import wandb
                    if wandb.run is not None:
                        wandb.log(payload)
                except Exception:
                    pass
            return True

    return LPCurriculumCallback()


def _make_reward_parts_callback():
    """Log per-rollout means of the env's reward components and tilt."""
    from stable_baselines3.common.callbacks import BaseCallback

    class RewardPartsCallback(BaseCallback):
        KEYS = ("reward_task", "reward_roll", "reward_pitch",
                "reward_height", "reward_gyro", "reward_action",
                "reward_action_delta", "reward_current", "reward_unload",
                "reward_rise_progress", "reward_rise_milestone",
                "reward_rise_finish",
                "reward_curl_progress", "reward_curl_milestone",
                "reward_walk", "reward_walk_prog", "reward_swing",
                "reward_current_hot", "reward_stance",
                "reward_clearance", "reward_flag_leg",
                "reward_current_max", "reward_termination",
                "reward_phase_contact",
                "reward_support_margin", "reward_load_even",
                "reward_step_event", "reward_drag", "reward_park_duty",
                "reward_end_posture", "walk_prog_factor",
                "reward_walk_yaw", "walk_yaw_err",
                "reward_quad_clear", "reward_quad_plant",
                "quad_clear_mm", "quad_fronts_off", "quad_planted_frac")

        def __init__(self):
            super().__init__()
            self._acc: dict[str, list[float]] = {}

        def _on_step(self) -> bool:
            for info in self.locals.get("infos", ()):
                for k in self.KEYS:
                    if k in info:
                        self._acc.setdefault(k, []).append(float(info[k]))
                for k in ("roll_deg", "pitch_deg"):
                    if k in info:
                        self._acc.setdefault(f"abs_{k}", []).append(
                            abs(float(info[k])))
                for k in ("track_err_deg", "height_err_mm",
                          "mean_current_a", "walk_vel_err", "walk_speed"):
                    if k in info:
                        self._acc.setdefault(k, []).append(
                            abs(float(info[k])))
                if "track_err_deg" in info:
                    self._acc.setdefault("pct_within_1deg", []).append(
                        1.0 if float(info["track_err_deg"]) <= 1.0 else 0.0)
            return True

        def _on_rollout_end(self) -> None:
            import wandb
            if not self._acc:
                return
            # No step= here: wandb ignores explicit steps under
            # tensorboard sync. Log global_step as a value instead so
            # env/ charts can share SB3's x-axis.
            payload = {f"env/{k}": float(np.mean(v))
                       for k, v in self._acc.items()}
            payload["global_step"] = self.num_timesteps
            wandb.log(payload)
            self._acc = {}

    return RewardPartsCallback()


def _annotate_frame(frame: np.ndarray, lines: list[str]) -> np.ndarray:
    """Burn telemetry text into the top-left of a rendered frame."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        return frame
    img = Image.fromarray(frame)
    draw = ImageDraw.Draw(img)
    y = 6
    for line in lines:
        # Cheap outline so text survives light backgrounds.
        for dx, dy in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            draw.text((8 + dx, y + dy), line, fill=(0, 0, 0))
        draw.text((8, y), line, fill=(255, 255, 80))
        y += 14
    return np.asarray(img)


EVAL_MODES = ("hold", "lean", "track", "unload", "raise", "rise")


def _rise_split_stats(env, act_fn, per_kind: int = 2) -> dict[str, tuple]:
    """Rise completion bucketed by start kind (flat / bridge / crouch).

    A pooled rise eval is uninformative: crouch starts have been solved
    since run 02, so 'rise 1/2' mostly reports which kinds were drawn.
    The split is THE improvement curve for the belly-rise effort.
    Assumes the caller already forced the goal generator to rise-only.
    """
    counts: dict[str, list[int]] = {
        "flat": [0, 0], "bridge": [0, 0], "crouch": [0, 0]}
    guard = 0
    while (any(v[1] < per_kind for v in counts.values())
           and guard < 40 * per_kind):
        guard += 1
        obs, _ = env.reset()
        traj = env._goal_traj
        kind = ("crouch" if traj.start_at == "crouch"
                else "bridge" if traj.start_curl > 0 else "flat")
        if counts[kind][1] >= per_kind:
            continue
        done = term = False
        h_err = None
        while not done:
            obs, r, term, trunc, info = env.step(act_fn(obs))
            h_err = abs(float(info.get("height_err_mm", 1e9)))
            done = term or trunc
        counts[kind][1] += 1
        if not term and h_err is not None and h_err <= 15.0:
            counts[kind][0] += 1
    return {k: (v[0], v[1]) for k, v in counts.items()}


def _run_periodic_eval(env, act, args, env_cls, step,
                       per_mode: int = 2) -> tuple[dict, str]:
    """Deterministic PER-MODE eval, returning (wandb payload, log brief).

    Training-rollout metrics are drowned in exploration noise (σ≈0.37 per
    action channel ≈ 1.8° of commanded tilt), and a mixed-mode eval hides
    per-task progress (hold's near-perfect stillness dilutes rise's 0%).
    Isolating each goal mode gives one clean improvement curve per skill:
    rise completion, raise success, per-mode tracking error.
    """
    gen = getattr(env, "_goal_gen", None)
    payload = {"global_step": step}
    brief = []
    # Env classes can override which modes get isolated (e.g.
    # the walk env adds "walk"); runs that enable an extra mode
    # via --goal-mix get it evaluated too.
    modes = list(getattr(env_cls, "EVAL_MODES", EVAL_MODES))
    for m in _parse_goal_mix(getattr(args, "goal_mix", None)):
        if m not in modes and m != "lean":
            modes.append(m)
    if gen is None:
        modes = ["balance"]
    for mode in modes:
        if gen is not None:
            # Zero every p_* the generator carries (including
            # injected ones like p_walk), then isolate the mode.
            for attr in [a for a in vars(gen) if a.startswith("p_")]:
                setattr(gen, attr, 0.0)
            setattr(gen, f"p_{mode}", 1.0)
        if mode == "rise" and gen is not None:
            # Split by start kind — the pooled number mostly
            # reports which kinds were drawn, not learning.
            split = _rise_split_stats(env, act)
            bits = []
            for kind, (done_n, tot) in split.items():
                if tot:
                    payload[f"eval/rise_{kind}_frac"] = done_n / tot
                bits.append(f"{kind[0]}{done_n}/{tot}")
            brief.append("rise " + " ".join(bits))
            continue
        stats = _rollout_stats(env, act, per_mode)
        payload[f"eval/{mode}/return"] = stats["return_mean"]
        payload[f"eval/{mode}/survived_frac"] = (
            stats["survived"] / stats["episodes"])
        if "track_err_deg_mean" in stats:
            payload[f"eval/{mode}/track_err_deg"] = (
                stats["track_err_deg_mean"])
        if "height_err_end_mm" in stats:
            payload[f"eval/{mode}/height_err_end_mm"] = (
                stats["height_err_end_mm"])
        if "raise_episodes" in stats:
            payload["eval/raise_success_frac"] = (
                stats["raise_completed"] / stats["raise_episodes"])
            brief.append(
                f"raise {stats['raise_completed']}"
                f"/{stats['raise_episodes']}")
        elif "lower_episodes" in stats:
            payload["eval/lower_success_frac"] = (
                stats["lower_completed"] / stats["lower_episodes"])
            brief.append(
                f"lower {stats['lower_completed']}"
                f"/{stats['lower_episodes']}")
        elif "walk_vel_err_mean" in stats:
            payload["eval/walk/vel_err_m_s"] = stats["walk_vel_err_mean"]
            payload["eval/walk/speed_m_s"] = stats["walk_speed_mean"]
            brief.append(f"walk err {stats['walk_vel_err_mean']:.3f} m/s")
        else:
            brief.append(
                f"{mode} {stats.get('track_err_deg_mean', 0.0):.2f}°")
    return payload, ", ".join(brief)


def _reel_modes(env, video_episodes: int) -> list[str]:
    """Modes for the video reel: every mode with sampling weight > 0,
    in a fixed priority order, cycled up to --video-episodes.

    Cycling matters for rise: its start kind (flat/bridge/crouch)
    is drawn per-episode, so repeats show different starts."""
    all_modes = ("rise", "walk", "lower", "raise",
                 "hold", "track", "lean", "unload")
    gen = getattr(env, "_goal_gen", None)
    active = [m for m in all_modes
              if gen is not None
              and getattr(gen, f"p_{m}", 0.0) > 0.0]
    if not active:
        return ["balance"] * max(1, video_episodes)
    return [active[i % len(active)] for i in range(max(1, video_episodes))]


def _render_reel(env, policy, args, step,
                 tag: str | None = None) -> tuple[str, str]:
    """Render a deterministic multi-episode reel; returns (path, caption).

    Frames carry a telemetry overlay (goal refs vs actual, reward, task)
    because return can improve while the behavior exploits a shortcut —
    the video is the audit.
    """
    import tempfile
    import imageio
    gen = getattr(env, "_goal_gen", None)
    saved_p = {m: getattr(gen, f"p_{m}")
               for m in ("hold", "lean", "track", "unload",
                         "raise", "rise", "lower", "walk")
               if gen is not None and hasattr(gen, f"p_{m}")}
    reel = _reel_modes(env, args.video_episodes)
    # Path MUST be unique per process: two runs sharing a pod (warm-started
    # twins hit identical step counts simultaneously) once interleaved
    # writes into the same /tmp/reel_<step>.mp4 and shipped corrupt video
    # to W&B. Write to a .part name and rename only when complete, so a
    # crash mid-render can never leave a plausible-looking file.
    path = (Path(tempfile.gettempdir())
            / f"reel_{os.getpid()}_{step}.mp4")
    part = path.with_name(path.name + ".part.mp4")
    # Stream to disk: a 4-episode reel held as one uint8 array
    # would be ~1 GB and OOM the process.
    # faststart + yuv420p: without leading moov metadata the W&B web
    # player misreads the duration (42 s reels showed as ~10 s).
    writer = imageio.get_writer(
        part, fps=25, macro_block_size=1, pixelformat="yuv420p",
        output_params=["-movflags", "+faststart"])
    outcomes = []
    try:
        for k, want in enumerate(reel):
            for m in saved_p:
                setattr(gen, f"p_{m}", 1.0 if m == want else 0.0)
            obs, info = env.reset()
            mode = info.get("goal_mode", want)
            header = f"[{k + 1}/{len(reel)}] task: {mode}"
            title = _annotate_frame(env.render(), [header])
            for _ in range(12):  # ~0.5 s title card per episode
                writer.append_data(title)
            done = term = False
            ret = 0.0
            while not done:
                action, _ = policy.predict(obs, deterministic=True)
                obs, r, term, trunc, info = env.step(action)
                ret += float(r)
                lines = [f"{header}   return: {ret:+.1f}"]
                if "roll_ref_deg" in info:
                    lines.append(
                        f"roll  ref {info['roll_ref_deg']:+5.2f}  "
                        f"act {info['roll_deg']:+5.2f}")
                    lines.append(
                        f"pitch ref {info['pitch_ref_deg']:+5.2f}  "
                        f"act {info['pitch_deg']:+5.2f}")
                    lines.append(
                        f"track err {info['track_err_deg']:.2f} deg")
                if "height_ref_mm" in info:
                    lines.append(
                        f"height ref {info['height_ref_mm']:+5.1f}  "
                        f"act {info['height_mm']:+5.1f} mm")
                if "walk_vel_err" in info:
                    lines.append(
                        f"vel err {info['walk_vel_err']:.3f}  "
                        f"speed {info.get('walk_speed', 0):.3f} m/s")
                if term:
                    lines.append(
                        f"TERMINATED: {info.get('termination_reason')}")
                writer.append_data(
                    _annotate_frame(env.render(), lines))
                done = term or trunc
            outcomes.append(
                f"{mode}:"
                + (f"TERM({info.get('termination_reason')})"
                   if term else "ok"))
    finally:
        writer.close()
        for m, p in saved_p.items():
            setattr(gen, f"p_{m}", p)
    part.rename(path)
    caption = (f"{tag or f'{step:,} steps'} | " + " ".join(outcomes))
    return str(path), caption


def _bg_eval_child(jobs, results, task, args) -> None:
    """Worker-process loop for background eval/video.

    Each job is a frozen checkpoint snapshot: load it, run the per-mode
    eval or render a reel, ship the result back over the queue. The
    training process does all wandb logging (single W&B writer).
    """
    from stable_baselines3 import PPO
    env_cls = ENV_CLASSES[task]
    params = SimServoParams.load()  # same file the trainer loaded from
    eval_env = None
    render_env = None
    while True:
        job = jobs.get()
        if job is None:
            return
        kind, ckpt, step, tag = job
        out = {"kind": kind, "step": step}
        try:
            policy = PPO.load(ckpt, device="cpu").policy
            policy.set_training_mode(False)
            if kind == "eval":
                if eval_env is None:
                    eval_env = _build_env(env_cls, params, args,
                                          seed=args.seed + 9999)
                act = lambda obs: policy.predict(  # noqa: E731
                    obs, deterministic=True)[0]
                payload, brief = _run_periodic_eval(
                    eval_env, act, args, env_cls, step)
                if getattr(args, "canary", False):
                    can = _run_canaries(eval_env, act)
                    for case, ok in can.items():
                        payload[f"canary/{case}"] = int(ok)
                    out["canary"] = can
                    brief += " | canary " + " ".join(
                        f"{c}={int(v)}" for c, v in can.items())
                out.update(payload=payload, brief=brief)
            else:
                if render_env is None:
                    # mesh_visuals=False: STL visual offsets are stale
                    # (June 2026 re-export), so render the primitive
                    # geometry that matches what physics simulates.
                    render_env = _build_env(
                        env_cls, params, args, seed=args.seed + 4242,
                        render_mode="rgb_array", mesh_visuals=False)
                path, caption = _render_reel(
                    render_env, policy, args, step, tag)
                out.update(path=path, caption=caption)
        except Exception as e:  # report, never crash the worker
            out["error"] = repr(e)
        finally:
            Path(ckpt).unlink(missing_ok=True)
        results.put(out)


class _BgEval:
    """Background eval/video in a separate PROCESS.

    A worker thread was tried first and cost ~60% of training
    throughput: eval rollouts, frame annotation, and mp4 encoding are
    GIL-heavy, and at any realistic cadence the worker is busy most of
    the time (the blocking in-process version before that cost ~65%).
    A process sidesteps the GIL entirely.

    submit() saves the live model to a temp zip (a snapshot — training
    keeps mutating the policy); the child loads it and evals/renders;
    drain() logs finished results to W&B from the training process.
    """

    def __init__(self, task: str, args):
        import multiprocessing as mp
        ctx = mp.get_context("spawn")  # fork is unsafe under wandb threads
        self._jobs = ctx.Queue()
        self._results = ctx.Queue()
        self._busy = {"eval": 0, "video": 0}
        self._canaries: list[dict] = []  # drained canary probe results
        self._proc = ctx.Process(
            target=_bg_eval_child,
            args=(self._jobs, self._results, task, args),
            daemon=True)
        self._proc.start()

    def submit(self, kind: str, model, step: int,
               tag: str | None = None) -> None:
        import tempfile
        ckpt = Path(tempfile.gettempdir()) / f"bgeval_{kind}_{step}.zip"
        model.save(ckpt)  # few MB; <1 s
        self._busy[kind] += 1
        self._jobs.put((kind, str(ckpt), step, tag))

    def busy(self, kind: str) -> bool:
        self.drain()
        return self._busy[kind] > 0

    def drain(self) -> None:
        """Log any finished background results; never blocks."""
        import queue as _queue
        import wandb
        while True:
            try:
                out = self._results.get_nowait()
            except _queue.Empty:
                return
            self._busy[out["kind"]] -= 1
            if "error" in out:
                print(f"[bg-{out['kind']}] skipped ({out['error']})")
            elif out["kind"] == "eval":
                wandb.log(out["payload"])
                if "canary" in out:
                    self._canaries.append(out["canary"])
                print(f"[periodic-eval] {out['step']:,}: {out['brief']}")
            else:
                wandb.log({"video/rollout": wandb.Video(
                               out["path"], format="mp4",
                               caption=out["caption"]),
                           "global_step": out["step"]})
                print(f"[video] logged reel ({out['caption']})")
                # wandb.Video copies the file into the run dir on log;
                # drop the /tmp original (unique names no longer overwrite
                # each other, so they'd otherwise accumulate).
                try:
                    Path(out["path"]).unlink()
                except OSError:
                    pass

    def pop_canaries(self) -> list[dict]:
        """Hand any drained canary probe results to the stop callback."""
        self.drain()
        out, self._canaries = self._canaries, []
        return out

    def wait(self, timeout_s: float = 1800.0) -> None:
        """Block until all in-flight jobs are logged (end of training)."""
        deadline = time.monotonic() + timeout_s
        while sum(self._busy.values()) > 0:
            self.drain()
            if time.monotonic() > deadline:
                print("[bg-eval] timed out waiting for background jobs")
                return
            time.sleep(0.5)

    def shutdown(self) -> None:
        self.wait()
        self._jobs.put(None)
        self._proc.join(timeout=30)
        if self._proc.is_alive():
            self._proc.terminate()


def _make_periodic_eval_callback(bg: "_BgEval", every: int = 200_000):
    """Trigger a background per-mode eval every N steps.

    If the previous eval is still in flight, the round is skipped —
    a late eval of a stale checkpoint is worthless.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class PeriodicEvalCallback(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = every  # skip step 0 (untrained; video covers it)

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                self._next = self.num_timesteps + every
                if bg.busy("eval"):
                    print("[periodic-eval] previous eval still running; "
                          "skipping this round")
                else:
                    bg.submit("eval", self.model, self.num_timesteps)
            bg.drain()
            return True

    return PeriodicEvalCallback()


def _make_video_callback(bg: "_BgEval", every: int, args):
    """Trigger a background telemetry-overlay video reel every N steps.

    The render env lives in the worker process (DR on, fresh draw each
    time) so the video shows the distribution the policy trains on.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class VideoCallback(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = 0  # first video = untrained policy, for contrast

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                self._next = self.num_timesteps + every
                if bg.busy("video"):
                    print("[video] previous reel still rendering; "
                          "skipping this round")
                else:
                    bg.submit("video", self.model, self.num_timesteps)
            return True

        def _on_training_end(self) -> None:
            # Final reel must make it out before the W&B run closes.
            bg.wait()
            bg.submit("video", self.model, self.num_timesteps, tag="final")

    return VideoCallback()


# ---------------------------------------------------------------------------
# Fixed-seed canaries + regression auto-stop (external review §5a/§5c).
#
# Identical cases every probe: the mid-run question becomes "did this
# checkpoint LOSE a behavior it previously demonstrated on identical
# cases?" — 2-episode random draws are binomial noise and hid the
# cw-walk-flag rise collapse for millions of steps.

CANARY_CASES: tuple[tuple[str, str, str | None, int], ...] = (
    # (case, goal mode, forced rise start kind, reset seed)
    ("rise_flat_a", "rise", "flat", 1001),
    ("rise_flat_b", "rise", "flat", 1002),
    ("rise_bridge_a", "rise", "bridge", 2001),
    ("rise_bridge_b", "rise", "bridge", 2002),
    ("rise_crouch_a", "rise", "crouch", 3001),
    ("rise_crouch_b", "rise", "crouch", 3002),
    ("lower_a", "lower", None, 4001),
    ("lower_b", "lower", None, 4002),
)
CANARY_GROUPS: dict[str, tuple[str, ...]] = {
    "rise_flat": ("rise_flat_a", "rise_flat_b"),
    "rise_bridge": ("rise_bridge_a", "rise_bridge_b"),
    "rise_crouch": ("rise_crouch_a", "rise_crouch_b"),
    "lower": ("lower_a", "lower_b"),
}


def _run_canaries(env, act_fn) -> dict[str, bool]:
    """Run the fixed-seed canary cases; success per case.

    reset(seed=N) reseeds env.rng, which drives the DR draw, the goal
    draw and the start pose — so with a deterministic policy each case
    is a repeatable episode. Mode is isolated via the generator's p_*
    (same pattern as _run_periodic_eval); the rise start kind is pinned
    through GoalGenerator.force_rise_start.
    """
    gen = getattr(env, "_goal_gen", None)
    if gen is None:
        return {}
    results: dict[str, bool] = {}
    for case, mode, force, seed in CANARY_CASES:
        if not hasattr(gen, f"p_{mode}"):
            continue
        for attr in [a for a in vars(gen) if a.startswith("p_")]:
            setattr(gen, attr, 0.0)
        setattr(gen, f"p_{mode}", 1.0)
        gen.force_rise_start = force
        obs, _ = env.reset(seed=seed)
        done = term = False
        h_err = None
        while not done:
            obs, _r, term, trunc, info = env.step(act_fn(obs))
            h_err = abs(float(info.get("height_err_mm", 1e9)))
            done = term or trunc
        results[case] = bool(
            not term and h_err is not None and h_err <= 15.0)
    gen.force_rise_start = None
    return results


def _protected_groups(baseline: dict[str, bool]) -> list[str]:
    """Groups the parent passed in FULL at launch — the protected set."""
    return sorted(
        g for g, cases in CANARY_GROUPS.items()
        if cases and all(baseline.get(c, False) for c in cases))


def _make_canary_stop_callback(bg: "_BgEval", protected: list[str],
                               stop_after: int = 3):
    """Auto-terminate when a protected skill fully collapses.

    A probe "fails" a group only when EVERY case in the group fails
    (conservative: 0/2, not 1/2 — single-case misses are noise).
    stop_after consecutive failing probes of any protected group stops
    training; the normal end-of-run save path still runs, so the last
    checkpoint survives for diagnosis. stop_after=0 monitors only.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class CanaryStopCallback(BaseCallback):
        def __init__(self):
            super().__init__()
            self.streak = {g: 0 for g in protected}

        def _on_step(self) -> bool:
            for can in bg.pop_canaries():
                for g in protected:
                    cases = CANARY_GROUPS[g]
                    seen = [can[c] for c in cases if c in can]
                    if not seen:
                        continue
                    if not any(seen):
                        self.streak[g] += 1
                    else:
                        self.streak[g] = 0
                bad = [g for g, s in self.streak.items()
                       if stop_after > 0 and s >= stop_after]
                if bad:
                    print(f"[canary] AUTO-STOP at {self.num_timesteps:,}: "
                          f"protected skill(s) {bad} failed "
                          f"{stop_after} consecutive probes "
                          f"(streaks {self.streak})")
                    try:
                        import wandb
                        wandb.log({"canary/auto_stop": 1,
                                   "global_step": self.num_timesteps})
                    except Exception:
                        pass
                    return False
            return True

    return CanaryStopCallback()


def train(args) -> int:
    from stable_baselines3 import PPO
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.callbacks import CheckpointCallback

    params = SimServoParams.load()
    _warn_if_defaults(params)
    POLICY_DIR.mkdir(parents=True, exist_ok=True)
    parent = _parent_record(args.init_from)
    run = _init_wandb(args, params, parent)

    env_cls = ENV_CLASSES[args.task]

    def env_fn():
        return _build_env(env_cls, params, args)

    # DummyVecEnv steps all envs serially in one process; --subproc gives
    # each env its own worker so n_envs can actually use n_envs cores
    # (essential on many-core cloud boxes, pointless for n_envs <= ~4).
    vec_cls = None
    if args.subproc:
        from stable_baselines3.common.vec_env import SubprocVecEnv
        vec_cls = SubprocVecEnv
    venv = make_vec_env(env_fn, n_envs=args.n_envs, seed=args.seed,
                        vec_env_cls=vec_cls)
    if args.init_from:
        # Warm start. Only works if obs/action spaces still match the
        # checkpoint (the 2026-08-07 redesign changed both, so runs from
        # before it must start fresh) — unless --obs-pad-transplant
        # bridges an obs WIDENING (new dims appended at the end).
        if args.obs_pad_transplant:
            if args.asym_critic:
                raise SystemExit("--obs-pad-transplant + --asym-critic "
                                 "is not implemented (privileged_idx "
                                 "would shift); do one at a time")
            old = PPO.load(args.init_from, device="cpu")
            model = PPO(
                "MlpPolicy", venv,
                n_steps=256,
                batch_size=min(2048, 256 * args.n_envs),
                learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                ent_coef=1e-3, clip_range=0.2,
                target_kl=(args.target_kl if args.target_kl > 0
                           else None),
                policy_kwargs=dict(net_arch=[128, 128],
                                   log_std_init=-1.0),
                seed=args.seed, verbose=1, device="cpu",
                tensorboard_log=(str(POLICY_DIR / "tb")
                                 if run else None),
            )
            pad_obs_transplant(old, model, args.obs_pad_transplant)
            model.num_timesteps = old.num_timesteps
            del old
        else:
            model = PPO.load(args.init_from, env=venv, device="cpu")
        model.verbose = 1  # checkpoints saved with verbose=0 stay silent
        if args.asym_critic:
            from .asym_policy import AsymActorCriticPolicy
            if not isinstance(model.policy, AsymActorCriticPolicy):
                # Transplant a stock-MlpPolicy champion into the
                # asymmetric policy: state_dict keys are identical (the
                # actor mask is a non-persistent buffer), so the load is
                # exact. Optimizer state is fresh (architecture change);
                # num_timesteps continues the lineage as usual.
                old = model
                model = PPO(
                    AsymActorCriticPolicy, venv,
                    n_steps=256,
                    batch_size=min(2048, 256 * args.n_envs),
                    learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                    ent_coef=1e-3, clip_range=0.2,
                    target_kl=(args.target_kl if args.target_kl > 0
                               else None),
                    policy_kwargs=dict(net_arch=[128, 128],
                                       log_std_init=-1.0,
                                       privileged_idx=_privileged_idx(
                                           args,
                                           venv.observation_space.shape[0])),
                    seed=args.seed, verbose=1, device="cpu",
                    tensorboard_log=(str(POLICY_DIR / "tb")
                                     if run else None),
                )
                res = model.policy.load_state_dict(
                    old.policy.state_dict(), strict=True)
                model.num_timesteps = old.num_timesteps
                del old
                print("[train] asym-critic transplant: champion weights "
                      f"loaded ({res}); actor masks obs dims "
                      f"{model.policy.privileged_idx}")
        # PPO.load runs _setup_model(), which calls
        # set_random_seed(self.seed) with the ANCESTOR's stored seed —
        # so warm-started "different seed" runs were bit-identical
        # clones (cw-walk-w08 vs -s1, and AGAIN cw-walk-w07 vs -s1:
        # max weight diff 0.0 after 5M steps). The 08-08 fix
        # (`model.seed = args.seed` AFTER load) was a no-op: nothing
        # re-seeds later — learn() never touches RNG. Actively re-seed
        # torch/numpy/action-space/env here instead.
        model.seed = args.seed
        model.set_random_seed(args.seed)
        # Checkpoints from before the 08-08 audit store target_kl=None;
        # apply the current flag regardless of what the parent had.
        model.target_kl = (args.target_kl if args.target_kl > 0
                           else None)
        print(f"[train] warm-started from {args.init_from} "
              f"(seed {args.seed}, target_kl {model.target_kl})")
        if args.reset_log_std:
            # Re-open exploration: the checkpoint's per-channel action
            # noise has annealed around its learned behavior — e.g. the
            # curl channel collapsed to "never curl", so zero-pose rise
            # is never even sampled. Back to the fresh-init -1.0.
            import torch
            with torch.no_grad():
                model.policy.log_std.fill_(-1.0)
            print("[train] log_std reset to -1.0 (exploration re-opened)")
        if args.shift_curl_bias:
            # Bias surgery on the curl channel (a5). Probes of runs 04
            # and 05 measured the policy's mean curl at -0.5..-0.75 on
            # belly starts: with std ~0.35 a positive curl is a ~2-sigma
            # event, so the ratchet/reward changes never even get
            # sampled. Shifting the action head's bias breaks the
            # anti-curl prior mechanically. Positive curl is a no-op
            # for every non-rise mode (anchors already at the plant
            # footprint), so the shift is nearly free elsewhere.
            import torch
            with torch.no_grad():
                model.policy.action_net.bias[5] += float(
                    args.shift_curl_bias)
            print(f"[train] curl bias shifted by "
                  f"{args.shift_curl_bias:+.2f}")
        if args.set_log_std is not None:
            # Quiet-consolidation mode. Measured on run 06's model: flat
            # rise succeeds 9/12 deterministically but 0/12 under the
            # std~0.34 training noise — so PPO, which optimizes the
            # NOISY policy's return, rationally walked away from the
            # behavior in runs 07/08. Lower noise lets the stochastic
            # return of the full rise dominate parking low.
            import torch
            with torch.no_grad():
                model.policy.log_std.fill_(float(args.set_log_std))
            print(f"[train] log_std set to {args.set_log_std:+.2f} "
                  f"(std {float(np.exp(args.set_log_std)):.2f})")
    else:
        policy_cls = "MlpPolicy"
        extra_pk = {}
        if args.asym_critic:
            from .asym_policy import AsymActorCriticPolicy
            policy_cls = AsymActorCriticPolicy
            extra_pk = dict(privileged_idx=_privileged_idx(
                args, venv.observation_space.shape[0]))
        model = PPO(
            policy_cls, venv,
            n_steps=256,
            batch_size=min(2048, 256 * args.n_envs),
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=1e-3,
            clip_range=0.2,
            target_kl=(args.target_kl if args.target_kl > 0 else None),
            # Historic default -1.0 protected the body-IK line from
            # flailing; the 08-08 audit directs 0.0 (std 1.0, field
            # standard) for from-scratch raw-joint gait arms via
            # --log-std-init.
            policy_kwargs=dict(net_arch=[128, 128],
                               log_std_init=args.log_std_init,
                               **extra_pk),
            seed=args.seed,
            verbose=1,
            device="cpu",  # MLP this small is faster on CPU than MPS
            tensorboard_log=str(POLICY_DIR / "tb") if run else None,
        )
    if args.ent_coef is not None:
        # Applies to fresh inits too (audit 08-08: from-scratch gait
        # arms use 0.005-0.01, not the historic 1e-3).
        model.ent_coef = float(args.ent_coef)
        print(f"[train] ent_coef overridden to {args.ent_coef}")
    # Output name: overridable so parallel experiments don't race to
    # overwrite the same checkpoint prefix and final .zip (2026-08-07:
    # two concurrent warm-starts were both writing ppo_goal.zip).
    name = args.out_name or f"ppo_{args.task}"
    # Fixed-seed canaries + regression auto-stop (review §5a/§5c): on by
    # default for every warm start — the failure class is a warm-started
    # run silently destroying a parent skill (cw-walk-flag's rise).
    args.canary = bool(args.init_from) and not args.no_canary
    canary_protected: list[str] = []
    if args.canary:
        cenv = _build_env(env_cls, params, args, seed=args.seed + 77777)
        act0 = lambda o: model.policy.predict(  # noqa: E731
            o, deterministic=True)[0]
        baseline = _run_canaries(cenv, act0)
        cenv.close()
        canary_protected = _protected_groups(baseline)
        print(f"[canary] parent baseline: "
              + " ".join(f"{c}={int(v)}" for c, v in baseline.items()))
        print(f"[canary] protected groups (parent passed 2/2): "
              f"{canary_protected or 'none'}")
        if run:
            run.config.update({
                "canary_baseline": {k: int(v) for k, v in baseline.items()},
                "canary_protected": canary_protected,
                "canary_stop_after": args.canary_stop_after,
            }, allow_val_change=True)
    callbacks = [CheckpointCallback(
        save_freq=max(10_000 // args.n_envs, 1000),
        save_path=str(POLICY_DIR), name_prefix=name)]
    bg = None
    if _parse_cfg_set(getattr(args, "cfg_set", None)).get(
            "goal.walk_lp_curriculum") == 1.0:
        callbacks.append(_make_lp_curriculum_callback())
        print("[train] learning-progress speed curriculum active "
              "(8 buckets 0.02-0.12 m/s, reweigh every 100k)")
    if run:
        callbacks.append(_make_reward_parts_callback())
        if args.eval_every > 0 or args.video_every > 0:
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
            callbacks.append(_make_video_callback(
                bg, args.video_every, args))
    t0 = time.monotonic()
    # Warm starts CONTINUE the step count (checkpoint num_timesteps →
    # checkpoint filenames, W&B x-axis and fork_from all line up across
    # the lineage instead of every run restarting at 0).
    #
    # NB: with reset_num_timesteps=False SB3's _setup_learn does
    # ``total_timesteps += self.num_timesteps`` itself, so pass ONLY the
    # additional steps. Passing steps_start + args.steps double-counts
    # (run 05 trained 520k of an intended 400k before this was caught).
    steps_start = int(model.num_timesteps) if args.init_from else 0
    model.learn(total_timesteps=args.steps,
                callback=callbacks, progress_bar=False,
                reset_num_timesteps=not args.init_from)
    if bg is not None:
        # Waits for the final video/eval to be logged, then reaps the
        # worker process.
        bg.shutdown()
    out = POLICY_DIR / f"{name}.zip"
    model.save(out)
    print(f"[train] {args.steps} steps in {time.monotonic() - t0:.0f}s → "
          f"{out}")
    venv.close()
    _append_lineage({
        "run_id": run.id if run else None,
        "run_name": run.name if run else args.run_name,
        "steps_start": steps_start,
        "steps_end": int(model.num_timesteps),
        "policy": str(out.resolve()),
        "parent_run": parent["run_id"] if parent else None,
        "notes": args.notes,
        "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    })

    # Quick post-train eval (DR on) so the run ends with a number.
    stats = evaluate(out, episodes=10, no_dr=args.no_dr,
                     dr_scale=args.dr_scale,
                     episode_seconds=args.episode_seconds, task=args.task,
                     cfg_set=getattr(args, "cfg_set", None))
    if run:
        import wandb
        for k, v in stats.items():
            run.summary[f"eval/{k}"] = v
        art = wandb.Artifact(name, type="model")
        art.add_file(str(out))
        run.log_artifact(art)
        run.finish()
    return 0


def _rollout_stats(env, act_fn, episodes: int) -> dict:
    """Run episodes and collect the smoke-gate metrics."""
    returns, tilts, track_errs, survived = [], [], [], 0
    within_1, within_05, currents = [], [], []
    rise_total, rise_done = 0, 0
    raise_total, raise_done = 0, 0
    lower_total, lower_done = 0, 0
    walk_errs, walk_speeds = [], []
    h_errs_end = []
    term_reasons: dict[str, int] = {}
    for _ in range(episodes):
        obs, info0 = env.reset()
        mode = info0.get("goal_mode", "balance")
        ret, max_tilt, done, term = 0.0, 0.0, False, False
        last_h_err = None
        while not done:
            obs, r, term, trunc, info = env.step(act_fn(obs))
            ret += r
            max_tilt = max(max_tilt, abs(info["roll_deg"]),
                           abs(info["pitch_deg"]))
            if "track_err_deg" in info:
                e = float(info["track_err_deg"])
                track_errs.append(e)
                within_1.append(1.0 if e <= 1.0 else 0.0)
                within_05.append(1.0 if e <= 0.5 else 0.0)
            if "height_err_mm" in info:
                last_h_err = abs(float(info["height_err_mm"]))
            if "mean_current_a" in info:
                currents.append(float(info["mean_current_a"]))
            if "walk_vel_err" in info:
                walk_errs.append(float(info["walk_vel_err"]))
                walk_speeds.append(float(info["walk_speed"]))
            done = term or trunc
        returns.append(ret)
        tilts.append(max_tilt)
        if last_h_err is not None:
            h_errs_end.append(last_h_err)
        if term:
            reason = str(info.get("termination_reason", "?"))
            term_reasons[reason] = term_reasons.get(reason, 0) + 1
        else:
            survived += 1
        if mode == "rise":
            rise_total += 1
            if not term and last_h_err is not None and last_h_err <= 15.0:
                rise_done += 1
        elif mode == "raise":
            raise_total += 1
            # Tight bar on purpose: the canary is trivial, so demand
            # the body actually parked at the commanded height.
            if not term and last_h_err is not None and last_h_err <= 5.0:
                raise_done += 1
        elif mode == "lower":
            lower_total += 1
            # Gentle-descent success: survived (no topple/overcurrent)
            # AND parked near the commanded belly height.
            if not term and last_h_err is not None and last_h_err <= 15.0:
                lower_done += 1
    stats = {
        "return_mean": float(np.mean(returns)),
        "return_std": float(np.std(returns)),
        "max_tilt_deg_mean": float(np.mean(tilts)),
        "survived": survived,
        "episodes": episodes,
        "term_reasons": term_reasons,
    }
    if track_errs:
        stats["track_err_deg_mean"] = float(np.mean(track_errs))
        stats["track_err_deg_median"] = float(np.median(track_errs))
        stats["pct_within_1deg"] = float(np.mean(within_1))
        stats["pct_within_0p5deg"] = float(np.mean(within_05))
    if currents:
        stats["mean_current_a"] = float(np.mean(currents))
    if h_errs_end:
        stats["height_err_end_mm"] = float(np.mean(h_errs_end))
    if rise_total:
        stats["rise_episodes"] = rise_total
        stats["rise_completed"] = rise_done
    if raise_total:
        stats["raise_episodes"] = raise_total
        stats["raise_completed"] = raise_done
    if lower_total:
        stats["lower_episodes"] = lower_total
        stats["lower_completed"] = lower_done
    if walk_errs:
        stats["walk_vel_err_mean"] = float(np.mean(walk_errs))
        stats["walk_speed_mean"] = float(np.mean(walk_speeds))
    return stats


def evaluate(policy_path: Path, *, episodes: int = 10, no_dr: bool = False,
             dr_scale: float = 1.0,
             episode_seconds: float | None = None,
             task: str = "balance",
             cfg_set: list[str] | None = None) -> dict:
    """Smoke-gate evaluation: policy vs zero-action baseline, same seeds.

    The gate (see RL_PLAN): the run passes only if tracking error clearly
    beats zero action, videos show movement toward targets, and safety
    terminations stay acceptable — NOT if raw return went up.
    """
    from stable_baselines3 import PPO

    params = SimServoParams.load()
    _warn_if_defaults(params)
    env_cls = ENV_CLASSES[task]
    model = PPO.load(policy_path, device="cpu")

    def make_env():
        kw = dict(params=params, randomize=not no_dr,
                  dr_scale=dr_scale,
                  episode_seconds=episode_seconds, seed=1234)
        # Honor --cfg-set: overrides can change obs WIDTH (e.g.
        # goal.walk_phase_obs) — an env built from the default cfg
        # crashed the post-train eval of any such run (found in the
        # cw-walk-phase smoke, cycle 11).
        overrides = _parse_cfg_set(cfg_set)
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
        return env_cls(**kw)

    env = make_env()
    stats = _rollout_stats(
        env, lambda obs: model.predict(obs, deterministic=True)[0],
        episodes)
    env.close()

    n_act = env.action_space.shape[0] if hasattr(env, "action_space") else 6
    env2 = make_env()
    base = _rollout_stats(env2, lambda obs: np.zeros(n_act), episodes)
    env2.close()

    def fmt(s: dict, name: str) -> str:
        msg = (f"[eval] {name}: return {s['return_mean']:+.3f} ± "
               f"{s['return_std']:.3f} | max tilt "
               f"{s['max_tilt_deg_mean']:.2f}° | survived "
               f"{s['survived']}/{s['episodes']}")
        if "track_err_deg_mean" in s:
            msg += (f" | track err {s['track_err_deg_mean']:.2f}° "
                    f"(median {s['track_err_deg_median']:.2f}°, "
                    f"within 1°: {100 * s['pct_within_1deg']:.0f}%)")
        if "rise_episodes" in s:
            msg += (f" | rise {s['rise_completed']}/{s['rise_episodes']}")
        if "raise_episodes" in s:
            msg += (f" | raise {s['raise_completed']}"
                    f"/{s['raise_episodes']}")
        if s["term_reasons"]:
            msg += f" | terms {s['term_reasons']}"
        return msg

    print(fmt(stats, policy_path.name))
    print(fmt(base, "zero-action baseline"))
    stats["baseline_return_mean"] = base["return_mean"]
    if "track_err_deg_mean" in base:
        stats["baseline_track_err_deg_mean"] = base["track_err_deg_mean"]
    stats.pop("term_reasons", None)
    return stats


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--task", choices=sorted(ENV_CLASSES), default="goal",
                    help="balance = hold level; goal = goal-conditioned "
                         "lean / track / weight-shift (default)")
    ap.add_argument("--steps", type=int, default=1_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--subproc", action="store_true",
                    help="run each env in its own process (SubprocVecEnv) "
                         "so n_envs scales across cores")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--episode-seconds", type=float, default=10.0,
                    help="sim episodes can be longer than hardware's 5 s")
    ap.add_argument("--no-dr", action="store_true",
                    help="disable domain randomization (debug only)")
    ap.add_argument("--dr-scale", type=float, default=1.0,
                    help="curriculum: 0..1 scales all randomization "
                         "ranges toward the calibrated nominal sim")
    ap.add_argument("--friction-range", type=float, nargs=2, default=None,
                    metavar=("LO", "HI"),
                    help="randomize geom friction scale over this FULL "
                         "range regardless of --dr-scale (targets the "
                         "belly-drag sim-to-real gap for rise)")
    ap.add_argument("--goal-mix", type=str, default=None,
                    help="override goal mode probabilities, e.g. "
                         "'rise=0.4,lower=0.3,hold=0.1' (unlisted modes "
                         "keep config.yaml values)")
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="DOTTED.KEY=VAL",
                    help="numeric config.yaml overrides for this run, "
                         "e.g. --cfg-set reward.k_current_max=0.05 "
                         "--cfg-set goal.rise_hold_min_s=0.5 (repeatable)")
    ap.add_argument("--init-from", type=Path, default=None,
                    help="warm-start from a saved policy (obs/action "
                         "spaces must match)")
    ap.add_argument("--reset-log-std", action="store_true",
                    help="with --init-from: reset exploration noise to "
                         "the fresh-init level (use when the checkpoint "
                         "has collapsed a channel it now needs)")
    ap.add_argument("--shift-curl-bias", type=float, default=0.0,
                    help="with --init-from: add this to the action "
                         "head's curl-channel bias (breaks a learned "
                         "anti-curl prior; harmless for non-rise modes)")
    ap.add_argument("--set-log-std", type=float, default=None,
                    help="with --init-from: set exploration log_std to "
                         "this value (e.g. -1.6 = std 0.20 for quiet "
                         "consolidation of noise-fragile behaviors)")
    ap.add_argument("--ent-coef", type=float, default=None,
                    help="override the entropy coefficient (0 lets "
                         "exploration noise anneal away instead of "
                         "being propped up)")
    ap.add_argument("--log-std-init", type=float, default=-1.0,
                    help="fresh-init exploration log_std (best-practices "
                         "audit 08-08: from-scratch gait arms should use "
                         "0.0 = std 1.0, the locomotion field standard; "
                         "the historic -1.0 default protected the old "
                         "body-IK line from flailing)")
    ap.add_argument("--target-kl", type=float, default=0.02,
                    help="PPO epoch early-stop KL threshold (audit "
                         "08-08: destructive-update guard, on for all "
                         "runs; <=0 disables)")
    ap.add_argument("--no-canary", action="store_true",
                    help="disable the fixed-seed canary probes + "
                         "regression auto-stop that warm starts get by "
                         "default (review §5a/§5c)")
    ap.add_argument("--canary-stop-after", type=int, default=3,
                    help="consecutive full-group canary failures of a "
                         "parent-passed skill before auto-stop "
                         "(0 = monitor only)")
    ap.add_argument("--asym-critic", action="store_true",
                    help="asymmetric actor-critic: mask the privileged "
                         "measured-velocity obs (last 2 dims) on the "
                         "actor path only; critic sees them (walk task)")
    ap.add_argument("--obs-pad-transplant", type=int, default=0,
                    help="warm start from a checkpoint whose obs space "
                         "is N dims NARROWER than the current env (new "
                         "dims appended at the end, e.g. the walk phase "
                         "clock): parent weights are transplanted with "
                         "zero first-layer columns for the new dims, so "
                         "behavior is bit-identical until training "
                         "learns to use them")
    ap.add_argument("--run-name", type=str, default=None,
                    help="W&B run display name (e.g. 04-200k-warm-...)")
    ap.add_argument("--notes", type=str, default=None,
                    help="W&B run notes: what this run is testing, what "
                         "changed since the parent, expected outcome")
    ap.add_argument("--out-name", type=str, default=None,
                    help="checkpoint prefix / final .zip name (default "
                         "ppo_<task>) — set for parallel experiments so "
                         "runs don't overwrite each other's policies")
    ap.add_argument("--smoke", action="store_true",
                    help="10k steps, 2 envs — pipeline check")
    ap.add_argument("--eval", type=Path, default=None,
                    help="evaluate a saved policy instead of training")
    ap.add_argument("--no-wandb", action="store_true",
                    help="disable Weights & Biases logging")
    ap.add_argument("--eval-every", type=int, default=200_000,
                    help="per-mode deterministic eval every N timesteps "
                         "(0 disables). Runs on a background thread on a "
                         "frozen policy copy, so training does not stall "
                         "(the old blocking version cost ~65%% of wall "
                         "clock); rounds are skipped if the previous eval "
                         "is still running.")
    ap.add_argument("--video-every", type=int, default=250_000,
                    help="log a rendered rollout video to W&B every N "
                         "timesteps (0 disables); renders on the same "
                         "background thread as eval")
    ap.add_argument("--video-episodes", type=int, default=4,
                    help="episodes per W&B video reel; cycles through the "
                         "active goal modes so one video covers the whole "
                         "training mix (~40 s at 4 x 10 s episodes)")
    args = ap.parse_args(argv)

    if args.eval:
        evaluate(args.eval, no_dr=args.no_dr, dr_scale=args.dr_scale,
                 episode_seconds=args.episode_seconds, task=args.task,
                 cfg_set=getattr(args, "cfg_set", None))
        return 0
    if args.smoke:
        args.steps, args.n_envs = 10_000, 2
    return train(args)


if __name__ == "__main__":
    raise SystemExit(main())

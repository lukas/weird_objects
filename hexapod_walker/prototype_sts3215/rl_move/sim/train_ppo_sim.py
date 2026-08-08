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
    """Parse --cfg-set 'reward.k_current_max=0.05' overrides."""
    out = {}
    for part in (specs or []):
        k, _, v = part.partition("=")
        out[k.strip()] = float(v)
    return out


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
        "log_std_init": -1.0,
        "n_steps": 256,
        "learning_rate": 3e-4,
        "gamma": 0.99,
        "gae_lambda": 0.95,
        "ent_coef": 1e-3,
        "clip_range": 0.2,
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
                "reward_clearance",
                "reward_current_max", "reward_termination")

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


def _make_periodic_eval_callback(env_cls, params, args, every: int = 20_000,
                                 per_mode: int = 2):
    """Deterministic PER-MODE eval every N steps, logged as eval/<mode>/*.

    Training-rollout metrics are drowned in exploration noise (σ≈0.37 per
    action channel ≈ 1.8° of commanded tilt), and a mixed-mode eval hides
    per-task progress (hold's near-perfect stillness dilutes rise's 0%).
    Isolating each goal mode gives one clean improvement curve per skill:
    rise completion, raise success, per-mode tracking error.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class PeriodicEvalCallback(BaseCallback):
        def __init__(self):
            super().__init__()
            self._next = every  # skip step 0 (untrained; video covers it)
            self._env = None

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                self._next = self.num_timesteps + every
                try:
                    self._eval()
                except Exception as e:  # never kill the run
                    print(f"[periodic-eval] skipped ({e})")
            return True

        def _eval(self) -> None:
            import wandb
            if self._env is None:
                self._env = _build_env(env_cls, params, args,
                                       seed=args.seed + 9999)
            gen = getattr(self._env, "_goal_gen", None)
            act = lambda obs: self.model.predict(  # noqa: E731
                obs, deterministic=True)[0]
            payload = {"global_step": self.num_timesteps}
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
                    for attr in [a for a in vars(gen)
                                 if a.startswith("p_")]:
                        setattr(gen, attr, 0.0)
                    setattr(gen, f"p_{mode}", 1.0)
                if mode == "rise" and gen is not None:
                    # Split by start kind — the pooled number mostly
                    # reports which kinds were drawn, not learning.
                    split = _rise_split_stats(self._env, act)
                    bits = []
                    for kind, (done_n, tot) in split.items():
                        if tot:
                            payload[f"eval/rise_{kind}_frac"] = done_n / tot
                        bits.append(f"{kind[0]}{done_n}/{tot}")
                    brief.append("rise " + " ".join(bits))
                    continue
                stats = _rollout_stats(self._env, act, per_mode)
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
                    payload["eval/walk/vel_err_m_s"] = (
                        stats["walk_vel_err_mean"])
                    payload["eval/walk/speed_m_s"] = (
                        stats["walk_speed_mean"])
                    brief.append(
                        f"walk err {stats['walk_vel_err_mean']:.3f} m/s")
                else:
                    brief.append(
                        f"{mode} {stats.get('track_err_deg_mean', 0.0):.2f}°")
            wandb.log(payload)
            print(f"[periodic-eval] {self.num_timesteps:,}: "
                  + ", ".join(brief))

    return PeriodicEvalCallback()


def _make_video_callback(env_cls, params, args):
    """Periodically render one deterministic rollout and log it to W&B.

    Runs in-process on a dedicated render env (DR on, fresh draw each
    time) so the video shows the same distribution the policy trains on.
    Frames carry a telemetry overlay (goal refs vs actual, reward, task)
    because return can improve while the behavior exploits a shortcut —
    the video is the audit.
    """
    from stable_baselines3.common.callbacks import BaseCallback

    class VideoCallback(BaseCallback):
        def __init__(self, every: int):
            super().__init__()
            self.every = every
            self._next = 0  # first video = untrained policy, for contrast
            self._env = None

        def _on_step(self) -> bool:
            if self.num_timesteps >= self._next:
                self._next = self.num_timesteps + self.every
                try:
                    self._record()
                except Exception as e:  # rendering must never kill a run
                    print(f"[video] skipped ({e})")
            return True

        def _on_training_end(self) -> None:
            try:
                self._record(tag="final")
            except Exception as e:
                print(f"[video] final render skipped ({e})")
            if self._env is not None:
                self._env.close()

        def _reel_modes(self, env) -> list[str]:
            """Modes for the reel: every mode with sampling weight > 0,
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
                return ["balance"] * max(1, args.video_episodes)
            return [active[i % len(active)]
                    for i in range(max(1, args.video_episodes))]

        def _record(self, tag: str | None = None) -> None:
            import tempfile
            import imageio
            import wandb
            if self._env is None:
                # mesh_visuals=False: STL visual offsets are stale (June
                # 2026 re-export), so render the primitive geometry that
                # matches what physics actually simulates.
                self._env = _build_env(
                    env_cls, params, args,
                    seed=args.seed + 4242, render_mode="rgb_array",
                    mesh_visuals=False)
            env = self._env
            gen = getattr(env, "_goal_gen", None)
            saved_p = {m: getattr(gen, f"p_{m}")
                       for m in ("hold", "lean", "track", "unload",
                                 "raise", "rise", "lower", "walk")
                       if gen is not None and hasattr(gen, f"p_{m}")}
            reel = self._reel_modes(env)
            path = Path(tempfile.gettempdir()) / (
                f"reel_{self.num_timesteps}.mp4")
            # Stream to disk: a 4-episode reel held as one uint8 array
            # would be ~1 GB and OOM the trainer.
            writer = imageio.get_writer(path, fps=25, macro_block_size=1)
            outcomes = []
            try:
                for k, want in enumerate(reel):
                    for m in saved_p:
                        setattr(gen, f"p_{m}",
                                1.0 if m == want else 0.0)
                    obs, info = env.reset()
                    mode = info.get("goal_mode", want)
                    header = f"[{k + 1}/{len(reel)}] task: {mode}"
                    title = _annotate_frame(env.render(), [header])
                    for _ in range(12):  # ~0.5 s title card per episode
                        writer.append_data(title)
                    done = term = False
                    ret = 0.0
                    while not done:
                        action, _ = self.model.predict(
                            obs, deterministic=True)
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
                                f"TERMINATED: "
                                f"{info.get('termination_reason')}")
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
            caption = (f"{tag or f'{self.num_timesteps:,} steps'} | "
                       + " ".join(outcomes))
            wandb.log({"video/rollout": wandb.Video(str(path),
                                                    format="mp4",
                                                    caption=caption),
                       "global_step": self.num_timesteps})
            print(f"[video] logged reel ({caption})")

    return VideoCallback(every=args.video_every)


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
        # before it must start fresh).
        model = PPO.load(args.init_from, env=venv, device="cpu")
        model.verbose = 1  # checkpoints saved with verbose=0 stay silent
        # PPO.load restores the ANCESTOR's seed, and learn() re-seeds
        # torch + env from model.seed — so warm-started "different seed"
        # runs were bit-identical clones (cw-walk-w08 vs -s1 proved it:
        # max weight diff 0.0 after 5M steps). Honor --seed for real.
        model.seed = args.seed
        print(f"[train] warm-started from {args.init_from} "
              f"(seed {args.seed})")
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
        if args.ent_coef is not None:
            model.ent_coef = float(args.ent_coef)
            print(f"[train] ent_coef overridden to {args.ent_coef}")
    else:
        model = PPO(
            "MlpPolicy", venv,
            n_steps=256,
            batch_size=min(2048, 256 * args.n_envs),
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=1e-3,
            clip_range=0.2,
            # log_std_init=-1: start exploring with ~1° body commands, not
            # full ±3° swings 25×/s — σ≈1 flailing physically tips the
            # robot and every episode dies before the policy sees signal.
            policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0),
            seed=args.seed,
            verbose=1,
            device="cpu",  # MLP this small is faster on CPU than MPS
            tensorboard_log=str(POLICY_DIR / "tb") if run else None,
        )
    # Output name: overridable so parallel experiments don't race to
    # overwrite the same checkpoint prefix and final .zip (2026-08-07:
    # two concurrent warm-starts were both writing ppo_goal.zip).
    name = args.out_name or f"ppo_{args.task}"
    callbacks = [CheckpointCallback(
        save_freq=max(10_000 // args.n_envs, 1000),
        save_path=str(POLICY_DIR), name_prefix=name)]
    if run:
        callbacks.append(_make_reward_parts_callback())
        if args.eval_every > 0:
            callbacks.append(_make_periodic_eval_callback(
                env_cls, params, args, every=args.eval_every))
        if args.video_every > 0:
            callbacks.append(_make_video_callback(env_cls, params, args))
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
                     episode_seconds=args.episode_seconds, task=args.task)
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
             task: str = "balance") -> dict:
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
        return env_cls(params=params, randomize=not no_dr,
                       dr_scale=dr_scale,
                       episode_seconds=episode_seconds, seed=1234)

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
    ap.add_argument("--eval-every", type=int, default=20_000,
                    help="per-mode deterministic eval every N timesteps; "
                         "eval runs serially in the main process, so raise "
                         "this when env stepping is fast (e.g. --subproc "
                         "on many cores)")
    ap.add_argument("--video-every", type=int, default=25_000,
                    help="log a rendered rollout video to W&B every N "
                         "timesteps (0 disables)")
    ap.add_argument("--video-episodes", type=int, default=4,
                    help="episodes per W&B video reel; cycles through the "
                         "active goal modes so one video covers the whole "
                         "training mix (~40 s at 4 x 10 s episodes)")
    args = ap.parse_args(argv)

    if args.eval:
        evaluate(args.eval, no_dr=args.no_dr, dr_scale=args.dr_scale,
                 episode_seconds=args.episode_seconds, task=args.task)
        return 0
    if args.smoke:
        args.steps, args.n_envs = 10_000, 2
    return train(args)


if __name__ == "__main__":
    raise SystemExit(main())

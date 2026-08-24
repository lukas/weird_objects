"""Sequence eval instrument: rise -> walk -> sit(lower) -> rise -> walk.

CODE item 3 of the operator's 08-13 ~21:00 UTC TRANSITIONS_DIRECTIVE
(rl_docs/tracks/arch/TRANSITIONS_DIRECTIVE.md): "a FIXED command
schedule ... reports PER SEGMENT ... This is the gate instrument for
BOTH arms -- build and baseline it on the two-specialist composition
(known zero-fall) before gating anything."

THE QUESTION this baseline answers: does composing the ALREADY-PROVEN
pairwise handoffs (`eval_handoff.py`: rise->walk, zero falls 08-11;
`eval_handoff_reverse.py`: walk->lower, zero falls 08-11) into a LONGER
chain -- specifically the SECOND rise, which starts from wherever
`lower` left the robot instead of a pristine cold reset -- still holds
zero falls? That second rise is exactly the directive's failure-ledger
risk #4/#5 (the two stances differ; height refs are start-relative).

Mechanism: generalizes both scripts' proven re-anchor trick
(`reanchor_keep_state` / `reanchor_lower_keep_state`) into one
`reanchor_to(mode, ...)` helper applied at EVERY segment boundary, N
times in a row -- goal references (height frame _z0, pad-z ref, tilt
ref, q_nom, safety nominal) are re-derived by a real env.reset() call
at the target mode, then the walker's/specialist's exact qpos/qvel/
ctrl/act and the safety layer's slew memory are restored on top, so
nothing is ever teleported. No env/reward code is touched by this
script -- it is pure external orchestration of two already-trained
policies, so it carries zero risk to shared training-time defaults.

This is NOT the training-time `goal.mode_seq` feature the directive
also calls for (CODE item 1, still open -- see arch/STATUS.md "Next"):
that item is required so PPO can train ON sequences natively inside
the vectorized MJX stack; this instrument only EVALUATES sequences
composed from already-trained specialists/single-mode checkpoints and
is the gate/baseline both future arms must beat.

Per segment: mode, start_kind (rise only), fall (or None), and the
segment's OWN existing criterion (rise/lower: `env.plant_report` +
height_err; walk: tracking error + gait_valid + prog_ratio, same
definitions as eval_checkpoint's walk letter). Per episode: zero_fall
over the whole grammar. Summary: per-segment-type success rate +
fraction of episodes with zero falls end to end, det and stochastic
passes both supported (--stochastic).

Two drive modes:

- TWO-SPECIALIST (default, the baseline): `--stand` drives rise/lower
  segments on the 68-obs prefix, `--walk` drives walk segments.
- SINGLE MODEL (`--single <ckpt>`, the gate path for the directive's
  Arm 1/2 artifacts): ONE checkpoint drives every segment on the full
  obs. GRU (RecurrentPPO) checkpoints keep their hidden state ACROSS
  segment switches within a sequence (reset only at true episode
  start) — exactly the continuous-stream contract `distill_gru
  --transitions` trains, so the eval never lobotomizes the memory at a
  switch. Dual-core GRU checkpoints (obs = env + mode one-hot) are
  auto-detected from the stored obs width and the env is rebuilt with
  `obs.mode_onehot=1`, same convention as eval_checkpoint.

Per-segment criteria (tightened 08-13 for the Arm 1 gate — failure-
ledger lesson 11: cheat gates must apply PER SEGMENT, pooled numbers
lie): walk success now ALSO requires gait_valid (no persistently
sacrificed leg, identical duty/swings formula to eval_checkpoint) —
a parked-leg walk segment can no longer read OK on tracking error
alone; prog_ratio and slip_per_m are reported per segment (evidence,
no bar here — the triaging cycle judges vs the baseline). Every
segment additionally reports switch_tilt_deg / switch_peak_a (max
|roll|/|pitch| and peak servo current over the first 1.5 s after its
re-anchor — the directive's switch-window evidence field, no bar in
v1; on the first, cold segment the window reads engagement, not a
switch).

    # baseline (two specialists)
    uv run python -m rl_move.sim.eval_modeseq \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip \
        --walk  rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --episodes 12 --grammar rise,walk,lower,rise,walk \
        --out logs/ckpt_eval/modeseq_baseline.json

    # one-model gate (Arm 1 / Arm 2 artifacts)
    uv run python -m rl_move.sim.eval_modeseq \
        --single rl_move/sim/policies/ppo_goal_cw_gru_dual_bc_transdagger1.zip \
        --episodes 12 --grammar rise,walk,lower,rise,walk \
        --out logs/ckpt_eval/modeseq_transdagger1.json

Verdict rule of thumb (matches the directive's Arm 1/2 gates): PASS if
zero_fall_frac >= 11/12 AND every segment TYPE's own success rate is
inside its pairwise-handoff band (rise ~= eval_handoff "direct", lower
~= eval_handoff_reverse "direct", walk tracking inside eval_handoff's
plant-arm band). A regression on the SECOND rise specifically (vs the
first) isolates the start-relative-_z0 risk named above.

SESSION-JOYSTICK GATE PATH (hw mainline, 08-14 — default OFF, the
flags below leave every legacy invocation bit-exact): `--drive-random`
replaces the fixed walk schedule with a per-episode randomized
joystick DRIVE — zero-command engage dwell (WALK_ENTRY), random
forward/fwd-diagonal segments at the trained speed band with a
guaranteed stop-go and a guaranteed direction flip, trailing
zero-command STOP_SETTLE window (guard: mean body speed < 0.02 m/s,
reported per segment) — and `--entry-slew 1.5,0.25` engages the
SafetyLayer entry slew ramp (rl_docs/TAKEOFF.md deploy design) at
each walk-segment policy handoff, exactly the runner's staged
gait-entry switch, scoped to walk engages only. Together with the
grammar `rise,walk,lower,rise,walk` this is the ~60 s guarded
specialist session REST->RISE->SETTLE->WALK_ENTRY->DRIVE->
STOP_SETTLE->LOWER->RISE->DRIVE:

    uv run python -m rl_move.sim.eval_modeseq \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_footlow2_hard1.zip \
        --walk  rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip \
        --episodes 12 --grammar rise,walk,lower,rise,walk \
        --drive-random --drive-speed-max 0.06 --entry-slew 1.5,0.25 \
        --cfg-set goal.walk_obs_body_vel=2 \
        --out logs/ckpt_eval/session_joystick_handoff1_det.json
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import math
from pathlib import Path

import numpy as np

# Walk-segment drive schedule: settle, then commanded forward, then stop
# -- identical shape to eval_handoff.py / eval_handoff_reverse.py so the
# per-segment walk numbers are directly comparable to their bands.
WALK_SCHEDULE = lambda v: [(1.0, 0.0, 0.0), (4.0, v, 0.0), (1.0, 0.0, 0.0)]

# --drive-random headings: forward-weighted, fwd-diagonal scope (the
# operator's ruled command envelope; matches the legacy training mix's
# 60% fwd / +-45deg band that every walk candidate saw in training).
DRIVE_HEADINGS = (0.0, 0.0, math.pi / 7.2, -math.pi / 7.2,   # 0deg x2, +-25deg
                  math.pi / 4, -math.pi / 4)                  # +-45deg
DRIVE_ENGAGE_S = 1.0        # zero-command dwell at policy handoff
DRIVE_STOP_SETTLE_S = 1.5   # trailing zero-command settle window


def _random_drive_schedule(rng, v_lo: float, v_hi: float,
                           seconds: float) -> list:
    """Randomized joystick DRIVE schedule (session-joystick gate,
    08-14): zero-command engage dwell, then random (dur, vx, vy)
    segments inside the trained envelope -- forward / fwd-diagonal
    headings at the trained speed band -- with a guaranteed mid-drive
    stop-go and a guaranteed direction flip (left-diag -> right-diag
    or vice versa), then a trailing zero-command STOP_SETTLE window.
    Purely additive: only used when --drive-random is passed."""
    segs = [(DRIVE_ENGAGE_S, 0.0, 0.0)]
    t, last_ang = 0.0, None
    flip_done = stop_done = False
    while t < seconds:
        if not stop_done and t >= seconds * 0.4:
            segs.append((1.5, 0.0, 0.0))          # stop-go
            stop_done = True
            t += 1.5
            continue
        dur = float(rng.uniform(1.5, 3.5))
        if not flip_done and last_ang not in (None, 0.0):
            ang = -last_ang                        # direction flip
            flip_done = True
        else:
            ang = float(DRIVE_HEADINGS[int(rng.integers(
                len(DRIVE_HEADINGS)))])
        sp = float(rng.uniform(v_lo, v_hi))
        segs.append((dur, sp * math.cos(ang), sp * math.sin(ang)))
        last_ang = ang
        t += dur
    segs.append((DRIVE_STOP_SETTLE_S, 0.0, 0.0))
    return segs
LOWER_PHASE_S = 10.0        # lower_hold_s(1) + lower_ramp_s(5) + settle
RISE_PHASE_S = 12.5         # eval_handoff.py's PHASE_A_S (worst-case rise)
TAIL_S = 0.5
END_CLEAR_BELLY_MM = 60.0   # eval_checkpoint.py posture-strict lower rule
HEIGHT_ERR_OK_MM = 15.0     # eval_checkpoint.py lower/rise success rule
RISE_START_KINDS = ("flat", "bridge", "crouch")   # cold-start rotation
CONTACT_N = 0.5             # eval_checkpoint.py touch-force threshold
SWITCH_WIN_S = 1.5          # switch-window evidence (directive item 3)


def _set_mix(gen, **p) -> None:
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 0.0
    for k, v in p.items():
        setattr(gen, f"p_{k}", v)


def rise_from_h_traj(env, cfg: dict):
    """Build a post-lower rise segment's goal trajectory with the REAL
    trained generator instead of the legacy cold-reset sampler --
    `SimHexapodGoalEnv._seq_segment_traj("rise", tick=0)` with
    `goal.mode_seq_rise_from_h` forced to 1 for the call only (cfg is
    restored immediately after, so nothing else this run reads a
    mutated cfg). Requires `env._z0` already installed (the belly
    frame `reanchor_to`'s inner `env.reset()` set BEFORE the physical
    state was restored) and `env.data`/`env.rng` to reflect the
    robot's real carried-over state (i.e. call AFTER the qpos
    restore + mj_forward, same precondition as `reanchor_to`'s
    return). Root cause this exists (2026-08-17, hw WAITING-ON): a
    naive `--cfg-set goal.mode_seq_rise_from_h=1` is a confirmed
    no-op through this harness -- that key is only read inside
    `_sample_mode_seq_stance`, gated on `goal.mode_seq_stance>0`, a
    code path the reanchor-based composed-specialist sequencing never
    enters (`_sample_goal` falls straight to the legacy generator).
    `_seq_segment_traj` is the actual training-time method that reads
    the key; calling it directly is the only way to get its exact
    hold_n=1.0s/ramp-jitter formula instead of re-deriving it.
    `env._seq_stand_z` is left at its current value (None for the
    first post-lower rise in a composed sequence, since these
    specialists don't track a running "last commanded stand height"
    across segments) -- the same amp draw the legacy sampler would
    make (`rng.uniform(*gen.rise_m)`), only the height SCHEDULE
    SHAPE changes. Returns (traj, h_target).
    """
    goal_cfg = cfg.setdefault("goal", {})
    had_key = "mode_seq_rise_from_h" in goal_cfg
    prev = goal_cfg.get("mode_seq_rise_from_h")
    goal_cfg["mode_seq_rise_from_h"] = 1
    try:
        traj, h_target, _ramp_i0 = env._seq_segment_traj("rise", 0)
    finally:
        if had_key:
            goal_cfg["mode_seq_rise_from_h"] = prev
        else:
            del goal_cfg["mode_seq_rise_from_h"]
    return traj, h_target


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stand", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_holdbc1_hard1.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
    ap.add_argument("--single", type=Path, default=None,
                    help="ONE checkpoint drives every segment (the "
                         "one-model gate path); overrides --stand/"
                         "--walk. GRU hidden state persists across "
                         "segment switches (reset at episode start "
                         "only); dual-core GRU obs one-hot handled "
                         "automatically")
    ap.add_argument("--episodes", type=int, default=12,
                    help="episodes total (cold-start kind rotates "
                         "flat/bridge/crouch)")
    ap.add_argument("--grammar", type=str, default="rise,walk,lower,"
                                                   "rise,walk",
                    help="comma list of rise|walk|lower segment tokens; "
                         "must start with rise or lower (walk needs a "
                         "prior stance) and each rise/lower after the "
                         "first re-anchors from wherever the previous "
                         "segment left the robot, not a cold reset")
    ap.add_argument("--speed", type=float, default=0.05)
    ap.add_argument("--drive-random", action="store_true",
                    help="walk segments use a per-episode randomized "
                         "joystick schedule (fwd/diagonal/stop-go/"
                         "direction-flip inside the trained envelope) "
                         "instead of the fixed settle-fwd-stop schedule "
                         "-- the session-joystick gate path (08-14)")
    ap.add_argument("--drive-seconds", type=float, default=14.0,
                    help="commanded portion of each --drive-random "
                         "walk segment (default 14 s)")
    ap.add_argument("--drive-speed-max", type=float, default=None,
                    help="upper speed for --drive-random draws "
                         "(default = --speed, i.e. fixed speed)")
    ap.add_argument("--entry-slew", type=str, default=None,
                    metavar="RAMP_S,START_DEG",
                    help="engage the SafetyLayer entry slew ramp at "
                         "each walk-segment policy handoff (e.g. "
                         "'1.5,0.25' = the TAKEOFF.md deploy design); "
                         "scoped to walk engages only, default off = "
                         "bit-exact legacy behavior")
    ap.add_argument("--rise-from-h", action="store_true",
                    help="mid-sequence (post-lower) rise segments use "
                         "the REAL trained schedule generator "
                         "(`SimHexapodGoalEnv._seq_segment_traj` with "
                         "`goal.mode_seq_rise_from_h=1`) instead of the "
                         "legacy cold-reset sampler, so the height "
                         "goal holds at the robot's CURRENT height "
                         "and ramps to the stand target with the "
                         "SAME 1.0s-hold/ramp-jitter formula "
                         "postlower4 was trained on -- a naive "
                         "`--cfg-set goal.mode_seq_rise_from_h=1` is a "
                         "confirmed no-op here (that key is only read "
                         "by the in-context `_sample_mode_seq_stance` "
                         "path, which this reanchor-based harness "
                         "never enters). WAITING-ON 08-17 hw: prices "
                         "the postlower operator fork (train==deploy "
                         "schedule) before a product-contract change. "
                         "Default off = bit-exact legacy reanchor "
                         "schedule; the first (cold) rise is never "
                         "affected.")
    ap.add_argument("--stochastic", action="store_true",
                    help="both policies predict stochastically (default "
                         "deterministic)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--torch-seed", type=int, default=None,
                    help="seed torch's global RNG at start (makes "
                         "--stochastic runs reproducible / resumable "
                         "shard-exact for the bulk session cohort); "
                         "default None = legacy behavior untouched")
    ap.add_argument("--strip-ep", type=int, default=0,
                    help="which episode index --strips renders "
                         "(default 0 = legacy; the bulk rerender path "
                         "re-runs a shard up to a failed episode and "
                         "saves THAT episode's strip)")
    ap.add_argument("--rise-height-mm", default="108,114",
                    help="specialist's trained plant band (eval_handoff "
                         "default)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--strips", type=Path, default=None,
                    help="dir for 1 fps frame-strip PNGs (episode 0 only)")
    ap.add_argument("--cfg-set", action="append", default=None,
                    metavar="K=V")
    args = ap.parse_args()

    import mujoco
    from stable_baselines3 import PPO

    if args.torch_seed is not None:
        import torch
        torch.manual_seed(args.torch_seed)

    from rl_move.config import load_config
    from rl_move.env import build_obs
    from .joint_task import q_rad_to_action  # noqa: F401  (parity import)
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    grammar = [t.strip() for t in args.grammar.split(",") if t.strip()]
    if not grammar:
        raise SystemExit("--grammar must name at least one segment")
    if grammar[0] not in ("rise", "lower"):
        raise SystemExit("--grammar must start with rise or lower "
                          "(walk needs a prior stance)")
    for t in grammar:
        if t not in ("rise", "walk", "lower"):
            raise SystemExit(f"--grammar unknown token: {t}")

    cfg = load_config()
    lo, hi = (float(x) for x in args.rise_height_mm.split(","))
    cfg.setdefault("actions", {})["max_height_mm"] = max(hi + 1.0, 115.0)
    g = cfg.setdefault("goal", {})
    g["rise_height_mm"] = [lo, hi]
    g["rise_ramp_s"] = 6.0
    g["rise_rsi_frac"] = 0.0
    for spec in (args.cfg_set or []):
        key, val = spec.split("=", 1)
        sect, name = key.split(".", 1)
        try:
            parsed: float | str = float(val)
        except ValueError:
            parsed = val.strip()
        cfg.setdefault(sect, {})[name] = parsed

    det = not args.stochastic

    entry_slew = None
    if args.entry_slew:
        ramp_s, start_deg = (float(x) for x in
                             args.entry_slew.split(","))
        entry_slew = (ramp_s, math.radians(start_deg))
    drive_rng = (np.random.default_rng(args.seed)
                 if args.drive_random else None)

    def make_env():
        return SimHexapodJointWalkEnv(
            params=SimServoParams.from_cfg(cfg), cfg=cfg,
            randomize=False, episode_seconds=20.0, seed=args.seed,
            render_mode="rgb_array" if args.strips else None)

    if args.single is not None:
        # ---- one-model gate path (Arm 1/2 artifacts) ----------------
        from .gru_policy import is_recurrent_checkpoint, \
            load_checkpoint_auto
        single = load_checkpoint_auto(args.single, device="cpu")
        n_model = int(single.observation_space.shape[0])
        env = make_env()
        n_env = int(env.observation_space.shape[0])
        if n_model != n_env:
            from .walk_task import N_MODE_OBS
            if n_model == n_env + N_MODE_OBS:
                # dual-core GRU convention (eval_checkpoint parity)
                print(f"[modeseq] checkpoint obs {n_model} = env "
                      f"{n_env} + {N_MODE_OBS}: enabling "
                      f"obs.mode_onehot (dual-core checkpoint)")
                env.close()
                cfg.setdefault("obs", {})["mode_onehot"] = 1.0
                env = make_env()
                n_env = int(env.observation_space.shape[0])
            if n_model != n_env:
                raise SystemExit(f"--single obs {n_model} does not "
                                 f"match env obs {n_env}")
        recurrent = is_recurrent_checkpoint(args.single)
        _rec = {"state": None, "start": np.ones((1,), dtype=bool)}

        def episode_begin() -> None:
            # Hidden state clears at TRUE episode start only; segment
            # switches keep it (the --transitions continuous-stream
            # contract — resetting here would evaluate a lobotomy).
            _rec["state"] = None
            _rec["start"] = np.ones((1,), dtype=bool)

        if recurrent:
            def act(obs, mode):
                a, _rec["state"] = single.policy.predict(
                    obs, state=_rec["state"],
                    episode_start=_rec["start"], deterministic=det)
                _rec["start"] = np.zeros((1,), dtype=bool)
                return a
        else:
            def act(obs, mode):
                return single.predict(obs, deterministic=det)[0]
    else:
        # ---- two-specialist baseline path ----------------------------
        env = make_env()
        stand = PPO.load(args.stand, device="cpu")
        walk = PPO.load(args.walk, device="cpu")
        n_stand = int(stand.observation_space.shape[0])
        n_env = int(env.observation_space.shape[0])
        assert walk.observation_space.shape[0] == n_env, (
            f"walk policy obs {walk.observation_space.shape} != env "
            f"{n_env}")
        assert n_stand < n_env, (
            "stand policy obs must be a prefix of the walk env obs "
            f"(got {n_stand} vs {n_env})")

        def episode_begin() -> None:
            pass

        def act(obs, mode):
            if mode == "walk":
                return walk.predict(obs, deterministic=det)[0]
            return stand.predict(obs[:n_stand], deterministic=det)[0]

    gen = env._goal_gen
    dt = env.dt
    tail_n = max(1, int(round(TAIL_S / dt)))

    def chassis_z() -> float:
        return float(env.data.xpos[env._chassis_bid, 2])

    def pad_z() -> np.ndarray:
        return np.array([float(env.data.xpos[b, 2])
                         for b in env._pad_bids])

    strip_frames: list = []

    def grab(final: bool = False) -> None:
        if args.strips is None:
            return
        if final or (grab.n % max(1, int(round(1.0 / dt)))) == 0:
            strip_frames.append(env.render())
        grab.n += 1
    grab.n = 0

    def save_strip(name: str) -> None:
        if args.strips is None or not strip_frames:
            return
        import imageio.v2 as imageio
        args.strips.mkdir(parents=True, exist_ok=True)
        imageio.imwrite(args.strips / f"{name}.png",
                        np.hstack(strip_frames))
        strip_frames.clear()

    def reanchor_to(mode: str, *, force_rise_start: str | None = None):
        """Fresh <mode> episode at a clean reference frame, physics kept.

        Generalizes eval_handoff.reanchor_keep_state /
        eval_handoff_reverse.reanchor_lower_keep_state to any target
        mode: env.reset() re-derives every start-relative goal
        reference (height frame _z0, pad-z ref, q_nom, tilt ref, safety
        nominal) from mujoco's own canonical reset pose for that mode,
        then the CURRENT physical state (qpos/qvel/ctrl/act + the
        safety layer's slew memory) is written back on top -- nothing
        is teleported, only the bookkeeping frame moves.
        """
        d = env.data
        keep_qpos, keep_qvel = d.qpos.copy(), d.qvel.copy()
        keep_ctrl = d.ctrl.copy()
        keep_act = d.act.copy() if d.act.size else None
        keep_safe = env.safety._last_safe.copy()
        _set_mix(gen, **{mode: 1.0})
        if mode == "rise" and force_rise_start is not None:
            gen.force_rise_start = force_rise_start
        env.reset()
        gen.force_rise_start = None
        d.qpos[:] = keep_qpos
        d.qvel[:] = keep_qvel
        d.ctrl[:] = keep_ctrl
        if keep_act is not None:
            d.act[:] = keep_act
        env.safety._last_safe = keep_safe
        mujoco.mj_forward(env.model, env.data)
        env._state = env._read_state()
        return env._final_obs(
            build_obs(env.cfg, env._state, env._q_nom,
                      env._prev_action, goal=env._current_goal(),
                      tilt_ref=env._tilt_ref0), reset=True)

    switch_win_n = max(1, int(round(SWITCH_WIN_S / dt)))

    class _SegMeter:
        """Switch-window evidence (directive item 3, no bar in v1):
        max |roll|/|pitch| and peak servo current over the first
        SWITCH_WIN_S after the segment's re-anchor (on the first,
        cold segment the window reads engagement, not a switch)."""

        def __init__(self) -> None:
            self.n, self.tilt, self.amp = 0, 0.0, 0.0

        def tick(self, info: dict) -> None:
            if self.n < switch_win_n:
                self.tilt = max(self.tilt,
                                abs(float(info.get("roll_deg", 0.0))),
                                abs(float(info.get("pitch_deg", 0.0))))
                cur = getattr(env._state, "servo_current", None)
                if cur is not None and len(cur):
                    self.amp = max(self.amp, float(np.max(cur)))
            self.n += 1

        def finalize(self, rec: dict) -> None:
            rec["switch_tilt_deg"] = round(self.tilt, 1)
            rec["switch_peak_a"] = round(self.amp, 2)

    def score_posture(rec: dict, pad_hist: list, h_err_mm) -> None:
        clear_mm = (np.asarray(pad_hist[-tail_n:]).mean(axis=0)
                    - env._pad_z_ref) * 1000.0
        rec["end_clear_mm"] = [round(float(c), 1) for c in clear_mm]
        rec["end_posture_ok"] = bool(
            (clear_mm <= END_CLEAR_BELLY_MM).all())
        rec["height_err_end_mm"] = (None if h_err_mm is None
                                    else round(abs(h_err_mm), 1))
        rec["success"] = bool(
            rec.get("fall") is None and rec["end_posture_ok"]
            and (h_err_mm is None
                 or abs(h_err_mm) <= HEIGHT_ERR_OK_MM))

    def run_rise(obs, rec: dict, cold: bool, start_kind: str | None):
        """Rise segment: cold (fresh reset, forced start kind) or a
        reanchor from wherever the prior segment left the robot."""
        if cold:
            _set_mix(gen, rise=1.0)
            gen.force_rise_start = start_kind
            obs, _ = env.reset()
            gen.force_rise_start = None
            rec["start_kind"] = start_kind
        else:
            obs = reanchor_to("rise", force_rise_start="flat")
            rec["start_kind"] = "reanchor_post_lower"
            if args.rise_from_h:
                traj, h_target = rise_from_h_traj(env, cfg)
                env._goal_traj = traj
                env._h_target = float(h_target)
                obs = env._final_obs(
                    build_obs(env.cfg, env._state, env._q_nom,
                              env._prev_action, goal=env._current_goal(),
                              tilt_ref=env._tilt_ref0), reset=True)
                rec["rise_from_h_start_mm"] = round(
                    (chassis_z() - env._z0) * 1000.0, 1)
        meter = _SegMeter()
        for _ in range(int(round(RISE_PHASE_S / dt))):
            a = act(obs, "rise")
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            meter.tick(info)
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                rec["success"] = False
                meter.finalize(rec)
                return obs, False
        meter.finalize(rec)
        h_err = chassis_z() - (env._z0 + env._h_target)
        ok, detail = env.plant_report(height_err_m=h_err)
        rec["success"] = bool(ok)
        rec["height_err_end_mm"] = round(h_err * 1000.0, 1)
        if not ok:
            rec["fail_detail"] = [k for k, v in detail.items()
                                  if k.endswith("_ok") and not v]
        return obs, True

    def run_walk(obs, rec: dict, schedule=None):
        obs = reanchor_to("walk")
        if entry_slew is not None:
            # Deploy-design walk engage (TAKEOFF.md): per-tick slew
            # starts at start_dq and ramps to max_dq over ramp_s after
            # this handoff. reanchor_to's inner env.reset() already
            # zeroed _entry_ticks via set_nominal(); scope the ramp to
            # WALK engages only by enabling it here and disabling it
            # at segment end (rise/lower handoffs stay legacy).
            env.safety.entry_ramp_s, env.safety.entry_start_dq = \
                entry_slew
            env.safety._entry_ticks = 0
        if schedule is None:
            schedule = WALK_SCHEDULE(args.speed)
        traj = env._goal_traj
        p0 = np.array(env.data.qpos[:2], dtype=float)
        n_err, err_sum = 0, 0.0
        cmd_dist_m, along_dist_m = 0.0, 0.0
        contact_hist: list = []      # (T, 6) bool
        pad_xy_hist: list = []       # (T, 6, 2) world
        pads = env._pad_bids
        meter = _SegMeter()

        def gait_metrics() -> None:
            # Identical definitions to eval_checkpoint (duty/swings/
            # sacrificed/gait_valid, prog_ratio, slip_per_m) so the
            # per-segment numbers are directly comparable to the
            # harness letter — failure-ledger lesson 11: cheat gates
            # apply PER SEGMENT, never pooled.
            if not contact_hist:
                rec["gait_valid"] = False
                return
            contact = np.asarray(contact_hist, dtype=bool)
            pad_xy = np.asarray(pad_xy_hist)
            duty = contact.mean(axis=0)
            swings, slips = [], []
            for f in range(6):
                c = contact[:, f]
                d = np.diff(c.astype(int))
                swings.append(int((d == -1).sum()))
                moved = np.linalg.norm(
                    np.diff(pad_xy[:, f], axis=0), axis=1)
                slips.append(float(moved[c[:-1]].sum()))
            rec["sacrificed_legs"] = [
                f for f in range(6)
                if duty[f] < 0.10 or (duty[f] > 0.95
                                      and swings[f] == 0)]
            rec["gait_valid"] = not rec["sacrificed_legs"]
            rec["prog_ratio"] = (round(along_dist_m / cmd_dist_m, 3)
                                 if cmd_dist_m > 1e-6 else None)
            rec["slip_per_m"] = round(
                float(np.sum(slips)) / max(along_dist_m, 0.05), 3)

        z_sum, z_n = 0.0, 0
        settle_speeds: list = []     # |v| samples in the final zero seg
        n_seg = len(schedule)
        for si_s, (seconds, vx, vy) in enumerate(schedule):
            last_zero = (si_s == n_seg - 1 and vx == 0.0 and vy == 0.0)
            for _ in range(max(1, int(round(seconds / dt)))):
                if hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = 0.0
                a = act(obs, "walk")
                obs, _rw, term, trunc, info = env.step(a)
                grab()
                meter.tick(info)
                if vx != 0.0 or vy != 0.0:
                    z_sum += chassis_z()
                    z_n += 1
                elif last_zero:
                    settle_speeds.append(
                        float(np.hypot(*env._body_vel_xy())))
                contact_hist.append([
                    float(env.data.sensordata[adr]) > CONTACT_N
                    for adr in env._touch_adr])
                pad_xy_hist.append(
                    [env.data.xpos[b, :2].copy() for b in pads])
                v = env._body_vel_xy()
                err_sum += math.hypot(v[0] - vx, v[1] - vy)
                n_err += 1
                s_ref = math.hypot(vx, vy)
                if s_ref > 1e-3:
                    cmd_dist_m += s_ref * dt
                    along_dist_m += (v[0] * vx + v[1] * vy) / s_ref * dt
                if term or trunc:
                    rec["fall"] = str(info.get("termination_reason")
                                      or "episode_end")
                    rec["success"] = False
                    meter.finalize(rec)
                    gait_metrics()
                    if entry_slew is not None:
                        env.safety.entry_ramp_s = 0.0
                    return obs, False
        if entry_slew is not None:
            env.safety.entry_ramp_s = 0.0
        if args.drive_random:
            # Session-gate evidence fields (report-only): commanded-
            # portion height vs the plant frame, and the STOP_SETTLE
            # guard -- mean body speed over the trailing zero window
            # (quiet = ready to lower / take a new direction).
            if z_n:
                rec["drive_z_mean_mm"] = round(z_sum / z_n * 1000.0, 1)
            if settle_speeds:
                rec["stop_settle_speed_mps"] = round(
                    float(np.mean(settle_speeds)), 4)
                rec["stop_settle_ok"] = bool(
                    np.mean(settle_speeds) < 0.02)
        rec["trk_err"] = round(err_sum / max(n_err, 1), 4)
        rec["dist_m"] = round(float(np.hypot(
            *(np.array(env.data.qpos[:2], dtype=float) - p0))), 3)
        meter.finalize(rec)
        gait_metrics()
        # Success = tracked the command without falling AND a valid
        # six-leg gait (a parked-leg segment must not read OK on
        # tracking alone — lesson 11). prog_ratio/slip_per_m are
        # reported as evidence, no bar here.
        rec["success"] = bool(rec["trk_err"] < 0.15
                              and rec["gait_valid"])
        return obs, True

    def run_lower(obs, rec: dict):
        obs = reanchor_to("lower")
        pad_hist, h_err_mm = [], None
        meter = _SegMeter()
        for _ in range(int(round(LOWER_PHASE_S / dt))):
            a = act(obs, "lower")
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            meter.tick(info)
            pad_hist.append(pad_z())
            if "height_err_mm" in info:
                h_err_mm = float(info["height_err_mm"])
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                rec["success"] = False
                meter.finalize(rec)
                return obs, False
        meter.finalize(rec)
        score_posture(rec, pad_hist, h_err_mm)
        return obs, True

    results: dict = {"cfg_set": args.cfg_set or [], "grammar": grammar,
                     "speed": args.speed, "deterministic": det,
                     "drive_random": bool(args.drive_random),
                     "entry_slew": args.entry_slew,
                     "rise_from_h": bool(args.rise_from_h),
                     "episodes": []}
    if args.single is not None:
        results["single"] = str(args.single)
        results["recurrent"] = bool(recurrent)
    else:
        results["stand"] = str(args.stand)
        results["walk"] = str(args.walk)

    seen_rise = 0
    for ep in range(args.episodes):
        ep_rec = {"ep": ep, "segments": [], "zero_fall": True}
        want_strip = args.strips is not None and ep == args.strip_ep
        strip_frames.clear()
        episode_begin()
        obs, alive = None, True
        for si, mode in enumerate(grammar):
            seg = {"i": si, "mode": mode}
            if not alive:
                seg["skipped"] = True
                ep_rec["segments"].append(seg)
                continue
            if mode == "rise":
                cold = (si == 0)
                kind = (RISE_START_KINDS[seen_rise % len(RISE_START_KINDS)]
                        if cold else None)
                seen_rise += 1 if cold else 0
                obs, alive = run_rise(obs, seg, cold, kind)
            elif mode == "walk":
                sched = None
                if drive_rng is not None:
                    sched = _random_drive_schedule(
                        drive_rng, args.speed,
                        args.drive_speed_max or args.speed,
                        args.drive_seconds)
                    seg["schedule"] = [
                        (round(s, 2), round(x, 4), round(y, 4))
                        for s, x, y in sched]
                obs, alive = run_walk(obs, seg, sched)
            else:
                obs, alive = run_lower(obs, seg)
            if not alive:
                ep_rec["zero_fall"] = False
            ep_rec["segments"].append(seg)
        if want_strip:
            grab(final=True)
            save_strip(f"modeseq_ep{ep}")
        results["episodes"].append(ep_rec)
        tail = " ".join(f"{s['mode']}:{'OK' if s.get('success') else 'FAIL' if not s.get('skipped') else '-'}"
                        for s in ep_rec["segments"])
        print(f"[ep{ep:2d}] zero_fall={ep_rec['zero_fall']} {tail}")

    # ---- per-segment-type summary -------------------------------------
    by_type: dict[str, list] = {}
    rise_ordinal: dict[int, list] = {}   # 0 = first rise in grammar, 1 = second, ...
    for ep_rec in results["episodes"]:
        seen = 0
        for seg in ep_rec["segments"]:
            if seg.get("skipped"):
                continue
            by_type.setdefault(seg["mode"], []).append(seg)
            if seg["mode"] == "rise":
                rise_ordinal.setdefault(seen, []).append(seg)
                seen += 1

    summary = {}
    for mode, segs in by_type.items():
        summary[mode] = {
            "n": len(segs),
            "success": sum(1 for s in segs if s.get("success")),
            "falls": sum(1 for s in segs if s.get("fall")),
        }
        if mode == "walk":
            summary[mode]["gait_valid"] = sum(
                1 for s in segs if s.get("gait_valid"))
            prs = [s["prog_ratio"] for s in segs
                   if s.get("prog_ratio") is not None]
            if prs:
                summary[mode]["prog_ratio_med"] = round(
                    float(np.median(prs)), 3)
            if args.drive_random:
                sso = [s for s in segs if "stop_settle_ok" in s]
                summary[mode]["stop_settle_ok"] = sum(
                    1 for s in sso if s["stop_settle_ok"])
                summary[mode]["stop_settle_n"] = len(sso)
                zs = [s["drive_z_mean_mm"] for s in segs
                      if "drive_z_mean_mm" in s]
                if zs:
                    summary[mode]["drive_z_mean_mm_med"] = round(
                        float(np.median(zs)), 1)
    tilts = [s["switch_tilt_deg"] for segs in by_type.values()
             for s in segs if "switch_tilt_deg" in s]
    if tilts:
        summary["switch_tilt_deg_med"] = round(
            float(np.median(tilts)), 1)
        summary["switch_tilt_deg_max"] = round(float(max(tilts)), 1)
    summary["rise_by_ordinal"] = {
        str(i): {"n": len(segs),
                 "success": sum(1 for s in segs if s.get("success")),
                 "falls": sum(1 for s in segs if s.get("fall"))}
        for i, segs in rise_ordinal.items()
    }
    n_ep = len(results["episodes"])
    n_zero_fall = sum(1 for e in results["episodes"] if e["zero_fall"])
    summary["zero_fall_episodes"] = n_zero_fall
    summary["episodes"] = n_ep
    results["summary"] = summary
    print(json.dumps(summary, indent=1))
    pass_ = n_zero_fall >= max(1, n_ep - 1)  # >= n-1 == the >=11/12 rule at n=12
    print("SEQUENCE (zero-fall end to end):",
          f"{n_zero_fall}/{n_ep}",
          "PASS" if pass_ else "below the >=n-1/n bar")
    if len(rise_ordinal) > 1:
        r0 = rise_ordinal.get(0, [])
        r1 = rise_ordinal.get(1, [])
        s0 = sum(1 for s in r0 if s.get("success"))
        s1 = sum(1 for s in r1 if s.get("success"))
        print(f"SECOND-RISE CHECK (start-relative-_z0 risk): "
              f"first rise {s0}/{len(r0)} vs later rise(s) {s1}/{len(r1)}")
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(results, indent=1))
        print(f"wrote {args.out}")
    return 0 if pass_ else 1


if __name__ == "__main__":
    raise SystemExit(main())

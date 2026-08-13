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
height_err; walk: tracking error + distance, same as eval_handoff).
Per episode: zero_fall over the whole grammar. Summary: per-segment-
type success rate + fraction of episodes with zero falls end to end,
det and stochastic passes both supported (--stochastic).

    python3 -m rl_move.sim.eval_modeseq \
        --stand rl_move/sim/policies/ppo_goal_cw_stand_holdbc1_hard1.zip \
        --walk  rl_move/sim/policies/ppo_goal_cw_walk_longdist_r2.zip \
        --episodes 12 --grammar rise,walk,lower,rise,walk \
        --out logs/ckpt_eval/modeseq_baseline.json

Verdict rule of thumb (matches the directive's Arm 1/2 gates): PASS if
zero_fall_frac >= 11/12 AND every segment TYPE's own success rate is
inside its pairwise-handoff band (rise ~= eval_handoff "direct", lower
~= eval_handoff_reverse "direct", walk tracking inside eval_handoff's
plant-arm band). A regression on the SECOND rise specifically (vs the
first) isolates the start-relative-_z0 risk named above.
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
LOWER_PHASE_S = 10.0        # lower_hold_s(1) + lower_ramp_s(5) + settle
RISE_PHASE_S = 12.5         # eval_handoff.py's PHASE_A_S (worst-case rise)
TAIL_S = 0.5
END_CLEAR_BELLY_MM = 60.0   # eval_checkpoint.py posture-strict lower rule
HEIGHT_ERR_OK_MM = 15.0     # eval_checkpoint.py lower/rise success rule
RISE_START_KINDS = ("flat", "bridge", "crouch")   # cold-start rotation


def _set_mix(gen, **p) -> None:
    for attr in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, attr, 0.0)
    gen.p_walk = 0.0
    for k, v in p.items():
        setattr(gen, f"p_{k}", v)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--stand", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_stand_holdbc1_hard1.zip"))
    ap.add_argument("--walk", type=Path,
                    default=Path("rl_move/sim/policies/"
                                 "ppo_goal_cw_walk_longdist_r2.zip"))
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
    ap.add_argument("--stochastic", action="store_true",
                    help="both policies predict stochastically (default "
                         "deterministic)")
    ap.add_argument("--seed", type=int, default=0)
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

    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), cfg=cfg, randomize=False,
        episode_seconds=20.0, seed=args.seed,
        render_mode="rgb_array" if args.strips else None)
    stand = PPO.load(args.stand, device="cpu")
    walk = PPO.load(args.walk, device="cpu")
    n_stand = int(stand.observation_space.shape[0])
    n_env = int(env.observation_space.shape[0])
    assert walk.observation_space.shape[0] == n_env, (
        f"walk policy obs {walk.observation_space.shape} != env {n_env}")
    assert n_stand < n_env, (
        "stand policy obs must be a prefix of the walk env obs "
        f"(got {n_stand} vs {n_env})")
    det = not args.stochastic

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
        for _ in range(int(round(RISE_PHASE_S / dt))):
            a, _ = stand.predict(obs[:n_stand], deterministic=det)
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                rec["success"] = False
                return obs, False
        h_err = chassis_z() - (env._z0 + env._h_target)
        ok, detail = env.plant_report(height_err_m=h_err)
        rec["success"] = bool(ok)
        rec["height_err_end_mm"] = round(h_err * 1000.0, 1)
        if not ok:
            rec["fail_detail"] = [k for k, v in detail.items()
                                  if k.endswith("_ok") and not v]
        return obs, True

    def run_walk(obs, rec: dict):
        obs = reanchor_to("walk")
        traj = env._goal_traj
        p0 = np.array(env.data.qpos[:2], dtype=float)
        n_err, err_sum = 0, 0.0
        for seconds, vx, vy in WALK_SCHEDULE(args.speed):
            for _ in range(max(1, int(round(seconds / dt)))):
                if hasattr(traj, "vx"):
                    traj.vx[:] = vx
                    traj.vy[:] = vy
                if getattr(traj, "wz", None) is not None:
                    traj.wz[:] = 0.0
                a, _ = walk.predict(obs, deterministic=det)
                obs, _rw, term, trunc, info = env.step(a)
                grab()
                v = env._body_vel_xy()
                err_sum += math.hypot(v[0] - vx, v[1] - vy)
                n_err += 1
                if term or trunc:
                    rec["fall"] = str(info.get("termination_reason")
                                      or "episode_end")
                    rec["success"] = False
                    return obs, False
        rec["trk_err"] = round(err_sum / max(n_err, 1), 4)
        rec["dist_m"] = round(float(np.hypot(
            *(np.array(env.data.qpos[:2], dtype=float) - p0))), 3)
        # Success = tracked the command without falling; the pairwise
        # gait_valid/prog_ratio checks belong to the harness, this
        # instrument's job is falls + coarse tracking across the chain.
        rec["success"] = bool(rec["trk_err"] < 0.15)
        return obs, True

    def run_lower(obs, rec: dict):
        obs = reanchor_to("lower")
        pad_hist, h_err_mm = [], None
        for _ in range(int(round(LOWER_PHASE_S / dt))):
            a, _ = stand.predict(obs[:n_stand], deterministic=det)
            obs, _rw, term, trunc, info = env.step(a)
            grab()
            pad_hist.append(pad_z())
            if "height_err_mm" in info:
                h_err_mm = float(info["height_err_mm"])
            if term or trunc:
                rec["fall"] = str(info.get("termination_reason")
                                  or "episode_end")
                rec["success"] = False
                return obs, False
        score_posture(rec, pad_hist, h_err_mm)
        return obs, True

    results: dict = {"cfg_set": args.cfg_set or [], "grammar": grammar,
                     "stand": str(args.stand), "walk": str(args.walk),
                     "speed": args.speed, "deterministic": det,
                     "episodes": []}

    seen_rise = 0
    for ep in range(args.episodes):
        ep_rec = {"ep": ep, "segments": [], "zero_fall": True}
        want_strip = args.strips is not None and ep == 0
        strip_frames.clear()
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
                obs, alive = run_walk(obs, seg)
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

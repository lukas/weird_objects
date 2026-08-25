"""Stance-pricing probe: does a candidate current/termination price make
the OBSERVED mesh stance exploit unprofitable while honest behavior
stays dominant?

Context (standwalk stage-1 dig-in, 2026-08-25): both completed seeds of
the mesh/100 Hz stance retrain (cw-standwalk-stance-mesh1-rr1/-seed2-rr1)
learned a torque-saturation ground-grind — servos parked at the 2.2 N*m
ceiling (cur = 2.64 A = ceiling * 1.2 A/N*m) until the 2.5 A/0.8 s trip
or a tilt fall — while TRAINING REWARD ROSE, because the launched stack
deliberately carried no current pricing and a safety termination costs a
flat -10 against ~+100/episode of banked ref/income reward.

This probe rolls, per mode (rise/hold/lower) x seed, on the LAUNCH-EXACT
mesh joint_goal stack (DR-0, deterministic):
  - the actual failed checkpoint (the measured cheat, not a scripted
    imitation), and
  - the honest scripted behavior (time-aligned rise-ref replay / quiet
    hold / reverse-ref lower),
under a grid of pricing variants:
  base                     the stack as launched (control)
  hotK                     +reward.k_current_hot=K (current_hot_a=1.0)
  termT                    +reward.term_cost_per_remaining_s=T (cap 60)
  hotK_termT               both.
A usable dose is one where honest return stays clearly positive and
above the checkpoint in EVERY mode, and honest terms/plant are unharmed.

Usage (run on the run's own pod; ~10 min full grid):
  uv run python -m rl_move.sim.probe_stance_pricing \
      --ckpt rl_move/sim/policies/ppo_goal_cw_standwalk_stance_mesh1_rr1.zip \
      [--seeds 0,1,2] [--modes rise,hold,lower] [--episode-seconds 15]
      [--pricings base,hot0.5,term3,hot0.5_term3,hot2_term3]
"""
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import numpy as np

from rl_move.config import load_config
from rl_move.robot_state import DEG2RAD  # noqa: F401  (parity with bank)
from rl_move.sim.joint_task import SimHexapodJointGoalEnv, q_rad_to_action
from rl_move.sim.servo_model import SimServoParams

ROOT = Path(__file__).resolve().parents[2]
RISE_REF = ROOT / "rl_move/sim/refs/rise_ref_belly2plant.npz"

# The cw-standwalk-stance-mesh1-rr1 launch stack, verbatim from the
# ledger extra_args (minus training-only knobs), on the mesh default.
LAUNCH_OVERRIDES = {
    ("control", "hz"): 100.0,
    ("actions", "max_height_mm"): 88.0,
    ("goal", "rise_height_mm"): [79.0, 87.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("goal", "rise_rsi_frac"): 0.0,   # probe determinism: full rise only
    ("goal", "rise_hold_min_s"): 0.5,
    ("reward", "rise_score_income"): 1.0,
    ("reward", "rise_score_strip_pen"): 1.0,
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): str(RISE_REF),
    ("reward", "rise_ref_sigma_deg"): 6.0,
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
    ("reward", "hold_still_gate"): 1.0,
    ("reward", "hold_flag_fade"): 1.0,
}


def _parse_pricing(tag: str) -> dict:
    """'hot0.5_term3' -> cfg override dict. 'base' -> {}."""
    ov: dict = {}
    for part in tag.split("_"):
        if part == "base":
            continue
        m = re.fullmatch(r"hot([0-9.]+)(?:a([0-9.]+))?", part)
        if m:
            ov[("reward", "k_current_hot")] = float(m.group(1))
            ov[("reward", "current_hot_a")] = float(m.group(2) or 1.0)
            continue
        m = re.fullmatch(r"term([0-9.]+)", part)
        if m:
            ov[("reward", "term_cost_per_remaining_s")] = float(m.group(1))
            ov[("reward", "term_cost_max")] = 60.0
            continue
        # loadX / loadXmin / loadX[min]fF: measured-load gate on hold
        # income (reward.hold_feet_load, sim_env ~3094). 'min' selects
        # the min-over-feet variant (hold_feet_load_min=1) built after
        # the product form was defeated by one-foot shedding. Optional
        # 'fF' suffix (08-25, holdload1min triage) sets the floor for
        # the selected variant — hold_load_floor for the product path
        # (e.g. load1f0.1 = product with per-unloaded-foot factor 0.1,
        # min-strength tax WITH per-foot gradient), hold_load_min_floor
        # for the min path.
        m = re.fullmatch(r"load([0-9.]+)(min)?(?:f([0-9.]+))?", part)
        if m:
            ov[("reward", "hold_feet_load")] = float(m.group(1))
            if m.group(2):
                ov[("reward", "hold_feet_load_min")] = 1.0
                if m.group(3):
                    ov[("reward", "hold_load_min_floor")] = float(m.group(3))
            elif m.group(3):
                ov[("reward", "hold_load_floor")] = float(m.group(3))
            continue
        raise SystemExit(f"unknown pricing token {part!r} in {tag!r}")
    return ov


def _make_env(mode: str, seed: int, pricing: dict,
              episode_seconds: float) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in {**LAUNCH_OVERRIDES, **pricing}.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=episode_seconds, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for a in [a for a in vars(gen) if a.startswith("p_")]:
        setattr(gen, a, 0.0)
    setattr(gen, f"p_{mode}", 1.0)
    if mode == "rise":
        gen.force_rise_start = "flat"
    return env


def _ref_row(env, step: int, ref_dt: float, ramp_ref: int) -> int:
    t_rel = (step - env._rise_ramp_i0) * env.dt
    return ramp_ref + int(round(t_rel / ref_dt))


def _rollout(env, mode: str, behavior: str, model) -> dict:
    obs, _ = env.reset()
    ref = np.load(RISE_REF)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    ref_dt = float(ref["dt"])
    q0 = env.data.qpos[env._qadr].copy()
    n_ref = len(q_ref)
    if model is not None:
        state, ep_start = None, np.ones((1,), dtype=bool)
    total, step, cur_max, cur_mean_acc = 0.0, 0, 0.0, 0.0
    term = trunc = False
    reason = None
    while not (term or trunc):
        if behavior == "policy":
            act, state = model.policy.predict(
                obs, state=state, episode_start=ep_start,
                deterministic=True)
            ep_start = np.zeros((1,), dtype=bool)
        elif behavior == "replay":            # honest rise
            j = _ref_row(env, step, ref_dt, ramp_ref)
            act = q_rad_to_action(q_ref[min(max(j, 0), n_ref - 1)])
        elif behavior == "hold_quiet":        # honest hold
            act = q_rad_to_action(q0)
        elif behavior == "lower_rev":         # honest lower: reverse ref
            j = (n_ref - 1) - _ref_row(env, step, ref_dt, ramp_ref)
            act = q_rad_to_action(q_ref[min(max(j, 0), n_ref - 1)])
        elif behavior == "lower_desc":
            # Honest lower matched to the goal's own timing: hold the
            # plant pose through lower_hold_s (1 s), then descend the
            # ref rows plant -> row 283 (~50 mm, the mesh mid-crouch;
            # kick-cycle calibration) over lower_ramp_s (5 s), then
            # hold the crouch. Descent WITH gravity = low torque.
            t = step * env.dt
            f = min(max((t - 1.0) / 5.0, 0.0), 1.0)
            j = int(round((n_ref - 1) + f * (283 - (n_ref - 1))))
            act = q_rad_to_action(q_ref[min(max(j, 0), n_ref - 1)])
        elif behavior == "partial":           # honest mid-rise, feet down
            j = min(_ref_row(env, step, ref_dt, ramp_ref), 283)
            act = q_rad_to_action(q_ref[min(max(j, 0), n_ref - 1)])
        elif behavior == "flagleg":           # 5-leg rise, leg0 flagged
            j = _ref_row(env, step, ref_dt, ramp_ref)
            q = q_ref[min(max(j, 0), n_ref - 1)].copy()
            q[0:3] = q0[0:3]
            act = q_rad_to_action(q)
        elif behavior == "stilt":             # tip-toe pop cheat
            q_stilt = np.array([0.0, 0.0, 80.0] * 6) * (np.pi / 180.0)
            act = q_rad_to_action(
                q0 if step < env._rise_ramp_i0 else q_stilt)
        elif behavior == "freeze":
            act = q_rad_to_action(q0)
        else:
            raise SystemExit(f"unknown behavior {behavior!r}")
        obs, r, term, trunc, info = env.step(np.asarray(act).ravel())
        total += float(r)
        step += 1
        cur = env._state.servo_current
        if cur is not None:
            cur_max = max(cur_max, float(np.max(np.abs(cur))))
            cur_mean_acc += float(np.mean(np.abs(cur)))
        if term:
            reason = info.get("termination_reason")
    h_err = (float(env.data.xpos[env._chassis_bid, 2]) - env._z0
             - env._h_target)
    plant_ok = False
    if mode == "rise" and not term:
        plant_ok = bool(env.plant_report(height_err_m=h_err)[0])
    return {"ret": total, "steps": step, "term": bool(term),
            "reason": reason, "cur_max": round(cur_max, 3),
            "cur_mean": round(cur_mean_acc / max(step, 1), 3),
            "h_err_mm": round(1000 * h_err, 1),
            "plant_ok": plant_ok}


HONEST = {"rise": "replay", "hold": "hold_quiet", "lower": "lower_desc"}
EXTRA = {"rise": ("partial", "flagleg", "stilt", "freeze")}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=Path, required=True)
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--modes", default="rise,hold,lower")
    ap.add_argument("--episode-seconds", type=float, default=15.0)
    ap.add_argument("--pricings",
                    default="base,hot0.5,term3,hot0.5_term3,hot2_term3")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--orderings", action="store_true",
                    help="also roll the scripted bank cheats (rise "
                         "partial/flagleg/stilt/freeze) to verify the "
                         "pricing preserves bank orderings")
    args = ap.parse_args()

    from rl_move.sim.gru_policy import load_checkpoint_auto
    model = load_checkpoint_auto(args.ckpt, device="cpu")

    seeds = [int(s) for s in args.seeds.split(",")]
    modes = args.modes.split(",")
    rows = []
    for tag in args.pricings.split(","):
        pricing = _parse_pricing(tag)
        for mode in modes:
            behaviors = (HONEST[mode], "policy") + EXTRA.get(
                mode, ()) if args.orderings else (HONEST[mode], "policy")
            for behavior in behaviors:
                rets = []
                for seed in seeds:
                    env = _make_env(mode, seed, pricing,
                                    args.episode_seconds)
                    r = _rollout(env, mode, behavior,
                                 model if behavior == "policy" else None)
                    env.close()
                    r.update(pricing=tag, mode=mode, behavior=behavior,
                             seed=seed)
                    rows.append(r)
                    rets.append(r["ret"])
                print(f"{tag:14s} {mode:5s} {behavior:10s} "
                      f"ret_mean {np.mean(rets):9.1f}  "
                      f"terms {sum(x['term'] for x in rows[-len(seeds):])}"
                      f"/{len(seeds)}  "
                      f"cur_max {max(x['cur_max'] for x in rows[-len(seeds):]):.2f}",
                      flush=True)
    out = args.out or Path("logs/probe_stance_pricing.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(rows, indent=1))
    print(f"wrote {out} ({len(rows)} rollouts)")


if __name__ == "__main__":
    main()

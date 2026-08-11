"""probe_walk_income.py — term-by-term WALK income decomposition.

WHY (RL_PLAN queue 2.1 / rl_docs/TURN.md, 08-11): three omni arms each
collapsed into a DIFFERENT degenerate gait (mirror1-r1 freeze,
mirror2/dr02 leg-sacrifice tripod, trans1 paddle-stall) while the
task-semantics bank kept passing — the bank compares full-command gait
vs march-in-place vs park, but the real attractors are INTERMEDIATE:
partial progress with degenerate leg usage. Before any new omni arm,
find which reward channel still pays for that.

WHAT IT DOES: rolls policies in the exact reward stack of a named run
(trans1 = the current deliverable stack; mirror2 = + yaw set) with a
pinned command, and accumulates EVERY per-term reward component the
env emits (info["reward_*"]) plus the income-gate factors, progress
ratio, and per-leg duty/swing fingerprints. Policies are scripted
references (honest gait, half-speed gait, 1/3-leg sacrifice, paddle,
freeze — built to match the video fingerprints) and/or real
checkpoints (ckpt:<path>).

    python -m rl_move.sim.probe_walk_income --stack trans1 \
        --policies gait,gait_slow,sac1,sac3,paddle,freeze,ckpt:rl_move/sim/policies/ppo_goal_cw_omni_trans1.zip \
        --dirs forward,backward,crab_left,diag_back_right \
        --seeds 0,1,2 --jobs 8 --out logs/probe_walk_income/trans1.json

Residual = episode return − Σ(labelled terms): non-zero residual is the
BASE stack (task kernel, stance/clearance/still, penalties) — printed
as its own row so no income channel can hide.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

WALK_PLANT = (20.0, 80.0)
CMD_V = 0.055           # mid of the trained 0.05-0.06 band
EPISODE_S = 15.0

# Exact env-relevant cfg of cw-omni-trans1 (ledger extra_args 08-11) —
# the CURRENT deliverable stack (turn de-scoped).
TRANS1_STACK = {
    ("reward", "k_step_event"): 1.0,
    ("reward", "k_drag_loaded"): 10.0,
    ("reward", "k_park_duty"): 1.0,
    ("goal", "walk_speed_min_m_s"): 0.05,
    ("goal", "walk_speed_max_m_s"): 0.06,
    ("reward", "walk_kernel_prog_gate"): 1.0,
    ("goal", "walk_park_start_frac"): 0.25,
    ("reward", "walk_anchor_gate"): 1.0,
    ("reward", "anchor_tol_mm"): 10.0,
    ("goal", "walk_heading_max_rad"): 3.14159,
    ("goal", "walk_cmd_resample_s"): 1.5,
    ("goal", "walk_cmd_resample_jitter"): 0.6,
    ("goal", "walk_cmd_blend_s_min"): 0.1,
    ("goal", "walk_cmd_blend_s_max"): 1.0,
    ("goal", "walk_stop_frac"): 0.2,
    ("obs", "history_frames"): 16,
    ("goal", "walk_obs_body_vel"): 2.0,
    ("safety", "max_roll_deg"): 25.0,
    ("safety", "max_pitch_deg"): 25.0,
    ("reward", "k_current"): 0.0,
}
# cw-omni-mirror2 adds the full yaw set (ledger extra_args 08-11).
MIRROR2_STACK = dict(TRANS1_STACK)
MIRROR2_STACK.update({
    ("goal", "walk_yaw_cmd"): 1,
    ("goal", "walk_yaw_max_rad_s"): 0.3,
    ("goal", "walk_yaw_zero_frac"): 0.5,
    ("goal", "walk_turn_in_place_frac"): 0.30,
    ("reward", "k_walk_yaw"): 1.0,
    ("reward", "walk_yaw_kernel_gate"): 1.0,
    ("reward", "k_yaw_prog"): 1.0,
    ("reward", "k_yaw_still"): 50.0,
    ("reward", "walk_kernel_yaw_gate"): 1.0,
})
# mirror2 + the 08-11 latent-defect fixes (walk_task.py): heading-hold
# yaw income gated on achieved linear progress, drift charge on the wz
# EMA instead of the instantaneous oscillation. The stack any future
# turn arm must train with (TURN_OVERRIDES in test_task_semantics.py).
MIRROR2FIX_STACK = dict(MIRROR2_STACK)
MIRROR2FIX_STACK.update({
    ("reward", "walk_yaw_hold_prog_gate"): 1.0,
    ("reward", "yaw_still_avg_s"): 1.0,
})
STACKS = {"trans1": TRANS1_STACK, "mirror2": MIRROR2_STACK,
          "mirror2fix": MIRROR2FIX_STACK}

DIRS = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "crab_left": (0.0, 1.0),
    "diag_back_right": (-0.707, -0.707),
}

# Video fingerprints being priced (rl_docs/TURN.md):
#   sac3   mirror2/dr02: one tripod held planted, body barely moves
#   sac1   the classic single flag-leg walk (leg pinned at plant)
#   paddle trans1: legs 1,4 planted 90-99% duty, other four rapid
#          ~0.01 m strides, partial progress
#   driftride  the yawcmd1/yawgate2/turnfix1 fingerprint: track the
#          linear command but rotate at the structural ~+0.09 rad/s
#          left drift, collecting the yaw kernel's heading-hold band.
#          Scripted omega 0.25 calibrated to ACHIEVE ~0.088 rad/s
#          while translating (the gait realizes ~40% of commanded
#          omega — measured, test_task_semantics.py DRIFT_RIDE_WZ).
SCRIPTED = ("gait", "gait_slow", "sac1", "sac3", "paddle", "freeze",
            "driftride")
PIN = {"sac1": (0,), "sac3": (0, 2, 4), "paddle": (1, 4)}
VEL_SCALE = {"gait_slow": 0.5, "paddle": 0.3}
DRIFT_RIDE_OMEGA = 0.25

# gate-factor / diagnostic means worth reporting alongside term sums
FACTOR_KEYS = ("walk_prog_factor", "walk_anchor_frac",
               "walk_loadslip_factor", "walk_height_factor",
               "walk_yaw_gate_factor", "walk_yaw_kernel_factor",
               "walk_yaw_hold_factor")


def make_env(seed: int, stack: dict, dr_scale: float = 0.0):
    from rl_move.sim.walk_task import SimHexapodJointWalkEnv
    cfg = load_config()
    for (sec, leaf), val in stack.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=dr_scale > 0.0,
        dr_scale=dr_scale, episode_seconds=EPISODE_S, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def pin_command(env, vx: float, vy: float) -> None:
    """Deterministic command: hold 1 s, ramp 1 s, then constant."""
    traj = env._goal_traj
    n = len(traj.vx)
    hold_n = ramp_n = int(round(1.0 / env.dt))
    ramp = np.linspace(0.0, 1.0, ramp_n)
    traj.vx[:] = vx
    traj.vx[:hold_n] = 0.0
    traj.vx[hold_n:hold_n + ramp_n] = vx * ramp
    traj.vy[:] = vy
    traj.vy[:hold_n] = 0.0
    traj.vy[hold_n:hold_n + ramp_n] = vy * ramp
    if getattr(traj, "wz", None) is not None:
        traj.wz[:] = 0.0


def rollout(policy: str, direction: str, seed: int, stack_name: str,
            deterministic: bool = True, dr_scale: float = 0.0) -> dict:
    from tripod_gait import TripodGait

    stack = STACKS[stack_name]
    env = make_env(seed, stack, dr_scale)
    obs, _ = env.reset()
    ux, uy = DIRS[direction]
    vx, vy = ux * CMD_V, uy * CMD_V
    pin_command(env, vx, vy)
    traj = env._goal_traj
    n = len(traj.vx)

    model = None
    gait = None
    plant_rad = np.array([0.0, *WALK_PLANT] * 6) * DEG2RAD
    if policy.startswith("ckpt:"):
        from stable_baselines3 import PPO
        model = PPO.load(str(ROOT / policy[5:]), device="cpu")
    else:
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()

    term_sums: dict[str, float] = defaultdict(float)
    factor_sums: dict[str, float] = defaultdict(float)
    factor_n: dict[str, int] = defaultdict(int)
    contact_hist = []
    total, step, cmd_dist, along_dist = 0.0, 0, 0.0, 0.0
    wz_sum, wz_n = 0.0, 0
    term_reason = None
    scale = VEL_SCALE.get(policy, 1.0)

    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if model is not None:
            act, _ = model.predict(obs, deterministic=deterministic)
        elif policy == "freeze":
            act = q_rad_to_action(plant_rad)
        else:
            omega = (DRIFT_RIDE_OMEGA if policy == "driftride"
                     and t >= 2.0 else 0.0)
            gait.set_velocity(vx=float(traj.vx[i]) * scale,
                              vy=float(traj.vy[i]) * scale, omega=omega)
            q = np.asarray(gait.desired_deg(t)) * DEG2RAD
            for leg in PIN.get(policy, ()):
                q[3 * leg:3 * leg + 3] = plant_rad[3 * leg:3 * leg + 3]
            act = q_rad_to_action(q)
        obs, r, term, trunc, info = env.step(act)
        total += float(r)
        for k, v in info.items():
            if k.startswith("reward_"):
                term_sums[k] += float(v)
            elif k in FACTOR_KEYS:
                factor_sums[k] += float(v)
                factor_n[k] += 1
        g = env._current_goal()
        if g is not None:
            s_ref = math.hypot(g.vx_ref, g.vy_ref)
            if s_ref > 1e-3:
                v_b = env._body_vel_xy()
                cmd_dist += s_ref * env.dt
                along_dist += ((v_b[0] * g.vx_ref + v_b[1] * g.vy_ref)
                               / s_ref) * env.dt
                wz_sum += env._body_wz()
                wz_n += 1
        contact_hist.append([
            float(env.data.sensordata[adr]) > 0.5
            for adr in env._touch_adr])
        step += 1
        if term or trunc:
            term_reason = info.get("termination_reason")
            break
    env.close()

    contact = np.asarray(contact_hist, dtype=bool)
    duty = contact.mean(axis=0)
    swings = [int(np.sum(np.diff(contact[:, f].astype(int)) == -1))
              for f in range(6)]
    labelled = float(sum(term_sums.values()))
    return {
        "policy": policy, "dir": direction, "seed": seed,
        "stack": stack_name, "dr_scale": dr_scale, "ticks": step, "return": total,
        "terms": dict(term_sums),
        "residual_base": total - labelled,
        "factors": {k: factor_sums[k] / max(factor_n[k], 1)
                    for k in factor_sums},
        "progress_ratio": along_dist / cmd_dist if cmd_dist > 0 else 0.0,
        "wz_mean": wz_sum / wz_n if wz_n else 0.0,
        "duty": [round(float(d), 3) for d in duty],
        "swings": swings,
        "terminated": term_reason if term_reason not in (None, "time")
        else None,
    }


def _job(args):
    return rollout(*args)


def summarize(records: list[dict]) -> None:
    by_pol: dict[str, list[dict]] = defaultdict(list)
    for r in records:
        by_pol[r["policy"]].append(r)
    pols = list(by_pol)
    all_terms = sorted({t for r in records for t in r["terms"]})

    def pname(p):
        return p if not p.startswith("ckpt:") else "ckpt"

    hdr = f"{'':28s}" + "".join(f"{pname(p):>14s}" for p in pols)
    print("\n=== per-episode MEAN income by term (all dirs/seeds) ===")
    print(hdr)
    for t in all_terms + ["residual_base", "TOTAL_RETURN"]:
        row = f"{t:28s}"
        for p in pols:
            rs = by_pol[p]
            if t == "TOTAL_RETURN":
                v = np.mean([r["return"] for r in rs])
            elif t == "residual_base":
                v = np.mean([r["residual_base"] for r in rs])
            else:
                v = np.mean([r["terms"].get(t, 0.0) for r in rs])
            row += f"{v:14.1f}"
        print(row)
    print("\n--- behavior fingerprints (mean) ---")
    for lab, key in (("progress_ratio", "progress_ratio"),
                     ("wz_mean", "wz_mean")):
        row = f"{lab:28s}"
        for p in pols:
            row += f"{np.mean([r[key] for r in by_pol[p]]):14.2f}"
        print(row)
    row = f"{'terminated_eps':28s}"
    for p in pols:
        row += f"{sum(1 for r in by_pol[p] if r['terminated']):14d}"
    print(row)
    for fk in FACTOR_KEYS:
        vals = {p: [r["factors"][fk] for r in by_pol[p]
                    if fk in r["factors"]] for p in pols}
        if not any(vals.values()):
            continue
        row = f"{fk:28s}"
        for p in pols:
            row += (f"{np.mean(vals[p]):14.2f}" if vals[p]
                    else f"{'-':>14s}")
        print(row)
    print("\n--- per-direction TOTAL return ---")
    dirs = sorted({r["dir"] for r in records})
    for d in dirs:
        row = f"{d:28s}"
        for p in pols:
            rs = [r for r in by_pol[p] if r["dir"] == d]
            row += (f"{np.mean([r['return'] for r in rs]):14.1f}"
                    if rs else f"{'-':>14s}")
        print(row)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stack", choices=list(STACKS), default="trans1")
    ap.add_argument("--policies", default=",".join(SCRIPTED))
    ap.add_argument("--dirs", default=",".join(DIRS))
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--stochastic", action="store_true")
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    pols = [p for p in a.policies.split(",") if p]
    dirs = [d for d in a.dirs.split(",") if d]
    seeds = [int(s) for s in a.seeds.split(",") if s != ""]
    jobs = [(p, d, s, a.stack, not a.stochastic, a.dr_scale)
            for p in pols for d in dirs for s in seeds]
    print(f"probe_walk_income: stack={a.stack} dr={a.dr_scale} {len(jobs)} rollouts "
          f"({len(pols)} policies x {len(dirs)} dirs x {len(seeds)} seeds)")
    if a.jobs > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            records = list(ex.map(_job, jobs))
    else:
        records = [_job(j) for j in jobs]
    summarize(records)
    if a.out:
        out = ROOT / a.out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(records, indent=1))
        print(f"\nWROTE {out}")


if __name__ == "__main__":
    main()

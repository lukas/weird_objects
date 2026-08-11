"""probe_tall_wall.py — TALL LADDER T5: is the −44 mm wall kinematic
or habitual? (RL_PLAN queue -0.5 P2.5, operator session 08-11 eve.)

The dep-line height-ref ladder stalls with the body ~44 mm below the
plant spawn no matter the commanded ref (tall30: err 15 mm @ −30;
tall15: err 29 mm @ −15). Two stories:
  (a) KINEMATIC/STABILITY: at −44 mm the stance geometry is pinned —
      hip pitch or knee near a joint limit, or the support polygon
      collapses — so no pricing scheme can buy more height. STOP the
      ladder; −30 is the envelope.
  (b) HABIT: joints sit mid-range with workspace to spare; the crouch
      is just where walk income converged. Keep pushing (gate/pricing
      arms can win).

WHAT IT DOES: deterministic steady-state rollouts (post-ramp window)
of the tall30 checkpoint vs the scripted plant-height tripod gait in
the same dep-contract env, reporting body-height offset, per-joint-
class mean angles + margin to the hard joint limits, and the stance
support radius. The scripted gait walks AT plant height, so its
numbers are the existence proof of a tall stance geometry.

    ../../.venv/bin/python -m rl_move.sim.probe_tall_wall
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "linux_control", ROOT / "linux_control" / "urt2_setup"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.sim.probe_walk_income import (  # noqa: E402
    CMD_V, STACKS, WALK_PLANT, make_env, pin_command,
)

CMD = CMD_V                    # override with --cmd (e.g. slow1's 0.035)

CKPT = "rl_move/sim/policies/ppo_goal_cw_dep_tall30.zip"
HEIGHT_OFF_MM = -30.0          # tall30's trained ref (override with --ref)
STEADY_FROM_S = 5.0            # skip hold+ramp+settle
EPISODE_S = 15.0


def rollout(policy: str, seed: int) -> dict:
    import mujoco
    from tripod_gait import TripodGait

    stack = dict(STACKS["vref1"])
    stack[("goal", "walk_height_off_mm")] = HEIGHT_OFF_MM
    env = make_env(seed, stack)
    obs, _ = env.reset()
    pin_command(env, CMD, 0.0)
    traj = env._goal_traj
    n = len(traj.vx)
    m = env.model

    model, gait = None, None
    if policy == "ckpt":
        from stable_baselines3 import PPO
        model = PPO.load(str(ROOT / CKPT), device="cpu")
    else:
        gait = TripodGait(vx=0.0)
        gait.sync_plant_stance(*WALK_PLANT)
        gait.reset_phase()

    # joint bookkeeping: 18 hinge joints after the free root
    cls_idx = {"yaw": [], "pitch": [], "knee": []}
    qadr, lo, hi = {}, {}, {}
    for j in range(m.njnt):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
        for cls in cls_idx:
            if name.endswith(cls):
                cls_idx[cls].append(j)
                qadr[j] = int(m.jnt_qposadr[j])
                lo[j], hi[j] = np.degrees(m.jnt_range[j])

    z0 = float(env._z0)
    heights, radii = [], []
    angles = {c: [] for c in cls_idx}
    margins = {c: [] for c in cls_idx}      # deg to NEAREST hard limit
    step = 0
    while True:
        t = step * env.dt
        i = min(step, n - 1)
        if model is not None:
            act, _ = model.predict(obs, deterministic=True)
        else:
            gait.set_velocity(vx=float(traj.vx[i]), vy=float(traj.vy[i]))
            act = q_rad_to_action(np.asarray(gait.desired_deg(t)) * DEG2RAD)
        obs, _, term, trunc, _ = env.step(act)
        if t >= STEADY_FROM_S:
            body = env.data.xpos[env._chassis_bid]
            heights.append((float(body[2]) - z0) * 1000.0)
            # support radius over feet in contact
            rr = []
            for f in range(6):
                adr = env._touch_adr[f]
                if adr >= 0 and float(env.data.sensordata[adr]) > 0.5:
                    pad = env.data.xpos[env._pad_bids[f]]
                    rr.append(float(np.hypot(pad[0] - body[0],
                                             pad[1] - body[1])))
            if rr:
                radii.append(float(np.mean(rr)) * 1000.0)
            for cls, joints in cls_idx.items():
                a = [np.degrees(float(env.data.qpos[qadr[j]])) for j in joints]
                angles[cls].append(float(np.mean(a)))
                margins[cls].append(float(min(
                    min(v - lo[j], hi[j] - v)
                    for j, v in zip(joints, a))))
        step += 1
        if term or trunc:
            break

    out = {
        "policy": policy, "seed": seed, "ticks_steady": len(heights),
        "height_off_mm": {"mean": float(np.mean(heights)),
                          "p10": float(np.percentile(heights, 10)),
                          "p90": float(np.percentile(heights, 90))},
        "support_radius_mm": float(np.mean(radii)) if radii else None,
    }
    for cls in cls_idx:
        out[f"{cls}_deg_mean"] = float(np.mean(angles[cls]))
        out[f"{cls}_limit_margin_deg_min"] = float(np.min(margins[cls]))
    env.close()
    return out


def main() -> None:
    global CKPT, HEIGHT_OFF_MM
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default=CKPT,
                    help="checkpoint path relative to prototype root")
    ap.add_argument("--ref", type=float, default=HEIGHT_OFF_MM,
                    help="goal.walk_height_off_mm the ckpt was trained with")
    ap.add_argument("--no-gait", action="store_true",
                    help="skip the scripted-gait reference rollouts")
    ap.add_argument("--cmd", type=float, default=CMD_V,
                    help="commanded speed m/s (use the ckpt's trained band)")
    args = ap.parse_args()
    global CMD
    CKPT, HEIGHT_OFF_MM, CMD = args.ckpt, args.ref, args.cmd

    rows = []
    for policy in (("ckpt",) if args.no_gait else ("ckpt", "gait")):
        for seed in (0, 1, 2):
            r = rollout(policy, seed)
            rows.append(r)
            print(json.dumps(r))
    # verdict helper
    ck = [r for r in rows if r["policy"] == "ckpt"]
    def agg(k, sub=None):
        vals = [(r[k][sub] if sub else r[k]) for r in ck]
        return float(np.mean(vals))
    print("\n--- ckpt (tall30) steady state ---")
    print(f"height offset mean {agg('height_off_mm', 'mean'):.1f} mm; "
          f"pitch mean {agg('pitch_deg_mean'):.1f} deg "
          f"(min margin {min(r['pitch_limit_margin_deg_min'] for r in ck):.1f}); "
          f"knee mean {agg('knee_deg_mean'):.1f} deg "
          f"(min margin {min(r['knee_limit_margin_deg_min'] for r in ck):.1f}); "
          f"support radius {agg('support_radius_mm'):.0f} mm")
    g = [r for r in rows if r["policy"] == "gait"]
    if not g:
        return
    print("--- scripted gait (plant height existence proof) ---")
    print(f"height offset mean {np.mean([r['height_off_mm']['mean'] for r in g]):.1f} mm; "
          f"pitch mean {np.mean([r['pitch_deg_mean'] for r in g]):.1f} deg; "
          f"knee mean {np.mean([r['knee_deg_mean'] for r in g]):.1f} deg; "
          f"support radius {np.mean([r['support_radius_mm'] for r in g]):.0f} mm")


if __name__ == "__main__":
    main()

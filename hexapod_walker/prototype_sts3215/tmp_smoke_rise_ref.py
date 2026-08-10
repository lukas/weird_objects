"""Pricing smoke for the rise-reference shaping (operator 08-10).

Question under test: with the shaping stack ON (ref-track + posture
gate + income prog-gate + signed finish gate) and a PLANT-HEIGHT
target (+111 mm, the walkable stance), does the known-good path
out-earn the two measured degenerate strategies?

  replay   feed the belly->plant reference (rise_ref_belly2plant.npz)
           as joint targets — the demonstrated stand-up
  freeze   hold the reset pose forever (the paid-plateau exploit)
  stilt    hip 0 / knee 80: torso high on leg tips, feet not planted
           (the posture exploit that gamed rise "6/6" in rfix-fresh1)

PASS = replay's return dominates both by a wide margin, and the
gap is visible in the income parts (task kernel, milestones, finish,
ref-track), not just the progress term.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_PROTO = Path(__file__).resolve().parent
_LINUX = _PROTO / "linux_control"
for p in (_PROTO, _LINUX, _LINUX / "urt2_setup"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from rl_move.config import load_config  # noqa: E402
from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import (  # noqa: E402
    SimHexapodJointGoalEnv, q_rad_to_action)
from rl_move.sim.servo_model import SimServoParams  # noqa: E402

REF_PATH = "rl_move/sim/refs/rise_ref_belly2plant.npz"
OVERRIDES = {
    ("actions", "max_height_mm"): 115.0,
    ("goal", "rise_height_mm"): [108.0, 114.0],
    ("goal", "rise_ramp_s"): 6.0,
    ("reward", "k_rise_ref_track"): 2.0,
    ("reward", "rise_ref_path"): REF_PATH,
    ("reward", "rise_posture_gate"): 1.0,
    ("reward", "rise_income_prog_gate"): 1.0,
    ("reward", "rise_finish_gate_signed"): 1.0,
}
PART_KEYS = ("reward_task", "reward_rise_progress", "reward_rise_milestone",
             "reward_rise_finish", "reward_rise_ref",
             "reward_curl_progress", "reward_curl_milestone")


def make_env(seed: int) -> SimHexapodJointGoalEnv:
    cfg = load_config()
    for (sec, leaf), val in OVERRIDES.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointGoalEnv(
        params=SimServoParams.from_cfg(None), randomize=False,
        dr_scale=0.0, episode_seconds=16.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "rise" else 0.0)
    gen.force_rise_start = "flat"
    return env


def rollout(policy: str, seed: int) -> dict:
    env = make_env(seed)
    obs, _ = env.reset()
    ref = np.load(_PROTO / REF_PATH)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    q0 = env.data.qpos[env._qadr].copy()
    q_stilt = np.array([0.0, 0.0, 80.0] * 6) * DEG2RAD
    bid = env.model.body("chassis").id
    z0 = float(env.data.xpos[bid, 2])

    total, sums = 0.0, {k: 0.0 for k in PART_KEYS}
    step, term_reason = 0, ""
    while True:
        if policy == "replay":
            j = ramp_ref + (step - env._rise_ramp_i0)
            act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        elif policy == "freeze":
            act = q_rad_to_action(q0)
        else:  # stilt: hold flat until the ramp starts, then pop up
            act = q_rad_to_action(
                q0 if step < env._rise_ramp_i0 else q_stilt)
        obs, r, term, trunc, info = env.step(act)
        total += r
        for k in PART_KEYS:
            sums[k] += info.get(k, 0.0)
        step += 1
        if term or trunc:
            term_reason = info.get("termination_reason") or ""
            break
    clear_mm = max(
        (float(env.data.xpos[b, 2]) - z) * 1000.0
        for b, z in zip(env._pad_bids, env._pad_z_ref) if b >= 0)
    return dict(total=total, sums=sums, steps=step,
                h_rel_mm=(float(env.data.xpos[bid, 2]) - z0) * 1000.0,
                clear_mm=clear_mm, term=term_reason)


def main() -> None:
    results = {}
    for policy in ("replay", "freeze", "stilt"):
        rs = [rollout(policy, seed) for seed in (0, 1, 2)]
        results[policy] = rs
        tot = np.mean([r["total"] for r in rs])
        per_seed = ", ".join(f"{r['total']:+.0f}" for r in rs)
        print(f"\n=== {policy}: return {tot:+.1f} (per-seed {per_seed})")
        for r in rs:
            parts = " ".join(f"{k.replace('reward_', '')}={v:+.1f}"
                             for k, v in r["sums"].items() if abs(v) > 0.05)
            print(f"    end h_rel {r['h_rel_mm']:+.0f}mm "
                  f"clear {r['clear_mm']:.0f}mm steps {r['steps']}"
                  f"{' TERM=' + r['term'] if r['term'] else ''} | {parts}")

    m = {p: np.mean([r["total"] for r in results[p]])
         for p in results}
    ok = m["replay"] > 2.0 * max(m["freeze"], m["stilt"]) \
        and m["replay"] > max(m["freeze"], m["stilt"]) + 50.0
    print(f"\nreplay {m['replay']:+.1f} vs freeze {m['freeze']:+.1f} "
          f"vs stilt {m['stilt']:+.1f} -> "
          f"{'PASS: demonstrated path dominates' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

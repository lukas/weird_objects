"""Throwaway probe: per-part episode totals, score+ref stack."""
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from rl_move.robot_state import DEG2RAD  # noqa: E402
from rl_move.sim.joint_task import q_rad_to_action  # noqa: E402
from rl_move.tests.test_task_semantics import (  # noqa: E402
    RISE_REF, SCORE_OVERRIDES, _make_rise_env)

SIGMA = float(sys.argv[1]) if len(sys.argv) > 1 else None
OV = dict(SCORE_OVERRIDES)
if SIGMA is not None:
    OV[("reward", "rise_ref_sigma_deg")] = SIGMA


def rollout(policy: str, seed: int) -> dict:
    env = _make_rise_env(seed, OV)
    env.reset()
    ref = np.load(ROOT / RISE_REF)
    q_ref, ramp_ref = ref["q_rad"], int(ref["ramp_i0"])
    q0 = env.data.qpos[env._qadr].copy()
    q_stilt = np.array([0.0, 0.0, 80.0] * 6) * DEG2RAD
    j_half = ramp_ref + (len(q_ref) - ramp_ref) // 2
    rng = np.random.default_rng(seed)
    sums: dict[str, float] = defaultdict(float)
    total, step = 0.0, 0
    while True:
        if policy == "replay":
            j = ramp_ref + (step - env._rise_ramp_i0)
            act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        elif policy == "partial":
            j = min(ramp_ref + (step - env._rise_ramp_i0), j_half)
            act = q_rad_to_action(q_ref[min(max(j, 0), len(q_ref) - 1)])
        elif policy == "flagleg":
            j = ramp_ref + (step - env._rise_ramp_i0)
            q = q_ref[min(max(j, 0), len(q_ref) - 1)].copy()
            q[0:3] = q0[0:3]
            act = q_rad_to_action(q)
        elif policy == "freeze":
            act = q_rad_to_action(q0)
        elif policy == "thrash":
            act = rng.uniform(-1.0, 1.0, size=18)
        else:
            act = q_rad_to_action(
                q0 if step < env._rise_ramp_i0 else q_stilt)
        _o, r, term, trunc, info = env.step(act)
        total += float(r)
        for k, v in info.items():
            if k.startswith("reward") and isinstance(v, (int, float)):
                sums[k] += float(v)
        step += 1
        if term or trunc:
            break
    env.close()
    sums["TOTAL"] = total
    return dict(sums)


for pol in ("replay", "partial", "stilt", "flagleg", "freeze"):
    s = rollout(pol, 0)
    big = {k: round(v, 1) for k, v in sorted(
        s.items(), key=lambda kv: abs(kv[1]), reverse=True)
        if abs(v) >= 1.0}
    print(f"{pol:8s} {big}")

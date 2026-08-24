"""probe_walk_kick.py — matched-parent check for cw-dep-tip1-kick1's
pre-registered gate (RL_PLAN queue -1, walk-kick DR axis).

WHY: the run's gate needs "terminal-fall rate under forced walk_kick
injection (prob 1, 14-22deg) < tip1 baseline by >=2x" AND "kicked-
episode tail |roll| median < 4deg" — neither metric exists in the
standard eval_checkpoint report (walk mode has no per-episode roll
trace). This is the matched-parent control the run's own gate
requires (RESEARCH_RULES: no verdict on an injected axis without it).

WHAT IT DOES: rolls the child (cw-dep-tip1-kick1) and the frozen
parent (cw-dep-tip1) deterministically through N seeds with the exact
training reward/goal stack, dr.walk_kick forced to prob=1.0 and the
gate's 14-22deg dose (absolute cfg override, bypasses dr_scale
scaling — see sim_env.py "ABSOLUTE values applied AFTER dr_scale
scaling"). Per episode: terminal fall (term_reason == tilt_roll) and
tail |roll_rel_deg| median over the last 1.0s. Forward command only
(the bench transient is a gait-start phenomenon, not direction-
specific per bench_report).

    uv run python -m rl_move.sim.probe_walk_kick
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

from rl_move.config import load_config  # noqa: E402
from rl_move.sim.probe_walk_income import VREF1_STACK, pin_command  # noqa: E402
from rl_move.sim.servo_model import SimServoParams  # noqa: E402
from rl_move.sim.walk_task import SimHexapodJointWalkEnv  # noqa: E402

CMD_V = 0.055
EPISODE_S = 15.0
TAIL_S = 1.0
KICK_DEG = (14.0, 22.0)   # the gate's dose

CKPTS = {
    "tip1_kick1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1_kick1.zip",
    "tip1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1.zip",
}


def kick_stack() -> dict:
    stack = dict(VREF1_STACK)
    stack[("dr", "walk_kick_prob")] = 1.0
    stack[("dr", "walk_kick_deg")] = KICK_DEG
    return stack


def make_kick_env(seed: int, dr_scale: float = 0.0):
    """Build the env with randomize=True EXPLICITLY (not gated on
    dr_scale>0 like probe_walk_income.make_env) so the absolute
    dr.walk_kick_* cfg override actually applies — dr_scale still
    zeroes every OTHER DR axis, isolating the kick (mirrors
    test_task_semantics.py's _make_kick_env)."""
    cfg = load_config()
    for (sec, leaf), val in kick_stack().items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=True,
        dr_scale=dr_scale, episode_seconds=EPISODE_S, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def rollout(ckpt_key: str, seed: int, deterministic: bool = True) -> dict:
    from stable_baselines3 import PPO

    env = make_kick_env(seed, dr_scale=0.0)
    obs, _ = env.reset()
    assert env._ep_rand is not None and env._ep_rand.walk_kick_roll_deg != 0.0, \
        "kick draw did not fire at prob 1 — injection is not applying"
    pin_command(env, CMD_V, 0.0)
    model = PPO.load(str(ROOT / CKPTS[ckpt_key]), device="cpu")

    tail_n = int(round(TAIL_S / env.dt))
    rolls: list[float] = []
    term_reason = None
    step = 0
    while True:
        act, _ = model.predict(obs, deterministic=deterministic)
        obs, _r, term, trunc, info = env.step(act)
        rolls.append(abs(float(info.get("roll_rel_deg", 0.0))))
        step += 1
        if term or trunc:
            term_reason = info.get("termination_reason")
            break
    env.close()
    tail = rolls[-tail_n:] if len(rolls) >= tail_n else rolls
    return {
        "ckpt": ckpt_key, "seed": seed, "ticks": step,
        "fell": bool(term_reason == "tilt_roll"),
        "term_reason": term_reason,
        "peak_roll_deg": round(float(max(rolls)), 1) if rolls else 0.0,
        "tail_roll_med_deg": round(float(np.median(tail)), 2) if tail else 0.0,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9,10,11")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]

    rows = []
    for ckpt_key in ("tip1_kick1", "tip1"):
        for seed in seeds:
            rows.append(rollout(ckpt_key, seed))

    print(f"{'ckpt':<12} {'seed':>4} {'fell':>5} {'peak':>6} {'tail_med':>9} "
          f"{'reason':<12}")
    for r in rows:
        print(f"{r['ckpt']:<12} {r['seed']:>4} {str(r['fell']):>5} "
              f"{r['peak_roll_deg']:>6.1f} {r['tail_roll_med_deg']:>9.2f} "
              f"{str(r['term_reason']):<12}")

    summary = {}
    for ckpt_key in ("tip1_kick1", "tip1"):
        sub = [r for r in rows if r["ckpt"] == ckpt_key]
        fell = sum(r["fell"] for r in sub)
        tails = [r["tail_roll_med_deg"] for r in sub if not r["fell"]]
        summary[ckpt_key] = {
            "n": len(sub), "fell": fell,
            "fall_rate": round(fell / max(len(sub), 1), 3),
            "tail_roll_med_deg_over_survivors": (
                round(float(np.median(tails)), 2) if tails else None),
        }
    print(json.dumps(summary, indent=2))

    out = {"rows": rows, "summary": summary}
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(out, indent=2))
        print(f"[probe_walk_kick] -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

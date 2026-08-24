"""probe_walk_push.py — matched-parent check for the walk-push torque
DR axis (RL_PLAN queue -1), generalized 08-12 (GAIT.md correction on
cw-dep-bcgait1-hard1-fric/-groundtilt5's triage: "needs a small
generalization first" before any push probe of the tall-walking
lineage) beyond its original cw-dep-tip1-push1 use.

WHY: a run's gate needs "terminal-fall rate under forced walk_push
injection (prob 1, 2.6 N*m fixed, 1.5s) < baseline by >=2x" AND
"nominal DR0 retention matches the baseline's own band" — neither
exists in the standard eval_checkpoint report (walk mode has no
per-episode roll trace). This is the matched-parent control any
such gate requires (RESEARCH_RULES: no verdict on an injected axis
without it). Twin of probe_walk_kick.py, parameterized for the xfrc
torque axis instead of the command-side kick.

GENERALIZATION NOTE: the dep-line stack (VREF1_STACK) and the
tall-walking BC-INIT lineage (cw-dep-bcgait1, cw-dep-bcgait1-hard1,
and its retention children) turn out to train with an IDENTICAL
physically-relevant stack (goal.walk_speed_min/max_m_s=0.05/0.06,
walk_obs_body_vel=2, safety.max_roll/pitch_deg=25, walk_park_start_frac
=0.25, anchor_tol_mm=10 — verified against both lineages' own launch
commands, 2026-08-12) — bcgait1 only ADDS reward-only terms
(walk_height_gate/sigma_mm) that shape training income, not physics
or the obs layout, so they cannot matter to this probe (which never
computes reward, only reads physical info fields). No new STACK was
needed; only the checkpoint registry (CKPTS) required entries for the
tall-walking lineage. If a future lineage's stack diverges on a
PHYSICALLY relevant axis (obs shape, DR ranges, safety envelope),
add a new STACKS-style entry rather than assuming VREF1_STACK still
matches — verify against that run's own launch command first.

    uv run python -m rl_move.sim.probe_walk_push
    uv run python -m rl_move.sim.probe_walk_push --ckpts bcgait1_hard1,tip1
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
PUSH_NM = (2.6, 2.6)   # the gate's calibrated fixed dose
PUSH_S = (1.5, 1.5)

CKPTS = {
    "tip1_push1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1_push1.zip",
    "tip1_push1_hard1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1_push1_hard1.zip",
    "tip1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1.zip",
    # Tall-walking BC-INIT lineage (added 08-12, see GENERALIZATION
    # NOTE above) — none of these ever trained with walk_push; the
    # comparison is diagnostic (does the tall gait share the crouch
    # lineage's takeoff-roll vulnerability?), not a push-hardening
    # matched-parent check yet.
    "bcgait1": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1.zip",
    "bcgait1_hard1": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
    "bcgait1_hard1_fric": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1_fric.zip",
    "bcgait1_hard1_groundtilt5":
        "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1_groundtilt5.zip",
}


def push_stack() -> dict:
    stack = dict(VREF1_STACK)
    stack[("dr", "walk_push_prob")] = 1.0
    stack[("dr", "walk_push_nm")] = PUSH_NM
    stack[("dr", "walk_push_s")] = PUSH_S
    return stack


def make_push_env(seed: int, dr_scale: float = 0.0):
    """randomize=True EXPLICITLY (not gated on dr_scale>0) so the
    absolute dr.walk_push_* cfg override applies regardless of
    dr_scale, which still zeroes every OTHER DR axis — isolating the
    push (mirrors probe_walk_kick.make_kick_env)."""
    cfg = load_config()
    for (sec, leaf), val in push_stack().items():
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

    env = make_push_env(seed, dr_scale=0.0)
    obs, _ = env.reset()
    assert env._ep_rand is not None and env._ep_rand.walk_push_peak_nm != 0.0, \
        "push draw did not fire at prob 1 — injection is not applying"
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
    ap.add_argument("--ckpts", default="tip1_push1,tip1",
                     help="comma-separated pair of CKPTS keys, child first")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    ckpt_keys = tuple(args.ckpts.split(","))
    assert len(ckpt_keys) == 2 and all(k in CKPTS for k in ckpt_keys), \
        f"--ckpts must be two comma-separated keys from {list(CKPTS)}"

    rows = []
    for ckpt_key in ckpt_keys:
        for seed in seeds:
            rows.append(rollout(ckpt_key, seed))

    print(f"{'ckpt':<12} {'seed':>4} {'fell':>5} {'peak':>6} {'tail_med':>9} "
          f"{'reason':<12}")
    for r in rows:
        print(f"{r['ckpt']:<12} {r['seed']:>4} {str(r['fell']):>5} "
              f"{r['peak_roll_deg']:>6.1f} {r['tail_roll_med_deg']:>9.2f} "
              f"{str(r['term_reason']):<12}")

    summary = {}
    for ckpt_key in ckpt_keys:
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
        print(f"[probe_walk_push] -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

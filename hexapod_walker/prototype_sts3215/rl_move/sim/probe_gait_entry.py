"""probe_gait_entry.py — staged gait-entry prototype A/B (operator
ruling 08-13: "INSTRUMENT the transient, then DESIGN a staged
gait-entry transition"; training arms only after an instrumented
design exists — this probe IS the instrumented design step, no
training).

MEASURED BASIS (08-11 bench tapes, 26 walks, analysis 08-13): the
takeoff roll transient is a DROP-IN POSTURE SNAP, not a response to
the velocity command — the policy saturates the 1.5 deg/tick slew on
ALL 18 joints from tick 0 at ZERO command (96-99% of joint-ticks in
the first second; all six legs move equally, ~160 deg cumulative
travel each in 1.5 s), and 14/26 tapes cross 5 deg of roll BEFORE the
runner's velocity ramp begins at t=1.04 s. Peak |gyro_x| median 32
dps (11-89). So "reduce the first step" cannot fix it; the entry
throttle must act at policy HANDOFF. Design under test: the
SafetyLayer entry slew ramp (safety.entry_slew_ramp_s /
entry_slew_start_deg, default-off), which caps the posture snap's
joint speed right after engage and ramps authority back up.

A/B: identical walk episodes from plant start, runner-shaped command
(1 s zero + 1 s ramp — pin_command), under the calibrated hardware
takeoff proxy dr.walk_push (2.6 N*m, 1.5 s, prob 1 — the axis that
reproduces the bench coin-flip regime policy-in-the-loop, SIM.md),
plus a clean no-push arm for entry quality. Metrics per episode:
fell / term_reason, peak |roll_rel| in the first 2.5 s and overall,
peak |roll rate|, tail roll (last 1 s), forward progress by t=10 s
(the throttle must not kill locomotion).

    python3 -m rl_move.sim.probe_gait_entry --ckpts tip1,bcgait1_hard1
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

RAD2DEG = 180.0 / np.pi
CMD_V = 0.055
EPISODE_S = 15.0
TAIL_S = 1.0
EARLY_S = 2.5          # the hardware transient window (+cmd ramp end)
PUSH_NM = (2.6, 2.6)   # calibrated fixed dose (probe_walk_push twin)
PUSH_S = (1.5, 1.5)

CKPTS = {
    "tip1": "rl_move/sim/policies/ppo_goal_cw_dep_tip1.zip",
    "bcgait1_hard1": "rl_move/sim/policies/ppo_goal_cw_dep_bcgait1_hard1.zip",
}

# Entry schedules under test. start=0.25 deg/tick ~ 6x slower than the
# trained 1.5; ramp 1.5 s covers the measured excursion window (roll-5deg
# crossings at 0.48-1.5 s) and the runner's settle+ramp start.
ENTRIES = {
    "off":        {},
    "ramp1.5_025": {("safety", "entry_slew_ramp_s"): 1.5,
                    ("safety", "entry_slew_start_deg"): 0.25},
    "ramp1.0_05":  {("safety", "entry_slew_ramp_s"): 1.0,
                    ("safety", "entry_slew_start_deg"): 0.5},
}


def make_env(seed: int, entry: dict, push: bool):
    cfg = load_config()
    stack = dict(VREF1_STACK)
    if push:
        stack[("dr", "walk_push_prob")] = 1.0
        stack[("dr", "walk_push_nm")] = PUSH_NM
        stack[("dr", "walk_push_s")] = PUSH_S
    stack.update(entry)
    for (sec, leaf), val in stack.items():
        cfg.setdefault(sec, {})[leaf] = val
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(None), randomize=push,
        dr_scale=0.0, episode_seconds=EPISODE_S, seed=seed, cfg=cfg)
    gen = env._goal_gen
    for m in ("hold", "lean", "track", "unload", "raise", "rise",
              "lower", "quad", "walk"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 1.0 if m == "walk" else 0.0)
    return env


def rollout(ckpt_key: str, entry_key: str, seed: int, push: bool) -> dict:
    from stable_baselines3 import PPO

    env = make_env(seed, ENTRIES[entry_key], push)
    obs, _ = env.reset()
    if push:
        assert env._ep_rand is not None and \
            env._ep_rand.walk_push_peak_nm != 0.0, "push draw missing"
    pin_command(env, CMD_V, 0.0)
    model = PPO.load(str(ROOT / CKPTS[ckpt_key]), device="cpu")

    early_n = int(round(EARLY_S / env.dt))
    tail_n = int(round(TAIL_S / env.dt))
    prog_n = int(round(10.0 / env.dt))
    x0 = float(env.data.qpos[0])
    rolls, rates, x_at_10 = [], [], None
    term_reason = None
    while True:
        act, _ = model.predict(obs, deterministic=True)
        obs, _r, term, trunc, info = env.step(act)
        rolls.append(abs(float(info.get("roll_rel_deg", 0.0))))
        rates.append(abs(float(env._state.imu_gyro[0])) * RAD2DEG)
        if len(rolls) == prog_n:
            x_at_10 = float(env.data.qpos[0]) - x0
        if term or trunc:
            term_reason = info.get("termination_reason")
            break
    if x_at_10 is None:      # early termination
        x_at_10 = float(env.data.qpos[0]) - x0
    env.close()
    tail = rolls[-tail_n:] if len(rolls) >= tail_n else rolls
    return {
        "ckpt": ckpt_key, "entry": entry_key, "push": push, "seed": seed,
        "ticks": len(rolls),
        "fell": bool(term_reason == "tilt_roll"),
        "term_reason": term_reason,
        "peak_roll_early_deg": round(max(rolls[:early_n]), 1),
        "peak_roll_deg": round(max(rolls), 1),
        "peak_rate_early_dps": round(max(rates[:early_n]), 0),
        "tail_roll_med_deg": round(float(np.median(tail)), 2),
        "x_prog_10s_m": round(x_at_10, 3),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", default="0,1,2,3,4,5,6,7,8,9,10,11")
    ap.add_argument("--ckpts", default="tip1")
    ap.add_argument("--entries", default="off,ramp1.5_025,ramp1.0_05")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    rows = []
    for ck in args.ckpts.split(","):
        for entry in args.entries.split(","):
            for push in (True, False):
                for sd in seeds:
                    r = rollout(ck, entry, sd, push)
                    rows.append(r)
                    print(json.dumps(r), flush=True)
    # summary
    print("\n=== SUMMARY (n=%d seeds) ===" % len(seeds))
    for ck in args.ckpts.split(","):
        for entry in args.entries.split(","):
            for push in (True, False):
                sel = [r for r in rows if r["ckpt"] == ck
                       and r["entry"] == entry and r["push"] == push]
                if not sel:
                    continue
                falls = sum(r["fell"] for r in sel)
                pk = np.median([r["peak_roll_early_deg"] for r in sel])
                rt = np.median([r["peak_rate_early_dps"] for r in sel])
                tl = np.median([r["tail_roll_med_deg"] for r in sel])
                xp = np.median([r["x_prog_10s_m"] for r in sel])
                print(f"{ck:14s} {entry:12s} push={int(push)} "
                      f"falls {falls}/{len(sel)}  peak2.5s {pk:.1f}deg  "
                      f"rate2.5s {rt:.0f}dps  tail {tl:.2f}  x10s {xp:+.3f}m")
    if args.out:
        Path(args.out).write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())

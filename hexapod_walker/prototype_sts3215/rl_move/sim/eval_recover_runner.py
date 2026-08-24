"""eval_recover_runner.py — DEPLOYMENT-RUNNER sim gate for recover mode.

Operator order 2026-08-20 (MCP operator lane): run the recover
champion's 23-rung ladder THROUGH the deployment runner code path
(rl_move/sim/recover_runner.py) — entry gating, entry hold, the
runner's OWN observation assembly from raw sensor reads, prev-action
bookkeeping, deploy-side success detector in shadow — instead of the
training env's obs builder. PASS evidence = every rung's env-truth
``recover_success`` termination plus detector-agreement stats.

Deployment-honest deltas vs the matched training-eval
(eval_checkpoint --modes recover):
  * q_nom = the entry ENCODER read (the robot's only option), not the
    env's passive-equilibrium capture ~0.3 s earlier (drift <= ~8
    mrad at DR-0; --parity-qnom uses the env's value to isolate this).
  * tilt reference = calibrated level-IMU bias, default (0,0) — at
    DR-0 identical to the env's gravity-truth anchoring; under DR the
    runner is the deploy truth (no privileged attitude).
  * the reset-history probe becomes a real 0.6 s ENTRY HOLD that
    consumes wall time; the env horizon is extended by exactly those
    15 ticks so the policy keeps its trained 16 s active budget.

Canonical invocation (controller smoke = 2-3 rungs; full gate on a
train pod):

    uv run python -m rl_move.sim.eval_recover_runner \
        --out logs/recover_runner_gate
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from rl_move.robot_state import RAD2DEG

from .recover_runner import (
    CHAMPION_ZIP, ENCODER_PT, HISTORY, RecoverRunner, contract_cfg,
    load_recover_policy)

_PROTO = Path(__file__).resolve().parents[2]
START_BANK = "rl_move/sim/park_banks/footlow2_hard1_lower_endpoints_full.npz"


def make_env(dr_scale: float, seed: int, episode_seconds: float):
    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv
    cfg = contract_cfg()
    # The runner performs the entry hold itself; the env-side probe
    # must be OFF or the probe frames would be spent twice.
    cfg.setdefault("obs", {})["reset_history_probe"] = 0.0
    cfg.setdefault("goal", {})["recover_start_bank"] = str(
        _PROTO / START_BANK)
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), randomize=dr_scale > 0,
        dr_scale=dr_scale, episode_seconds=episode_seconds, seed=seed,
        render_mode=None, cfg=cfg)
    gen = env._goal_gen
    for m in ("walk", "hold", "lean", "track", "unload", "raise",
              "rise", "lower", "quad"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    gen.p_recover = 1.0
    return env


def run_episode(env, model, kind: str, active_budget_s: float,
                parity_qnom: bool) -> dict:
    env.force_recover_start = kind
    env.reset()
    state = env._state
    runner = RecoverRunner(model, cfg=env.cfg,
                           plant_rad=env._plant_deg
                           * np.pi / 180.0,
                           active_budget_s=active_budget_s,
                           shadow_detector=True)
    for _ in range(runner.quiet_ticks):
        runner.observe_idle(state)
    ok, why = runner.start(
        state, q_nom_override=(env._q_nom if parity_qnom else None))
    wait_ticks = 0
    while not ok and wait_ticks < int(3.0 / env.dt):
        # Deployment semantics: the runner refuses while tumbling and
        # the session controller waits (freezing on the current pose)
        # until the body lands, then re-issues the command.
        from .joint_task import q_rad_to_action
        _o, _r, term, trunc, _i = env.step(
            q_rad_to_action(np.asarray(env._state.joint_position)))
        wait_ticks += 1
        if term or trunc:
            return {"kind": kind, "env_success": False,
                    "term_reason": f"ended while waiting to land ({why})",
                    "ticks": wait_ticks, "recover_s": None,
                    "runner_state": runner.state,
                    "detector_fire_tick": None, "detector_fired": False,
                    "roll_tail_deg": None, "end_tilt_deg": None,
                    "wall_s": 0.0}
        ok, why = runner.start(
            env._state,
            q_nom_override=(env._q_nom if parity_qnom else None))
    if not ok:
        return {"kind": kind, "env_success": False,
                "term_reason": f"entry refused 3s: {why}",
                "ticks": wait_ticks, "recover_s": None,
                "runner_state": runner.state,
                "detector_fire_tick": None, "detector_fired": False,
                "roll_tail_deg": None, "end_tilt_deg": None,
                "wall_s": 0.0}
    state = env._state
    action = runner.entry_action()
    t0 = time.monotonic()
    ticks = 0
    term_reason = "none"
    env_success = False
    roll_tail: list[float] = []
    while True:
        _obs, _r, term, trunc, info = env.step(action)
        ticks += 1
        roll_tail.append(abs(float(info.get("roll_deg", 0.0))))
        if term or trunc:
            term_reason = str(info.get("termination_reason") or
                              ("truncated" if trunc else "terminated"))
            env_success = term_reason == "recover_success"
            break
        action = runner.tick(env._state)
        runner.note_action(action)
        if runner.state == "timeout":
            term_reason = "runner_timeout"
            break
    return {
        "kind": kind,
        "env_success": bool(env_success),
        "term_reason": term_reason,
        "ticks": ticks,
        "recover_s": round(ticks * env.dt, 2),
        "runner_state": runner.state,
        "detector_fire_tick": runner.detector_fire_tick,
        "detector_fired": runner.detector_fire_tick is not None,
        "roll_tail_deg": round(float(np.mean(roll_tail[-25:])), 2)
        if roll_tail else None,
        "end_tilt_deg": round(max(
            abs(float(env._state.imu_roll)),
            abs(float(env._state.imu_pitch))) * RAD2DEG, 2),
        "wall_s": round(time.monotonic() - t0, 1),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, default=CHAMPION_ZIP)
    ap.add_argument("--encoder", type=Path, default=ENCODER_PT)
    ap.add_argument("--dr-scale", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--active-seconds", type=float, default=16.0,
                    help="policy budget after the entry hold "
                    "(trained episode length)")
    ap.add_argument("--kinds", type=str, default=None,
                    help="comma list; default = full 23-rung ladder")
    ap.add_argument("--parity-qnom", action="store_true",
                    help="use the env's captured q_nom instead of the "
                    "entry encoder read (contract-isolation probe)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    # horizon = entry hold (15 ticks) + trained active budget + margin
    episode_seconds = args.active_seconds + (HISTORY - 1) / 25.0 + 0.2
    env = make_env(args.dr_scale, args.seed, episode_seconds)
    kinds = ([k.strip() for k in args.kinds.split(",") if k.strip()]
             if args.kinds else
             [k for b in range(len(env.RECOVER_FAMILIES))
              for k in env._recover_family_kinds(b)])
    model = load_recover_policy(args.checkpoint, args.encoder)

    rows = []
    for kind in kinds:
        row = run_episode(env, model, kind, args.active_seconds,
                          args.parity_qnom)
        rows.append(row)
        print(f"[recover-runner-gate] {kind:16s} "
              f"success={row['env_success']} "
              f"reason={row.get('term_reason'):18s} "
              f"t={row.get('recover_s')}s "
              f"detector={'fires@' + str(row.get('detector_fire_tick')) if row.get('detector_fired') else 'silent'} "
              f"end_tilt={row.get('end_tilt_deg')}deg",
              flush=True)

    n_ok = sum(r["env_success"] for r in rows)
    det_agree = sum(1 for r in rows
                    if r["env_success"] == bool(r.get("detector_fired")))
    summary = {
        "checkpoint": str(args.checkpoint), "dr_scale": args.dr_scale,
        "seed": args.seed, "parity_qnom": bool(args.parity_qnom),
        "n_kinds": len(rows), "n_success": int(n_ok),
        "detector_agreement": f"{det_agree}/{len(rows)}",
        "rows": rows,
    }
    print(f"[recover-runner-gate] TOTAL {n_ok}/{len(rows)} "
          f"env-truth recoveries; detector agrees {det_agree}/{len(rows)}")
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        (args.out / "report.json").write_text(json.dumps(summary,
                                                         indent=1))
        print(f"[recover-runner-gate] report: {args.out}/report.json")


if __name__ == "__main__":
    main()

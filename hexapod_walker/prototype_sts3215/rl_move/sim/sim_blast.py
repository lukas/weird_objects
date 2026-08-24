"""Sim blast — every pre-bench decision probe in one command (Mac CPU).

Companion to ``rl_move/scripts/bench_blast.py`` (the hardware half).
Each experiment answers a named decision for the NEXT bench session;
none of this is training. Run from prototype_sts3215:

    uv run python -m rl_move.sim.sim_blast            # full (~30-45 min CPU)
    uv run python -m rl_move.sim.sim_blast --quick    # ~third the episodes

Experiments (see RL_PLAN queue -1 / HARDWARE.md for the bench story):

  rolltrap   vref1-r1 vs tip1 under the mid-gait roll-torque trap
             (_roll_trap_stats — the sim proxy for the hardware
             runaway) on the NOMINAL floor (already mu 2.0, the
             friction saturation) and slicker ones (mu 1.4 / 1.0).
             Decides: does sim rank tip1 above vref1 on the runaway
             mechanism, and — since grippier-than-trained does not
             exist in this contact model — whether ANY friction
             magnitude reproduces the hardware failure (if none does,
             the no-skate/pinning contact-gap story stands).
  panel      eval_drive joystick gate for both checkpoints, naked
             (45 deg envelope) and rot60-wrapped (full circle).
             Decides: is tip1 rot60-safe (it is the ACTIVE walk slot
             but only vref1-r1 was full-circle validated), and the
             fwd-scenario distance is the sim tape prediction for the
             bench reading (6 s at 0.05 m/s).
  artifacts  pytest test_stand_runner.py + test_rot60_runner.py — the
             deployed numpy exports the re-push will ship. Decides:
             is the repo export safe to push before STAND is pressed.

Outputs: logs/sim_blast_<stamp>/summary.json + per-run eval_drive
reports, and a printed decision table.
"""
from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_v, "2")

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

_PROTO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROTO))

POLICIES = Path(__file__).resolve().parent / "policies"
CKPTS = {
    "vref1": POLICIES / "ppo_goal_cw_dep_vref1_r1.zip",
    "tip1": POLICIES / "ppo_goal_cw_dep_tip1.zip",
}
# Floor variants for the roll trap: name -> foot-ground slide mu
# (0 = keep the XML default). NOTE the default pair is ALREADY mu 2.0
# (max grip; sim travel saturates >=1.5 per the calibrate_slip replay,
# RL_PLAN problem 1) — so "grippier than trained" does not exist in
# this contact model, and the informative spread is DOWNWARD: if the
# runaway only appears on slicker floors the hardware gap is not a
# friction-magnitude story at all (supports the no-skate/pinning
# model lever).
FLOORS = {"nominal(mu2.0)": 0.0, "mu1.4": 1.4, "mu1.0": 1.0}


def _make_walk_env(mu: float, seed: int):
    from rl_move.config import load_config

    from .servo_model import SimServoParams
    from .walk_task import SimHexapodJointWalkEnv

    cfg = load_config()
    if mu > 0:
        cfg.setdefault("env", {})["foot_friction_slide"] = mu
    env = SimHexapodJointWalkEnv(
        params=SimServoParams.from_cfg(cfg), randomize=False,
        dr_scale=0.0, episode_seconds=10.0, seed=seed, cfg=cfg)
    gen = env._goal_gen
    gen.p_walk = 1.0
    for m in ("hold", "lean", "track", "unload", "raise", "rise", "lower"):
        if hasattr(gen, f"p_{m}"):
            setattr(gen, f"p_{m}", 0.0)
    return env


def run_rolltrap(episodes: int, seed: int) -> dict:
    from .gru_policy import load_checkpoint_auto
    from .train_ppo_sim import _ActFn, _roll_trap_stats

    out: dict = {}
    for cname, ckpt in CKPTS.items():
        model = load_checkpoint_auto(ckpt, device="cpu")
        act = _ActFn(model)
        for fname, mu in FLOORS.items():
            t0 = time.time()
            env = _make_walk_env(mu, seed)
            rt = _roll_trap_stats(env, act, episodes, walk=True)
            env.close()
            key = f"{cname}/{fname}"
            out[key] = {k: round(float(v), 3) for k, v in rt.items()}
            print(f"[rolltrap] {key:16s} pass={rt['pass_frac']:.2f} "
                  f"trapped={rt['trapped_frac']:.2f} "
                  f"max_roll={rt['max_roll_deg']:.1f} "
                  f"end_roll={rt['end_roll_deg']:.1f} "
                  f"speed_frac={rt['speed_frac']:.2f} "
                  f"legs={rt['legs_cycling']:.1f} "
                  f"({time.time() - t0:.0f}s)", flush=True)
    return out


def run_panels(out_dir: Path, quick: bool) -> dict:
    """eval_drive joystick panels, subprocessed so a fall/crash in one
    run cannot take the whole blast down."""
    runs = []
    for cname, ckpt in CKPTS.items():
        runs.append((f"{cname}-naked", ckpt, []))
        runs.append((f"{cname}-rot60", ckpt,
                     ["--rot60", "--heading-max-deg", "180"]))
    results: dict = {}
    flips = "1" if quick else "3"
    for name, ckpt, extra in runs:
        rep = out_dir / f"panel_{name}.json"
        cmd = [sys.executable, "-m", "rl_move.sim.eval_drive", str(ckpt),
               "--speed", "0.05", "--dr-scale", "0",
               "--flip-episodes", flips, "--out", str(rep), *extra]
        t0 = time.time()
        p = subprocess.run(cmd, cwd=_PROTO, capture_output=True, text=True)
        if rep.exists():
            r = json.loads(rep.read_text())
            fwd = r["scenarios"].get("fwd", {})
            results[name] = {
                "gate": r["gate"], "gate_falls": r["gate_falls"],
                "fwd_dist_m": fwd.get("dist_m"),
                "fwd_trk_err": fwd.get("tracking_err"),
                "report": rep.name,
            }
            print(f"[panel] {name:14s} gate={r['gate']} "
                  f"falls={r['gate_falls']} fwd_dist={fwd.get('dist_m')}m "
                  f"({time.time() - t0:.0f}s)", flush=True)
        else:
            results[name] = {"error": (p.stderr or p.stdout)[-400:]}
            print(f"[panel] {name}: FAILED to produce a report\n"
                  f"{(p.stderr or p.stdout)[-400:]}", flush=True)
    return results


def run_artifact_tests() -> dict:
    tests = ["rl_move/tests/test_stand_runner.py",
             "rl_move/tests/test_rot60_runner.py"]
    p = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", *tests],
        cwd=_PROTO, capture_output=True, text=True)
    tail = (p.stdout or "").strip().splitlines()[-1:]
    print(f"[artifacts] pytest rc={p.returncode} {' '.join(tail)}",
          flush=True)
    return {"rc": p.returncode, "tail": " ".join(tail),
            "tests": tests}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--quick", action="store_true",
                    help="4 roll-trap episodes + 1 flip episode instead "
                         "of 8/3")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--skip", nargs="*", default=[],
                    choices=["rolltrap", "panel", "artifacts"],
                    help="experiments to skip")
    args = ap.parse_args()

    missing = [str(p) for p in CKPTS.values() if not p.is_file()]
    if missing:
        raise SystemExit(f"missing checkpoints (ops.sh pullckpt): "
                         f"{missing}")
    out_dir = _PROTO / "logs" / f"sim_blast_{time.strftime('%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: dict = {"quick": args.quick, "seed": args.seed,
                     "stamp": time.strftime("%Y-%m-%d %H:%M:%S")}

    if "artifacts" not in args.skip:
        summary["artifacts"] = run_artifact_tests()
    if "rolltrap" not in args.skip:
        summary["rolltrap"] = run_rolltrap(4 if args.quick else 8,
                                           args.seed)
    if "panel" not in args.skip:
        summary["panel"] = run_panels(out_dir, args.quick)

    (out_dir / "summary.json").write_text(json.dumps(summary, indent=1))
    print(f"\nwrote {out_dir / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

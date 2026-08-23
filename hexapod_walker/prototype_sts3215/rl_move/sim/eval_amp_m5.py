"""Fixed MuJoCo cross-engine suite for the AMP track's M5 DONE gate.

AMP_LOCOMOTION.md §13 M5: "The same actor weights must replay in MuJoCo
with no retraining and preserve: command response; recognizable gait;
push recovery; at least partial fault adaptation."

The trainer is MJX/Warp; this suite is the OTHER engine: it composes the
existing plain-CPU-MuJoCo tools (eval_checkpoint.py walk panels, eval_yaw
turn panel) plus the dr.ext_push_* / dr.fault_* injection cfgs into ONE
versioned invocation with pre-registered bars, and writes a single
m5_verdict.json. No new physics, no new metrics -- pure composition, so
a section read here is directly comparable to the per-milestone gates
that produced the bars.

Sections and bars (v1, recorded in OPERATOR_QUESTIONS.md; sources named):
  walk   eval_checkpoint, own cfg, DR-0, det+sto: 0 terminations,
         gait_valid 12/12, det prog med >= 0.75, det slip med <= 3.5
         (M2 bcinit-sec5 lineage gates).
  yaw    eval_yaw, own cfg: tip-left AND tip-right |wz_err| med <= 0.20,
         0 falls (turnclone lineage bar, 08-23). Skipped (marked
         not_capable, fails overall) if the cfg has no goal.walk_yaw_cmd=1.
  push   walk section + dr.ext_push_prob=1.0 (base 10-25N single shove):
         terminations <= 2/6 det and <= 3/6 sto, gait_valid >= 10/12
         (M3 pushsmoke/pushhard bars).
  fault  walk section + dr.fault_prob=1.0 (one weak/frozen/disabled-leg
         fault per episode): terminations <= 2/12, gait_valid >= 10/12,
         det forward_dist med >= 0.10 m (no statue; M4 faultobs2 bars).

Usage (run on a pod; each section is a full harness eval with videos):
  python3 -m rl_move.sim.eval_amp_m5 ckpt.zip --out-dir logs/ckpt_eval/<name>_m5 \
      [--cfg-set k=v ...]   # the checkpoint's OWN training cfg-sets
      [--per-mode 6] [--seed 0] [--speed 0.08]
      [--skip walk,yaw,push,fault]
"""
from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
from pathlib import Path

BARS = {
    "walk": dict(max_terms=0, min_gait_valid=12, min_det_prog_med=0.75,
                 max_det_slip_med=3.5),
    "yaw": dict(max_tip_err=0.20, max_falls=0),
    "push": dict(max_det_terms=2, max_sto_terms=3, min_gait_valid=10),
    "fault": dict(max_terms=2, min_gait_valid=10, min_det_fwd_med=0.10),
}


def _run(cmd: list[str]) -> int:
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def _eval_ckpt(ckpt: str, out: Path, cfg_sets: list[str], per_mode: int,
               seed: int, extra_cfg: list[str]) -> dict | None:
    out.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, "-m", "rl_move.sim.eval_checkpoint", ckpt,
           "--task", "joint_walk", "--modes", "walk",
           "--per-mode", str(per_mode), "--dr-scale", "0",
           "--seed", str(seed), "--video-every", "1",
           "--out", str(out)]
    for c in cfg_sets + extra_cfg:
        cmd += ["--cfg-set", c]
    rc = _run(cmd)
    rep = out / "report.json"
    if rc != 0 or not rep.exists():
        return None
    return json.loads(rep.read_text())


def _walk_stats(report: dict) -> dict:
    eps = report["episodes"]
    det = eps.get("walk/det", [])
    sto = eps.get("walk/sto", [])
    both = det + sto
    return dict(
        n=len(both),
        terms=sum(1 for e in both if e.get("terminated")),
        det_terms=sum(1 for e in det if e.get("terminated")),
        sto_terms=sum(1 for e in sto if e.get("terminated")),
        gait_valid=sum(1 for e in both if e.get("gait_valid")),
        sacrificed=sorted({leg for e in both for leg in (e.get("sacrificed_legs") or [])}),
        det_prog_med=statistics.median([e["progress_ratio"] for e in det]) if det else None,
        det_slip_med=statistics.median([e["slip_per_m"] for e in det]) if det else None,
        det_fwd_med=statistics.median([e["forward_dist_m"] for e in det]) if det else None,
        sto_prog_med=statistics.median([e["progress_ratio"] for e in sto]) if sto else None,
        sto_slip_med=statistics.median([e["slip_per_m"] for e in sto]) if sto else None,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint")
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--cfg-set", action="append", default=[],
                    help="the checkpoint's OWN training cfg (obs contract)")
    ap.add_argument("--per-mode", type=int, default=6)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=0.08,
                    help="eval_yaw commanded translation speed")
    ap.add_argument("--wz-max", type=float, default=0.3)
    ap.add_argument("--skip", default="",
                    help="comma-separated sections to skip")
    args = ap.parse_args()

    skip = {s.strip() for s in args.skip.split(",") if s.strip()}
    out: Path = args.out_dir
    out.mkdir(parents=True, exist_ok=True)
    verdict: dict = {"checkpoint": args.checkpoint, "suite": "amp-m5-v1",
                     "bars": BARS, "sections": {}}

    # -- walk (command response + recognizable gait) ----------------------
    if "walk" not in skip:
        rep = _eval_ckpt(args.checkpoint, out / "walk", args.cfg_set,
                         args.per_mode, args.seed, [])
        if rep is None:
            verdict["sections"]["walk"] = {"pass": False, "error": "harness failed"}
        else:
            s = _walk_stats(rep)
            b = BARS["walk"]
            s["pass"] = (s["terms"] <= b["max_terms"]
                         and s["gait_valid"] >= min(b["min_gait_valid"], s["n"])
                         and not s["sacrificed"]
                         and (s["det_prog_med"] or 0) >= b["min_det_prog_med"]
                         and (s["det_slip_med"] or 99) <= b["max_det_slip_med"])
            verdict["sections"]["walk"] = s

    # -- yaw (turn both ways) ---------------------------------------------
    if "yaw" not in skip:
        yaw_capable = any(c.startswith("goal.walk_yaw_cmd=1") for c in args.cfg_set)
        if not yaw_capable:
            verdict["sections"]["yaw"] = {"pass": False, "not_capable": True}
        else:
            yjson = out / "yaw.json"
            cmd = [sys.executable, "-m", "rl_move.sim.eval_yaw", args.checkpoint,
                   "--speed", str(args.speed), "--wz-max", str(args.wz_max),
                   "--seed", str(args.seed), "--out", str(yjson)]
            for c in args.cfg_set:
                cmd += ["--cfg-set", c]
            _run(cmd)  # its own gate rc is the strict 0.10 default; we re-judge
            if not yjson.exists():
                verdict["sections"]["yaw"] = {"pass": False, "error": "eval_yaw failed"}
            else:
                y = json.loads(yjson.read_text())
                sc = y.get("scenarios", {})
                tl = sc.get("tip-left", {}).get("wz_med")
                tr = sc.get("tip-right", {}).get("wz_med")
                b = BARS["yaw"]
                ok = (tl is not None and tr is not None
                      and tl <= b["max_tip_err"] and tr <= b["max_tip_err"]
                      and y.get("falls", 99) <= b["max_falls"])
                verdict["sections"]["yaw"] = {
                    "pass": bool(ok), "tip_left_err": tl, "tip_right_err": tr,
                    "turn_err_med": y.get("turn_wz_err_med"),
                    "hold_wz_med": y.get("hold_wz_med"), "falls": y.get("falls")}

    # -- push recovery ------------------------------------------------------
    if "push" not in skip:
        rep = _eval_ckpt(args.checkpoint, out / "push", args.cfg_set,
                         args.per_mode, args.seed, ["dr.ext_push_prob=1.0"])
        if rep is None:
            verdict["sections"]["push"] = {"pass": False, "error": "harness failed"}
        else:
            s = _walk_stats(rep)
            b = BARS["push"]
            s["pass"] = (s["det_terms"] <= b["max_det_terms"]
                         and s["sto_terms"] <= b["max_sto_terms"]
                         and s["gait_valid"] >= min(b["min_gait_valid"], s["n"]))
            verdict["sections"]["push"] = s

    # -- fault adaptation ---------------------------------------------------
    if "fault" not in skip:
        rep = _eval_ckpt(args.checkpoint, out / "fault", args.cfg_set,
                         args.per_mode, args.seed, ["dr.fault_prob=1.0"])
        if rep is None:
            verdict["sections"]["fault"] = {"pass": False, "error": "harness failed"}
        else:
            s = _walk_stats(rep)
            b = BARS["fault"]
            s["pass"] = (s["terms"] <= b["max_terms"]
                         and s["gait_valid"] >= min(b["min_gait_valid"], s["n"])
                         and (s["det_fwd_med"] or 0) >= b["min_det_fwd_med"])
            verdict["sections"]["fault"] = s

    ran = [k for k in ("walk", "yaw", "push", "fault") if k in verdict["sections"]]
    verdict["m5_pass"] = bool(ran) and all(
        verdict["sections"][k].get("pass") for k in ran)
    (out / "m5_verdict.json").write_text(json.dumps(verdict, indent=1))
    print(json.dumps({k: verdict["sections"][k].get("pass") for k in ran}
                     | {"m5_pass": verdict["m5_pass"]}, indent=1))
    print(f"m5 artifacts: {out}")
    return 0 if verdict["m5_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())

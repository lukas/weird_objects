"""analyze_scale.py — aggregate the representation-scaling sweep.

Reads the newest eval JSON per dyn_scale_* checkpoint from
rl_move/dynamics/logs/ (written by eval_model.py) and prints one table:
per-horizon model MSE vs the matched linear baseline, latent MSEs, and
both gate verdicts, sorted by (data, history, size). The scaling
question (operator next-steps 08-13) is whether prediction quality
improves with model size / context / data — not whether any single
cell passes the gate.

    uv run python -m rl_move.dynamics.analyze_scale
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOG_DIR = ROOT / "rl_move" / "dynamics" / "logs"

SIZE_ORDER = {"S": 0, "M": 1, "L": 2}


def newest_reports() -> dict[str, dict]:
    reports: dict[str, tuple[str, dict]] = {}
    for p in sorted(LOG_DIR.glob("eval_dyn_scale_*.json")):
        m = re.match(r"eval_(dyn_scale_.+)_(\d{8}_\d{6})\.json$", p.name)
        if not m:
            continue
        name, ts = m.group(1), m.group(2)
        if name not in reports or ts > reports[name][0]:
            reports[name] = (ts, json.loads(p.read_text()))
    return {k: v[1] for k, v in reports.items()}


def main() -> None:
    reps = newest_reports()
    if not reps:
        sys.exit(f"no eval_dyn_scale_*.json under {LOG_DIR}")

    def key(name: str):
        m = re.match(r"dyn_scale_([SML])_h(\d+)_(\w+)$", name)
        assert m, name
        return (m.group(3), int(m.group(2)), SIZE_ORDER[m.group(1)])

    print(f"{'name':<24} {'k1 mdl/lin':>12} {'k2 mdl/lin':>12} "
          f"{'k5 mdl/lin':>12} {'z10':>7} {'z25':>7} {'G1':>5} {'G1.1':>5}")
    for name in sorted(reps, key=key):
        r = reps[name]
        h = r["horizons"]
        cells = []
        for k in ("1", "2", "5"):
            row = h.get(k) or h.get(int(k))
            cells.append(f"{row['model_mse']:.3f}/{row['linear_mse']:.3f}")
        z = []
        for k in ("10", "25"):
            row = h.get(k) or h.get(int(k))
            z.append(f"{row['latent_mse']:.3f}")
        print(f"{name:<24} {cells[0]:>12} {cells[1]:>12} {cells[2]:>12} "
              f"{z[0]:>7} {z[1]:>7} "
              f"{'PASS' if r.get('gate_g1_pass') else 'fail':>5} "
              f"{'PASS' if r.get('gate_g1_1_pass') else 'fail':>5}")
    print("\nread: model/linear normalized state MSE per short horizon "
          "(lower is better; <1x linear = beats matched ridge); z10/z25 "
          "= latent MSE vs the same model's unchanged-z reference. "
          "Compare cells DOWN a column: size S->M->L within one "
          "(data, history) block is the scaling curve.")


if __name__ == "__main__":
    main()

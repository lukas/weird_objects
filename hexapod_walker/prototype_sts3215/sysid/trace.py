"""Load / write sysid traces (the on-robot ``sysid_*.csv`` schema).

A *trace* is the synchronized telemetry recorded while a protocol ran —
on hardware (``linux_control/sysid_runner.py``) or in simulation
(``sysid.replay`` writes the identical schema for synthetic traces and
round-trip tests). Raw traces are never modified.
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from . import PROTO_DIR  # noqa: F401  (bootstraps sys.path)
from sysid_protocol import N_JOINTS  # noqa: E402

CSV_FIELDS = (["t_s", "tick", "seg", "phase", "joint",
               "t_send_s", "t_recv_s", "overrun"]
              + [f"q{j}_deg" for j in range(N_JOINTS)]
              + [f"cmd{j}_deg" for j in range(N_JOINTS)]
              + ["cur_a", "load_pct", "volt", "temp_c"])


def _f(v: str) -> float:
    return float(v) if v not in ("", None) else float("nan")


def load(csv_path: Path | str) -> dict:
    """Parse a sysid CSV (+ its ``_summary.json`` sidecar if present)."""
    csv_path = Path(csv_path)
    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        raise ValueError(f"{csv_path}: empty trace")
    tr = {
        "name": csv_path.name,
        "path": csv_path,
        "t": np.array([_f(r["t_s"]) for r in rows]),
        "tick": np.array([int(r["tick"]) for r in rows]),
        "seg": np.array([int(r["seg"]) for r in rows]),
        "phase": [r["phase"] for r in rows],
        "joint": np.array([int(r["joint"]) for r in rows]),
        "t_send": np.array([_f(r["t_send_s"]) for r in rows]),
        "t_recv": np.array([_f(r["t_recv_s"]) for r in rows]),
        "overrun": np.array([int(r.get("overrun") or 0) for r in rows]),
        "q": np.array([[_f(r[f"q{j}_deg"]) for j in range(N_JOINTS)]
                       for r in rows]),
        "cmd": np.array([[_f(r[f"cmd{j}_deg"]) for j in range(N_JOINTS)]
                         for r in rows]),
        "cur_a": np.array([_f(r.get("cur_a", "")) for r in rows]),
    }
    sp = csv_path.with_name(csv_path.stem + "_summary.json")
    tr["summary"] = json.loads(sp.read_text()) if sp.exists() else {}
    tr["protocol"] = tr["summary"].get("protocol")
    return tr


def write(csv_path: Path | str, *, t: np.ndarray, tick: np.ndarray,
          seg: np.ndarray, phase: list[str], joint: np.ndarray,
          t_send: np.ndarray, t_recv: np.ndarray, q: np.ndarray,
          cmd: np.ndarray, summary: dict | None = None) -> Path:
    """Write a trace in the on-robot CSV schema (sim / synthetic use)."""
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_FIELDS)
        for k in range(len(t)):
            w.writerow(
                [f"{t[k]:.4f}", int(tick[k]), int(seg[k]), phase[k],
                 int(joint[k]), f"{t_send[k]:.4f}", f"{t_recv[k]:.4f}", 0]
                + [f"{v:.3f}" for v in q[k]]
                + [f"{v:.3f}" for v in cmd[k]]
                + ["", "", "", ""])
    if summary is not None:
        csv_path.with_name(csv_path.stem + "_summary.json").write_text(
            json.dumps(summary, indent=1))
    return csv_path


def segments(tr: dict) -> list[dict]:
    """Per-segment view: protocol metadata + row slice into the trace.

    Requires the embedded protocol (summary sidecar); the runner always
    writes it.
    """
    proto = tr.get("protocol")
    if not proto:
        raise ValueError(f"{tr['name']}: no embedded protocol "
                         "(missing _summary.json)")
    out = []
    for i, s in enumerate(proto["segments"]):
        idx = np.flatnonzero(tr["seg"] == i)
        if idx.size == 0:
            continue
        out.append({"index": i, "spec": s,
                    "kind": s["kind"],
                    "joint": s.get("joint", -1),
                    "label": s.get("label", f"seg{i}"),
                    "sl": slice(int(idx[0]), int(idx[-1]) + 1)})
    return out

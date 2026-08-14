"""Sysid harness: measured hardware-to-MuJoCo calibration pipeline.

See ``sysid/README.md`` for the workflow. Modules assume they are run
from ``prototype_sts3215/`` as ``python -m sysid.<module>`` (repo
.venv); this package bootstraps ``sys.path`` so both the robot-side
protocol code (``linux_control/sysid_protocol.py``) and the sim stack
(``rl_move.sim``) import cleanly.
"""
from __future__ import annotations

import sys
from pathlib import Path

_PROTO = Path(__file__).resolve().parents[1]
for _p in (str(_PROTO), str(_PROTO / "linux_control")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

PROTO_DIR = _PROTO
PROTOCOL_DIR = Path(__file__).resolve().parent / "protocols"
DATASET_DIR = Path(__file__).resolve().parent / "datasets"
REPORT_DIR = Path(__file__).resolve().parent / "reports"

#!/usr/bin/env python3
"""Paint a deploy/restart notice on the robot TFT.

This is meant to run while ``hexapod-web.service`` is stopped, so it can
briefly claim the MCU serial bridge without contending with the web/control
process. It uses the MCU ``DJ`` job screen only; no servo reads or motion.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from mcu_feetech_bus import MCU_PORT_DEFAULT, McuFeetechBus  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=MCU_PORT_DEFAULT)
    ap.add_argument("--title", default="DEPLOYING")
    ap.add_argument("--line", action="append", default=[])
    ap.add_argument("--footer", default="web restart")
    ap.add_argument("--pct", type=int, default=-1)
    args = ap.parse_args()

    lines = [args.title]
    body = list(args.line) or ["code updated", "web restarting", "please wait"]
    lines.extend(body[:4])
    while len(lines) < 5:
        lines.append("")
    lines.append(args.footer)

    bus = McuFeetechBus(args.port)
    try:
        ok = bus.display_job(lines, pct=args.pct, timeout=6.0)
        if not ok:
            bus.display_recover(attempts=2, timeout=6.0)
            ok = bus.display_job(lines, pct=args.pct, timeout=6.0)
        return 0 if ok else 1
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())

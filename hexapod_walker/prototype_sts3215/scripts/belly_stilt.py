#!/usr/bin/env python3
"""RETIRED — belly stilts replaced by yaw_servo_retainer stands.

Aug 2026: each ``yaw_servo_retainer`` carries a permanent 38 mm open-cage
ground stand (tip at chassis underside − 38 mm).  Print 6×
``yaw_servo_retainer.stl`` instead of these velcro props.

This module remains only so old commands fail with a clear message.
"""
from __future__ import annotations

import sys


def main() -> None:
    print(
        "belly_stilt is retired.\n"
        "  Use hexapod_prototype.make_yaw_servo_retainer() / "
        "stl_prototype/yaw_servo_retainer.stl\n"
        "  (6× footed saddles, 38 mm tip, stay on while walking).",
        file=sys.stderr,
    )
    raise SystemExit(1)


if __name__ == "__main__":
    main()

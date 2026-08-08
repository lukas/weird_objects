#!/usr/bin/env python3
"""RETIRED — no printed belly stand exists any more.

The velcro belly stilts were first replaced by ground stands on the
``yaw_servo_retainer`` (Aug 2026), and those stands were then removed in
the flat-belly rework (also Aug 2026): the chassis underside is flat
except the hanging yaw servos + their retainer saddles.  Use an external
bench support when the robot must be held off its feet.

This module remains only so old commands fail with a clear message.
"""
from __future__ import annotations

import sys


def main() -> None:
    print(
        "belly_stilt is retired, and the retainer ground stands that "
        "replaced it were\nremoved in the Aug 2026 flat-belly rework.  "
        "There is no printed belly stand;\nuse an external bench support.",
        file=sys.stderr,
    )
    raise SystemExit(1)


if __name__ == "__main__":
    main()

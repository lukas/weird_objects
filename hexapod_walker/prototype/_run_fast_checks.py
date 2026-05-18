"""Run all _verify_prototype checks EXCEPT the slow workspace self-
collision sweep.  Used for fast iteration on geometry edits.
"""
from __future__ import annotations
import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from _verify_prototype import (
    check_watertight,
    check_cradle_openness,
    check_bolt_holes,
    check_wire_slot,
    check_self_collision,
    check_servo_clearance,
    check_flimsy_joints,
    check_thin_sheets,
)

if __name__ == "__main__":
    results = [
        ("Mesh watertightness",  check_watertight()),
        ("Cradle openness",       check_cradle_openness()),
        ("Bolt-hole engagement",  check_bolt_holes()),
        ("Wire-exit slot",        check_wire_slot()),
        ("Self-collision",        check_self_collision()),
        ("Servo clearance",       check_servo_clearance()),
        ("Flimsy joints",         check_flimsy_joints()),
        ("Thin sheets",           check_thin_sheets()),
    ]
    print()
    print("=" * 72)
    print("FAST-CHECK Summary (workspace self-collision NOT run):")
    all_ok = True
    for name, ok in results:
        print(f"   [{'PASS' if ok else 'FAIL'}]  {name}")
        all_ok &= ok
    print("=" * 72)
    sys.exit(0 if all_ok else 1)

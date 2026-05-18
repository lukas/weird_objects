"""Run JUST the new thin-sheet check, to iterate quickly on geometry."""
from __future__ import annotations

import os
import sys

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from _verify_prototype import check_thin_sheets


if __name__ == "__main__":
    ok = check_thin_sheets()
    print()
    print(f"thin-sheet check: {'PASS' if ok else 'FAIL'}")
    sys.exit(0 if ok else 1)

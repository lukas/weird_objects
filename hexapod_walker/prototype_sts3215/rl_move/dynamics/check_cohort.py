"""Check script-owned dynrep transfer cohorts.

These cohorts predate launcher integration, so this helper gives the
orchestrator a mechanical truth source: manifest events, live transfer
processes, and per-seed logs. It intentionally does not infer success
from STATUS prose.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys


def _cmdlines() -> list[str]:
    out: list[str] = []
    proc = pathlib.Path("/proc")
    if not proc.exists():
        return out
    for p in proc.glob("[0-9]*/cmdline"):
        try:
            raw = p.read_bytes().replace(b"\0", b" ").strip()
        except OSError:
            continue
        if raw:
            out.append(raw.decode("utf-8", "replace"))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cohort", default="risewalk")
    ap.add_argument("--logs", default="rl_move/dynamics/logs")
    args = ap.parse_args()

    logs = pathlib.Path(args.logs)
    manifest = logs / f"{args.cohort}_manifest.jsonl"
    if not manifest.exists():
        print(f"NO_MANIFEST {manifest}")
        return 2

    events = []
    for line in manifest.read_text().splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError as exc:
            print(f"BAD_MANIFEST_LINE {exc}: {line}")
            return 2

    live = [c for c in _cmdlines() if "rl_move.dynamics.train_ppo_transfer" in c]
    seed_done = [e for e in events if e.get("event") == "seed_done"]
    done = [e for e in events if e.get("event") == "done"]
    starts = [e for e in events if e.get("event") == "start"]
    print(f"COHORT {args.cohort}")
    print(f"manifest={manifest}")
    if starts:
        s = starts[-1]
        print(f"started={s.get('utc')} host={s.get('host')} seeds={s.get('seeds')}")
        print(f"encoder={s.get('encoder')} data={s.get('data')}")
    print(f"events={len(events)} seed_done={len(seed_done)} final_done={bool(done)}")
    print(f"live_train_ppo_transfer={len(live)}")
    for c in live[:12]:
        print("  " + c[:220])

    if done:
        return 0
    if live:
        return 1
    print("NOT_RUNNING_AND_NOT_DONE")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())

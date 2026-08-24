"""Run a sysid protocol on the hexapod over HTTP and pull the trace.

SAFETY: this tool moves the robot. It refuses without ``--go``, and the
default posture for every standard protocol is *robot on a stand, feet
off the ground, operator watching*. No hand-posing is needed: the
on-robot runner glides the legs to the protocol's start pose by itself
(slow, trip-protected, pose-verified) before the experiment starts.
Preflight is read-only; ``Ctrl-C`` (or ``--abort``) sends
``/api/rl/stop``, and the runner limps on any trip. Never run this
unless the operator asked for this exact experiment.

Flow: read-only preflight (pose + IMU sanity) -> POST /api/sysid/run
(protocol JSON in the body — nothing to deploy per-experiment) -> poll
until the job finishes -> download the CSV + summary into
``sysid/datasets/<protocol>_<stamp>/`` (raw traces are never
overwritten).

Run (from prototype_sts3215/, repo .venv)::

    uv run python -m sysid.run_hw --protocol sysid/protocols/steps_air_v1.json --go
    uv run python -m sysid.run_hw --abort          # emergency stop the job
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

from . import DATASET_DIR, PROTO_DIR  # noqa: F401
from sysid_protocol import (  # noqa: E402
    duration_s, protocol_hash, validate,
)
from rl_move.remote import HexapodClient  # noqa: E402


def _pull(client: HexapodClient, name: str, dst_dir: Path) -> Path | None:
    url = f"{client.base}/api/logs/{name}"
    dst = dst_dir / name
    try:
        dst_dir.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(url, timeout=30) as resp:
            dst.write_bytes(resp.read())
        return dst
    except Exception as e:
        print(f"  !! pull failed ({name}): {e}")
        return None


def _newest_sysid_csv(client: HexapodClient, after_unix: float,
                      wait_s: float = 15.0) -> str | None:
    """Newest sysid_*.csv written after ``after_unix``, size-stable."""
    deadline = time.time() + wait_s
    last = None
    while True:
        logs = client._req("GET", "/api/logs")
        found = None
        for f in logs.get("files", []):
            n = f.get("name", "")
            if (n.startswith("sysid_") and n.endswith(".csv")
                    and f.get("mtime_unix", 0) > after_unix):
                found = (n, int(f.get("bytes", 0)))
                break
        if found is not None and found == last:
            return found[0]
        last = found
        if time.time() >= deadline:
            return found[0] if found else None
        time.sleep(1.5)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--protocol", type=Path)
    ap.add_argument("--url", default=None)
    ap.add_argument("--go", action="store_true",
                    help="actually move the robot (default: dry-run plan)")
    ap.add_argument("--force", action="store_true",
                    help="required for whole-body traj protocols")
    ap.add_argument("--abort", action="store_true",
                    help="just send /api/rl/stop and exit")
    args = ap.parse_args(argv)

    client = HexapodClient(args.url)
    if args.abort:
        print(json.dumps(client.stop(), indent=2))
        return 0
    if not args.protocol:
        ap.error("need --protocol (or --abort)")

    doc = json.loads(args.protocol.read_text())
    errs = validate(doc)
    if errs:
        raise SystemExit("invalid protocol: " + "; ".join(errs))
    secs = duration_s(doc)
    has_traj = any(s.get("kind") == "traj" for s in doc["segments"])
    print(f"protocol '{doc['name']}' hash {protocol_hash(doc)}: "
          f"{len(doc['segments'])} segments, {secs:.0f} s @ "
          f"{doc.get('hz', 25)} Hz"
          + (" — WHOLE-BODY traj (needs --force)" if has_traj else ""))
    print(f"posture: {doc.get('description', '(none)')}")

    if not args.go:
        print("\nDRY RUN (no motion). Re-run with --go when the robot is "
              "on the stand, feet off the ground, and you are watching.")
        return 0

    # Read-only preflight: bus + IMU answering, robot idle.
    fb = client.feedback()
    if not fb.get("ok") or fb.get("live", 0) < 18:
        raise SystemExit(f"preflight failed: feedback={fb}")
    print(f"preflight: {fb['live']}/18 servos, roll {fb.get('roll_deg')} "
          f"pitch {fb.get('pitch_deg')}")

    t_start = time.time()
    kick = client._req("POST", "/api/sysid/run",
                       {"protocol": doc, "force": bool(args.force)})
    print(json.dumps({k: v for k, v in kick.items()
                      if k != "calibrate"}, indent=2))
    if not kick.get("ok"):
        return 1

    try:
        res = client.wait_idle(timeout_s=secs + 120.0, poll_s=1.0)
    except KeyboardInterrupt:
        print("\n^C — sending stop (robot limps)")
        client.stop()
        res = client.wait_idle(timeout_s=15.0)
    result = res.get("result") or {}
    print(f"result: ok={result.get('ok')} "
          f"ticks {result.get('ticks_done')}/{result.get('ticks_planned')}"
          f" overruns={result.get('overruns')} "
          f"error={result.get('error')}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out_dir = DATASET_DIR / f"{doc['name']}_{stamp}"
    csv_name = (Path(result["csv"]).name if result.get("csv")
                else _newest_sysid_csv(client, t_start))
    if not csv_name:
        print("no trace CSV found on the robot")
        return 1
    got = _pull(client, csv_name, out_dir)
    sum_name = csv_name.replace(".csv", "_summary.json")
    _pull(client, sum_name, out_dir)
    (out_dir / "protocol.json").write_text(json.dumps(doc, indent=1,
                                                      sort_keys=True))
    print(f"dataset: {out_dir}" + (f" ({got.name})" if got else ""))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())

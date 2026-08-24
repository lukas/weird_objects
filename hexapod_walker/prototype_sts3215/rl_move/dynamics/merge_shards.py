"""merge_shards.py — merge per-seed collection subdirs into one dataset.

collect.py's append-safe shard numbering is NOT safe under concurrent
writers (parallel collects into one dir race on the next shard index).
The pod pipelines therefore collect into per-seed SUBDIRS in parallel
and merge afterwards:

    uv run python -m rl_move.dynamics.merge_shards \
        --src datasets/v3pod/s0 datasets/v3pod/s1 datasets/v3pod/s2 \
        --out datasets/v3pod --require-actor noslip:0.05

Shards are copied (renumbered sequentially, sources sorted by path so
the merge is deterministic) and the per-subdir meta.json run entries are
concatenated. --require-actor NAME:MINFRAC hard-fails (exit 4) if the
merged dataset's episode share of that actor falls below the minimum —
the v3 pipeline uses it to make the v2pod noslip->tripod drift
impossible to reproduce silently.
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rl_move.dynamics import data as dd          # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--src", nargs="+", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--require-actor", action="append", default=[],
                    metavar="NAME:MINFRAC",
                    help="fail (exit 4) unless the merged episode share "
                         "of NAME is >= MINFRAC")
    args = ap.parse_args()

    out = ROOT / args.out
    out.mkdir(parents=True, exist_ok=True)
    if list(out.glob("shard_*.npz")):
        sys.exit(f"ABORT: {out} already contains shards — refusing to "
                 f"merge into a non-empty dataset (idempotency guard)")

    srcs = sorted((ROOT / s) for s in args.src)
    meta = None
    idx = 0
    for src in srcs:
        shards = sorted(src.glob("shard_*.npz"))
        if not shards:
            sys.exit(f"ABORT: no shards under {src}")
        m = json.loads((src / "meta.json").read_text())
        if meta is None:
            meta = {k: v for k, v in m.items() if k != "runs"}
            meta["runs"] = []
        elif m.get("layout_version") != meta.get("layout_version"):
            sys.exit(f"ABORT: layout mismatch: {src} has "
                     f"{m.get('layout_version')!r}, expected "
                     f"{meta.get('layout_version')!r}")
        meta["runs"].extend(m.get("runs", []))
        for sh in shards:
            shutil.copyfile(sh, out / f"shard_{idx:03d}.npz")
            idx += 1
    meta["merged_from"] = [str(s.relative_to(ROOT)) for s in srcs]
    (out / "meta.json").write_text(json.dumps(meta, indent=2) + "\n")

    eps = dd.load_dataset(out)
    print(dd.describe(eps))
    counts: dict[str, int] = {}
    for e in eps:
        counts[e.actor] = counts.get(e.actor, 0) + 1
    fracs = {a: n / len(eps) for a, n in sorted(counts.items())}
    print("actor shares: " + "  ".join(f"{a}={f:.3f}"
                                       for a, f in fracs.items()))
    for req in args.require_actor:
        name, minfrac = req.split(":")
        got = fracs.get(name, 0.0)
        if got < float(minfrac):
            print(f"MERGE_ACTOR_REQUIREMENT_FAIL: {name} share {got:.3f} "
                  f"< required {float(minfrac):.3f} — dataset recipe "
                  f"drifted (see dynrep STATUS 08-13: this is the v2pod "
                  f"noslip drift guard)")
            sys.exit(4)
    print(f"merged {idx} shards from {len(srcs)} subdirs -> {out}")


if __name__ == "__main__":
    main()

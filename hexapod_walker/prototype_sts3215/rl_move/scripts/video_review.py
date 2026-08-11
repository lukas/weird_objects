"""Video review — turn a bench_blast --video recording into readable
frame sheets aligned with the session log.

The operator films the session once (see bench_blast VIDEO MODE);
this script does the bullshit so nobody has to: it maps the session's
unix-stamped events onto video time, cuts a frame sheet (2 fps contact
grid) per walk / turn / stand event plus full-res frames of the
tape-zoom segment, and writes an index the analysis agent reads to
extract distance, turn sign, falls, and gait quality.

Sync: bench_blast announces "sync mark" at a known t_unix right after
recording starts. Give --sync <video-seconds-of-that-announcement> if
you know it (listen for it near the start of the footage); otherwise
the script guesses it from the first sustained motion in the frame and
prints the guess — re-run with --sync to correct if the clips look
shifted.

    python -m rl_move.scripts.video_review IMG_1234.mov \
        [--session rl_move/hardware_traces/bench_blast_<stamp>] \
        [--sync 12.5]

Outputs into <session>/video/: <tag>_sheet.jpg per event (frames at
2 fps, timestamp burned into each cell), tape_zoom_*.jpg full-res
frames, and index.json.

Requires opencv (repo venv has it). iPhone HEVC .mov that cv2 cannot
open: convert first with
``ffmpeg -i in.mov -c:v libx264 -crf 20 out.mp4``.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve()
TRACES_DIR = _HERE.parents[1] / "hardware_traces"

SHEET_FPS = 2.0          # frames per second sampled into a sheet
SHEET_COLS = 6
CELL_W = 480             # sheet cell width px (keeps sheets readable)
PRE_S, POST_S = 1.0, 2.0  # context around each event window


def newest_session() -> Path:
    dirs = sorted(TRACES_DIR.glob("bench_blast_*"),
                  key=lambda p: p.stat().st_mtime)
    if not dirs:
        raise SystemExit("no bench_blast_* session under hardware_traces/")
    return dirs[-1]


def detect_first_motion(cap, max_scan_s: float = 120.0) -> float:
    """Video time (s) of the first sustained motion — the fallback sync
    anchor (assumed to be the first announced walk's start).

    Threshold is RELATIVE to the clip's own energy range: a small robot
    in a wide frame only shifts a few hundred pixels per step, so any
    fixed cutoff either misses it or fires on sensor noise."""
    import cv2
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(fps / 4))          # ~4 samples/s
    prev = None
    samples: list[tuple[float, float]] = []
    idx = 0
    while idx / fps < max_scan_s:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            break
        small = cv2.cvtColor(cv2.resize(frame, (160, 90)),
                             cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev is not None:
            samples.append((idx / fps,
                            float(np.mean(np.abs(small - prev)))))
        prev = small
        idx += step
    if not samples:
        return 0.0
    energies = np.array([e for _t, e in samples])
    floor = float(np.median(energies))
    peak = float(energies.max())
    if peak <= floor + 1e-6:
        return 0.0
    thresh = floor + 0.25 * (peak - floor)
    hot_since = None
    for (t, e) in samples:
        if e > thresh:
            if hot_since is None:
                hot_since = t
            elif t - hot_since >= 1.0:   # 1 s sustained
                return hot_since
        else:
            hot_since = None
    return 0.0


def grab(cap, t_video: float):
    import cv2
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(round(t_video * fps)))
    ok, frame = cap.read()
    return frame if ok else None


def sheet(cap, t0: float, t1: float, label: str, out: Path) -> int:
    """Contact sheet of frames sampled at SHEET_FPS over [t0, t1]."""
    import cv2
    cells = []
    t = t0
    while t <= t1 + 1e-6:
        f = grab(cap, t)
        if f is not None:
            h, w = f.shape[:2]
            scale = CELL_W / w
            f = cv2.resize(f, (CELL_W, int(h * scale)))
            cv2.putText(f, f"{label} t={t - t0:.1f}s", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(f, f"{label} t={t - t0:.1f}s", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
            cells.append(f)
        t += 1.0 / SHEET_FPS
    if not cells:
        return 0
    ch = cells[0].shape[0]
    cells = [c for c in cells if c.shape[0] == ch]
    rows = []
    for i in range(0, len(cells), SHEET_COLS):
        row = cells[i:i + SHEET_COLS]
        row += [np.zeros_like(cells[0])] * (SHEET_COLS - len(row))
        rows.append(np.hstack(row))
    cv2.imwrite(str(out), np.vstack(rows),
                [cv2.IMWRITE_JPEG_QUALITY, 85])
    return len(cells)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", type=Path)
    ap.add_argument("--session", type=Path, default=None,
                    help="bench_blast_<stamp> dir (default: newest)")
    ap.add_argument("--sync", type=float, default=None,
                    help="video time (s) of the spoken 'sync mark' "
                         "announcement; default: guess from first "
                         "sustained motion mapped to the first walk")
    args = ap.parse_args()

    import cv2

    session = args.session or newest_session()
    summary = json.loads((session / "summary.json").read_text())
    events = summary.get("events", [])
    if not events:
        raise SystemExit(f"{session}/summary.json has no events — was "
                         "the session run with --video?")

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"cv2 cannot open {args.video} (HEVC? convert: "
                         "ffmpeg -i in.mov -c:v libx264 -crf 20 out.mp4)")

    # unix -> video time mapping
    sync_ev = next((e for e in events if "sync mark" in e["text"]),
                   events[0])
    if args.sync is not None:
        video_of_sync = args.sync
        how = "operator --sync"
    else:
        first_walk = next((e for e in events
                           if e["text"].startswith("walk ")
                           and e["text"].endswith("starting")), None)
        anchor_ev = first_walk or sync_ev
        video_of_anchor = detect_first_motion(cap)
        video_of_sync = video_of_anchor - (anchor_ev["t_unix"]
                                           - sync_ev["t_unix"])
        how = (f"guessed: first motion at {video_of_anchor:.1f}s = "
               f"'{anchor_ev['text']}' — re-run with --sync to correct")

    def to_video(t_unix: float) -> float:
        return t_unix - sync_ev["t_unix"] + video_of_sync

    out_dir = session / "video"
    out_dir.mkdir(exist_ok=True)
    dur = ((cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
           / (cap.get(cv2.CAP_PROP_FPS) or 30.0))
    index = {"video": str(args.video), "session": str(session),
             "sync": {"video_of_sync_s": round(video_of_sync, 2),
                      "how": how},
             "video_duration_s": round(dur, 1), "items": []}
    print(f"sync: {how} (sync mark at video {video_of_sync:.1f}s)")

    # paired start/done events -> one sheet each
    starts = [e for e in events if e["text"].endswith("starting")]
    for e in starts:
        base = e["text"].rsplit(" starting", 1)[0]
        done = next((d for d in events
                     if d["text"].startswith(base)
                     and d["text"].endswith("done")
                     and d["t_unix"] > e["t_unix"]), None)
        t0 = to_video(e["t_unix"]) - PRE_S
        t1 = (to_video(done["t_unix"]) if done
              else t0 + PRE_S + 8.0) + POST_S
        if t1 < 0 or t0 > dur:
            print(f"  [skip] {base}: outside the footage")
            continue
        tag = base.replace(" ", "_").replace("+", "p").replace(".", "")
        f = out_dir / f"{tag}_sheet.jpg"
        n = sheet(cap, max(0.0, t0), min(dur, t1), base, f)
        index["items"].append({"event": base, "sheet": f.name,
                               "video_t0_s": round(t0, 1),
                               "video_t1_s": round(t1, 1), "frames": n})
        print(f"  {base}: {n} frames -> {f.name}")

    # tape-zoom segment: full-res frames (the number gets read here)
    zoom = next((e for e in events if "point the camera" in e["text"]),
                None)
    if zoom is not None:
        t0 = to_video(zoom["t_unix"])
        for i, dt in enumerate(np.arange(1.0, 6.5, 1.0)):
            f = grab(cap, t0 + float(dt))
            if f is not None:
                p = out_dir / f"tape_zoom_{i}.jpg"
                cv2.imwrite(str(p), f, [cv2.IMWRITE_JPEG_QUALITY, 92])
                index["items"].append({"event": "tape zoom",
                                       "frame": p.name,
                                       "video_t_s": round(t0 + dt, 1)})
        print("  tape zoom frames extracted")

    (out_dir / "index.json").write_text(json.dumps(index, indent=1))
    print(f"wrote {out_dir / 'index.json'}\n"
          "Next: hand the sheets to the analysis agent (distance off "
          "the tape frames, turn sign off the turn sheets, falls / "
          "gait quality off the walk sheets).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Robot-in-frame check for bench videos.

Answers, per frame: is the hexapod fully inside the camera frame?
Meant to run over a bench_blast camera.mp4 after (or during) a session so
framing problems are caught before a whole run is filmed with the robot
half out of shot.

Usage:
  python robot_in_frame.py <video.mp4> [--stride 2] [--annotate out.mp4]
      [--jsonl out.jsonl]
  python robot_in_frame.py --image frame.png

Prints a per-video summary (fractions full / partial / not visible, plus
the time ranges of any bad stretches) and exits 1 if more than
--fail-threshold of frames are not fully in frame.
"""

import argparse
import json
import sys
import time

import cv2

from detect import RobotDetector, annotate


def spans(flags, fps, stride):
    """Contiguous True stretches as (t0_s, t1_s)."""
    out, start = [], None
    for i, bad in enumerate(flags):
        if bad and start is None:
            start = i
        elif not bad and start is not None:
            out.append((start * stride / fps, i * stride / fps))
            start = None
    if start is not None:
        out.append((start * stride / fps, len(flags) * stride / fps))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", nargs="?", help="path to camera.mp4")
    ap.add_argument("--image", help="classify a single image instead")
    ap.add_argument("--stride", type=int, default=1, help="process every Nth frame")
    ap.add_argument("--annotate", help="write annotated video to this path")
    ap.add_argument("--jsonl", help="write per-frame verdicts to this path")
    ap.add_argument("--fail-threshold", type=float, default=0.05,
                    help="exit 1 if more than this fraction of frames is not 'full'")
    args = ap.parse_args()

    det = RobotDetector()

    if args.image:
        frame = cv2.imread(args.image)
        d = det.detect(frame)
        print(json.dumps({"verdict": d.verdict, "box": d.box, "score": d.score}))
        return

    if not args.video:
        ap.error("need a video path or --image")

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        sys.exit(f"cannot open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0

    writer = None
    jsonl = open(args.jsonl, "w") if args.jsonl else None
    verdicts = []
    t_start = time.perf_counter()
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % args.stride:
            idx += 1
            continue
        d = det.detect(frame)
        verdicts.append(d.verdict)
        if jsonl:
            jsonl.write(json.dumps(
                {"frame": idx, "t_s": round(idx / fps, 3),
                 "verdict": d.verdict, "box": d.box, "score": d.score}) + "\n")
        if args.annotate:
            if writer is None:
                writer = cv2.VideoWriter(
                    args.annotate, cv2.VideoWriter_fourcc(*"mp4v"),
                    fps / args.stride, (frame.shape[1], frame.shape[0]))
            writer.write(annotate(frame, d))
        idx += 1
    elapsed = time.perf_counter() - t_start
    cap.release()
    if writer:
        writer.release()
    if jsonl:
        jsonl.close()

    n = len(verdicts)
    if not n:
        sys.exit("no frames read")
    frac = {v: verdicts.count(v) / n for v in ("full", "partial", "not_visible")}
    print(f"{args.video}: {n} frames analyzed "
          f"({n / elapsed:.0f} fps on this machine)")
    print(f"  full        {frac['full']*100:5.1f}%")
    print(f"  partial     {frac['partial']*100:5.1f}%")
    print(f"  not_visible {frac['not_visible']*100:5.1f}%")
    bad = [v != "full" for v in verdicts]
    for t0, t1 in spans(bad, fps, args.stride):
        if t1 - t0 >= 0.5:
            print(f"  BAD {t0:7.1f}s – {t1:7.1f}s")

    if 1 - frac["full"] > args.fail_threshold:
        sys.exit(1)


if __name__ == "__main__":
    main()

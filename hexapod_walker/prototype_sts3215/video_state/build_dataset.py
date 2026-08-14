"""Build a (robot crop image -> body/leg state) dataset from bench sessions.

Sources: bench_blast_* sessions that have BOTH camera.mp4 and per-tick walk
trace CSVs. The camera metadata gives an exact video<->unix mapping
(camera.t0_unix) and each walk gives t_start_unix, so trace time for video
frame f is:  (t0_unix + f/fps) - t_start_unix  (+ a small refined offset).

Labels per frame (interpolated from the 25 Hz trace):
  [roll_deg, pitch_deg, q0..q17_deg]   -> 20 values

The per-walk sync offset is refined by cross-correlating video motion
energy (frame differencing over the robot region) against mean joint speed
from the trace; offsets are reported so a bad sync is visible.

Stand-up episodes (rl_stand traces in stand_fail_20260811, videoed in the
same sessions) are included too: their CSV start time is only loosely known
(the "learned stand up starting" event fires before a multi-second safe-zero
acquisition), so their sync is anchored on the event and refined over a
+/-6 s window — legitimate here because a stand is aperiodic, unlike a
gait. Stand episodes that fail to correlate are dropped.

Output: data/state_dataset.npz with
  images  (N, 144, 144, 3) uint8 RGB crops (144 so training can random-crop
          to the 128 model input)
  labels  (N, 20) float32
  bbox    (N, 4) float32 normalized detector box (cx, cy, w, h) in the full
          frame — cropping destroys the tilt reference, this restores it
  walk_id (N,) int16, walk_names list, offsets, meta
"""

import csv
import glob
import json
import os

import cv2
import numpy as np

from detect import RobotDetector, VERDICT_NOT_VISIBLE

ROOT = os.path.join(os.path.dirname(__file__), "..", "rl_move", "hardware_traces")
STAND_DIR = os.path.join(ROOT, "stand_fail_20260811")
OUT_DIR = os.path.join(os.path.dirname(__file__), "data")
CROP = 144   # model input is 128; margin allows random-crop augmentation
LABEL_NAMES = ["roll_deg", "pitch_deg"] + [f"q{i}_deg" for i in range(18)]


def read_trace(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t_s"]) for r in rows])
    lab = np.array(
        [[float(r["roll_deg"]), float(r["pitch_deg"])]
         + [float(r[f"q{i}_deg"]) for i in range(18)] for r in rows],
        dtype=np.float32,
    )
    return t, lab


def square_crop(frame, box, pad=1.45):
    x0, y0, x1, y1 = box
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    side = max(x1 - x0, y1 - y0) * pad
    h, w = frame.shape[:2]
    side = min(side, h, w)
    x0 = int(np.clip(cx - side / 2, 0, w - side))
    y0 = int(np.clip(cy - side / 2, 0, h - side))
    crop = frame[y0:y0 + int(side), x0:x0 + int(side)]
    return cv2.resize(crop, (CROP, CROP), interpolation=cv2.INTER_AREA)


def refine_offset(gray_crops, frame_times, trace_t, trace_lab, max_shift=0.35, fps=15.0):
    """Best time shift of trace vs video, via motion-energy correlation.

    The shift window must stay well under half a gait period (~1.2 s):
    walking motion is periodic, so a wider search aliases to period
    multiples and corrupts the (already exact) camera t0_unix sync.
    """
    if len(gray_crops) < 20:
        return 0.0, 0.0
    mv = np.array([np.mean(cv2.absdiff(a, b)) for a, b in zip(gray_crops, gray_crops[1:])])
    mv_t = frame_times[1:]
    dq = np.mean(np.abs(np.diff(trace_lab[:, 2:], axis=0)), axis=1)
    dq_t = trace_t[1:]

    def z(x):
        s = x.std()
        return (x - x.mean()) / (s + 1e-9)

    best, best_off = -np.inf, 0.0
    for off in np.arange(-max_shift, max_shift + 1e-9, 1.0 / fps):
        mt = np.interp(mv_t + off, dq_t, dq, left=np.nan, right=np.nan)
        m = ~np.isnan(mt)
        if m.sum() < 15:
            continue
        c = float(np.mean(z(mv[m]) * z(mt[m])))
        if c > best:
            best, best_off = c, off
    return best_off, best


class Builder:
    def __init__(self):
        self.det = RobotDetector()
        self.images, self.labels, self.bboxes = [], [], []
        self.walk_ids, self.walk_names, self.offsets = [], [], []

    def add_episode(self, video, cam0, fps, t_start, csv_path, name, *,
                    max_shift=0.35, min_corr=None):
        """Extract labeled frames for one episode.

        max_shift: sync refinement window (small for periodic walks, wide
        for aperiodic stands). min_corr: if set, drop the episode when the
        refinement correlation is below it (used for stands, whose anchor
        is only approximate).
        """
        trace_t, trace_lab = read_trace(csv_path)
        f0 = max(0, int(np.floor((t_start - cam0 - max_shift) * fps)))
        f1 = int(np.ceil((t_start + trace_t[-1] - cam0 + max_shift) * fps))
        cap = cv2.VideoCapture(video)
        cap.set(cv2.CAP_PROP_POS_FRAMES, f0)
        frames, boxes, fidx = [], [], []
        fw = fh = None
        for f in range(f0, f1 + 1):
            ok, frame = cap.read()
            if not ok:
                break
            fh, fw = frame.shape[:2]
            d = self.det.detect(frame)
            if d.verdict == VERDICT_NOT_VISIBLE:
                continue
            frames.append(square_crop(frame, d.box))
            boxes.append(d.box)
            fidx.append(f)
        cap.release()
        if len(frames) < 10:
            print(f"{name}: only {len(frames)} usable frames, skipped")
            return

        frame_times = np.array(fidx) / fps + cam0 - t_start  # trace time
        grays = [cv2.cvtColor(c, cv2.COLOR_BGR2GRAY) for c in frames]
        off, corr = refine_offset(grays, frame_times, trace_t, trace_lab,
                                  max_shift=max_shift, fps=fps)
        if min_corr is not None and corr < min_corr:
            print(f"{name}: sync corr {corr:.2f} too weak, episode dropped")
            return
        if corr < 0.2 and min_corr is None:
            off = 0.0
        wid = len(self.walk_names)
        kept = 0
        for i, ft in enumerate(frame_times):
            tt = ft + off
            if tt < trace_t[0] or tt > trace_t[-1]:
                continue
            lab = np.array(
                [np.interp(tt, trace_t, trace_lab[:, j]) for j in range(20)],
                dtype=np.float32,
            )
            x0, y0, x1, y1 = boxes[i]
            self.images.append(cv2.cvtColor(frames[i], cv2.COLOR_BGR2RGB))
            self.labels.append(lab)
            self.bboxes.append(np.array(
                [(x0 + x1) / 2 / fw, (y0 + y1) / 2 / fh,
                 (x1 - x0) / fw, (y1 - y0) / fh], dtype=np.float32))
            self.walk_ids.append(wid)
            kept += 1
        self.walk_names.append(name)
        self.offsets.append(off)
        print(f"{name}: {kept} frames, sync offset {off:+.2f}s (corr {corr:.2f})")


def main():
    b = Builder()

    for sdir in sorted(glob.glob(os.path.join(ROOT, "bench_blast_*"))):
        video = os.path.join(sdir, "camera.mp4")
        summary_path = os.path.join(sdir, "summary.json")
        if not (os.path.exists(video) and os.path.exists(summary_path)):
            continue
        summary = json.load(open(summary_path))
        cam = summary.get("camera") or {}
        if not cam.get("t0_unix"):
            continue
        cam0, fps = cam["t0_unix"], cam.get("fps", 15.0)
        session = os.path.basename(sdir)

        for walk in summary.get("walks", []):
            csv_name = walk.get("csv") or (walk.get("result") or {}).get("log")
            if not csv_name or not os.path.exists(os.path.join(sdir, csv_name)):
                continue
            b.add_episode(video, cam0, fps, walk["t_start_unix"],
                          os.path.join(sdir, csv_name),
                          f"{session}/{walk.get('tag', csv_name)}")

        # stand-up attempts: pair "learned stand up starting" events with
        # mode=stand transitions, in order; CSVs live in stand_fail_20260811
        stand_events = [e["t_unix"] for e in summary.get("events", [])
                        if e["text"] == "learned stand up starting"]
        stands = [tr for tr in summary.get("transitions", [])
                  if tr.get("mode") == "stand" and (tr.get("result") or {}).get("log")]
        if len(stand_events) == len(stands):
            for t_ev, tr in zip(stand_events, stands):
                log = tr["result"]["log"]
                csv_path = os.path.join(STAND_DIR, log)
                if not os.path.exists(csv_path):
                    continue
                # trace t=0 lags the event by the safe-zero acquisition,
                # measured at ~2.4 s on the well-correlated episodes
                b.add_episode(video, cam0, fps, t_ev + 2.4, csv_path,
                              f"{session}/stand_{log.split('_')[-1].split('.')[0]}",
                              max_shift=1.5, min_corr=0.35)

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "state_dataset.npz")
    np.savez_compressed(
        out,
        images=np.stack(b.images),
        labels=np.stack(b.labels),
        bbox=np.stack(b.bboxes),
        walk_id=np.array(b.walk_ids, dtype=np.int16),
        walk_names=np.array(b.walk_names),
        offsets=np.array(b.offsets, dtype=np.float32),
        label_names=np.array(LABEL_NAMES),
    )
    print(f"\nwrote {out}: {len(b.images)} frames from {len(b.walk_names)} episodes")


if __name__ == "__main__":
    main()

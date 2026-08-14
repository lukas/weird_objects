"""Run the trained state CNN over one bench walk and compare to the trace.

Produces a timeline plot (predicted vs encoder/IMU truth) for body roll,
pitch, and the three joints of one leg, plus overall MAE numbers.

Usage:
  python infer_state.py [--walk 193306/vref1-r1] [--leg 0]
      [--out /tmp/state_timeline.png]
"""

import argparse
import os

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

from train_state import GROUPS, StateCNN, group_mae

DATA = os.path.join(os.path.dirname(__file__), "data", "state_dataset.npz")
CKPT = os.path.join(os.path.dirname(__file__), "models", "state_cnn.pt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--walk", default="193306/vref1-r1",
                    help="substring of the walk name to analyze")
    ap.add_argument("--leg", type=int, default=0)
    ap.add_argument("--out", default="/tmp/state_timeline.png")
    args = ap.parse_args()

    d = np.load(DATA, allow_pickle=True)
    walk_names = [str(w) for w in d["walk_names"]]
    match = [i for i, w in enumerate(walk_names) if args.walk in w]
    if not match:
        raise SystemExit(f"no walk matches {args.walk!r}; have {walk_names}")
    wid = match[0]
    sel = d["walk_id"] == wid
    images, labels = d["images"][sel], d["labels"][sel]
    bbox = d["bbox"][sel].astype(np.float32)
    print(f"walk {walk_names[wid]}: {len(images)} frames")

    ck = torch.load(CKPT, map_location="cpu", weights_only=False)
    mu, sd, inp = ck["mu"], ck["sd"], ck["input"]
    model = StateCNN()
    model.load_state_dict(ck["state_dict"])
    model.eval()

    c = (images.shape[1] - inp) // 2
    x = images[:, c:c + inp, c:c + inp].astype(np.float32)
    x = (x / 255.0 - 0.5) / 0.25
    with torch.no_grad():
        pred = model(torch.from_numpy(x.transpose(0, 3, 1, 2)),
                     torch.from_numpy(bbox)).numpy()
    pred = pred * sd + mu

    print("MAE deg:", {k: round(v, 2) for k, v in group_mae(pred, labels).items()})

    t = np.arange(len(labels)) / 15.0
    leg = args.leg
    panels = [
        ("body roll (deg)", 0), ("body pitch (deg)", 1),
        (f"leg{leg} coxa (deg)", 2 + 3 * leg),
        (f"leg{leg} femur (deg)", 3 + 3 * leg),
        (f"leg{leg} knee (deg)", 4 + 3 * leg),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(10, 12), sharex=True)
    for ax, (title, j) in zip(axes, panels):
        ax.plot(t, labels[:, j], label="encoder/IMU truth", lw=1.5)
        ax.plot(t, pred[:, j], label="CNN from video", lw=1.2, alpha=0.85)
        ax.set_ylabel(title)
        ax.grid(alpha=0.3)
    axes[0].set_title(f"{walk_names[wid]} — vision state estimate vs logged truth")
    axes[0].legend(loc="upper right")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

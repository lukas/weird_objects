"""Train a small CNN: robot crop image -> body orientation + 18 joint angles.

Targets (degrees): [roll, pitch, q0..q17]. Labels come from the IMU and the
servo encoders via build_dataset.py, so this learns a vision-only estimate
of the state the robot believes it is in — useful as an independent check
when it tips over.

Split is by whole walk episodes (default: every walk from one session held
out) so val frames are never near-duplicates of train frames.

Usage:
  python train_state.py [--epochs 50] [--val-session 193306]
      [--synth data/synth_dataset.npz --synth-epochs 30]

With --synth, training runs two phases: pretrain on the MuJoCo-rendered
synthetic set (gen_synth.py), then fine-tune on the real frames at a lower
learning rate with a slice of synthetic data mixed in against forgetting.
"""

import argparse
import os

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

DATA = os.path.join(os.path.dirname(__file__), "data", "state_dataset.npz")
OUT = os.path.join(os.path.dirname(__file__), "models", "state_cnn.pt")
INPUT = 128  # 160 was tried for the thin tibia rods; no measurable gain

GROUPS = {
    "roll": [0],
    "pitch": [1],
    "coxa": [2 + 3 * i for i in range(6)],
    "femur": [3 + 3 * i for i in range(6)],
    "knee": [4 + 3 * i for i in range(6)],
}


class CropDataset(Dataset):
    def __init__(self, images, bboxes, labels, train):
        self.images, self.bboxes = images, bboxes
        self.labels, self.train = labels, train

    def __len__(self):
        return len(self.images)

    def __getitem__(self, i):
        img = self.images[i]
        m = img.shape[0] - INPUT
        if self.train:
            x0, y0 = np.random.randint(0, m + 1, 2)
            img = img[y0:y0 + INPUT, x0:x0 + INPUT].astype(np.float32)
            img *= np.random.uniform(0.7, 1.3)                    # brightness
            img += np.random.uniform(-20, 20)                     # offset
            img += np.random.uniform(-12, 12, size=(1, 1, 3))     # color cast
            img = np.clip(img, 0, 255)
        else:
            c = m // 2
            img = img[c:c + INPUT, c:c + INPUT].astype(np.float32)
        img = (img / 255.0 - 0.5) / 0.25
        return (torch.from_numpy(img.transpose(2, 0, 1)),
                torch.from_numpy(self.bboxes[i]),
                torch.from_numpy(self.labels[i]))


class StateCNN(nn.Module):
    """Crop image + normalized detector bbox -> 20 state values.

    The bbox (cx, cy, w, h in full-frame coords) is fed to the head because
    cropping removes where-in-the-frame information, which carries the
    perspective/tilt reference of the fixed bench camera.
    """

    def __init__(self, n_out=20, n_aux=4):
        super().__init__()
        chans = [3, 24, 48, 96, 160, 224]
        layers = []
        for cin, cout in zip(chans, chans[1:]):
            layers += [nn.Conv2d(cin, cout, 3, stride=2, padding=1),
                       nn.BatchNorm2d(cout), nn.ReLU(inplace=True)]
        self.features = nn.Sequential(*layers)
        self.pool = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten())
        self.head = nn.Sequential(
            nn.Linear(chans[-1] + n_aux, 128), nn.ReLU(inplace=True),
            nn.Linear(128, n_out),
        )

    def forward(self, x, aux):
        return self.head(torch.cat([self.pool(self.features(x)), aux], dim=1))


def group_mae(pred_deg, true_deg):
    err = np.abs(pred_deg - true_deg)
    return {g: float(err[:, idx].mean()) for g, idx in GROUPS.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=50)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--val-session", default="193306",
                    help="walks whose name contains this string form the val set")
    ap.add_argument("--synth", default=None,
                    help="synthetic npz from gen_synth.py to pretrain on")
    ap.add_argument("--synth-epochs", type=int, default=30)
    ap.add_argument("--mix-synth", action="store_true",
                    help="mix a synthetic slice into the fine-tune phase")
    args = ap.parse_args()

    d = np.load(DATA, allow_pickle=True)
    images, labels = d["images"], d["labels"]
    walk_names = [str(w) for w in d["walk_names"]]
    is_val_walk = np.array([args.val_session in w for w in walk_names])
    val_mask = is_val_walk[d["walk_id"]]
    print(f"train {int((~val_mask).sum())} frames / val {int(val_mask.sum())} frames "
          f"({int(is_val_walk.sum())} val walks: "
          f"{[w for w in walk_names if args.val_session in w]})")

    synth = np.load(args.synth, allow_pickle=True) if args.synth else None
    # stats over synth+real train labels: real-only stats (which up-weight
    # knees in the normalized loss) were tried and measured slightly worse
    train_labels = labels[~val_mask]
    stat_labels = (np.concatenate([train_labels, synth["labels"]])
                   if synth is not None else train_labels)
    mu = stat_labels.mean(0)
    sd = stat_labels.std(0) + 1e-6
    norm = lambda y: (y - mu) / sd

    bbox = d["bbox"].astype(np.float32)
    real_train = CropDataset(images[~val_mask], bbox[~val_mask],
                             norm(labels[~val_mask]), train=True)
    val_ds = CropDataset(images[val_mask], bbox[val_mask],
                         norm(labels[val_mask]), train=False)
    val_dl = DataLoader(val_ds, batch_size=args.batch)

    device = "mps" if torch.backends.mps.is_available() else "cpu"
    model = StateCNN().to(device)
    n_params = sum(p.numel() for p in model.parameters())
    print(f"model: {n_params/1e6:.2f}M params, device {device}")
    loss_fn = nn.SmoothL1Loss()

    base = group_mae(np.tile(mu, (int(val_mask.sum()), 1)), labels[val_mask])
    print("baseline (predict train mean):",
          {k: round(v, 2) for k, v in base.items()})

    state = {"best": np.inf}

    def evaluate_and_save(tag, ep, tl, n_train):
        model.eval()
        preds, trues = [], []
        with torch.no_grad():
            for x, aux, y in val_dl:
                preds.append(model(x.to(device), aux.to(device)).cpu().numpy())
                trues.append(y.numpy())
        pred_deg = np.concatenate(preds) * sd + mu
        true_deg = np.concatenate(trues) * sd + mu
        g = group_mae(pred_deg, true_deg)
        total = float(np.abs(pred_deg - true_deg).mean())
        marker = ""
        if total < state["best"]:
            state["best"] = total
            os.makedirs(os.path.dirname(OUT), exist_ok=True)
            torch.save({"state_dict": model.state_dict(), "mu": mu, "sd": sd,
                        "input": INPUT, "label_names": d["label_names"]}, OUT)
            marker = "  *saved*"
        print(f"{tag} ep {ep:3d} train {tl/n_train:.4f} | val MAE deg "
              + " ".join(f"{k} {v:.2f}" for k, v in g.items())
              + f" | all {total:.2f}{marker}")

    def run_phase(tag, ds, epochs, lr):
        dl = DataLoader(ds, batch_size=args.batch, shuffle=True, drop_last=True)
        opt = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-4)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, epochs)
        for ep in range(1, epochs + 1):
            model.train()
            tl = 0.0
            for x, aux, y in dl:
                x, aux, y = x.to(device), aux.to(device), y.to(device)
                opt.zero_grad()
                loss = loss_fn(model(x, aux), y)
                loss.backward()
                opt.step()
                tl += loss.item() * len(x)
            sched.step()
            evaluate_and_save(tag, ep, tl, len(ds))

    if synth is not None:
        synth_ds = CropDataset(synth["images"], synth["bbox"].astype(np.float32),
                               norm(synth["labels"]), train=True)
        run_phase("synth", synth_ds, args.synth_epochs, args.lr)
        if args.mix_synth:
            # fine-tune on real + a synthetic slice against forgetting
            rng = np.random.default_rng(0)
            keep = rng.choice(len(synth["images"]),
                              min(len(synth["images"]), len(real_train)),
                              replace=False)
            tune_ds = torch.utils.data.ConcatDataset([
                real_train,
                CropDataset(synth["images"][keep],
                            synth["bbox"][keep].astype(np.float32),
                            norm(synth["labels"][keep]), train=True)])
        else:
            tune_ds = real_train
        run_phase("tune", tune_ds, args.epochs, args.lr * 0.3)
    else:
        run_phase("real", real_train, args.epochs, args.lr)

    print(f"\nbest overall val MAE {state['best']:.2f} deg -> {OUT}")


if __name__ == "__main__":
    main()

"""probe_latents.py — linear probes from z to physical concepts (G3).

Operator next-steps 08-13: beyond future-state prediction, test whether
SIMPLE probes from the latent recover useful physical state — the goal
is to know whether larger/better dynrep models actually organize
trajectories into physical structure, not to deploy the probes.

Input: a latents npz dumped by eval_model.py --dump-latents (z +
per-window labels from the newest history frame). For each label a
closed-form ridge probe is fit on 80% of windows and scored on the
held-out 20%: R^2 for continuous targets, balanced accuracy for the
binary per-foot contacts. A shuffled-target control column guards
against trivially-high scores from label imbalance.

    python -m rl_move.dynamics.probe_latents \
        --latents rl_move/dynamics/logs/latents_dyn_v3_obs_<ts>.npz
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def ridge_fit_predict(Ztr, ytr, Zte, lam_rel: float = 1e-3):
    Z1 = np.concatenate([Ztr, np.ones((len(Ztr), 1))], axis=1)
    lam = lam_rel * len(Z1)
    A = Z1.T @ Z1 + lam * np.eye(Z1.shape[1])
    w = np.linalg.solve(A.astype(np.float64),
                        (Z1.T @ ytr).astype(np.float64))
    Zte1 = np.concatenate([Zte, np.ones((len(Zte), 1))], axis=1)
    return Zte1 @ w


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--latents", required=True)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    d = np.load(ROOT / args.latents)
    z, labels = d["z"], d["labels"]
    names = [str(n) for n in d["label_names"]]
    rng = np.random.default_rng(args.seed)
    idx = rng.permutation(len(z))
    n_tr = int(0.8 * len(z))
    tr, te = idx[:n_tr], idx[n_tr:]
    print(f"{len(z)} windows, z dim {z.shape[1]}; "
          f"train {len(tr)} / test {len(te)}")
    print(f"{'target':<12} {'kind':>6} {'score':>7} {'shuffled':>9}")
    for j, name in enumerate(names):
        y = labels[:, j:j + 1].astype(np.float64)
        pred = ridge_fit_predict(z[tr], y[tr], z[te])
        y_sh = y[rng.permutation(len(y))]
        pred_sh = ridge_fit_predict(z[tr], y_sh[tr], z[te])
        if name.startswith("contact_"):
            yt = y[te, 0] > 0.5
            score = 0.5 * (np.mean(pred[yt, 0] > 0.5)
                           + np.mean(pred[~yt, 0] <= 0.5)) \
                if 0 < yt.sum() < len(yt) else float("nan")
            yts = y_sh[te, 0] > 0.5
            sh = 0.5 * (np.mean(pred_sh[yts, 0] > 0.5)
                        + np.mean(pred_sh[~yts, 0] <= 0.5)) \
                if 0 < yts.sum() < len(yts) else float("nan")
            kind = "bacc"
        else:
            ss_res = float(np.sum((pred[:, 0] - y[te, 0]) ** 2))
            ss_tot = float(np.sum((y[te, 0] - y[te, 0].mean()) ** 2))
            score = 1.0 - ss_res / max(ss_tot, 1e-12)
            ss_res = float(np.sum((pred_sh[:, 0] - y_sh[te, 0]) ** 2))
            ss_tot = float(np.sum((y_sh[te, 0]
                                   - y_sh[te, 0].mean()) ** 2))
            sh = 1.0 - ss_res / max(ss_tot, 1e-12)
            kind = "R2"
        print(f"{name:<12} {kind:>6} {score:>7.3f} {sh:>9.3f}")
    print("\nread: linear ridge probe from z on held-out windows; the "
          "shuffled column is the chance floor. High roll/pitch/gyro "
          "R^2 + per-foot contact bacc >> shuffled = the latent "
          "organizes attitude, angular rate and support state (G3).")


if __name__ == "__main__":
    main()

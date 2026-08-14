"""Overlay plots: q_commanded(t) / q_hardware(t) / q_sim(t) per segment."""
from __future__ import annotations

from pathlib import Path

import numpy as np


def overlay_trace(tr: dict, sims: dict[str, np.ndarray], out: Path,
                  *, max_segs: int = 24) -> Path:
    """One panel per segment: cmd (grey), hardware q (black), sim q
    (colored per param set). ``sims`` maps tag -> (n,18) sim stream
    aligned with the trace rows; empty dict plots the trace alone."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from .trace import segments
    segs = segments(tr)[:max_segs]
    if not segs:
        raise ValueError("trace has no segments")
    ncol = min(4, len(segs))
    nrow = (len(segs) + ncol - 1) // ncol
    fig, axs = plt.subplots(nrow, ncol, figsize=(4.2 * ncol, 2.8 * nrow),
                            squeeze=False)
    colors = plt.cm.tab10.colors
    for i, seg in enumerate(segs):
        ax = axs[i // ncol][i % ncol]
        sl, j = seg["sl"], seg["joint"]
        t = tr["t"][sl] - tr["t"][sl][0]
        if j >= 0:
            ax.plot(t, tr["cmd"][sl, j], color="0.6", lw=1.0, label="cmd")
            ax.plot(t, tr["q"][sl, j], "k-", lw=1.6, label="hw")
            for ci, (tag, q_sim) in enumerate(sims.items()):
                ax.plot(t, q_sim[sl, j], lw=1.2, color=colors[ci % 10],
                        ls="--", label=f"sim {tag}")
        else:  # traj: show the 4 most-travelled joints
            top = np.argsort(np.ptp(tr["cmd"][sl], axis=0))[::-1][:4]
            for ci, jj in enumerate(top):
                c = colors[ci % 10]
                ax.plot(t, tr["cmd"][sl, jj], color=c, lw=0.8, alpha=0.4)
                ax.plot(t, tr["q"][sl, jj], color=c, lw=1.5,
                        label=f"q{jj}")
                for tag, q_sim in sims.items():
                    ax.plot(t, q_sim[sl, jj], color=c, lw=1.0, ls="--")
        ax.set_title(seg["label"], fontsize=8)
        ax.grid(alpha=0.3)
        if i == 0:
            ax.legend(fontsize=7)
    for i in range(len(segs), nrow * ncol):
        axs[i // ncol][i % ncol].axis("off")
    fig.suptitle(tr["name"], fontsize=10)
    fig.tight_layout()
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print(f"  wrote {out}")
    return out

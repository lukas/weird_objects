# cw-walk-diag45

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T12:22:36+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: 5hvlqt8d

**hypothesis**: OPERATOR WISHLIST + rulings s2/s5 command-steering promotion scope: forward-diagonal competence. Champion retrained with heading cap pi/4 (100% of episodes fwd-diagonal vs legacy 60% pure-fwd mix). If-true: det tracking of +-45deg headings matches pure-forward within noise; if-false: diagonal commands collapse to forward drift (steering needs dedicated curriculum).

**gate**: DR0 det 6/6 at |heading|<=45deg: gait_valid, zero terminations, vel tracking err within 2x pure-forward baseline

**verdict**: No verdict - killed by operator at 11.2M/20M (08-09): near-duplicate of cw-steer-fdiag, whose eval already refuted the class (diag tracking within noise, sto worse); class stopped, GPU freed.


# cw-walk-steer-explore

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:21:27+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: 32iq9xjm

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST exploratory (non-promotion, rulings allow wider hemisphere as exploratory arm): full-heading steering. Champion retrained with heading uniform in +-pi. If-true: usable omnidirectional base emerges; if-false: rear-hemisphere commands destroy forward gait (confirms deferral).

**gate**: exploratory: DR0 det per-quadrant tracking report + gait_valid forward retention 6/6 (non-promotion)

**verdict**: FAIL (exploratory gate, non-promotion) / no omnidirectional base emerged; forward gait NOT destroyed either. Fwd-only DR0 eval: det 6/6 gait_valid, 0 term, fwd dist 0.70m vs champion 0.73m, det slip/m 1.36 (vs champ ~1.15, near noise). Mixed-heading (own cfg, +-pi) eval bimodal: forward-ish draws track (prog 0.79-0.99) but off-axis/rear draws do not transport — prog 0.22-0.37, slip/m 4.6-7.2, body keeps creeping forward instead of along the command (e.g. det0: fwd 0.35m vs along-cmd 0.23m). Paddle gait cannot push off-axis — same contact-pricing root as skating/speed. Confirms rear-hemisphere deferral (ruling 4) + c39 steering stop; NO requeue.


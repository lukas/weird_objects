# cw-walk-speedband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T13:14:30+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: 240yq995

**hypothesis**: OPERATOR WISHLIST 8b: operator-tunable speed. One policy tracking the FULL 0.02-0.12 m/s band with mid-episode command resampling, so speed becomes a runtime knob instead of a retrain. If-true: tracking error flat across the band with gait_valid; if-false: the band is too wide for one head (fast end shuffles or slow end parks) and we bucket it.

**gate**: DR0 det+sto 6/6 at commanded 0.03/0.07/0.11 m/s: gait_valid, zero terminations, per-speed tracking err within 25% of champion at 0.05


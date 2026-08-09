# cw-walk-speedband

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: KILLED

**created**: 2026-08-09T13:14:30+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: 240yq995

**hypothesis**: OPERATOR WISHLIST 8b: operator-tunable speed. One policy tracking the FULL 0.02-0.12 m/s band with mid-episode command resampling, so speed becomes a runtime knob instead of a retrain. If-true: tracking error flat across the band with gait_valid; if-false: the band is too wide for one head (fast end shuffles or slow end parks) and we bucket it.

**gate**: DR0 det+sto 6/6 at commanded 0.03/0.07/0.11 m/s: gait_valid, zero terminations, per-speed tracking err within 25% of champion at 0.05

**verdict**: No verdict - killed at 6M/20M by cycle 43: stale backlog spec launched by the drain AFTER cycle 42 cw-walk-fast refuted its premise (gait ceiling ~0.065 m/s makes the 0.02-0.12 band upper half unreachable and the 0.11 gate leg fail by construction; half the command distribution would confound the achievable-band question). Revised 0.02-0.06 band requeued as cw-walk-speedband-r1.


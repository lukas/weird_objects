# cw-walk-backforth

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:51:53+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: xi5bkjlt

**hypothesis**: OPERATOR WISHLIST 4: back and forth. Full-circle heading commands (+-180deg) with resampling and 15% stops — walk out, stop, walk BACK. Backward walking is the new behavior; the champion never saw heading beyond +-45deg. If-true: reverse tracking with gait_valid; if-false: backward gait fails (leg geometry asymmetry) and backward needs its own reward shaping.

**gate**: DR0 det+sto 6/6 incl reverse commands: gait_valid, zero terminations, reverse tracking err <= 2x forward err


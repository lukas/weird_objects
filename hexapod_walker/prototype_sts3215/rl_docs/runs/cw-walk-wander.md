# cw-walk-wander

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-09T12:32:26+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: tz52k9yt

**hypothesis**: OPERATOR WISHLIST: walk around and CHANGE DIRECTION. Command resampled every 5 s within +-45deg incl. 15% full stops (goal.walk_cmd_resample_s=5) — the policy must learn start/steer/stop transitions, which the frozen-command champion never saw. If-true: tracking through command changes with gait_valid; if-false: transitions break the gait (stumbles/park at each change).

**gate**: DR0 det+sto 6/6 with resampled commands: gait_valid, zero terminations, post-change tracking recovers within 2 s


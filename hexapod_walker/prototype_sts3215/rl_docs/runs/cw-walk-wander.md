# cw-walk-wander

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-09T12:32:26+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-anchorgate

**wandb_id**: tz52k9yt

**hardware_ready**: no

**hypothesis**: OPERATOR WISHLIST: walk around and CHANGE DIRECTION. Command resampled every 5 s within +-45deg incl. 15% full stops (goal.walk_cmd_resample_s=5) — the policy must learn start/steer/stop transitions, which the frozen-command champion never saw. If-true: tracking through command changes with gait_valid; if-false: transitions break the gait (stumbles/park at each change).

**gate**: DR0 det+sto 6/6 with resampled commands: gait_valid, zero terminations, post-change tracking recovers within 2 s

**verdict**: PASS — mid-episode command changes (5s resample, ±45°, 15% stops): DR0 det+sto gv 12/12, 0 term, prog_ratio ~1.0 in every ep incl. multi-change ones (recovery well within the 2s gate or per-ep prog would lag). Cost: direction-change eps run slip/m ~1.8-2.1 vs ~1.1 straight — the paddle gait shuffles through re-orientation. First policy with start/steer/stop transitions.


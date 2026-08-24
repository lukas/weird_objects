# cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace-hi

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T02:29:02+00:00

**pod**: hexapod-mjx-train-3

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr7

**wandb_id**: gscw2nh1

**hypothesis**: Dose sibling of cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace (grace_s=0.4): tests a LONGER grace window, reward.walk_stop_grace_s=0.8, on the exact same single-lever recipe/parent. Read the 3-point bracket (0.2/0.4/0.8) jointly against joyfullcurr7 (0.0, the FAIL baseline): the over_current fall count and the b1 stop-cert speed should move monotonically with grace length if the transient-relief mechanism is real; a non-monotonic or flat-at-every-dose result means grace length isn't the operative variable.

**gate**: Same as joyfullcurr8-stopgrace: walkcurr V6 b1 cert clears its stop check and frontier promotes past side90_60s; joygate falls <= joyfullcurr6's 8/48 with over_current no longer dominant; DR0+ownDR walk gates stay >=5/6 gait_valid; video all six feet cycling


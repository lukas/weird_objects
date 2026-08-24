# cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace-lo

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-24T02:27:12+00:00

**pod**: hexapod-mjx-train-1

**steps**: 40000000

**parent**: cw-arch-hist16-dep1-c1-joyfullcurr7

**wandb_id**: 9ftrzr8y

**hypothesis**: Dose sibling of cw-arch-hist16-dep1-c1-joyfullcurr8-stopgrace (grace_s=0.4): tests a SHORTER grace window, reward.walk_stop_grace_s=0.2, on the exact same single-lever recipe/parent. Read the pair jointly: if 0.2 already clears over_current and gets closer to the b1 cert bar, a short grace is enough (transient is brief); if 0.2 still shows over_current-dominated falls while 0.4 doesn't, the transient needs more settle time; if BOTH still show over_current, the transient-relief hypothesis itself needs revisiting (a torque/current-rate charge instead of stop-speed pricing) regardless of grace length.

**gate**: Same as joyfullcurr8-stopgrace: walkcurr V6 b1 cert clears its stop check and frontier promotes past side90_60s; joygate falls <= joyfullcurr6's 8/48 with over_current no longer dominant; DR0+ownDR walk gates stay >=5/6 gait_valid; video all six feet cycling


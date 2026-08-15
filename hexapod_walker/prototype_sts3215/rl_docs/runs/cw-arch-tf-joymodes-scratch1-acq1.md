# cw-arch-tf-joymodes-scratch1-acq1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-15T18:30:34+00:00

**pod**: hexapod-mjx-train-3

**steps**: 38000000

**parent**: cw-arch-tf-joymodes-scratch1

**wandb_id**: 7sualbv3

**hypothesis**: Keep training the same random-initialized joystick Transformer lineage for its honest 40M total learning budget. The 2M checkpoint proved the machinery works but was only 5% of the budget this architecture previously needed; if the direction reward and command curriculum work, the immature stilt posture should give way to a stable six-leg gait as command-aligned speed and mode-switch tracking continue improving through acquisition.

**gate**: Do not issue a final behavior or reward verdict before this 38M continuation completes unless training crashes, produces NaNs, or a separately measured task metric irreversibly diverges. At 40M total: PASS = six-leg gait with no sacrificed legs or falls, progress ratio >=0.85 and wrong-way fraction <=0.15 across random holds, 180 flips, square turns, continuous circles, stop-go, and jitter; video must show physical direction changes rather than stilt/dragging. FAIL at full budget = those task/video gates remain clearly missed despite healthy optimization; report checkpoint trajectory before attributing cause.


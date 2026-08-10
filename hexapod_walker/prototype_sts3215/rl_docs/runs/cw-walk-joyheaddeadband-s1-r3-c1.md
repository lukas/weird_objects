# cw-walk-joyheaddeadband-s1-r3-c1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:21:05+00:00

**pod**: hexapod-mjx-train-3

**steps**: 16000000

**parent**: cw-walk-joyheaddeadband-s1-r3

**hypothesis**: REBALANCE continuation (not a new variable): cw-walk-joyheaddeadband-s1-r3 was killed at ~3.8M/20M because node g142d86 was host-starved (loadavg ~245/128, fps declined 8961->5000, neighbor groundtilt8-s1-r1 on same node down to 2465). Same hypothesis as parent: ruling-7 seed-1 deadband twin, unchanged spec. If-true: same gate as parent. If-false: flag for dig-in per c67 precedent (isolated-spec failure, not fleet storm).

**gate**: own-cfg DR0 6+6 harness gv>=10/12, no sacrificed legs, det med fwd within champion band; JOYSTICK-style deadband/friction DR retention clean vs parent lineage

**failed_reason**: run never appeared as 'running' in W&B within 240s


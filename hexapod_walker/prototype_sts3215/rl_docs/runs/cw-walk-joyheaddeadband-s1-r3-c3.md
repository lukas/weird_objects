# cw-walk-joyheaddeadband-s1-r3-c3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:41:08+00:00

**pod**: hexapod-mjx-train-7

**steps**: 16000000

**parent**: cw-walk-joyheaddeadband-s1-r3

**hypothesis**: REBALANCE continuation, 3rd attempt (c1 + c2 both lost launch-collision EOFError races during a heavy concurrent-launch window, gotcha 13b, 0 steps each -- no science result either time). Root cause unchanged: cw-walk-joyheaddeadband-s1-r3 was killed at ~3.8M/20M because node g142d86 was host-starved (loadavg ~245/128, fps declined 8961->5000). Same hypothesis as parent: ruling-7 seed-1 deadband twin, unchanged spec. If-true: same gate as parent. If-false: flag for dig-in per c67 precedent (isolated-spec failure, not fleet storm).

**gate**: own-cfg DR0 6+6 harness gv>=10/12, no sacrificed legs, det med fwd within champion band; JOYSTICK-style deadband/friction DR retention clean vs parent lineage

**failed_reason**: 4th consecutive EOFError (c1/c2/c3 on 3 different pods); the drain's own requeue-and-rename self-repair then auto-retried this exact spec a 5th time as cw-walk-joyheaddeadband-s1-r3-c3-rr1 on train-9, which landed cleanly (RUNNING, fps~14.5k) -- so this was collision noise after all (4 unlucky pod assignments in a heavy-launch window), not a checkpoint/cfg defect. Downgrading my earlier dig-in suspicion for this spec; no further action needed, -rr1 is the live continuation.


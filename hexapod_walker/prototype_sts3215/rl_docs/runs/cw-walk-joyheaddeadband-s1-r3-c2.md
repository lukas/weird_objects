# cw-walk-joyheaddeadband-s1-r3-c2

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T02:33:46+00:00

**pod**: hexapod-mjx-train-11

**steps**: 16000000

**parent**: cw-walk-joyheaddeadband-s1-r3

**hardware_ready**: False

**hypothesis**: REBALANCE continuation, 2nd attempt (c1 lost a launch-collision EOFError race with a concurrent cycle's simultaneous launch on the same node g131eec, gotcha 13b, 0 steps, no science). Root cause unchanged: cw-walk-joyheaddeadband-s1-r3 was killed at ~3.8M/20M because node g142d86 was host-starved (loadavg ~245/128, fps declined 8961->5000). Same hypothesis as parent: ruling-7 seed-1 deadband twin, unchanged spec. If-true: same gate as parent. If-false: flag for dig-in per c67 precedent.

**gate**: own-cfg DR0 6+6 harness gv>=10/12, no sacrificed legs, det med fwd within champion band; JOYSTICK-style deadband/friction DR retention clean vs parent lineage

**verdict**: INFRA, not science: died at 0 steps to the same launch-collision EOFError (gotcha 13b, mjx_sharded_vec_env.py reset_finalize) that killed its sibling -c1. This is now 3 consecutive infra deaths for the joyheaddeadband-s1 lineage's continuation (r3 host-starved kill, c1 collision, c2 collision) -- matches the c67/c71-flagged isolated-spec pattern, not generic storm (other launches around it succeeded). Leaving further mechanical retries to the drain; the underlying deadband-seed1 hypothesis is still unverified.

**failed_reason**: run never appeared as 'running' in W&B within 240s


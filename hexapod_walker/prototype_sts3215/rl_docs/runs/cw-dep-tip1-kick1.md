# cw-dep-tip1-kick1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-12T00:34:26+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tip1

**hypothesis**: Hardware takeoff falls are a DYNAMIC roll-rate problem, not a static-lean problem (takeoff25-r1 closed the dose lever: sim already recovers static 20-25deg starts). If tip1 trains with the new dr.walk_kick_* transient fold pulse (half-sine over the first 0.5-1.2s of gait, 10-22deg peak = the measured 11-46deg/s roll-rate regime from bench_report over 18 hardware walks), it will learn active takeoff leveling that static tipped-start DR never taught.

**gate**: eval walk panel retained (dist ratio >=0.9 of tip1 baseline, zero new falls at DR0) AND under forced walk_kick injection (prob 1, 14-22deg) terminal-fall rate < tip1 baseline under the same injection by >=2x; kicked-episode tail |roll| median < 4deg.


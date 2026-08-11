# cw-dep-tall-kh3

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-11T22:03:48+00:00

**pod**: hexapod-mjx-train-0

**steps**: 2000000

**parent**: cw-dep-tall30

**hypothesis**: TALL LADDER T2a: k_height crank 3x (100->300) at ref -15, warm from tall30. The base quadratic height charge is ~0.36/tick at 60mm vs ~3/tick walk income, so the crouch outbids it; T1 proved the bare ref is tradeable. Does a 3x charge change the trade?

**gate**: PASS: height_err_end <=8mm at -15 ref, speed >=0.028, survived 1, slip <=1.8, no park. FAIL: err >=25mm or walk broken. Compare against T2b (10x) for dose-response.

**refused_reason**: hexapod-mjx-train-0 already runs cw-dep-tall-kh3 — GPU pods host exactly one run; pick a free GPU pod.


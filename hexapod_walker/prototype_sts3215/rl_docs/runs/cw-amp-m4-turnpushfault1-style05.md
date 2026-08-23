# cw-amp-m4-turnpushfault1-style05

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-23T02:23:29+00:00

**pod**: hexapod-mjx-train-4

**steps**: 2000000

**parent**: cw-amp-m3-turnpush1-style05-acq1-r2

**hypothesis**: Plain English: turn+push just closed clean at acquisition budget (8M, PASS: prog 1.11, topples 0/6+2/6, tip errs 0.14/0.12) after its own 2M discovery read looked broken (prog 0.37) -- exactly the same undertrained pattern push-alone and fault-alone each showed before their own 3x acquisition step. Does grafting fault onto this SOLID two-axis substrate (rather than turnfault1-style05's fresh 3-way stack, which missed its safety bar at 2M: gait_valid 9/12 vs 10/12) let the mechanism-safety bar clear at first discovery, since two of the three axes no longer need simultaneous discovery? Single lever vs turnfault1-style05: --init-from swapped from the turn-only checkpoint to the turn+push checkpoint; everything else (fault wiring, amp weights, yaw/heading cfg, budget) identical.

**gate**: Discovery (2M, DR-0, own cfg: fault+push+yaw all active). Mechanism-safety bar (M4 new-mechanism discipline, matching turnfault1-style05's own bar): gait_valid >=10/12 det+sto, faulted episodes limp not statue on video, no crouch, zero NaN/crash. If it clears where the fresh-stack turnfault1-style05 missed (9/12): the sequential route is confirmed as the fix for 3-way composition, extend to acquisition budget next. If it ALSO misses the bar: 3-way composition is hard regardless of substrate solidity, budget (not order) is the lever -- acquisition-continue this arm instead of respending on ordering. Compare det/sto prog+slip against turnpush1-style05-acq1-r2's own numbers (1.11/1.20 prog, no sacrificed) and run eval_yaw manually to check turn tracking survived triple-stacking.


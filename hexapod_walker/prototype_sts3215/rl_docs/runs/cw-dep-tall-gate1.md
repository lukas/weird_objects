# cw-dep-tall-gate1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-11T21:31:07+00:00

**pod**: hexapod-mjx-train-2

**steps**: 2000000

**parent**: cw-dep-tall30

**hypothesis**: TALL LADDER T3: height gate AT A REACHABLE REF. T1 (cw-dep-tall15-h1, 6M) proved the height ref alone is TRADEABLE: given budget the policy reverted to a -67mm crouch to buy speed 0.051 m/s (fastest dep walker ever, in the command band) - walk income dominates and posture is the currency. Fix: make height part of the income. cw-dep-hgt1s gate failed at ref 0 = an unreachable 50mm one-shot (factor collapsed to 0.24); here the gate (sigma 30mm) is applied at ref -30 warm from cw-dep-tall30 which ALREADY tracks that ref at 15mm err = gate factor 0.88 at start, smooth gradient up, nothing to give up. Charge stack retained from parent.

**gate**: PASS: height_err_end <=8mm at -30 ref AND speed >=0.030 AND survived 1, slip <=1.8, no park -> then rung the REF upward (-15, 0) with the gate riding along, and check whether T1s speed discovery survives the gate. FAIL if factor collapses (hgt1 mode) or speed <0.025: gate+ref cannot coexist with walk income at this sigma - try sigma 45 once, else the wall answer comes from T5s probe.


# cw-dep-vref1-r1-badstartjoints

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T19:57:53+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly if MORE joints are simultaneously mis-set at session start, separate from tests of how OFTEN a bad start happens (cw-dep-vref1-r1-badstart) or how BIG one joint's miss can be (cw-dep-vref1-r1-badstartdeg, PASSed this cycle)? This is the third and last dimension of the bad-start mechanism -- breadth. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- breadth alone is absorbed like frequency/magnitude. If-false: more simultaneously-wrong joints break gait_valid or inflate slip -- breadth, not just frequency/magnitude, is a real pre-attempt-#2 risk.

**gate**: own-cfg (DR0.35 + dr.bad_start_max_joints=6, doubled from nominal 3) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; known fixed-draw crater (det/4) pre-allowed as baseline


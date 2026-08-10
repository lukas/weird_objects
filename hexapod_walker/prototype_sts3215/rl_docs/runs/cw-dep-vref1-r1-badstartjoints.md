# cw-dep-vref1-r1-badstartjoints

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T19:57:53+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: a88hlzbg

**hardware_ready**: False

**hypothesis**: Plain English: does the hardware checkpoint still walk cleanly if MORE joints are simultaneously mis-set at session start, separate from tests of how OFTEN a bad start happens (cw-dep-vref1-r1-badstart) or how BIG one joint's miss can be (cw-dep-vref1-r1-badstartdeg, PASSed this cycle)? This is the third and last dimension of the bad-start mechanism -- breadth. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv (or 5/6 allowing the known crater), 0 term, slip/m within vref1-r1's own band -- breadth alone is absorbed like frequency/magnitude. If-false: more simultaneously-wrong joints break gait_valid or inflate slip -- breadth, not just frequency/magnitude, is a real pre-attempt-#2 risk.

**gate**: own-cfg (DR0.35 + dr.bad_start_max_joints=6, doubled from nominal 3) det+sto @15s: gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; known fixed-draw crater (det/4) pre-allowed as baseline

**verdict**: PASS -- breadth of simultaneously-bad-start joints (6, doubled from nominal 3) composes free onto the hardware candidate, same fingerprint as every other floor/DR axis tonight. DR0-gate: gait_valid 6/6 det+sto, 0 term, slip med 0.97 det/0.94 sto (in vref1-r1's own band); only the known fixed-draw det/4 crater fails (pre-allowed baseline). Own-cfg DR0.35: gv 6/6 both modes, 0 term, slip med 1.02 det/1.19 sto (in band); det/5 + sto/0-1 form the already-root-caused fixed-seed DR0.35 crater cluster (sac=[], gv=True, no flag-leg/fall on video), not new. Not independently hardware-ready; confirms bad-start breadth is absorbed like frequency/magnitude already were.


# cw-walk-groundtilt8-s1-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T02:07:52+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-groundtilt8-s1

**wandb_id**: 9yt8ia2y

**hardware_ready**: no

**hypothesis**: Retry of cw-walk-groundtilt8-s1 (died at init, gotcha 13b, W&B global_step 0->0, no science result). Same spec unchanged: seed twin of cw-walk-groundtilt8-r3 (this cycle's concurrent triage) testing whether the 8deg-tilt crater pattern (3/6 det draws) reproduces at similar severity or was seed luck.

**gate**: own-cfg tilt u(0,8deg) det+sto gv 6/6, 0 term; DR0 retention gv 6/6; frames watched det -- crater-fraction comparison against r3's 3/6 is informational, not gating (severity band, not pass/fail).

**verdict**: Seed-1 twin of cw-walk-groundtilt8-r3 confirms the crater pattern is a real recipe trait, not seed luck: own-cfg det gait_valid 6/6, 0 term, but the SAME 3/6 fraction (det eps 3-5) craters into a high-slip shuffle (slip/m up to 3.26, prog down to 0.48) vs the clean half (slip~1.1-1.4, prog~0.85-1.04) -- identical severity band to r3's own 3/6. No falls, no flag leg, no sacrificed leg in any of the 12 det+sto episodes; flat/no-tilt retention is clean (gv 6/6, slip 1.16, prog 0.95). Meets its own gate (informational crater-fraction comparison only, not pass/fail).


# cw-dep-vref1-r1-contactstiff

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T17:07:43+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**wandb_id**: orru7zxb

**hardware_ready**: True

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: ground/foot contact COMPLIANCE (table vs carpet vs the operator's floor) has never been varied on vref1-r1 -- friction (fric, PASSED) and floor slope (groundtilt5, PASSED) are tested but contact stiffness (solref timeconst, a genuinely different physical axis: how squishy the foot-ground contact is, not how slippery or how tilted) is not. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- compliance variation composes free like friction/slope did. If-false: soft/stiff contact changes the effective loaded settling the fixed-gain controller relies on, breaking gait timing -- a real pre-attempt-#2 surface risk (the operator's floor is unknown compliance).

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**verdict**: PASS -- ground/foot contact compliance (0.7-2.0x stiffness), a new physical axis never before varied on this checkpoint, does not erode the hardware candidate. Own-cfg det+sto 6/6 gait_valid, 0 term; slip/m med det 1.16 sto 1.01, comfortably inside the pre-registered vref1-r1 band (0.89-1.36). Known idx4 fixed-seed stall reproduces (prog 0.07 slip 23.8, clean halt, gait_valid True) same as every other lineage child -- not new. Contact sheet + frame strips show the standard six-leg creep gait, no timing/settling breakdown from softer or stiffer contact.


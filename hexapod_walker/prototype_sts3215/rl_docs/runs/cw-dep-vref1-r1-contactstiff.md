# cw-dep-vref1-r1-contactstiff

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: REFUSED

**created**: 2026-08-10T17:04:31+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-dep-vref1-r1

**hypothesis**: PROTECT THE NAMED HARDWARE CANDIDATE: ground/foot contact COMPLIANCE (table vs carpet vs the operator's floor) has never been varied on vref1-r1 -- friction (fric, PASSED) and floor slope (groundtilt5, PASSED) are tested but contact stiffness (solref timeconst, a genuinely different physical axis: how squishy the foot-ground contact is, not how slippery or how tilted) is not. Per P0 rule 3, k_current=0. If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- compliance variation composes free like friction/slope did. If-false: soft/stiff contact changes the effective loaded settling the fixed-gain controller relies on, breaking gait timing -- a real pre-attempt-#2 surface risk (the operator's floor is unknown compliance).

**gate**: own-cfg det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band (0.89-1.36); frames watched det for flag-leg/skate

**refused_reason**: hexapod-mjx-train-7 code marker e29e76d811a7ffe01ee76eabc404e178553b2c78 != local HEAD ecf653ce395e9e22b0e36cbaaeefe0279a9f4eda. Sync first: snapshot.sh --sync hexapod-mjx-train-7 (and snapshot/commit before that if the tree is dirty).


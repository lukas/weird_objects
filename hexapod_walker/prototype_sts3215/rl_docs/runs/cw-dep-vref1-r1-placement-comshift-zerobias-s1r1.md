# cw-dep-vref1-r1-placement-comshift-zerobias-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T20:01:04+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1-placement-comshift-zerobias

**hypothesis**: Confirm tonight's three-way assembly-tolerance PASS is a property of the recipe, not seed luck, before the operator trusts it around hardware. Seed twin (11->12) of the just-PASSed stack: joint placement slop (6deg) + off-center CoM (0.03m) + hand-zeroed servo horns (3deg) all at once on the hardware-candidate contract, k_current=0 per P0. Same precedent as fric-s1/encnoise-latency-s1. (r1 suffix = relaunch; first -s1 attempt was REFUSED for a taken pod and its git tag blocks name reuse — no training occurred.) If-true: own-cfg det+sto 12/12 gait_valid, 0 term, slip within vref1-r1's own band — the assembly-QA armor is seed-robust. If-false (a seed twin erodes): the 3-axis compose PASS was seed luck and the stack gets re-flagged as an assembly-QA risk. Strongest alternative: known fixed-draw crater episodes vary in stall posture between seeds (expected, not a fail — see the source run's dig-in verdict).

**gate**: own-cfg (DR0.35 + placement6 + com0.03 + zerobias3) det+sto 6/6 each @15s gait_valid 12/12, 0 term, slip/m within vref1-r1 own band (±20%); DR0 retention matching the lineage fingerprint (known idx4 crater tolerated, stall-posture variation tolerated; any sacrificed leg while TRANSPORTING = FAIL); frames watched det


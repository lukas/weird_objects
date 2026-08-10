# cw-dep-vref1-r1-placement-comshift-zerobias-s1r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T20:01:04+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-dep-vref1-r1-placement-comshift-zerobias

**wandb_id**: tuhtxis5

**hardware_ready**: False

**hypothesis**: Confirm tonight's three-way assembly-tolerance PASS is a property of the recipe, not seed luck, before the operator trusts it around hardware. Seed twin (11->12) of the just-PASSed stack: joint placement slop (6deg) + off-center CoM (0.03m) + hand-zeroed servo horns (3deg) all at once on the hardware-candidate contract, k_current=0 per P0. Same precedent as fric-s1/encnoise-latency-s1. (r1 suffix = relaunch; first -s1 attempt was REFUSED for a taken pod and its git tag blocks name reuse — no training occurred.) If-true: own-cfg det+sto 12/12 gait_valid, 0 term, slip within vref1-r1's own band — the assembly-QA armor is seed-robust. If-false (a seed twin erodes): the 3-axis compose PASS was seed luck and the stack gets re-flagged as an assembly-QA risk. Strongest alternative: known fixed-draw crater episodes vary in stall posture between seeds (expected, not a fail — see the source run's dig-in verdict).

**gate**: own-cfg (DR0.35 + placement6 + com0.03 + zerobias3) det+sto 6/6 each @15s gait_valid 12/12, 0 term, slip/m within vref1-r1 own band (±20%); DR0 retention matching the lineage fingerprint (known idx4 crater tolerated, stall-posture variation tolerated; any sacrificed leg while TRANSPORTING = FAIL); frames watched det

**verdict**: PASS -- seed twin (12) confirms the 3-axis assembly-tolerance compose (placement 6deg + CoM offset 0.03m + zero-bias 3deg) is a property of the recipe, not seed-11 luck. OBSERVATIONS: DR0 gate det gv6/6 5/6 ok (only idx4, the lineage's universal fixed-draw stall crater, prog -0.08); sto gv6/6 6/6 ok mostly (1 minor at idx0). Own-cfg (DR0.35+stack) det gv6/6 med slip 1.11/prog 1.00, 3/6 ok=False (idx0,2,5); sto gv6/6 med slip 1.09/prog 0.92, 2/6 ok=False (idx0,1) -- 0 term, 0 sacrificed legs. Directly comparable to the seed-11 parent's own PASS numbers (own-cfg det med slip 1.26/2 fails, sto med slip 1.28/3 fails, 5/12 degraded total) -- this seed twin lands at 5/12 degraded too, same magnitude, in some places slightly better (lower slip medians). Video (gate+owncfg det contact sheets) shows the family's normal clean six-leg creep gait throughout, no flag-leg/drag/skate. VERDICT: PASS, hypothesis if-true confirmed -- the 3-axis assembly-QA stack is seed-robust, part of the now-CLOSED dep-vref1-r1 DR-compose sweep (20+-for-20+ no-effect). hardware_ready=false (inherits the lineage's paddle-gait economics like every sibling).


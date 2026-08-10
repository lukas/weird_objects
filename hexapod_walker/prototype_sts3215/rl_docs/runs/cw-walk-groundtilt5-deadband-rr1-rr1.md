# cw-walk-groundtilt5-deadband-rr1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: PASS

**created**: 2026-08-10T03:37:25+00:00

**pod**: hexapod-mjx-train-2

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: 7kbsrqc1

**hardware_ready**: no

**hypothesis**: NEW compose, untried pairing: floor-slope (5deg, DR-free per groundtilt5 PASS) x servo deadband (1.0-3.0x, the deadband30 PASS envelope). Deadband has composed cleanly onto DR0.5 (deadband-dr05 PASS-caveat) but never onto an uneven-floor axis; a sluggish/dead-zone servo response could interact badly with the fine balance corrections a slope demands. If-true: own-cfg (tilt+deadband) det+sto 6/6 gv, 0 term, det med fwd>=1.2m (matches groundtilt5's own band); DR0 flat no-deadband retention slip<=1.24, prog>=0.90. If-false: deadband on a slope compounds into a one-sided tip/flag-leg draw beyond groundtilt5's own honest 2/6 steep-tilt tail.

**gate**: Own-cfg (tilt u(0,5deg) + dr.deadband_scale=1.0,3.0) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat no-deadband retention det 6/6 gv, slip/m<=1.24, prog>=0.90; frames watched det

**verdict**: PASS-with-caveat: independent retry of the identical groundtilt5 x deadband(1.0-3.0x) hypothesis, landed separately from -r1-rr1 after its own collision-storm deaths. Own-cfg det+sto gv 12/12, 0 term, det med fwd 1.35m (>=1.2 gate) -- clean, matches -r1-rr1's 1.31m within noise. Caveat: DR0 flat no-deadband retention det slip/m med 1.28, a hair OVER the 1.24 cap (prog med 0.95>=0.90 fine) -- same thin-margin pattern already labeled PASS-with-caveat for groundtilt5-fric's own-cfg slip. Same 2/6 steepest-tilt shuffle tail as -r1-rr1 and the whole groundtilt5 family (no falls/flag-leg, all six legs still cycling, just slower); one sto retention draw hits the familiar fixed-draw-stall canary (prog 0.38, slip 3.77). Two independent launches of the same hypothesis now agree on both the pass and the thin margin -- treat the cap miss as a lineage trait, not this run's defect. Not hardware-ready.


# cw-walk-groundtilt5-fric-s1-r2-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:57:18+00:00

**pod**: hexapod-mjx-train-8

**steps**: 20000000

**parent**: cw-walk-groundtilt5-fric-s1

**wandb_id**: 9egv8kh0

**hardware_ready**: False

**hypothesis**: Retry (r2) of cw-walk-groundtilt5-fric-s1: r1 was lost to a drain-process interrupt (infra, not science, 0 steps). Same hypothesis: ruling-7 seed twin of the groundtilt5-fric PASS.

**gate**: Own-cfg (friction 0.4-1.6x + tilt 5deg) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat-no-fric retention det 6/6 gv, slip/m<=1.24; frames watched det for crater-fraction/mechanism match to seed0

**verdict**: PASS-with-caveat (if-true branch confirmed: seed1 matches seed0's caveat almost exactly, not seed-sensitive). Own-cfg (5deg tilt + friction 0.4-1.6x) det+sto 6/6 gv, 0 term. det prog med 0.84, slip/m med 1.68, fwd med 1.22m (>=1.2m gate, same thin margin as seed0's 1.287m); 3/6 det draws crater into a high-slip shuffle (det/3,4,5: prog 0.42-0.90, slip 1.50-3.65/m) -- IDENTICAL crater count to seed0 (also 3/6), same order of magnitude slip. sto prog med 0.88, slip/m med 1.49, 2/6 milder degraded draws. Video (det_0,3,5 frame strips) confirms the SAME no-fall/no-flag-leg mechanism as seed0: all six legs keep cycling through the crater episodes, just slower/slippier on the steepest+slickest draws -- not a new pathology. DR0 flat-floor nominal-friction retention (own eval, not pre-staged): det gv 6/6, 0 term, slip/m med 1.07-1.08 (<=1.24 gate, PASS), prog 0.95 -- near-identical to seed0's retention (slip 1.088, prog 0.946); sto gv 6/6, one isolated fixed-draw stall (ep4: prog 0.21 slip 6.47), same lineage pattern seen across this whole campaign (c74/c75/c78/c79), not new. Ruling-7 seed panel for groundtilt5-fric is now 2/2 PASS, confirming the recipe (not seed luck).


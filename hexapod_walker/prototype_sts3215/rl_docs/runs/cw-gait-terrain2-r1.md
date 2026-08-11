# cw-gait-terrain2-r1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-11T18:07:13+00:00

**pod**: hexapod-mjx-train-3

**steps**: 2000000

**parent**: cw-dep-vref1-r1

**wandb_id**: tpdzk4dp

**hypothesis**: GAIT CLEANUP P3 lever 1, pre-registered single retry one rung down (operator gate text on cw-gait-terrain2): 72mm from scratch collapsed into a WORSE leg-sacrifice cheat (gate det gait_valid 2/6 leg[3] parked, slip/m 6.6-15.3, fwd 0.12-0.26m over 15s -- far worse than the closed paddle band 1.1-1.5, both on-terrain AND flat-retention). Retry ONE rung down at env.terrain_amp=3.0 (54mm, the zero-training champion probe's mildest amplitude where the champion still gaits cleanly with slip flat at 1.50, no sacrifice) to see if a less extreme difficulty avoids the leg-sacrifice collapse and lets plain progress income select stepping. Same contract stack as terrain1/2 otherwise (from scratch, k_drag_loaded=10 stock, seed 11).

**gate**: By 2M: gait_valid >=5/6 on own terrain (no leg-sacrifice) AND either (i) travels with slip/m<0.6 at matched flat-DR0 travel (lever confirmed) or (ii) paddles at slip 1.1-1.5 matching the champion band with NO sacrifice (lever refuted cleanly, matches the zero-shot champion evidence) -- either way this is the SECOND and LAST attempt at terrain-as-teacher (two-miss rule): any leg-sacrifice or worse-than-champion-band result here closes lever 1 for good, no third rung.


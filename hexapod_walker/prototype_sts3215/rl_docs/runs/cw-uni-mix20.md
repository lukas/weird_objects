# cw-uni-mix20

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FAILED

**created**: 2026-08-10T00:39:28+00:00

**pod**: hexapod-mjx-train-3

**steps**: 18000000

**parent**: cw-uni-blend1-r2

**hypothesis**: Bracket rung of the inverse mix ladder (pair with cw-uni-mix40; blend1-r2 FAILED with rise/lower flat 0 at 10% share each). ONE variable vs blend1-r2: goal-mix walk=0.2/hold=0.1/rise=0.35/lower=0.35, same driving-champion warm start, fixed sim. Walk deliberately minority — stage-1 skill acquisition; a later walk-heavy re-blend consolidates if walk erodes. If-true: rise/lower reach >=5/6 det (even if walk/joystick retention slips, that is stage-2's job to recover). If-false AND mix40 also stays at 0 success: mix share refuted as the lever (two misses = change hypothesis) -> rise/lower income scale audit next (make not-descending unprofitable by construction). If mix20 learns rise but mix40 does not: acquisition threshold is between 25-35% share, re-blend from mix20's checkpoint.

**gate**: rise/lower success >=5/6 det each AND VIDEO: no leg-through-floor; walk retention scored but non-blocking at this rung (JOYSTICK GATE recorded; stage-2 re-blend recovers walk if eroded); frames watched det

**failed_reason**: run never appeared as 'running' in W&B within 240s


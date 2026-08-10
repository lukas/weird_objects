# cw-walk-groundtilt5-fric-s1-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T03:45:38+00:00

**pod**: hexapod-mjx-train-7

**steps**: 20000000

**parent**: cw-walk-groundtilt5-fric

**hypothesis**: Ruling-7 seed twin of cw-walk-groundtilt5-fric PASS (floor-friction 0.4-1.6x composes onto the 5deg-slope axis, but with a caveat: 3/6 own-cfg det draws crater to a high-slip shuffle vs groundtilt5-alone's 2/6). Seed-1 confirms whether that worse-tail fraction is a real recipe or seed luck. If-true: seed1 matches seed0's own-cfg gv 12/12, DR0 retention clean, similarly-sized crater fraction (2-4/6), same no-fall/no-flag-leg mechanism. If-false: seed1's tail is materially better/worse (0-1 or 5-6/6 craters) -- seed-sensitive, don't bank as a recipe.

**gate**: Own-cfg (friction 0.4-1.6x + tilt 5deg) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m; DR0 flat-no-fric retention det 6/6 gv, slip/m<=1.24; frames watched det for crater-fraction/mechanism match to seed0


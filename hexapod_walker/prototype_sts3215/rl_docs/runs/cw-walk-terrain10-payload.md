# cw-walk-terrain10-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: INTENT

**created**: 2026-08-10T02:31:08+00:00

**pod**: hexapod-mjx-train-4

**steps**: 20000000

**parent**: cw-walk-terrain10

**hypothesis**: Terrain (18-36mm bumps, PASSED both rungs, saturated/closed as a standalone line) has never been composed with any other axis. Chassis payload (1.0-1.4x mass, proven free on flat DR0/DR0.5 driving packages -- joyheadfric-payload, lowgait-dr035-payload) hasn't been tested on uneven ground, where added mass changes foot-loading and impact dynamics on the bumps. One variable off cw-walk-terrain10: add dr.mass_scale=1.0,1.4. If-true: own-cfg terrain(amp1.0)+payload det+sto 6/6 gv, 0 term, prog>=0.9, no new tilt/falls on the bumps; flat DR0 no-payload retention clean (slip/m<=1.35). If-false: added mass on uneven ground causes tilt terminations or a flag-leg the flat-payload composes never showed -- terrain+load is NOT free like terrain+flat was.

**gate**: Own-cfg (env.terrain_amp=1.0 + dr.mass_scale=1.0,1.4) det+sto 6/6 @15s: gait_valid 12/12, 0 term, prog median>=0.9, no falls/tilt-term on the bumps; flat DR0 no-payload retention det 6/6 gv, slip/m<=1.35; frames watched det


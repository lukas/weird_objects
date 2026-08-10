# cw-walk-deadband30-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T03:34:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-deadband30

**wandb_id**: doaw60zp

**hypothesis**: NEW compose, untried pairing: servo deadband exposure (1-3x, the deadband30 PASS -- 2/6 own-cfg det draws crater to a slow shuffle, no falls) x payload (mass_scale 1.0-1.5x, the validated payload50 range). Deadband alone already costs precision on the hardest draws; extra chassis mass changes the torque/backlash interaction at the same joints. If-true: own-cfg (deadband+payload) det+sto 6/6 gv, 0 term, det med fwd>=1.2m, crater fraction stays ~2-3/6 (no worse than deadband30 alone); DR0 no-deadband-no-payload retention det 6/6 gv, slip/m<=1.24. If-false: payload pushes the deadband-crater draws into falls or a majority-crater tail -- the two axes don't compose, matches the c61 pattern where mass-axis DR composes sometimes charge nominal walking.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m, crater fraction<=3/6, 0 falls; DR0 nominal retention det 6/6 gv, slip/m<=1.24; frames watched det


# cw-walk-deadband30-payload

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:34:18+00:00

**pod**: hexapod-mjx-train-0

**steps**: 20000000

**parent**: cw-walk-deadband30

**wandb_id**: doaw60zp

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: servo deadband exposure (1-3x, the deadband30 PASS -- 2/6 own-cfg det draws crater to a slow shuffle, no falls) x payload (mass_scale 1.0-1.5x, the validated payload50 range). Deadband alone already costs precision on the hardest draws; extra chassis mass changes the torque/backlash interaction at the same joints. If-true: own-cfg (deadband+payload) det+sto 6/6 gv, 0 term, det med fwd>=1.2m, crater fraction stays ~2-3/6 (no worse than deadband30 alone); DR0 no-deadband-no-payload retention det 6/6 gv, slip/m<=1.24. If-false: payload pushes the deadband-crater draws into falls or a majority-crater tail -- the two axes don't compose, matches the c61 pattern where mass-axis DR composes sometimes charge nominal walking.

**gate**: Own-cfg (dr.deadband_scale=1.0,3.0 + dr.mass_scale=1.0,1.5) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.2m, crater fraction<=3/6, 0 falls; DR0 nominal retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: FAIL vs pre-registered numeric gate (named baseline: parent cw-walk-deadband30/-s1 DR0 retention slip/m 1.00-1.27, own-cfg det med fwd 1.33-1.41m). This run: own-cfg det+sto gv 12/12, 0 term, 0 falls, crater fraction 3/6 (eps 3-5: prog 0.37-0.86, slip 1.28-3.28) at the cap but det med fwd 1.13m misses the 1.2m target; DR0 nominal retention gv 6/6 but det slip/m 1.32-1.45 (med 1.39) is CONSISTENTLY above both the 1.24 cap and the parent's clean 1.09-1.27 band on all 6 draws (not a noisy outlier -- systematic), plus one sto retention draw catastrophically stalled (prog 0.19, slip 8.17, fwd 0.61m, no fall/flag-leg, matches the known rare fixed-draw stochastic-stall canary class seen elsewhere e.g. payload70/longdist-r2). Root cause: same mechanism c61 identified (mass-DR composed onto plain-walk-lineage erodes nominal retention; driving-lineage packages with friction/lateral headroom do NOT show this per joyfric-payload/joyheadfric-payload-r1 PASSes) -- deadband30 is plain-walk lineage, so this is the SAME erosion mechanism recurring, just milder (~15-30% vs c61's +45%). Frames (det+sto, own-cfg + retention): six legs cycling throughout, level body, no flag leg, no dragging drama -- mechanically valid gait, just measurably less efficient. Not a new pathology, not hardware-ready. Closes this compose: plain-walk-lineage + payload keeps costing retention efficiency; payload should be composed onto driving/friction-hardened packages (established headroom) rather than bare deadband/plain-walk lines going forward.


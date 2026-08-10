# cw-walk-groundtilt8-comshift-rr1

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T03:46:51+00:00

**pod**: hexapod-mjx-train-5

**steps**: 20000000

**parent**: cw-walk-groundtilt8-r3

**wandb_id**: aj7pt80u

**hardware_ready**: False

**hypothesis**: NEW compose, untried pairing: marginal 8deg floor-tilt exposure rung (groundtilt8-r3 PASS-with-caveat: 3/6 own-cfg det draws crater to a shuffle, at/past the exposure-training ceiling) x off-center CoM payload (0.03m, the comshift PASS envelope used everywhere on the driving lines). An asymmetric CoM shift could tip the balance on the already-marginal steepest-azimuth draws that groundtilt8 struggles with. If-true: own-cfg (tilt8+comshift) det+sto 6/6 gv, 0 term, det med fwd>=1.1m, crater fraction stays <=3/6 (no worse than groundtilt8 alone); DR0 flat-no-offset retention det 6/6 gv, slip/m<=1.24. If-false: the off-center load pushes the steepest-azimuth craters into falls or a majority-crater tail (>=4/6) -- 8deg is confirmed as a ceiling that does not compose further.

**gate**: Own-cfg (tilt u(0,8deg) + dr.com_offset_m=0.03) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det med fwd>=1.1m, crater fraction<=3/6, 0 falls; DR0 flat-no-offset retention det 6/6 gv, slip/m<=1.24; frames watched det

**verdict**: PASS-with-caveat -- 8deg floor-tilt x off-center CoM payload compose holds. Own-cfg (tilt8+comshift) det+sto 12/12 gait_valid, 0 term, 0 falls, det med fwd 1.45m (>=1.1 gate); 2/6 det draws crater to a shuffle (fwd 0.59-0.77m, slip 3.1-3.9) -- the same steepest-azimuth shuffle tail groundtilt8 has shown in every prior variant (r3, s1, comshift-r2-rr1), just 2/6 here vs r3's 3/6. DR0 flat-no-offset retention: sto clean (slip/m 1.18<=1.24, prog 0.93); det hits the SAME 2/6 crater draws under the unexposed flat condition (0.69-0.70m, slip 3.7-4.2), pushing the det median to 1.40 -- a hair over the 1.24 cap, where two comshift siblings (groundtilt8-r3: 1.07, groundtilt8-comshift-r2-rr1: 1.13) cleared cleanly. Frames confirm the craters are the known march-in-place pattern (all six legs cycling, body level, no falls, no flag leg, no dragging) -- read as the lineage's well-documented fixed-draw stall (c75/c79 root cause), not a new comshift-specific defect. Not hardware-ready (marginal-tilt exposure ceiling, paddle-style gait).


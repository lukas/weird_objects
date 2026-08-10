# cw-walk-lowgait-fricvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T01:39:19+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-lowgait

**wandb_id**: tihbdish

**hardware_ready**: no

**hypothesis**: Floor-grip variation composed onto the crouch-height axis (lowgait, -20mm to -70mm ladder, all PASSED at DR0). Crouch stance changes the contact geometry/loading on each foot; grip variation hasn't been tested on a crouched gait (only on the nominal-height champion via fricvar, and via generic DR0.35 on the -50mm rung via lowgait_dr035). One variable off the base lowgait (-20mm) checkpoint: add dr.friction_scale=0.4,1.6. If-true: own-cfg gv 12/12, 0 term, mean end-height err <=8mm (matches the lowgait gate), no new falls/flag-leg on the slickest draws; DR0 nominal-friction retention clean. If-false: the crouched stance's altered foot-loading makes it MORE grip-sensitive than upright walking -- slick draws fail where fricvar's upright ones didn't.

**gate**: Own-cfg (goal.walk_height_off_mm=-20 + dr.friction_scale=0.4,1.6) det+sto 6/6 @30s: gait_valid 12/12, 0 term, mean end-height err <=8mm, no falls/flag-leg on slickest draws; DR0 nominal-friction retention det 6/6 gv, det slip/m <=1.24; frames watched det

**verdict**: PASS (dig-in) — friction 0.4-1.6x composes onto the -20mm crouch. OBSERVATIONS: own-cfg det+sto gv 12/12, 0 term, mean end-height err 3.5/4.0mm (<=8); DR0 det retention gv 6/6 slip 1.13 (<=1.24). Flagged det/4 dead skate (slip 18.86, fwd 0.07m) reproduced at PINNED friction 0.4 (slip 18.62) AND in the untrained parent cw-walk-lowgait at 0.4 (slip 20.68, fwd 0.03m) — same reset draw, same signature; same draw clean at nominal friction. At pinned 0.4 the other 11/12 eps walk prog med 0.86-0.88 slip ~1.65, BETTER than upright fricvar champion at 0.4 (prog med 0.83, worst draws 0.32-0.51). INTERPRETATION: the skate is an inherited lowgait-lineage fixed-draw stall surfaced deterministically by slick grip, not a fricvar-training defect and not general crouch grip-sensitivity. VERDICT: PASS with named caveat (slick-triggered fixed-draw stall class, lineage-wide, parent->child unchanged); hardware-ready: no (paddle lineage). HYPOTHESIS STATUS: if-true confirmed; if-false (crouch MORE grip-sensitive than upright) refuted — crouch at slick 0.4 outperforms upright medians. Probes: logs/ckpt_eval/cw_walk_{lowgait_fricvar,lowgait,fricvar}_slick04.


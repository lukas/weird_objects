# cw-walk-lowgait-fricvar

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T01:39:19+00:00

**pod**: hexapod-mjx-train-3

**steps**: 20000000

**parent**: cw-walk-lowgait

**wandb_id**: tihbdish

**hypothesis**: Floor-grip variation composed onto the crouch-height axis (lowgait, -20mm to -70mm ladder, all PASSED at DR0). Crouch stance changes the contact geometry/loading on each foot; grip variation hasn't been tested on a crouched gait (only on the nominal-height champion via fricvar, and via generic DR0.35 on the -50mm rung via lowgait_dr035). One variable off the base lowgait (-20mm) checkpoint: add dr.friction_scale=0.4,1.6. If-true: own-cfg gv 12/12, 0 term, mean end-height err <=8mm (matches the lowgait gate), no new falls/flag-leg on the slickest draws; DR0 nominal-friction retention clean. If-false: the crouched stance's altered foot-loading makes it MORE grip-sensitive than upright walking -- slick draws fail where fricvar's upright ones didn't.

**gate**: Own-cfg (goal.walk_height_off_mm=-20 + dr.friction_scale=0.4,1.6) det+sto 6/6 @30s: gait_valid 12/12, 0 term, mean end-height err <=8mm, no falls/flag-leg on slickest draws; DR0 nominal-friction retention det 6/6 gv, det slip/m <=1.24; frames watched det


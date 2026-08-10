# cw-walk-groundtilt5-fric

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T01:40:36+00:00

**pod**: hexapod-mjx-train-1

**steps**: 20000000

**parent**: cw-walk-groundtilt5

**wandb_id**: 6o8yclfc

**hypothesis**: Floor-grip variation composed onto the sloped-floor axis (groundtilt5 PASS: 0-5deg tilt, isolated). Grip and slope are physically correlated on a real floor (a tilted surface is often also a different material) but have only been tested independently or composed with generic DR0.5 (groundtilt_dr05). One variable off groundtilt5: add dr.friction_scale=0.4,1.6 (the same spread that composed cleanly onto champion in fricvar and onto driving packages in joyfric/joyheadfric). If-true: own-cfg gv 12/12, 0 term, det med fwd >=1.2m, same 2/6 steepest-tilt shuffle tail as groundtilt5 alone (no NEW pathology); DR0 flat/no-tilt retention clean. If-false: grip variation interacts with the steepest tilt draws (which already shuffle) to produce falls or a flag leg -- slope+slip is a harder combination than either alone.

**gate**: Own-cfg (dr.ground_tilt_deg=5 + dr.friction_scale=0.4,1.6) det+sto 6/6 @30s: gait_valid 12/12, 0 term, det median fwd >=1.2m, no falls/flag-leg even on steepest+slickest draws; DR0 flat-floor nominal-friction retention det 6/6 gv, det slip/m <=1.24; frames watched det


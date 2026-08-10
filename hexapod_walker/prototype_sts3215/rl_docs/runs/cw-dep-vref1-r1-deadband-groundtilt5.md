# cw-dep-vref1-r1-deadband-groundtilt5

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T18:24:35+00:00

**pod**: hexapod-mjx-train-3

**steps**: 8000000

**parent**: cw-dep-vref1-r1-deadband

**wandb_id**: 1wam62gi

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if a sluggish/dead-zone servo response happens on an unlevel floor at the same time? Both PASSed alone on vref1-r1 (deadband 1.0-3.0x; ground tilt 5deg) but never together -- a sluggish push-off is more likely to matter exactly when the floor is already sloped against you, unlike axes that don't interact mechanically (e.g. IMU mount rotation). Per P0 rule 3, k_current=0 (inherited). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- deadband+slope composes free like every other axis pairing tonight. If-false: sluggish response on a slope defeats the contract-exact obs in a way neither did alone -- flag as a real hardware risk (uneven, imperfectly-responsive floor+servo combo) before deployment.

**gate**: own-cfg (DR0.35 + dr.deadband_scale=1.0,3.0 + dr.ground_tilt_deg=5.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto


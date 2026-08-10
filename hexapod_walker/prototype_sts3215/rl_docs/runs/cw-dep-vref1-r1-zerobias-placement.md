# cw-dep-vref1-r1-zerobias-placement

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: RUNNING

**created**: 2026-08-10T17:55:25+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-zerobias

**wandb_id**: qptzd9zp

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if we combine the two errors that happen at the START of every real session -- an imperfect hand set_zero (per-joint zero bias) AND imperfect hand placement of the robot on the floor -- instead of testing them one at a time? Both PASSed alone on vref1-r1 (zerobias 3deg, placement 6deg) but every real hardware session has BOTH simultaneously (the operator sets zero by hand, then places the robot by hand), unlike some other pairs already tested (e.g. comshift is a body property, not a session-start action) -- this is the natural real-world co-occurrence, not yet tested together. Per P0 rule 3, k_current=0 (already inherited from zerobias's args). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- session-start noise composes free like every other axis pairing tonight. If-false: the two session-start errors interact badly (plausible: a zero-biased joint compounds with placement slop on the SAME joint) -- flag as a real pre-flight risk before the operator's hardware attempt.

**gate**: own-cfg (DR0.35 + joint_zero_bias_deg=3.0 + placement_noise_deg=6.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto


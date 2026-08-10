# cw-dep-vref1-r1-zerobias-placement

<!-- GENERATED from experiments.json by launch_run.py — do not edit -->

**status**: FINISHED

**created**: 2026-08-10T17:55:25+00:00

**pod**: hexapod-mjx-train-11

**steps**: 20000000

**parent**: cw-dep-vref1-r1-zerobias

**wandb_id**: qptzd9zp

**hardware_ready**: False

**hypothesis**: Plain English: does the checkpoint headed for tonight's hardware attempt still walk cleanly if we combine the two errors that happen at the START of every real session -- an imperfect hand set_zero (per-joint zero bias) AND imperfect hand placement of the robot on the floor -- instead of testing them one at a time? Both PASSed alone on vref1-r1 (zerobias 3deg, placement 6deg) but every real hardware session has BOTH simultaneously (the operator sets zero by hand, then places the robot by hand), unlike some other pairs already tested (e.g. comshift is a body property, not a session-start action) -- this is the natural real-world co-occurrence, not yet tested together. Per P0 rule 3, k_current=0 (already inherited from zerobias's args). If-true: own-cfg det+sto 6/6 gv, 0 term, slip/m within vref1-r1's own band -- session-start noise composes free like every other axis pairing tonight. If-false: the two session-start errors interact badly (plausible: a zero-biased joint compounds with placement slop on the SAME joint) -- flag as a real pre-flight risk before the operator's hardware attempt.

**gate**: own-cfg (DR0.35 + joint_zero_bias_deg=3.0 + placement_noise_deg=6.0) det+sto 6/6 @15s gait_valid 12/12, 0 term, slip/m within vref1-r1's own band +-20%; DR0-no-override retention det 6/6 gv reproducing vref1-r1's own band; video frames watched det+sto

**verdict**: PASS -- combining the two session-start errors (3deg joint zero-bias + 6deg hand-placement noise) TOGETHER composes free onto the contract-exact hardware-candidate checkpoint, same as each did alone. Own-cfg (DR0.35+both overrides) det gv 6/6 (prog med 1.01, slip med 1.08), sto gv 6/6 (prog med 0.84, slip med 1.27) -- both medians inside vref1-r1's own band (det 0.89-1.13, sto 1.13-1.36). 0 term either pass, 0 sacrificed legs any episode. Degraded draws (det/5 crater prog 0.63/slip 2.74; sto/0-1 crater prog 0.60/0.43) match the EXACT lineage fixed-draw fingerprint (det/5, sto/0, sto/1) already seen on multiple sibling 2-axis composes today (tiltnoise_gyronoise, torquescale_deadband) -- video-checked (det/0 clean, det/5, sto/0, sto/1 craters): level body, all six legs still cycling in every frame, no flag-leg/drag/skate, just a slow march-in-place on that fixed draw. DR0-no-override retention clean: det gv 6/6, prog med 1.01, slip med 1.00 (matches parent exactly). If-false (bad interaction between the two session-start errors) is refuted -- they compose free like every other axis pairing tonight.

